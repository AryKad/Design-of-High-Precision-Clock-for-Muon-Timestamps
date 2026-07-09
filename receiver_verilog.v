// ============================================================================
// MUON RECEIVER v4 -- SIMPLE FIFO, NO TIMER
// Nexys A7-100T | 100 MHz | 921600 baud
// Top module: receiver_top
//
// Simplified approach:
//   - Single BRAM FIFO (write pointer + read pointer)
//   - Validator writes 4 edges directly to FIFO
//   - UART reader drains FIFO continuously
//   - No timer, no bank swap, no double buffer
//   - If UART is slower than writes, FIFO absorbs the difference
//
// At 1kHz pairs: 4000 writes/sec, UART handles ~15000 edges/sec
// So FIFO drains faster than it fills -- no overflow
// ============================================================================

`timescale 1ns / 1ps


// ============================================================================
// MODULE 1: EDGE DETECTOR WITH PRE-DEBOUNCE TIMESTAMP
// Timestamps are captured on the RAW edge (before debounce validation)
// Debounce runs in parallel for FILTER_CYCLES
// If signal stays stable for FILTER_CYCLES -> edge_valid pulses with
// the timestamp from when the transition FIRST occurred
// If signal reverts before FILTER_CYCLES -> timestamp discarded silently
//
// This eliminates the debounce delay from timestamp measurements
// so width = T_fall - T_rise = actual pulse width with no systematic offset
// ============================================================================
module edge_detector (
    input  wire        clk,
    input  wire        sig_in,       // raw JA_IN (after 2-FF sync only)
    input  wire [30:0] timestamp,
    output reg         edge_valid,
    output reg         edge_rising,
    output reg  [30:0] edge_ts,
    output reg         sig_out       // debounced output (for LED display)
);

parameter FILTER_CYCLES = 5;  // 50ns at 100MHz

// 2-FF synchroniser
reg [1:0] sync_ff = 2'b00;
always @(posedge clk) sync_ff <= {sync_ff[0], sig_in};
wire sig_sync = sync_ff[1];

reg [3:0]  cnt         = 0;
reg        state       = 0;    // current debounced state
reg [30:0] ts_captured = 0;    // timestamp captured at first transition
reg        in_transit  = 0;    // currently waiting for debounce confirmation
reg        transit_dir = 0;    // direction of pending transition (1=rising)

always @(posedge clk) begin
    edge_valid <= 0;

    if (sig_sync == state) begin
        // Signal matches debounced state -- stable, reset counter
        cnt        <= 0;
        in_transit <= 0;
    end else begin
        // Signal differs from debounced state -- potential transition
        if (!in_transit) begin
            // First cycle of this transition -- capture timestamp NOW
            ts_captured <= timestamp;
            transit_dir <= sig_sync;
            in_transit  <= 1;
            cnt         <= 1;
        end else begin
            if (cnt == FILTER_CYCLES - 1) begin
                // Stable for FILTER_CYCLES -- confirmed edge
                state      <= sig_sync;
                cnt        <= 0;
                in_transit <= 0;
                // Output edge with timestamp from when transition STARTED
                edge_valid  <= 1;
                edge_rising <= transit_dir;
                edge_ts     <= ts_captured;
            end else begin
                cnt <= cnt + 1;
            end
        end
    end

    sig_out <= state;
end

endmodule


// ============================================================================
// MODULE 3: PULSE VALIDATOR
// ============================================================================
module pulse_validator (
    input  wire        clk,
    input  wire        edge_valid,
    input  wire        edge_rising,
    input  wire [30:0] edge_ts,

    output reg         out_valid,
    output reg         out_rising,
    output reg  [30:0] out_ts,
    output reg  [30:0] last_delta_t,
    output reg         pair_blink
);

localparam GAP_MAX = 31'd5000;

localparam V_WAIT_R1 = 3'd0;
localparam V_WAIT_F1 = 3'd1;
localparam V_WAIT_R2 = 3'd2;
localparam V_WAIT_F2 = 3'd3;
localparam V_EMIT    = 3'd4;

reg [2:0]  v_state   = V_WAIT_R1;
reg [30:0] t_rise1   = 0;
reg [30:0] t_fall1   = 0;
reg [30:0] t_rise2   = 0;
reg [30:0] t_fall2   = 0;
reg [1:0]  emit_cnt  = 0;
reg [19:0] blink_cnt = 0;

function [30:0] gap_calc;
    input [30:0] t_new, t_old;
    begin
        if (t_new >= t_old)
            gap_calc = t_new - t_old;
        else
            gap_calc = (31'h7FFFFFFF - t_old) + t_new + 1;
    end
endfunction

always @(posedge clk) begin
    out_valid <= 0;

    if (blink_cnt > 0) begin
        blink_cnt  <= blink_cnt - 1;
        pair_blink <= 1;
    end else begin
        pair_blink <= 0;
    end

    case (v_state)
        V_WAIT_R1: begin
            if (edge_valid && edge_rising) begin
                t_rise1 <= edge_ts;
                v_state <= V_WAIT_F1;
            end
        end
        V_WAIT_F1: begin
            if (edge_valid && !edge_rising) begin
                t_fall1 <= edge_ts;
                v_state <= V_WAIT_R2;
            end
        end
        V_WAIT_R2: begin
            if (edge_valid && edge_rising) begin
                begin : gap_check
                    reg [30:0] gap;
                    gap = gap_calc(edge_ts, t_rise1);
                    if (gap > GAP_MAX) begin
                        t_rise1 <= edge_ts;
                        v_state <= V_WAIT_F1;
                    end else begin
                        t_rise2      <= edge_ts;
                        last_delta_t <= gap;
                        v_state      <= V_WAIT_F2;
                    end
                end
            end
        end
        V_WAIT_F2: begin
            if (edge_valid && !edge_rising) begin
                t_fall2   <= edge_ts;
                emit_cnt  <= 0;
                blink_cnt <= 20'd999_999;
                v_state   <= V_EMIT;
            end
        end
        V_EMIT: begin
            out_valid <= 1;
            case (emit_cnt)
                2'd0: begin out_rising<=1; out_ts<=t_rise1; end
                2'd1: begin out_rising<=0; out_ts<=t_fall1; end
                2'd2: begin out_rising<=1; out_ts<=t_rise2; end
                2'd3: begin out_rising<=0; out_ts<=t_fall2; end
            endcase
            if (emit_cnt == 2'd3)
                v_state <= V_WAIT_R1;
            else
                emit_cnt <= emit_cnt + 1;
        end
        default: v_state <= V_WAIT_R1;
    endcase
end

endmodule


// ============================================================================
// MODULE 4: BRAM FIFO -- inferred BRAM
// (* ram_style = "block" *) forces Vivado to use BRAM tiles
// 1024 x 32-bit = 1 BRAM tile
// ============================================================================
module bram_fifo (
    input  wire        clk,
    input  wire        wr_en,
    input  wire [31:0] wr_data,
    input  wire        rd_en,
    output reg  [31:0] rd_data,
    output wire        empty,
    output wire        full,
    output wire        dbg_not_empty
);

localparam DEPTH  = 1024;
localparam ADDR_W = 10;

(* ram_style = "block" *)
reg [31:0] mem [0:DEPTH-1];

reg [ADDR_W-1:0] wr_ptr = 0;
reg [ADDR_W-1:0] rd_ptr = 0;
reg [ADDR_W:0]   count  = 0;

assign empty         = (count == 0);
assign full          = (count == DEPTH);
assign dbg_not_empty = (count > 0);

always @(posedge clk)
    if (wr_en) mem[wr_ptr] <= wr_data;

always @(posedge clk)
    if (rd_en) rd_data <= mem[rd_ptr];

always @(posedge clk) begin
    if (wr_en && !full && rd_en && !empty) begin
        wr_ptr <= wr_ptr + 1;
        rd_ptr <= rd_ptr + 1;
    end else if (wr_en && !full) begin
        wr_ptr <= wr_ptr + 1;
        count  <= count + 1;
    end else if (rd_en && !empty) begin
        rd_ptr <= rd_ptr + 1;
        count  <= count - 1;
    end
end

endmodule


// ============================================================================
// MODULE 5: UART TX
// Based on working uart_tx_packet from generator code
// Counts UP like the original (proven to work at 115200 baud)
// Adapted for 921600 baud and 6-byte packet format
//
// Packet: [{RF,TS[30:24]}, TS[23:16], TS[15:8], TS[7:0], 0x0D, 0x0A]
// tx_ready HIGH when idle and ready for next edge
// ============================================================================
module uart_tx (
    input  wire        clk,
    input  wire        rst,
    input  wire        tx_valid,
    input  wire        tx_rising,
    input  wire [30:0] tx_ts,
    output reg         tx,
    output wire        tx_ready,
    output wire [1:0]  dbg_tx_state
);

parameter CLK_FREQ  = 100_000_000;
parameter BAUD_RATE = 921_600;
localparam integer BIT_PERIOD = CLK_FREQ / BAUD_RATE;  // 108 cycles

// States -- same as working old code
localparam S_IDLE  = 2'd0;
localparam S_START = 2'd1;
localparam S_DATA  = 2'd2;
localparam S_STOP  = 2'd3;

reg [1:0]  state       = S_IDLE;
reg [31:0] bit_cnt     = 0;   // 32-bit counter -- safe, no overflow
reg [2:0]  bit_idx     = 0;
reg [3:0]  byte_idx    = 0;
reg [7:0]  tx_byte     = 0;
reg [47:0] packet      = 0;   // 6 bytes

assign tx_ready     = (state == S_IDLE);
assign dbg_tx_state = state;

always @(posedge clk) begin
    if (rst) begin
        state    <= S_IDLE;
        tx       <= 1;
        bit_cnt  <= 0;
        bit_idx  <= 0;
        byte_idx <= 0;
    end else begin
        case (state)

            S_IDLE: begin
                tx <= 1;
                if (tx_valid) begin
                    // Pack 6 bytes: byte0 at bits[7:0] sent first
                    // Format: [0xAA][{RF,TS[30:24]}][TS[23:16]][TS[15:8]][TS[7:0]][checksum]
                    // checksum = XOR of bytes 1-4
                    packet[7:0]   <= 8'hAA;
                    packet[15:8]  <= {tx_rising, tx_ts[30:24]};
                    packet[23:16] <= tx_ts[23:16];
                    packet[31:24] <= tx_ts[15:8];
                    packet[39:32] <= tx_ts[7:0];
                    packet[47:40] <= {tx_rising, tx_ts[30:24]}
                                   ^ tx_ts[23:16]
                                   ^ tx_ts[15:8]
                                   ^ tx_ts[7:0];
                    byte_idx <= 0;
                    bit_cnt  <= 0;
                    state    <= S_START;
                end
            end

            S_START: begin
                tx <= 0;  // start bit
                if (bit_cnt >= BIT_PERIOD - 1) begin
                    bit_cnt  <= 0;
                    bit_idx  <= 0;
                    // Load current byte
                    tx_byte  <= packet[byte_idx * 8 +: 8];
                    state    <= S_DATA;
                end else begin
                    bit_cnt <= bit_cnt + 1;
                end
            end

            S_DATA: begin
                tx <= tx_byte[bit_idx];  // LSB first
                if (bit_cnt >= BIT_PERIOD - 1) begin
                    bit_cnt <= 0;
                    if (bit_idx == 7) begin
                        state <= S_STOP;
                    end else begin
                        bit_idx <= bit_idx + 1;
                    end
                end else begin
                    bit_cnt <= bit_cnt + 1;
                end
            end

            S_STOP: begin
                tx <= 1;  // stop bit
                if (bit_cnt >= BIT_PERIOD - 1) begin
                    bit_cnt <= 0;
                    if (byte_idx == 5) begin
                        state <= S_IDLE;  // all 6 bytes sent
                    end else begin
                        byte_idx <= byte_idx + 1;
                        state    <= S_START;
                    end
                end else begin
                    bit_cnt <= bit_cnt + 1;
                end
            end

            default: state <= S_IDLE;

        endcase
    end
end

endmodule


// ============================================================================
// MODULE 6: FIFO DRAIN -- reads FIFO and feeds UART TX
// Fixed: properly handles BRAM read latency and UART busy wait
// State machine:
//   IDLE: wait for data in FIFO and UART ready
//   READ: issue rd_en, latch data next cycle
//   SEND: assert uart_valid, wait here until UART accepts
// ============================================================================
module fifo_drain (
    input  wire        clk,
    input  wire        fifo_empty,
    input  wire [31:0] fifo_data,
    input  wire        uart_ready,

    output reg         fifo_rd_en,
    output reg         uart_valid,
    output reg         uart_rising,
    output reg  [30:0] uart_ts
);

// State machine for draining FIFO to UART
// Correct BRAM read timing:
//   Cycle 0 (D_IDLE):  assert rd_en, go to D_WAIT
//   Cycle 1 (D_WAIT):  rd_en was high, BRAM now computing rd_data
//                      rd_en goes low this cycle
//   Cycle 2 (D_LATCH): BRAM output rd_data is now stable and valid
//                      capture into data_latch
//   Cycle 3 (D_SEND):  send data_latch to UART

localparam D_IDLE  = 2'd0;
localparam D_WAIT  = 2'd1;   // wait for BRAM read latency
localparam D_LATCH = 2'd2;   // capture stable BRAM output
localparam D_SEND  = 2'd3;   // send to UART

reg [1:0]  d_state    = D_IDLE;
reg [31:0] data_latch = 0;

always @(posedge clk) begin
    fifo_rd_en <= 0;
    uart_valid <= 0;

    case (d_state)

        D_IDLE: begin
            if (!fifo_empty && uart_ready) begin
                fifo_rd_en <= 1;   // pulse rd_en: BRAM reads mem[rd_ptr]
                d_state    <= D_WAIT;
            end
        end

        D_WAIT: begin
            // rd_en fired last cycle, BRAM is computing output
            // rd_data will be valid at END of this cycle (next posedge)
            d_state <= D_LATCH;
        end

        D_LATCH: begin
            // rd_data is now fully stable -- safe to capture
            data_latch <= fifo_data;
            d_state    <= D_SEND;
        end

        D_SEND: begin
            if (uart_ready) begin
                uart_valid  <= 1;
                uart_rising <= data_latch[31];
                uart_ts     <= data_latch[30:0];
                d_state     <= D_IDLE;
            end
        end

        default: d_state <= D_IDLE;
    endcase
end

endmodule


// ============================================================================
// MODULE 7: SEVEN SEGMENT DISPLAY
// ============================================================================
module seven_seg (
    input  wire        clk,
    input  wire [30:0] delta_t_cycles,
    output reg  [6:0]  seg,
    output reg  [7:0]  an
);

reg [3:0] d0=0, d1=0, d2=0, d3=0;

always @(posedge clk) begin
    d0 <= (delta_t_cycles / 100) % 10;
    d1 <= (delta_t_cycles / 1000) % 10;
    d2 <= (delta_t_cycles / 10000) % 10;
    d3 <= (delta_t_cycles / 100000) % 10;
end

function [6:0] seg_decode;
    input [3:0] digit;
    case (digit)
        4'd0: seg_decode = 7'b1000000;
        4'd1: seg_decode = 7'b1111001;
        4'd2: seg_decode = 7'b0100100;
        4'd3: seg_decode = 7'b0110000;
        4'd4: seg_decode = 7'b0011001;
        4'd5: seg_decode = 7'b0010010;
        4'd6: seg_decode = 7'b0000010;
        4'd7: seg_decode = 7'b1111000;
        4'd8: seg_decode = 7'b0000000;
        4'd9: seg_decode = 7'b0010000;
        default: seg_decode = 7'b1111111;
    endcase
endfunction

reg [16:0] mux_cnt   = 0;
reg [2:0]  digit_sel = 0;

always @(posedge clk) begin
    if (mux_cnt == 17'd99_999) begin
        mux_cnt   <= 0;
        digit_sel <= digit_sel + 1;
    end else mux_cnt <= mux_cnt + 1;
end

always @(posedge clk) begin
    case (digit_sel)
        3'd0: begin an<=8'b11111110; seg<=seg_decode(d0); end
        3'd1: begin an<=8'b11111101; seg<=seg_decode(d1); end
        3'd2: begin an<=8'b11111011; seg<=seg_decode(d2); end
        3'd3: begin an<=8'b11110111; seg<=seg_decode(d3); end
        3'd4: begin an<=8'b11101111; seg<=7'b1111111;     end
        3'd5: begin an<=8'b11011111; seg<=7'b1111111;     end
        3'd6: begin an<=8'b10111111; seg<=7'b1111111;     end
        3'd7: begin an<=8'b01111111; seg<=7'b1111111;     end
        default: begin an<=8'b11111111; seg<=7'b1111111;  end
    endcase
end

endmodule


// ============================================================================
// MODULE 8: TOP LEVEL
// ============================================================================
module receiver_top (
    input  wire        CLK100MHZ,
    input  wire        JA_IN,
    output wire        UART_RXD_OUT,
    output wire [6:0]  SEG,
    output wire [7:0]  AN,
    output wire [15:0] LED
);

// Startup reset -- hold reset for 16 cycles to ensure clean init
reg [3:0] rst_cnt   = 0;
reg       sys_rst   = 1;
always @(posedge CLK100MHZ) begin
    if (rst_cnt < 4'hF) begin
        rst_cnt <= rst_cnt + 1;
        sys_rst <= 1;
    end else begin
        sys_rst <= 0;
    end
end

// Free running timestamp
reg [30:0] timestamp = 0;
always @(posedge CLK100MHZ)
    if (!sys_rst) timestamp <= timestamp + 1;

// Edge detector with built-in pre-debounce timestamp capture
// Takes raw JA_IN, timestamps on first transition, validates over 5 cycles
wire        edge_valid;
wire        edge_rising;
wire [30:0] edge_ts;
wire        ja_clean;   // debounced output for LED

edge_detector u_edge (
    .clk        (CLK100MHZ),
    .sig_in     (JA_IN),
    .timestamp  (timestamp),
    .edge_valid (edge_valid),
    .edge_rising(edge_rising),
    .edge_ts    (edge_ts),
    .sig_out    (ja_clean)
);

// Pulse validator
wire        val_valid;
wire        val_rising;
wire [30:0] val_ts;
wire [30:0] last_delta_t;
wire        pair_blink;
pulse_validator u_val (
    .clk         (CLK100MHZ),
    .edge_valid  (edge_valid),
    .edge_rising (edge_rising),
    .edge_ts     (edge_ts),
    .out_valid   (val_valid),
    .out_rising  (val_rising),
    .out_ts      (val_ts),
    .last_delta_t(last_delta_t),
    .pair_blink  (pair_blink)
);

// BRAM FIFO
wire        fifo_empty;
wire        fifo_full;
wire [31:0] fifo_rd_data;
wire        fifo_rd_en;
wire        fifo_not_empty;

bram_fifo u_fifo (
    .clk         (CLK100MHZ),
    .wr_en       (val_valid),
    .wr_data     ({val_rising, val_ts}),
    .rd_en       (fifo_rd_en),
    .rd_data     (fifo_rd_data),
    .empty       (fifo_empty),
    .full        (fifo_full),
    .dbg_not_empty(fifo_not_empty)
);

// UART TX
wire uart_ready;
wire drain_valid;
wire drain_rising;
wire [30:0] drain_ts;

fifo_drain u_drain (
    .clk        (CLK100MHZ),
    .fifo_empty (fifo_empty),
    .fifo_data  (fifo_rd_data),
    .uart_ready (uart_ready),
    .fifo_rd_en (fifo_rd_en),
    .uart_valid (drain_valid),
    .uart_rising(drain_rising),
    .uart_ts    (drain_ts)
);

wire [1:0] tx_state_dbg;
uart_tx u_tx (
    .clk         (CLK100MHZ),
    .rst         (sys_rst),
    .tx_valid    (drain_valid),
    .tx_rising   (drain_rising),
    .tx_ts       (drain_ts),
    .tx          (UART_RXD_OUT),
    .tx_ready    (uart_ready),
    .dbg_tx_state(tx_state_dbg)
);

// Seven segment
seven_seg u_seg (
    .clk           (CLK100MHZ),
    .delta_t_cycles(last_delta_t),
    .seg           (SEG),
    .an            (AN)
);

// Heartbeat
reg [25:0] hb_cnt = 0;
reg        hb_led = 0;
always @(posedge CLK100MHZ) begin
    if (hb_cnt == 26'd49_999_999) begin
        hb_cnt <= 0;
        hb_led <= ~hb_led;
    end else hb_cnt <= hb_cnt + 1;
end

// LEDs
assign LED[0]  = ja_clean;        // live input
assign LED[1]  = pair_blink;      // valid pair
assign LED[2]  = drain_valid;     // UART sending
assign LED[3]  = uart_ready;      // UART idle
assign LED[4]  = fifo_not_empty;  // FIFO has data
assign LED[5]  = fifo_full;       // FIFO full (overflow warning)
assign LED[6]  = val_valid;       // validator output
assign LED[7]  = fifo_rd_en;      // FIFO being read
assign LED[8]  = drain_valid;     // drain sending to UART
assign LED[9]  = hb_led;          // heartbeat
assign LED[10] = tx_state_dbg[0];  // tx_state bit0
assign LED[11] = tx_state_dbg[1];  // tx_state bit1
assign LED[12] = 0;
assign LED[13] = 0;
assign LED[14] = 0;
assign LED[15] = 0;

endmodule
