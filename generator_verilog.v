// =============================================================================
//  pulse_gen.v  -  Dual Pulse Pair Generator with UART control
//  Target  : Basys 3 (100 MHz)
//
//  Generates a repeating pair of pulses:
//    [PULSE_A]---inter-pulse gap---[PULSE_B]---pair gap---[PULSE_A]...
//
//  UART protocol (115200 8N1), ASCII commands terminated with '\n':
//    W<nnn>\n   -> pulse width      in 10ns steps  (20..50    => 200ns..500ns)
//    I<nnnn>\n  -> inter-pulse gap  in 100ns steps (100..1000 => 10us..100us)
//    G<nnnnn>\n -> pair gap         in 100ns steps (3334..10000 => 333.4us..1000us)
//    S\n        -> start generation
//    T\n        -> stop  generation
//    ?\n        -> query: replies "W<nnn>,I<nnnn>,G<nnnnn>,RUN\n" or "...,STP\n"
//
//  Outputs:
//    pulse_out -> JA1 (J1) + LED0
//    LED1      -> running indicator
//    LED2      -> RX activity blink
//    LED3      -> TX ready
// =============================================================================
`timescale 1ns/1ps

module pulse_gen (
    input        clk,        // 100 MHz  W5
    input        btnc,       // reset    T17
    input        uart_rx,    // B18  (FTDI TX -> FPGA RX)
    output       uart_tx,    // A18  (FPGA TX -> FTDI RX)
    output       pulse_out,  // JA1  J1
    output [3:0] led
);

wire rst_n = ~btnc;

// -------------------------------------------------------------------------
//  Clock parameters: 100 MHz => 1 tick = 10 ns
// -------------------------------------------------------------------------
localparam BAUD_DIV    = 868;    // 100e6 / 115200
localparam GAP_TICKS   = 10;     // 1 gap unit = 100 ns = 10 ticks

localparam WIDTH_MIN   = 20;     // units of 10ns  (200ns)
localparam WIDTH_MAX   = 50;     // units of 10ns  (500ns)

localparam INTER_MIN   = 100;    // units of 100ns (10us)
localparam INTER_MAX   = 1000;   // units of 100ns (100us)

localparam GAP_MIN     = 3334;   // units of 100ns (333.4us)
localparam GAP_MAX     = 10000;  // units of 100ns (1000us)

// =========================================================================
//  UART RX
// =========================================================================
reg [1:0] rx_st;
reg [9:0] rx_cnt;
reg [2:0] rx_bit;
reg [7:0] rx_sr;
reg       rx_valid;
reg [7:0] rx_data;
reg       rx1, rx2;

localparam [1:0] RXI=0, RXS=1, RXD=2, RXP=3;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx1<=1; rx2<=1; rx_st<=RXI; rx_cnt<=0;
        rx_bit<=0; rx_sr<=0; rx_valid<=0; rx_data<=0;
    end else begin
        rx1 <= uart_rx; rx2 <= rx1;
        rx_valid <= 0;
        case (rx_st)
            RXI: if (!rx2) begin rx_cnt<=BAUD_DIV/2; rx_st<=RXS; end
            RXS: if (rx_cnt==0) begin
                     if (!rx2) begin rx_cnt<=BAUD_DIV-1; rx_bit<=0; rx_st<=RXD; end
                     else rx_st<=RXI;
                 end else rx_cnt<=rx_cnt-1;
            RXD: if (rx_cnt==0) begin
                     rx_sr<={rx2,rx_sr[7:1]};
                     rx_cnt<=BAUD_DIV-1;
                     if (rx_bit==7) rx_st<=RXP;
                     else rx_bit<=rx_bit+1;
                 end else rx_cnt<=rx_cnt-1;
            RXP: if (rx_cnt==0) begin
                     if (rx2) begin rx_data<=rx_sr; rx_valid<=1; end
                     rx_st<=RXI;
                 end else rx_cnt<=rx_cnt-1;
        endcase
    end
end

// =========================================================================
//  UART TX
// =========================================================================
reg       tx_start;
reg [7:0] tx_byte;
wire      tx_ready;

uart_tx_simple #(.BAUD_DIV(BAUD_DIV)) u_tx (
    .clk(clk), .rst_n(rst_n),
    .tx_start(tx_start), .tx_byte(tx_byte),
    .tx_ready(tx_ready), .uart_tx(uart_tx)
);

// =========================================================================
//  ASCII decimal encoder function (5 digits, MSB first)
// =========================================================================
function [39:0] to_ascii5;
    input [16:0] val;
    reg [16:0] v;
    reg [3:0]  d4,d3,d2,d1,d0;
    begin
        v  = val;
        d4 = v/10000; v = v % 10000;
        d3 = v/1000;  v = v % 1000;
        d2 = v/100;   v = v % 100;
        d1 = v/10;    d0 = v % 10;
        to_ascii5 = {d4+"0", d3+"0", d2+"0", d1+"0", d0+"0"};
    end
endfunction

// =========================================================================
//  Registers
// =========================================================================
// TX queue
reg [7:0]  txq     [0:31];
reg [4:0]  txq_wr;
reg [4:0]  txq_rd;
reg [5:0]  txq_cnt;

// Reply buffer
reg [7:0]  rep_buf  [0:23];
reg [4:0]  rep_len;
reg [4:0]  rep_idx;
reg        rep_sending;

// Command parser
reg [1:0]  cmd_st;
reg [7:0]  cmd_char;
reg [13:0] cmd_accum;
reg [2:0]  cmd_digits;

// Pulse config
reg [5:0]  cfg_width;    // 20..50    x10ns
reg [9:0]  cfg_inter;    // 100..1000 x100ns
reg [13:0] cfg_gap;      // 3334..10000 x100ns
reg        running;

// ASCII temp regs
reg [39:0] wa_tmp;
reg [39:0] ia_tmp;
reg [39:0] ga_tmp;

localparam [1:0] CS_IDLE=0, CS_NUM=1;

// =========================================================================
//  Single always block: TX drainer + reply drainer + command parser
// =========================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        txq_wr      <= 0;
        txq_rd      <= 0;
        txq_cnt     <= 0;
        tx_start    <= 0;
        tx_byte     <= 0;
        rep_sending <= 0;
        rep_idx     <= 0;
        rep_len     <= 0;
        cmd_st      <= CS_IDLE;
        cmd_char    <= 0;
        cmd_accum   <= 0;
        cmd_digits  <= 0;
        cfg_width   <= 20;
        cfg_inter   <= 400;    // default 40us
        cfg_gap     <= 3334;
        running     <= 0;
        wa_tmp      <= 0;
        ia_tmp      <= 0;
        ga_tmp      <= 0;
    end else begin

        // -----------------------------------------------------------------
        //  TX drainer
        // -----------------------------------------------------------------
        tx_start <= 0;
        if (txq_cnt > 0 && tx_ready && !tx_start) begin
            tx_byte  <= txq[txq_rd];
            tx_start <= 1;
            txq_rd   <= txq_rd + 1;
            txq_cnt  <= txq_cnt - 1;
        end

        // -----------------------------------------------------------------
        //  Reply buffer drainer
        // -----------------------------------------------------------------
        if (rep_sending) begin
            if (rep_idx < rep_len) begin
                if (txq_cnt < 30) begin
                    txq[txq_wr] <= rep_buf[rep_idx];
                    txq_wr      <= txq_wr + 1;
                    txq_cnt     <= txq_cnt + 1;
                    rep_idx     <= rep_idx + 1;
                end
            end else begin
                rep_sending <= 0;
            end
        end

        // -----------------------------------------------------------------
        //  Command parser
        // -----------------------------------------------------------------
        if (rx_valid && !rep_sending) begin
            case (cmd_st)
                CS_IDLE: begin
                    if (rx_data=="W" || rx_data=="G" || rx_data=="I") begin
                        cmd_char   <= rx_data;
                        cmd_accum  <= 0;
                        cmd_digits <= 0;
                        cmd_st     <= CS_NUM;
                    end else if (rx_data=="S") begin
                        running     <= 1;
                        rep_buf[0]  <= "O"; rep_buf[1] <= "K";
                        rep_buf[2]  <= 8'h0A;
                        rep_len <= 3; rep_idx <= 0; rep_sending <= 1;
                    end else if (rx_data=="T") begin
                        running     <= 0;
                        rep_buf[0]  <= "O"; rep_buf[1] <= "K";
                        rep_buf[2]  <= 8'h0A;
                        rep_len <= 3; rep_idx <= 0; rep_sending <= 1;
                    end else if (rx_data=="?") begin
                        // "W<nnn>,I<nnnn>,G<nnnnn>,RUN\n"  (21 bytes max)
                        wa_tmp = to_ascii5(cfg_width);
                        ia_tmp = to_ascii5(cfg_inter);
                        ga_tmp = to_ascii5(cfg_gap);
                        rep_buf[0]  <= "W";
                        rep_buf[1]  <= wa_tmp[31:24];
                        rep_buf[2]  <= wa_tmp[23:16];
                        rep_buf[3]  <= wa_tmp[15: 8];
                        rep_buf[4]  <= ",";
                        rep_buf[5]  <= "I";
                        rep_buf[6]  <= ia_tmp[31:24];
                        rep_buf[7]  <= ia_tmp[23:16];
                        rep_buf[8]  <= ia_tmp[15: 8];
                        rep_buf[9]  <= ia_tmp[ 7: 0];
                        rep_buf[10] <= ",";
                        rep_buf[11] <= "G";
                        rep_buf[12] <= ga_tmp[39:32];
                        rep_buf[13] <= ga_tmp[31:24];
                        rep_buf[14] <= ga_tmp[23:16];
                        rep_buf[15] <= ga_tmp[15: 8];
                        rep_buf[16] <= ga_tmp[ 7: 0];
                        rep_buf[17] <= ",";
                        rep_buf[18] <= running ? "R" : "S";
                        rep_buf[19] <= running ? "U" : "T";
                        rep_buf[20] <= running ? "N" : "P";
                        rep_buf[21] <= 8'h0A;
                        rep_len     <= 22;
                        rep_idx     <= 0;
                        rep_sending <= 1;
                    end
                end

                CS_NUM: begin
                    if (rx_data >= "0" && rx_data <= "9") begin
                        cmd_accum  <= cmd_accum * 10 + (rx_data - 8'h30);
                        cmd_digits <= cmd_digits + 1;
                    end else if (rx_data==8'h0A || rx_data==8'h0D) begin
                        if (cmd_char=="W") begin
                            if (cmd_accum>=WIDTH_MIN && cmd_accum<=WIDTH_MAX)
                                cfg_width <= cmd_accum[5:0];
                        end else if (cmd_char=="I") begin
                            if (cmd_accum>=INTER_MIN && cmd_accum<=INTER_MAX)
                                cfg_inter <= cmd_accum[9:0];
                        end else begin
                            if (cmd_accum>=GAP_MIN && cmd_accum<=GAP_MAX)
                                cfg_gap <= cmd_accum;
                        end
                        rep_buf[0]  <= "O"; rep_buf[1] <= "K";
                        rep_buf[2]  <= 8'h0A;
                        rep_len <= 3; rep_idx <= 0; rep_sending <= 1;
                        cmd_st  <= CS_IDLE;
                    end
                end

                default: cmd_st <= CS_IDLE;
            endcase
        end

    end
end

// =========================================================================
//  Pulse Generation FSM
//  Sequence: PULSE_A -> INTER(cfg_inter) -> PULSE_B -> GAP(cfg_gap) -> repeat
// =========================================================================
reg [2:0]  pg_st;
reg [16:0] pg_cnt;
reg        pulse_reg;

localparam [2:0]
    PG_IDLE    = 3'd0,
    PG_PULSE_A = 3'd1,
    PG_INTER   = 3'd2,
    PG_PULSE_B = 3'd3,
    PG_GAP     = 3'd4;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pg_st     <= PG_IDLE;
        pg_cnt    <= 0;
        pulse_reg <= 0;
    end else begin
        case (pg_st)
            PG_IDLE: begin
                pulse_reg <= 0;
                if (running) begin
                    pulse_reg <= 1;
                    pg_cnt    <= cfg_width - 1;
                    pg_st     <= PG_PULSE_A;
                end
            end
            PG_PULSE_A: begin
                if (pg_cnt==0) begin
                    pulse_reg <= 0;
                    pg_cnt    <= (cfg_inter * GAP_TICKS) - 1;
                    pg_st     <= PG_INTER;
                end else pg_cnt <= pg_cnt - 1;
            end
            PG_INTER: begin
                if (pg_cnt==0) begin
                    pulse_reg <= 1;
                    pg_cnt    <= cfg_width - 1;
                    pg_st     <= PG_PULSE_B;
                end else pg_cnt <= pg_cnt - 1;
            end
            PG_PULSE_B: begin
                if (pg_cnt==0) begin
                    pulse_reg <= 0;
                    pg_cnt    <= (cfg_gap * GAP_TICKS) - 1;
                    pg_st     <= PG_GAP;
                end else pg_cnt <= pg_cnt - 1;
            end
            PG_GAP: begin
                if (pg_cnt==0) begin
                    if (running) begin
                        pulse_reg <= 1;
                        pg_cnt    <= cfg_width - 1;
                        pg_st     <= PG_PULSE_A;
                    end else begin
                        pg_st <= PG_IDLE;
                    end
                end else pg_cnt <= pg_cnt - 1;
            end
            default: pg_st <= PG_IDLE;
        endcase
    end
end

// =========================================================================
//  RX activity blink (LED2)
// =========================================================================
reg        rx_blink;
reg [21:0] rx_blink_cnt;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin rx_blink<=0; rx_blink_cnt<=0; end
    else if (rx_valid) begin
        rx_blink     <= 1;
        rx_blink_cnt <= 22'd2_000_000;
    end else if (rx_blink_cnt > 0) rx_blink_cnt <= rx_blink_cnt - 1;
    else rx_blink <= 0;
end

// =========================================================================
//  Outputs
// =========================================================================
assign pulse_out = pulse_reg;
assign led[0]    = pulse_reg;
assign led[1]    = running;
assign led[2]    = rx_blink;
assign led[3]    = tx_ready;

endmodule


// =============================================================================
//  Simple UART TX
// =============================================================================
module uart_tx_simple #(parameter BAUD_DIV=868) (
    input            clk, rst_n, tx_start,
    input      [7:0] tx_byte,
    output reg       tx_ready,
    output reg       uart_tx
);
localparam [1:0] TXI=0, TXS=1, TXD=2, TXP=3;
reg [1:0]  st;
reg [15:0] bc;
reg [2:0]  bi;
reg [7:0]  sr;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        st<=TXI; uart_tx<=1; tx_ready<=1; bc<=0; bi<=0; sr<=0;
    end else case(st)
        TXI: begin uart_tx<=1; tx_ready<=1;
             if(tx_start) begin sr<=tx_byte; bc<=BAUD_DIV-1;
                 tx_ready<=0; uart_tx<=0; st<=TXS; end end
        TXS: if(bc==0) begin uart_tx<=sr[0]; sr<={1'b1,sr[7:1]};
                 bc<=BAUD_DIV-1; bi<=1; st<=TXD; end
             else bc<=bc-1;
        TXD: if(bc==0) begin
                 if(bi==7) begin uart_tx<=1; bc<=BAUD_DIV-1; st<=TXP; end
                 else begin uart_tx<=sr[0]; sr<={1'b1,sr[7:1]};
                     bi<=bi+1; bc<=BAUD_DIV-1; end
             end else bc<=bc-1;
        TXP: if(bc==0) begin tx_ready<=1; st<=TXI; end
             else bc<=bc-1;
    endcase
end
endmodule
