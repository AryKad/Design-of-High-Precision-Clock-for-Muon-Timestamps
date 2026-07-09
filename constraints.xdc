## =============================================================================
##  pulse_gen.xdc  -  Basys 3  (100 MHz)
##  Pulse pair generator constraints
## =============================================================================

## Clock
set_property PACKAGE_PIN W5      [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports clk]

## Reset (BTNC)
set_property PACKAGE_PIN T17     [get_ports btnc]
set_property IOSTANDARD LVCMOS33 [get_ports btnc]

## Onboard USB-UART  (FTDI chip)
## B18 = FTDI-TX  -> FPGA RX input
## A18 = FTDI-RX  -> FPGA TX output
set_property PACKAGE_PIN B18     [get_ports uart_rx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_rx]
set_property PACKAGE_PIN A18     [get_ports uart_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_tx]

## Pulse output  -  JA connector, pin JA4 (top row, leftmost)
set_property PACKAGE_PIN G2     [get_ports pulse_out]
set_property IOSTANDARD LVCMOS33 [get_ports pulse_out]

## LEDs
set_property PACKAGE_PIN U16     [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]
set_property PACKAGE_PIN E19     [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
set_property PACKAGE_PIN U19     [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]
set_property PACKAGE_PIN V19     [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]

## Bitstream
set_property CFGBVS VCCO         [current_design]
set_property CONFIG_VOLTAGE 3.3  [current_design]