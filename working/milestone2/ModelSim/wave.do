onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -label CLOCK_50 -radix binary /testbench/CLOCK_50
add wave -noupdate -label SW -radix binary /testbench/SW
add wave -noupdate -label KEY -radix binary /testbench/KEY
add wave -noupdate -divider vga_demo
add wave -noupdate -label state -radix binary /testbench/U1/y_Q
add wave -noupdate -label next -radix binary /testbench/U1/Y_D
add wave -noupdate -label bodyCount -radix binary /testbench/U1/drawBodyCount
add wave -noupdate -label X -radix hexadecimal /testbench/U1/X
add wave -noupdate -label Y -radix hexadecimal /testbench/U1/Y
add wave -noupdate -label xsl -radix hexadecimal /testbench/U1/XSnakeLong
add wave -noupdate -label ysl -radix hexadecimal /testbench/U1/YSnakeLong
add wave -noupdate -label go -radix hexadecimal /testbench/U1/go
add wave -noupdate -label VGA_Xreg -radix hexadecimal /testbench/U1/VGA_X_reg
add wave -noupdate -label VGA_Yreg -radix hexadecimal /testbench/U1/VGA_Y_reg
add wave -noupdate -label VGA_Y -radix hexadecimal /testbench/U1/VGA_Y
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {10000 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 80
configure wave -valuecolwidth 40
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {0 ps} {120 ns}
