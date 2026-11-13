// Waveform recording for tb_sys: compile with -s tb_sys -s wave.
module wave;
initial begin
    $dumpfile("build/wave.vcd");
    $dumpvars(1, tb_sys.ep[0].u);
    $dumpvars(1, tb_sys.ep[1].u);
    $dumpvars(1, tb_sys.ep[2].u);
end
endmodule
