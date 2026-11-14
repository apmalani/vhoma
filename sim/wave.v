// Waveform recording for tb_sys: compile with -s tb_sys -s wave.
module wave;
initial begin
    $dumpfile("build/wave.vcd");
    $dumpvars(3, tb_sys);   // one tb_sys scope; depth 3 reaches each endpoint's top-level signals
end
endmodule
