// Waveform recording for tb_sys: compile with -s tb_sys -s wave; +vcd=<file> (default build/wave.vcd).
// Besides tb_sys, it records the name of each packet on every endpoint's network ports, decoded from
// the wire, for viewers to show as text (sim/wave.json does, in VaporView).
module wave;
reg [8*512-1:0] path;
initial begin
    if (!$value$plusargs("vcd=%s", path)) path = "build/wave.vcd";
    $dumpfile(path);
    $dumpvars(3, tb_sys);   // one tb_sys scope; depth 3 reaches each endpoint's top-level signals
    $dumpvars(2, wave);
end

wave_pkt out0 (tb_sys.clk, tb_sys.ep[0].u.m_net_tvalid, tb_sys.ep[0].u.m_net_tready, tb_sys.ep[0].u.m_net_tlast, tb_sys.ep[0].u.m_net_tdata);
wave_pkt out1 (tb_sys.clk, tb_sys.ep[1].u.m_net_tvalid, tb_sys.ep[1].u.m_net_tready, tb_sys.ep[1].u.m_net_tlast, tb_sys.ep[1].u.m_net_tdata);
wave_pkt out2 (tb_sys.clk, tb_sys.ep[2].u.m_net_tvalid, tb_sys.ep[2].u.m_net_tready, tb_sys.ep[2].u.m_net_tlast, tb_sys.ep[2].u.m_net_tdata);
wave_pkt in0  (tb_sys.clk, tb_sys.ep[0].u.s_net_tvalid, 1'b1, tb_sys.ep[0].u.s_net_tlast, tb_sys.ep[0].u.s_net_tdata);
wave_pkt in1  (tb_sys.clk, tb_sys.ep[1].u.s_net_tvalid, 1'b1, tb_sys.ep[1].u.s_net_tlast, tb_sys.ep[1].u.s_net_tdata);
wave_pkt in2  (tb_sys.clk, tb_sys.ep[2].u.s_net_tvalid, 1'b1, tb_sys.ep[2].u.s_net_tlast, tb_sys.ep[2].u.s_net_tdata);
endmodule

// The type of the Homa packet on an AXI-Stream port (byte 11, in the second beat), as text: set
// from the second beat through the last.
module wave_pkt (input wire clk, input wire valid, input wire ready, input wire last, input wire [63:0] data);
reg [63:0] name = 0;
reg [15:0] beat = 0;
reg        clear = 0;
always @(posedge clk) begin
    if (clear) name <= 0;
    clear <= valid && ready && last;
    if (valid && ready) begin
        beat <= last ? 0 : beat + 1;
        if (beat == 1)
            case (data[31:24])
            16:      name <= "DATA";
            17:      name <= "GRANT";
            18:      name <= "RESEND";
            19:      name <= "UNKNOWN";
            20:      name <= "BUSY";
            21:      name <= "CUTOFFS";
            23:      name <= "NEED_ACK";
            24:      name <= "ACK";
            25:      name <= "START";
            default: name <= "?";
            endcase
    end
end
endmodule
