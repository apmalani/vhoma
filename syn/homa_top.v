// Place-and-route harness for homa_endpoint at its default parameters. The endpoint has about
// 1100 port bits, more than any FPGA package has pins, so its inputs shift in from one pin (none
// is constant, so no logic is optimized away) and its outputs are registered and folded into one
// pin (none is unused). Resource counts come from the endpoint alone; this is only for timing.
module homa_top (
    input  wire clk,
    input  wire si,
    output reg  so
);

localparam NI = 625, NO = 518, NP = (NO + 15) / 16;

reg  [NI-1:0] in;
wire [NO-1:0] out;
reg  [16*NP-1:0] out_q;
reg  [NP-1:0] part;
integer i;

always @(posedge clk) begin
    in    <= {in[NI-2:0], si};
    out_q <= out;
    for (i = 0; i < NP; i = i + 1) part[i] <= ^out_q[16*i +: 16];
    so    <= ^part;
end

wire         rst, s_net_tvalid, s_net_tlast, s_net_tuser, m_net_tready, s_app_valid, s_app_resp,
             s_app_tvalid, m_app_tready, ab_ready;
wire [255:0] cfg_cutoffs;
wire [15:0]  cfg_cver, s_app_rport, s_app_lport;
wire [3:0]   cfg_sched_lvls;
wire [63:0]  cfg_id_base, s_net_tdata, s_app_tdata;
wire [7:0]   s_net_tkeep;
wire [31:0]  s_net_saddr, s_app_addr, s_app_len;
wire [10:0]  s_app_handle;
assign {rst, cfg_cutoffs, cfg_cver, cfg_sched_lvls, cfg_id_base, s_net_tdata, s_net_tkeep, s_net_tvalid,
        s_net_tlast, s_net_tuser, s_net_saddr, m_net_tready, s_app_valid, s_app_resp, s_app_addr,
        s_app_rport, s_app_lport, s_app_handle, s_app_len, s_app_tdata, s_app_tvalid, m_app_tready,
        ab_ready} = in;

wire         s_net_tready, m_net_tvalid, m_net_tlast, s_app_ready, s_app_tready, st_valid, st_ok,
             st_resp, m_app_tvalid, m_app_tlast, m_app_resp, ab_valid, rx_drop;
wire [63:0]  m_net_tdata, st_id, m_app_tdata, m_app_id, ab_id;
wire [7:0]   m_net_tkeep, m_app_tkeep;
wire [31:0]  m_net_daddr, m_app_addr, m_app_len;
wire [2:0]   m_net_prio;
wire [15:0]  m_net_len, m_app_rport, m_app_lport;
wire [10:0]  st_handle, m_app_handle;
assign out = {s_net_tready, m_net_tdata, m_net_tkeep, m_net_tvalid, m_net_tlast, m_net_daddr, m_net_prio,
              m_net_len, s_app_ready, s_app_tready, st_valid, st_ok, st_resp, st_id, st_handle,
              m_app_tdata, m_app_tkeep, m_app_tvalid, m_app_tlast, m_app_resp, m_app_addr, m_app_rport,
              m_app_lport, m_app_id, m_app_handle, m_app_len, ab_valid, ab_id, rx_drop};

homa_endpoint u (
    .clk(clk), .rst(rst),
    .cfg_cutoffs(cfg_cutoffs), .cfg_cver(cfg_cver), .cfg_sched_lvls(cfg_sched_lvls), .cfg_id_base(cfg_id_base),
    .s_net_tdata(s_net_tdata), .s_net_tkeep(s_net_tkeep), .s_net_tvalid(s_net_tvalid), .s_net_tlast(s_net_tlast),
    .s_net_tuser(s_net_tuser), .s_net_tready(s_net_tready), .s_net_saddr(s_net_saddr),
    .m_net_tdata(m_net_tdata), .m_net_tkeep(m_net_tkeep), .m_net_tvalid(m_net_tvalid), .m_net_tlast(m_net_tlast),
    .m_net_tready(m_net_tready), .m_net_daddr(m_net_daddr), .m_net_prio(m_net_prio), .m_net_len(m_net_len),
    .s_app_valid(s_app_valid), .s_app_ready(s_app_ready), .s_app_resp(s_app_resp), .s_app_addr(s_app_addr),
    .s_app_rport(s_app_rport), .s_app_lport(s_app_lport), .s_app_handle(s_app_handle), .s_app_len(s_app_len),
    .s_app_tdata(s_app_tdata), .s_app_tvalid(s_app_tvalid), .s_app_tready(s_app_tready),
    .st_valid(st_valid), .st_ok(st_ok), .st_resp(st_resp), .st_id(st_id), .st_handle(st_handle),
    .m_app_tdata(m_app_tdata), .m_app_tkeep(m_app_tkeep), .m_app_tvalid(m_app_tvalid), .m_app_tlast(m_app_tlast),
    .m_app_tready(m_app_tready), .m_app_resp(m_app_resp), .m_app_addr(m_app_addr), .m_app_rport(m_app_rport),
    .m_app_lport(m_app_lport), .m_app_id(m_app_id), .m_app_handle(m_app_handle), .m_app_len(m_app_len),
    .ab_valid(ab_valid), .ab_id(ab_id), .ab_ready(ab_ready), .rx_drop(rx_drop));

endmodule
