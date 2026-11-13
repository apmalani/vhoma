// Homa endpoint (draft-ousterhout-tsvwg-homa-00): RPC transport for one network link.
// Network side carries Homa packets starting at the common header; IP framing is
// outside this module (see homa_ipv4). 64-bit AXI-Stream, byte 0 in tdata[7:0].
module homa_endpoint #(
    parameter SEG_BYTES     = 1024,
    parameter MAX_MSG       = 16384,
    parameter UNSCHED_MAX   = 4096,
    parameter RTT_BYTES     = 4096,
    parameter NRPC          = 8,
    parameter SRV_RESERVE   = 2,
    parameter OVERCOMMIT    = 4,
    parameter NPEER         = 4,
    parameter TICK          = 15625,
    parameter RESEND_TICKS  = 50,
    parameter ABORT_RESENDS = 5,
    parameter NEEDACK_TICKS = 50,
    parameter ACK_GIVEUP    = 20,
    parameter EVQ           = 4,
    parameter SLQ           = 8,
    parameter CUT_COOL      = 1024
) (
    input  wire              clk,
    input  wire              rst,

    input  wire [255:0]      cfg_cutoffs,
    input  wire [15:0]       cfg_cver,
    input  wire [3:0]        cfg_sched_lvls,
    input  wire [63:0]       cfg_id_base,      // first client RPC id after reset; vary it per reset

    // network in
    input  wire [63:0]       s_net_tdata,
    input  wire [7:0]        s_net_tkeep,
    input  wire              s_net_tvalid,
    input  wire              s_net_tlast,
    input  wire              s_net_tuser,     // packet is bad, valid with tlast
    output wire              s_net_tready,
    input  wire [31:0]       s_net_saddr,
    // network out
    output wire [63:0]       m_net_tdata,
    output wire [7:0]        m_net_tkeep,
    output wire              m_net_tvalid,
    output wire              m_net_tlast,
    input  wire              m_net_tready,
    output wire [31:0]       m_net_daddr,
    output wire [2:0]        m_net_prio,
    output wire [15:0]       m_net_len,

    // send a request (s_app_resp=0) or a response (s_app_resp=1, s_app_handle from delivery)
    input  wire              s_app_valid,
    output wire              s_app_ready,
    input  wire              s_app_resp,
    input  wire [31:0]       s_app_addr,
    input  wire [15:0]       s_app_rport,
    input  wire [15:0]       s_app_lport,
    input  wire [$clog2(NRPC)+7:0] s_app_handle,
    input  wire [31:0]       s_app_len,
    input  wire [63:0]       s_app_tdata,
    input  wire              s_app_tvalid,
    output wire              s_app_tready,
    output wire              st_valid,
    output wire              st_ok,
    output wire              st_resp,
    output wire [63:0]       st_id,
    output wire [$clog2(NRPC)+7:0] st_handle,

    // received requests and responses
    output wire [63:0]       m_app_tdata,
    output wire [7:0]        m_app_tkeep,
    output wire              m_app_tvalid,
    output wire              m_app_tlast,
    input  wire              m_app_tready,
    output wire              m_app_resp,
    output wire [31:0]       m_app_addr,
    output wire [15:0]       m_app_rport,
    output wire [15:0]       m_app_lport,
    output wire [63:0]       m_app_id,
    output wire [$clog2(NRPC)+7:0] m_app_handle,
    output wire [31:0]       m_app_len,

    // client RPCs that failed (no answer from the server)
    output wire              ab_valid,
    output wire [63:0]       ab_id,
    input  wire              ab_ready,

    output wire              rx_drop
);

localparam SL     = $clog2(NRPC);
localparam MSG_LG = $clog2(MAX_MSG);
localparam AW     = SL + MSG_LG - 3;

wire          adm_req, adm_ok, cmt_req, cmt_ok, ev_push, ev_full;
wire [SL-1:0] adm_slot, cmt_slot;
wire [31:0]   d_saddr, d_len, d_off, ev_saddr;
wire [15:0]   d_sport, d_dport, d_cver, d_plen, ev_len;
wire [63:0]   d_id, d_ackid;
wire [639:0]  ev_hdr;
wire          rb_we;
wire [AW-1:0] rb_waddr, rb_raddr, tb_waddr, tb_raddr;
wire [63:0]   rb_wdata, rb_rdata, tb_wdata, tb_rdata;
wire          tb_we;

wire          tx_valid, tx_ready;
wire [7:0]    tx_type, tx_pf;
wire [31:0]   tx_daddr, tx_off, tx_len;
wire [15:0]   tx_sport, tx_dport, tx_plen, tx_cver;
wire [63:0]   tx_id;
wire [2:0]    tx_prio;
wire [SL-1:0] tx_slot;
wire [NRPC-1:0] eg_lock;

wire          ld_req, ld_resp, ld_ack, ld_ok, fin_req, fin_ack;
wire [31:0]   ld_addr, ld_mlen;
wire [15:0]   ld_rport, ld_lport;
wire [SL+7:0] ld_handle, ld_hout;
wire [63:0]   ld_id;
wire [SL-1:0] fin_slot;

wire          dq_push, dq_resp, dd_req, dd_ack;
wire [SL+7:0] dq_handle;
wire [31:0]   dq_addr, dq_len;
wire [15:0]   dq_rport, dq_lport;
wire [63:0]   dq_id;
wire [SL-1:0] dd_slot;

homa_rx_ingress #(.SEG_BYTES(SEG_BYTES), .MAX_MSG(MAX_MSG), .NRPC(NRPC)) u_rx (
    .clk(clk), .rst(rst),
    .s_tdata(s_net_tdata), .s_tkeep(s_net_tkeep), .s_tvalid(s_net_tvalid), .s_tlast(s_net_tlast),
    .s_tuser(s_net_tuser), .s_tready(s_net_tready), .s_saddr(s_net_saddr),
    .adm_req(adm_req), .adm_ok(adm_ok), .adm_slot(adm_slot),
    .d_saddr(d_saddr), .d_sport(d_sport), .d_dport(d_dport), .d_id(d_id), .d_len(d_len), .d_off(d_off),
    .d_ackid(d_ackid), .d_cver(d_cver), .d_plen(d_plen),
    .cmt_req(cmt_req), .cmt_ok(cmt_ok), .cmt_slot(cmt_slot),
    .ev_push(ev_push), .ev_hdr(ev_hdr), .ev_saddr(ev_saddr), .ev_len(ev_len), .ev_full(ev_full),
    .rb_we(rb_we), .rb_waddr(rb_waddr), .rb_wdata(rb_wdata),
    .drop(rx_drop));

homa_core #(
    .SEG_BYTES(SEG_BYTES), .MAX_MSG(MAX_MSG), .UNSCHED_MAX(UNSCHED_MAX), .RTT_BYTES(RTT_BYTES),
    .NRPC(NRPC), .SRV_RESERVE(SRV_RESERVE), .OVERCOMMIT(OVERCOMMIT), .NPEER(NPEER), .TICK(TICK),
    .RESEND_TICKS(RESEND_TICKS), .ABORT_RESENDS(ABORT_RESENDS), .NEEDACK_TICKS(NEEDACK_TICKS), .ACK_GIVEUP(ACK_GIVEUP),
    .EVQ(EVQ), .SLQ(SLQ), .CUT_COOL(CUT_COOL)
) u_core (
    .clk(clk), .rst(rst),
    .cfg_cutoffs(cfg_cutoffs), .cfg_cver(cfg_cver), .cfg_sched_lvls(cfg_sched_lvls), .cfg_id_base(cfg_id_base),
    .adm_req(adm_req), .adm_ok(adm_ok), .adm_slot(adm_slot),
    .d_saddr(d_saddr), .d_sport(d_sport), .d_dport(d_dport), .d_id(d_id), .d_len(d_len), .d_off(d_off),
    .d_ackid(d_ackid), .d_cver(d_cver), .d_plen(d_plen),
    .cmt_req(cmt_req), .cmt_ok(cmt_ok), .cmt_slot(cmt_slot),
    .ev_push(ev_push), .ev_hdr(ev_hdr), .ev_saddr(ev_saddr), .ev_len(ev_len), .ev_full(ev_full),
    .tx_valid(tx_valid), .tx_ready(tx_ready), .tx_type(tx_type), .tx_daddr(tx_daddr),
    .tx_sport(tx_sport), .tx_dport(tx_dport), .tx_id(tx_id), .tx_prio(tx_prio), .tx_off(tx_off),
    .tx_len(tx_len), .tx_plen(tx_plen), .tx_pf(tx_pf), .tx_cver(tx_cver), .tx_slot(tx_slot),
    .eg_lock(eg_lock),
    .ld_req(ld_req), .ld_resp(ld_resp), .ld_addr(ld_addr), .ld_rport(ld_rport), .ld_lport(ld_lport),
    .ld_handle(ld_handle), .ld_mlen(ld_mlen), .ld_ack(ld_ack), .ld_ok(ld_ok), .ld_hout(ld_hout),
    .ld_id(ld_id), .fin_req(fin_req), .fin_slot(fin_slot), .fin_ack(fin_ack),
    .dq_push(dq_push), .dq_handle(dq_handle), .dq_resp(dq_resp), .dq_addr(dq_addr),
    .dq_rport(dq_rport), .dq_lport(dq_lport), .dq_id(dq_id), .dq_len(dq_len),
    .dd_req(dd_req), .dd_slot(dd_slot), .dd_ack(dd_ack),
    .ab_valid(ab_valid), .ab_id(ab_id), .ab_ready(ab_ready));

homa_tx_egress #(.SEG_BYTES(SEG_BYTES), .MAX_MSG(MAX_MSG), .NRPC(NRPC)) u_tx (
    .clk(clk), .rst(rst), .cfg_cutoffs(cfg_cutoffs), .cfg_cver(cfg_cver),
    .d_valid(tx_valid), .d_ready(tx_ready), .d_type(tx_type), .d_daddr(tx_daddr), .d_sport(tx_sport),
    .d_dport(tx_dport), .d_id(tx_id), .d_prio(tx_prio), .d_off(tx_off), .d_len(tx_len),
    .d_plen(tx_plen), .d_pf(tx_pf), .d_cver(tx_cver), .d_slot(tx_slot), .lock(eg_lock),
    .tb_raddr(tb_raddr), .tb_rdata(tb_rdata),
    .m_tdata(m_net_tdata), .m_tkeep(m_net_tkeep), .m_tvalid(m_net_tvalid), .m_tlast(m_net_tlast),
    .m_tready(m_net_tready), .m_daddr(m_net_daddr), .m_prio(m_net_prio), .m_len(m_net_len));

homa_app_load #(.MAX_MSG(MAX_MSG), .NRPC(NRPC)) u_load (
    .clk(clk), .rst(rst),
    .s_valid(s_app_valid), .s_ready(s_app_ready), .s_resp(s_app_resp), .s_addr(s_app_addr),
    .s_rport(s_app_rport), .s_lport(s_app_lport), .s_handle(s_app_handle), .s_len(s_app_len),
    .s_tdata(s_app_tdata), .s_tvalid(s_app_tvalid), .s_tready(s_app_tready),
    .st_valid(st_valid), .st_ok(st_ok), .st_resp(st_resp), .st_id(st_id), .st_handle(st_handle),
    .ld_req(ld_req), .ld_resp(ld_resp), .ld_addr(ld_addr), .ld_rport(ld_rport), .ld_lport(ld_lport),
    .ld_handle(ld_handle), .ld_mlen(ld_mlen), .ld_ack(ld_ack), .ld_ok(ld_ok), .ld_hout(ld_hout),
    .ld_id(ld_id), .fin_req(fin_req), .fin_slot(fin_slot), .fin_ack(fin_ack),
    .tb_we(tb_we), .tb_waddr(tb_waddr), .tb_wdata(tb_wdata));

homa_deliver #(.MAX_MSG(MAX_MSG), .NRPC(NRPC)) u_dlv (
    .clk(clk), .rst(rst),
    .dq_push(dq_push), .dq_handle(dq_handle), .dq_resp(dq_resp), .dq_addr(dq_addr),
    .dq_rport(dq_rport), .dq_lport(dq_lport), .dq_id(dq_id), .dq_len(dq_len),
    .dd_req(dd_req), .dd_slot(dd_slot), .dd_ack(dd_ack),
    .rb_raddr(rb_raddr), .rb_rdata(rb_rdata),
    .m_tdata(m_app_tdata), .m_tkeep(m_app_tkeep), .m_tvalid(m_app_tvalid), .m_tlast(m_app_tlast),
    .m_tready(m_app_tready), .m_resp(m_app_resp), .m_addr(m_app_addr), .m_rport(m_app_rport),
    .m_lport(m_app_lport), .m_id(m_app_id), .m_handle(m_app_handle), .m_len(m_app_len));

homa_ram #(.AW(AW), .DW(64)) u_rxbuf (
    .clk(clk), .we(rb_we), .waddr(rb_waddr), .wdata(rb_wdata), .raddr(rb_raddr), .rdata(rb_rdata));

homa_ram #(.AW(AW), .DW(64)) u_txbuf (
    .clk(clk), .we(tb_we), .waddr(tb_waddr), .wdata(tb_wdata), .raddr(tb_raddr), .rdata(tb_rdata));

endmodule
