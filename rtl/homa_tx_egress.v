// Homa transmit path: turns core descriptors into packets (draft §3 formats).
// DATA payload is read from the transmit buffer; output starts at the Homa
// common header with destination, priority and length as sideband.
module homa_tx_egress #(
    parameter SEG_BYTES = 1024,
    parameter MAX_MSG   = 16384,
    parameter NRPC      = 8
) (
    input  wire              clk,
    input  wire              rst,
    input  wire [255:0]      cfg_cutoffs,
    input  wire [15:0]       cfg_cver,

    input  wire              d_valid,
    output wire              d_ready,
    input  wire [7:0]        d_type,
    input  wire [31:0]       d_daddr,
    input  wire [15:0]       d_sport,
    input  wire [15:0]       d_dport,
    input  wire [63:0]       d_id,
    input  wire [2:0]        d_prio,
    input  wire [31:0]       d_off,
    input  wire [31:0]       d_len,
    input  wire [15:0]       d_plen,
    input  wire [7:0]        d_pf,
    input  wire [15:0]       d_cver,
    input  wire [$clog2(NRPC)-1:0] d_slot,
    output wire [NRPC-1:0]   lock,          // slots whose transmit buffer is still being read

    output wire [$clog2(NRPC)+$clog2(MAX_MSG)-4:0] tb_raddr,
    input  wire [63:0]       tb_rdata,

    output reg  [63:0]       m_tdata,
    output reg  [7:0]        m_tkeep,
    output wire              m_tvalid,
    output reg               m_tlast,
    input  wire              m_tready,
    output wire [31:0]       m_daddr,       // sideband, constant per packet
    output wire [2:0]        m_prio,
    output wire [15:0]       m_len          // Homa header + payload bytes
);

`include "homa_defs.vh"

localparam SL     = $clog2(NRPC);
localparam MSG_LG = $clog2(MAX_MSG);
localparam AW     = SL + MSG_LG - 3;

// one descriptor waiting, one being sent
reg          n_v;
reg  [7:0]   n_type;
reg  [31:0]  n_daddr, n_off, n_len;
reg  [15:0]  n_sport, n_dport, n_plen, n_cver;
reg  [63:0]  n_id;
reg  [2:0]   n_prio;
reg  [7:0]   n_pf;
reg  [SL-1:0] n_slot;

reg          c_v;
reg  [7:0]   c_type;
reg  [31:0]  c_daddr, c_off, c_len;
reg  [15:0]  c_sport, c_dport, c_plen, c_cver;
reg  [63:0]  c_id;
reg  [2:0]   c_prio;
reg  [7:0]   c_pf;
reg  [SL-1:0] c_slot;

assign d_ready = !n_v;

genvar g;
generate
    for (g = 0; g < NRPC; g = g + 1) begin : lk
        assign lock[g] = (c_v && c_type == T_DATA && c_slot == g) || (n_v && n_type == T_DATA && n_slot == g);
    end
endgenerate

// header bytes for the packet being sent
reg  [511:0] hv;
reg  [6:0]   hlen;
integer      b;
always @* begin
    hv = 512'd0;
    hv[0 +: 8]  = c_sport[15:8];  hv[8 +: 8]  = c_sport[7:0];
    hv[16 +: 8] = c_dport[15:8];  hv[24 +: 8] = c_dport[7:0];
    hv[88 +: 8] = c_type;
    for (b = 0; b < 8; b = b + 1) hv[8*(20+b) +: 8] = c_id[63 - 8*b -: 8];
    case (c_type)
    T_DATA: begin
        for (b = 0; b < 4; b = b + 1) hv[8*(28+b) +: 8] = c_len[31 - 8*b -: 8];
        hv[8*42 +: 8] = c_cver[15:8]; hv[8*43 +: 8] = c_cver[7:0];
        hv[8*44 +: 8] = c_pf;
        hlen = 7'd52;
    end
    T_START: begin
        for (b = 0; b < 4; b = b + 1) hv[8*(28+b) +: 8] = c_len[31 - 8*b -: 8];
        hlen = 7'd32;
    end
    T_GRANT: begin
        for (b = 0; b < 4; b = b + 1) hv[8*(28+b) +: 8] = c_off[31 - 8*b -: 8];
        hv[8*32 +: 8] = c_pf;
        hlen = 7'd33;
    end
    T_RESEND: begin
        for (b = 0; b < 4; b = b + 1) hv[8*(28+b) +: 8] = c_off[31 - 8*b -: 8];
        for (b = 0; b < 4; b = b + 1) hv[8*(32+b) +: 8] = c_len[31 - 8*b -: 8];
        hv[8*36 +: 8] = c_pf;
        hlen = 7'd37;
    end
    T_CUTOFFS: begin
        for (b = 0; b < 32; b = b + 1) hv[8*(28+b) +: 8] = cfg_cutoffs[32*(b/4) + 24 - 8*(b%4) +: 8];
        hv[8*60 +: 8] = cfg_cver[15:8]; hv[8*61 +: 8] = cfg_cver[7:0];
        hlen = 7'd62;
    end
    T_ACK:   hlen = 7'd30;
    default: hlen = 7'd28;
    endcase
end

localparam [1:0] IDLE = 2'd0, HEAD = 2'd1, PAY = 2'd2;
reg  [1:0]    st;
reg  [2:0]    hb;
reg  [15:0]   pleft;
reg  [31:0]   resid;
reg           first, flush;
reg  [2:0]    fbytes;
reg  [AW-1:0] raddr;

wire [2:0]  hbeats  = (c_type == T_DATA) ? 3'd5 : (hlen + 7'd7) / 8 - 1;   // index of last header beat
wire [2:0]  hrem    = hlen[2:0];
wire        last_in = (pleft <= 16'd8);
wire        hs      = m_tvalid && m_tready;
wire [AW-1:0] base  = (c_slot << (MSG_LG - 3)) + (c_off >> 3);

assign m_tvalid = (st != IDLE);
assign m_daddr  = c_daddr;
assign m_prio   = c_prio;
assign m_len    = hlen + ((c_type == T_DATA) ? c_plen : 16'd0);

// read address moves on the cycle a payload word is consumed, so data is ready next cycle
assign tb_raddr = (st == PAY && hs && !flush) ? raddr + 1'b1 : (st == PAY) ? raddr : base;

always @* begin
    m_tdata = hv[64*hb +: 64];
    m_tkeep = 8'hFF;
    m_tlast = 1'b0;
    if (st == HEAD) begin
        if (c_type != T_DATA && hb == hbeats) begin
            m_tlast = 1'b1;
            if (hrem != 0) m_tkeep = ~(8'hFF << hrem);
        end
    end else if (st == PAY) begin
        if (flush) begin
            m_tdata = {32'd0, resid};
            m_tkeep = ~(8'hFF << fbytes);
            m_tlast = 1'b1;
        end else begin
            m_tdata = {tb_rdata[31:0], first ? be32(c_off) : resid};
            if (last_in && pleft <= 16'd4) begin
                m_tkeep = ~(8'hFF << ({1'b0, pleft[2:0]} + 4'd4));
                m_tlast = 1'b1;
            end
        end
    end
end

always @(posedge clk) begin
    if (rst) begin
        n_v <= 1'b0; c_v <= 1'b0; st <= IDLE; flush <= 1'b0;
    end else begin
        if (d_valid && d_ready) begin
            n_v <= 1'b1; n_type <= d_type; n_daddr <= d_daddr; n_off <= d_off; n_len <= d_len;
            n_sport <= d_sport; n_dport <= d_dport; n_plen <= d_plen; n_cver <= d_cver;
            n_id <= d_id; n_prio <= d_prio; n_pf <= d_pf; n_slot <= d_slot;
        end
        if (st == IDLE && n_v) begin
            c_v <= 1'b1; c_type <= n_type; c_daddr <= n_daddr; c_off <= n_off; c_len <= n_len;
            c_sport <= n_sport; c_dport <= n_dport; c_plen <= n_plen; c_cver <= n_cver;
            c_id <= n_id; c_prio <= n_prio; c_pf <= n_pf; c_slot <= n_slot;
            n_v <= 1'b0;
            st <= HEAD; hb <= 0;
        end
        if (st == HEAD && hs) begin
            hb <= hb + 1'b1;
            if (hb == hbeats) begin
                if (c_type == T_DATA) begin
                    st <= PAY; first <= 1'b1; pleft <= c_plen;
                    raddr <= base;
                end else begin
                    st <= IDLE; c_v <= 1'b0;
                end
            end
        end
        if (st == PAY && hs) begin
            if (!flush) begin
                raddr  <= raddr + 1'b1;
                resid  <= tb_rdata[63:32];
                first  <= 1'b0;
                pleft  <= pleft - 16'd8;
                if (last_in && pleft > 16'd4) begin
                    flush  <= 1'b1;
                    fbytes <= pleft[2:0] - 3'd4;
                end
            end
            if (m_tlast) begin
                flush <= 1'b0;
                st    <= IDLE;
                c_v   <= 1'b0;
            end
        end
    end
end

endmodule
