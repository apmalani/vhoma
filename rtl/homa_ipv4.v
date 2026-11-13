// IPv4 framing for Homa (IP protocol 146). Priority goes in the top 3 bits of DSCP (draft §5).
// TX prepends a 20-byte header; RX checks and strips it. No IP options or fragments.
module homa_ipv4 (
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] cfg_addr,        // our address: source on TX, required destination on RX

    // TX: Homa packet in, IPv4 packet out
    input  wire [63:0] s_tx_tdata,
    input  wire [7:0]  s_tx_tkeep,
    input  wire        s_tx_tvalid,
    input  wire        s_tx_tlast,
    output wire        s_tx_tready,
    input  wire [31:0] s_tx_daddr,
    input  wire [2:0]  s_tx_prio,
    input  wire [15:0] s_tx_len,        // Homa bytes in this packet
    output reg  [63:0] m_tx_tdata,
    output reg  [7:0]  m_tx_tkeep,
    output reg         m_tx_tvalid,
    output reg         m_tx_tlast,
    input  wire        m_tx_tready,

    // RX: IPv4 packet in, Homa packet out
    input  wire [63:0] s_rx_tdata,
    input  wire [7:0]  s_rx_tkeep,
    input  wire        s_rx_tvalid,
    input  wire        s_rx_tlast,
    input  wire        s_rx_tuser,      // frame error, valid with tlast
    output wire        s_rx_tready,
    output reg  [63:0] m_rx_tdata,
    output reg  [7:0]  m_rx_tkeep,
    output reg         m_rx_tvalid,
    output reg         m_rx_tlast,
    output reg         m_rx_tuser,      // packet is bad: drop it
    input  wire        m_rx_tready,
    output wire [31:0] m_rx_saddr,
    output reg         rx_drop
);

localparam [7:0] PROTO_HOMA = 8'd146;

function [3:0] ones(input [7:0] k);
    integer j;
    begin
        ones = 4'd0;
        for (j = 0; j < 8; j = j + 1) ones = ones + k[j];
    end
endfunction

function [15:0] csum(input [159:0] h);   // h holds header bytes 0..19, byte i at [8*i +: 8]
    integer j;
    reg [19:0] s;
    begin
        s = 0;
        for (j = 0; j < 10; j = j + 1) s = s + {h[16*j +: 8], h[16*j + 8 +: 8]};
        s = s[15:0] + s[19:16];
        s = s[15:0] + s[19:16];
        csum = ~s[15:0];
    end
endfunction

// TX
reg  [1:0]   tst;          // 0 idle, 1-2 header beats, 3 body
reg  [159:0] th;
reg  [31:0]  tres;
reg          tfirst, tflush;
reg  [2:0]   tfb;
reg  [15:0]  tid;
reg  [159:0] th_n;
integer      b;

always @* begin
    th_n = 160'd0;
    th_n[0 +: 8]  = 8'h45;
    th_n[8 +: 8]  = {s_tx_prio, 5'd0};
    th_n[16 +: 8] = (s_tx_len + 16'd20) >> 8;
    th_n[24 +: 8] = (s_tx_len + 16'd20);
    th_n[32 +: 8] = tid[15:8];
    th_n[40 +: 8] = tid[7:0];
    th_n[48 +: 8] = 8'h40;              // don't fragment
    th_n[64 +: 8] = 8'd64;
    th_n[72 +: 8] = PROTO_HOMA;
    for (b = 0; b < 4; b = b + 1) th_n[96 + 8*b +: 8]  = cfg_addr[31 - 8*b -: 8];
    for (b = 0; b < 4; b = b + 1) th_n[128 + 8*b +: 8] = s_tx_daddr[31 - 8*b -: 8];
    {th_n[80 +: 8], th_n[88 +: 8]} = csum(th_n);
end

wire [3:0] tk = ones(s_tx_tkeep);
assign s_tx_tready = (tst == 2'd3) && !tflush && m_tx_tready;

always @* begin
    m_tx_tdata  = 64'd0;
    m_tx_tkeep  = 8'hFF;
    m_tx_tvalid = 1'b0;
    m_tx_tlast  = 1'b0;
    case (tst)
    2'd1: begin m_tx_tdata = th[63:0];   m_tx_tvalid = 1'b1; end
    2'd2: begin m_tx_tdata = th[127:64]; m_tx_tvalid = 1'b1; end
    2'd3: if (tflush) begin
        m_tx_tdata = {32'd0, tres}; m_tx_tkeep = ~(8'hFF << tfb); m_tx_tvalid = 1'b1; m_tx_tlast = 1'b1;
    end else begin
        m_tx_tdata  = {s_tx_tdata[31:0], tfirst ? th[159:128] : tres};
        m_tx_tvalid = s_tx_tvalid;
        if (s_tx_tlast && tk <= 4) begin
            m_tx_tkeep = ~(8'hFF << (tk + 4'd4));
            m_tx_tlast = 1'b1;
        end
    end
    default: ;
    endcase
end

always @(posedge clk) begin
    if (rst) begin
        tst <= 0; tflush <= 1'b0; tid <= 0;
    end else begin
        case (tst)
        2'd0: if (s_tx_tvalid) begin th <= th_n; tid <= tid + 1'b1; tst <= 2'd1; end
        2'd1: if (m_tx_tready) tst <= 2'd2;
        2'd2: if (m_tx_tready) begin tst <= 2'd3; tfirst <= 1'b1; end
        2'd3: if (m_tx_tvalid && m_tx_tready) begin
            if (tflush) begin
                tflush <= 1'b0; tst <= 2'd0;
            end else begin
                tres <= s_tx_tdata[63:32];
                tfirst <= 1'b0;
                if (s_tx_tlast) begin
                    if (tk > 4) begin tflush <= 1'b1; tfb <= tk - 4'd4; end
                    else tst <= 2'd0;
                end
            end
        end
        endcase
    end
end

// RX
localparam [2:0] R_HDR = 3'd0, R_BODY = 3'd1, R_FLUSH = 3'd2, R_SKIP = 3'd3;
reg  [2:0]   rst_s;
reg  [1:0]   rb;            // header beat 0..2
reg  [127:0] rh;
reg  [31:0]  rres;
reg  [2:0]   rresn;         // bytes held in rres
reg  [15:0]  rleft;         // Homa bytes still to forward
reg  [31:0]  rsaddr;
reg          rbad;
assign m_rx_saddr = rsaddr;

wire [3:0]   rk   = ones(s_rx_tkeep);
wire [159:0] rh3  = {s_rx_tdata[31:0], rh};
wire [15:0]  tot  = {rh[16 +: 8], rh[24 +: 8]};
wire         hdr_ok = (rh[0 +: 8] == 8'h45) && (rh[72 +: 8] == PROTO_HOMA) && (csum(rh3) == 16'd0) &&
                      ((rh[48 +: 8] & 8'h3F) == 0) && (rh[56 +: 8] == 0) &&   // no fragments
                      (tot >= 16'd48) && ({rh3[128 +: 8], rh3[136 +: 8], rh3[144 +: 8], rh3[152 +: 8]} == cfg_addr);
// bytes this beat contributes in the body: 4 from the previous beat plus up to 4 new
wire [3:0]   r_new = (rk > 4) ? 4'd4 : rk;
wire [4:0]   r_out = rresn + r_new;
wire         r_end = ({11'd0, r_out} >= rleft);          // forwarded everything IP said is there
wire [15:0]  r_rem = rleft - r_out;                       // still owed after this beat
wire [3:0]   r_lo  = (rk > 4) ? rk - 4'd4 : 4'd0;          // bytes this beat leaves for a flush
wire         r_trunc = ({12'd0, r_lo} < r_rem);
// body beats must be full; the last one must be filled from lane 0 up
wire         r_kbad  = s_rx_tlast ? (s_rx_tkeep == 0 || (s_rx_tkeep & (s_rx_tkeep + 8'd1)) != 0)
                                  : (s_rx_tkeep != 8'hFF);
reg          rkb;

assign s_rx_tready = (rst_s == R_HDR) || (rst_s == R_SKIP) || (rst_s == R_BODY && m_rx_tready);

always @* begin
    m_rx_tdata  = {s_rx_tdata[31:0], rres};
    m_rx_tkeep  = 8'hFF;
    m_rx_tvalid = 1'b0;
    m_rx_tlast  = 1'b0;
    m_rx_tuser  = 1'b0;
    if (rst_s == R_BODY) begin
        m_rx_tvalid = s_rx_tvalid;
        if (r_end) begin
            m_rx_tkeep = ~(8'hFF << rleft[3:0]);
            if (rleft == 8) m_rx_tkeep = 8'hFF;
            m_rx_tlast = 1'b1;
            // bytes after the IP length mean a malformed frame (FCS must already be stripped)
            m_rx_tuser = !s_rx_tlast || s_rx_tuser || r_kbad || rkb;
        end else if (s_rx_tlast && r_lo == 0) begin
            // IP packet ends early and nothing is left over
            m_rx_tkeep = ~(8'hFF << r_out);
            if (r_out >= 8) m_rx_tkeep = 8'hFF;
            m_rx_tlast = 1'b1;
            m_rx_tuser = 1'b1;
        end
    end else if (rst_s == R_FLUSH) begin
        m_rx_tdata  = {32'd0, rres};
        m_rx_tkeep  = ~(8'hFF << rresn);
        m_rx_tvalid = 1'b1;
        m_rx_tlast  = 1'b1;
        m_rx_tuser  = rbad;
    end
end

always @(posedge clk) begin
    rx_drop <= 1'b0;
    if (rst) begin
        rst_s <= R_HDR; rb <= 0;
    end else begin
        case (rst_s)
        R_HDR: if (s_rx_tvalid) begin
            if (rb == 0) rh[63:0] <= s_rx_tdata;
            if (rb == 1) rh[127:64] <= s_rx_tdata;
            rb <= rb + 1'b1;
            if (s_rx_tlast || (rb != 2 && s_rx_tkeep != 8'hFF)) begin
                rb <= 0; rx_drop <= 1'b1;
                if (!s_rx_tlast) rst_s <= R_SKIP;
            end else if (rb == 2) begin
                rb <= 0;
                if (hdr_ok && s_rx_tkeep == 8'hFF) begin
                    rst_s <= R_BODY;
                    rres <= s_rx_tdata[63:32]; rresn <= 3'd4; rkb <= 1'b0;
                    rleft <= tot - 16'd20;
                    rsaddr <= {rh[96 +: 8], rh[104 +: 8], rh[112 +: 8], rh[120 +: 8]};
                end else begin
                    rx_drop <= 1'b1;
                    rst_s <= R_SKIP;
                end
            end
        end
        R_BODY: if (s_rx_tvalid && m_rx_tready) begin
            rres  <= s_rx_tdata[63:32];
            rresn <= (rk > 4) ? rk - 4'd4 : 3'd0;
            rleft <= rleft - r_out;
            if (r_kbad) rkb <= 1'b1;
            if (r_end) rst_s <= s_rx_tlast ? R_HDR : R_SKIP;
            else if (s_rx_tlast) begin
                rbad  <= r_trunc || s_rx_tuser || r_kbad || rkb;
                rresn <= r_trunc ? r_lo[2:0] : r_rem[2:0];
                rst_s <= (r_lo != 0) ? R_FLUSH : R_HDR;
            end
        end
        R_FLUSH: if (m_rx_tready) begin rst_s <= R_HDR; rbad <= 1'b0; end
        R_SKIP: if (s_rx_tvalid && s_rx_tlast) rst_s <= R_HDR;
        default: rst_s <= R_HDR;
        endcase
    end
end

endmodule
