// Homa receiver, unscheduled path: puts DATA packets back together into whole
// messages (draft-ousterhout-tsvwg-homa-00 §3.1) and outputs each message when it is complete.
// Input starts at the Homa common header; IP framing is removed upstream.
// Packets may arrive in any order. Each segment must be exactly SEG_BYTES long and
// SEG_BYTES-aligned, except the last segment of a message.
module homa_rx_unsched #(
    parameter SEG_BYTES = 1024,   // must match sender; power of 2, >= 8
    parameter MAX_MSG   = 16384,  // power of 2, >= SEG_BYTES
    parameter NSLOT     = 8,      // messages being assembled at once; power of 2, >= 2
    parameter TICK      = 156250, // clock cycles per age tick
    parameter AGE_MAX   = 15      // idle ticks before a partial message is discarded (>= 1)
) (
    input  wire              clk,
    input  wire              rst,

    input  wire [63:0]       s_axis_tdata,
    input  wire [7:0]        s_axis_tkeep,
    input  wire              s_axis_tvalid,
    input  wire              s_axis_tlast,
    output wire              s_axis_tready,  // always 1: bad packets are dropped, never stalled
    input  wire [31:0]       s_axis_saddr,   // sideband, constant per packet

    output wire [63:0]       m_axis_tdata,
    output wire [7:0]        m_axis_tkeep,
    output wire              m_axis_tvalid,
    output wire              m_axis_tlast,
    input  wire              m_axis_tready,
    output wire [31:0]       m_axis_saddr,   // sideband, constant per message
    output wire [15:0]       m_axis_sport,
    output wire [15:0]       m_axis_dport,
    output wire [63:0]       m_axis_id,
    output wire [31:0]       m_axis_len,

    output reg               ev_drop,        // pulse: packet discarded
    output reg  [NSLOT-1:0]  ev_expire       // pulse per slot: partial message discarded
);

localparam [7:0] TYPE_DATA = 8'd16;
localparam SEG_LG = $clog2(SEG_BYTES);
localparam MSG_LG = $clog2(MAX_MSG);
localparam NSEG   = MAX_MSG / SEG_BYTES;
localparam SEGW   = SEG_BYTES / 8;
localparam SL     = $clog2(NSLOT);
localparam AW     = SL + MSG_LG - 3;
localparam LW     = MSG_LG + 1;
localparam BW     = $clog2(SEGW + 8);
localparam AGW    = $clog2(AGE_MAX + 1);
localparam TW     = $clog2(TICK + 1);
localparam [BW-1:0] BC_MAX = SEGW + 7;  // one beat past the longest legal packet

// elaboration fails here if a parameter is out of range
generate
    if (SEG_BYTES < 8 || (SEG_BYTES & (SEG_BYTES - 1)) != 0 ||
        MAX_MSG < SEG_BYTES || (MAX_MSG & (MAX_MSG - 1)) != 0 ||
        NSLOT < 2 || (NSLOT & (NSLOT - 1)) != 0 || TICK < 1 || AGE_MAX < 1) begin : bad_parameter
        homa_rx_unsched_invalid_parameter u_fail ();
    end
endgenerate

function [15:0] be16(input [15:0] v); be16 = {v[7:0], v[15:8]}; endfunction
function [31:0] be32(input [31:0] v); be32 = {v[7:0], v[15:8], v[23:16], v[31:24]}; endfunction
function [3:0] ones(input [7:0] k);
    integer j;
    begin
        ones = 4'd0;
        for (j = 0; j < 8; j = j + 1) ones = ones + k[j];
    end
endfunction

assign s_axis_tready = 1'b1;
wire [63:0] d = s_axis_tdata;
wire        v = s_axis_tvalid;

// slot table
reg  [NSLOT-1:0] busy, done;
reg  [31:0]      k_saddr [0:NSLOT-1];
reg  [63:0]      k_id    [0:NSLOT-1];
reg  [15:0]      k_sport [0:NSLOT-1];
reg  [15:0]      k_dport [0:NSLOT-1];
reg  [LW-1:0]    k_len   [0:NSLOT-1];
reg  [NSEG-1:0]  bm      [0:NSLOT-1];
reg  [AGW-1:0]   age     [0:NSLOT-1];

reg  [63:0]      mem [0:(1<<AW)-1];

// parser
reg  [BW-1:0] bc;
reg  [15:0]   p_sport, p_dport;
reg  [7:0]    p_type;
reg  [63:0]   p_id;
reg  [31:0]   p_len;
reg           kbad;
reg           r_acc, r_new;
reg  [SL-1:0] r_slot;
reg  [31:0]   r_off;
reg  [31:0]   resid;

// beat 6 carries the offset: decide here, before any payload is written
wire [31:0] off6 = be32(d[31:0]);
reg          hit6, free6;
reg [SL-1:0] hslot, fslot;
integer i;
always @* begin
    hit6 = 1'b0; free6 = 1'b0; hslot = 0; fslot = 0;
    for (i = NSLOT - 1; i >= 0; i = i - 1) begin
        if (busy[i] && k_saddr[i] == s_axis_saddr && k_id[i] == p_id) begin
            hit6 = 1'b1; hslot = i[SL-1:0];
        end
        if (!busy[i]) begin
            free6 = 1'b1; fslot = i[SL-1:0];
        end
    end
end

wire hdr_ok6 = (p_type == TYPE_DATA) && (p_len != 0) && (p_len <= MAX_MSG) &&
               (off6[SEG_LG-1:0] == 0) && (off6 < p_len);
// a segment already received is never rewritten, so duplicates cannot corrupt it
wire acc6 = hdr_ok6 && (hit6 ? (!done[hslot] && k_len[hslot] == p_len &&
                                k_sport[hslot] == p_sport && k_dport[hslot] == p_dport &&
                                !bm[hslot][off6 >> SEG_LG])
                             : free6);

wire          at6    = v && (bc == 6);
wire          c_acc  = at6 ? acc6 : r_acc;   // also locks c_slot against expiry
wire          c_new  = at6 ? !hit6 : r_new;
wire [SL-1:0] c_slot = at6 ? (hit6 ? hslot : fslot) : r_slot;
wire [31:0]   c_off  = at6 ? off6 : r_off;

wire [3:0]  kc      = ones(s_axis_tkeep);
wire        keep_ok = (s_axis_tkeep != 0) && ((s_axis_tkeep & (s_axis_tkeep + 8'd1)) == 0);
wire [31:0] nbytes  = {bc, 3'b000} + kc;
wire [31:0] rem     = p_len - c_off;
wire [31:0] exp_b   = ((rem > SEG_BYTES) ? SEG_BYTES : rem) + 52;
wire        pkt_ok  = c_acc && !kbad && keep_ok && (nbytes == exp_b);

wire [NSEG-1:0] need   = ~({NSEG{1'b1}} << ((p_len + SEG_BYTES - 1) >> SEG_LG));
wire [NSEG-1:0] bm_new = (c_new ? {NSEG{1'b0}} : bm[c_slot]) | (1'b1 << (c_off >> SEG_LG));
wire            cmpl   = &(bm_new | ~need);
wire            commit = v && s_axis_tlast && pkt_ok;

// payload writes
// payload starts 4 bytes into beat 6; realign to 8-byte words
reg           fl_v;
reg  [AW-1:0] fl_addr;
reg  [31:0]   fl_data;
reg           push_v;
reg  [SL-1:0] push_slot;

wire [31:0]   widx    = bc - 7;
wire [AW-1:0] r_base  = (r_slot << (MSG_LG - 3)) + (r_off >> 3);
wire          beat_we = v && r_acc && (bc >= 7) && (widx < SEGW);
wire          we      = fl_v || beat_we;
wire [AW-1:0] waddr   = fl_v ? fl_addr : r_base + widx;
wire [63:0]   wdata   = fl_v ? {32'd0, fl_data} : {d[31:0], resid};

always @(posedge clk)
    if (we) mem[waddr] <= wdata;

always @(posedge clk) begin
    ev_drop <= 1'b0;
    fl_v    <= 1'b0;
    push_v  <= 1'b0;
    if (rst) begin
        bc    <= 0;
        r_acc <= 1'b0;
        kbad  <= 1'b0;
    end else if (v) begin
        case (bc)
        0: begin p_sport <= be16(d[15:0]); p_dport <= be16(d[31:16]); end
        1: p_type <= d[31:24];
        2: p_id[63:32] <= be32(d[63:32]);
        3: begin p_id[31:0] <= be32(d[31:0]); p_len <= be32(d[63:32]); end
        6: begin r_acc <= acc6; r_new <= !hit6; r_slot <= c_slot; r_off <= off6; end
        default: ;
        endcase
        resid <= d[63:32];
        if (!s_axis_tlast && s_axis_tkeep != 8'hFF) kbad <= 1'b1;
        if (s_axis_tlast) begin
            bc        <= 0;
            r_acc     <= 1'b0;
            kbad      <= 1'b0;
            ev_drop   <= !pkt_ok;
            // last beat may hold up to 4 more payload bytes: write them next cycle
            fl_v      <= pkt_ok && (kc > 4);
            fl_addr   <= (c_slot << (MSG_LG - 3)) + (c_off >> 3) + (bc - 6);
            fl_data   <= d[63:32];
            // queue for delivery only after that final write has landed
            push_v    <= pkt_ok && cmpl;
            push_slot <= c_slot;
        end else if (bc != BC_MAX) begin
            bc <= bc + 1'b1;
        end
    end
end

// delivery
reg  [SL-1:0]     q [0:NSLOT-1];  // completed slots, in completion order
reg  [SL:0]       wp, rp;
reg               d_act;
reg  [SL-1:0]     d_slot;
reg  [MSG_LG-3:0] d_left;
reg  [AW-1:0]     raddr;
reg  [63:0]       rdata;

wire          q_ne    = (wp != rp);
wire [SL-1:0] q_head  = q[rp[SL-1:0]];
wire          start   = !d_act && q_ne;
wire          d_hs    = d_act && m_axis_tready;
wire          d_fin   = d_hs && (d_left == 1);
wire [AW-1:0] raddr_n = start ? (q_head << (MSG_LG - 3)) : d_hs ? raddr + 1'b1 : raddr;

always @(posedge clk) begin
    raddr <= raddr_n;
    rdata <= mem[raddr_n];
end

always @(posedge clk)
    if (push_v) q[wp[SL-1:0]] <= push_slot;

always @(posedge clk) begin
    if (rst) begin
        wp    <= 0;
        rp    <= 0;
        d_act <= 1'b0;
    end else begin
        if (push_v) wp <= wp + 1'b1;
        if (start) begin
            rp     <= rp + 1'b1;
            d_act  <= 1'b1;
            d_slot <= q_head;
            d_left <= (k_len[q_head] + 7) >> 3;
        end else if (d_hs) begin
            d_left <= d_left - 1'b1;
            if (d_left == 1) d_act <= 1'b0;
        end
    end
end

wire [2:0] lb = k_len[d_slot][2:0];
assign m_axis_tvalid = d_act;
assign m_axis_tdata  = rdata;
assign m_axis_tlast  = (d_left == 1);
assign m_axis_tkeep  = (m_axis_tlast && lb != 0) ? ~(8'hFF << lb) : 8'hFF;
assign m_axis_saddr  = k_saddr[d_slot];
assign m_axis_sport  = k_sport[d_slot];
assign m_axis_dport  = k_dport[d_slot];
assign m_axis_id     = k_id[d_slot];
assign m_axis_len    = k_len[d_slot];

// slot state
reg [TW-1:0] tcnt;
wire tick = (tcnt == TICK - 1);
integer n;

always @(posedge clk) begin
    ev_expire <= 0;
    if (rst) begin
        busy <= 0;
        done <= 0;
        tcnt <= 0;
    end else begin
        tcnt <= tick ? 0 : tcnt + 1'b1;
        if (tick)
            for (n = 0; n < NSLOT; n = n + 1)
                if (busy[n] && !done[n] && !(c_acc && c_slot == n)) begin
                    if (age[n] == AGE_MAX) begin
                        busy[n]      <= 1'b0;
                        ev_expire[n] <= 1'b1;
                    end else
                        age[n] <= age[n] + 1'b1;
                end
        if (d_fin) begin
            busy[d_slot] <= 1'b0;
            done[d_slot] <= 1'b0;
        end
        if (commit) begin
            busy[c_slot] <= 1'b1;
            done[c_slot] <= cmpl;
            age[c_slot]  <= 0;
            bm[c_slot]   <= bm_new;
            if (c_new) begin
                k_saddr[c_slot] <= s_axis_saddr;
                k_id[c_slot]    <= p_id;
                k_sport[c_slot] <= p_sport;
                k_dport[c_slot] <= p_dport;
                k_len[c_slot]   <= p_len[LW-1:0];
            end
        end
    end
end

endmodule
