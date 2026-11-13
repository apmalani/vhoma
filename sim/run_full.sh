#!/bin/sh
# Full regression: fake-peer protocol tests, IPv4 shim, and three-endpoint system runs
# (in parallel). Usage: sim/run_full.sh      Logs: build/reg_*.log
cd "$(dirname "$0")/.." || exit 1
mkdir -p build
RTL=$(cat sim/rtl_files.txt)
i=0
start() {   # start <name> <top> <tb file> [params...]
    name=$1; top=$2; tb=$3; shift 3
    ( iverilog -g2005 -I rtl -o build/reg_$name.vvp -s $top "$@" $tb $RTL > build/reg_$name.log 2>&1 &&
      vvp -n build/reg_$name.vvp >> build/reg_$name.log 2>&1 ) &
}
start peer1  tb_peer sim/tb_homa_peer.v -Ptb_peer.SEED=1
start peer2  tb_peer sim/tb_homa_peer.v -Ptb_peer.SEED=2
start ipv4_1 tb_ipv4 sim/tb_homa_ipv4.v -Ptb_ipv4.SEED=1
start ipv4_2 tb_ipv4 sim/tb_homa_ipv4.v -Ptb_ipv4.SEED=2
start sys01 tb_sys sim/tb_homa_system.v -Ptb_sys.SEED=1
start sys02 tb_sys sim/tb_homa_system.v -Ptb_sys.SEED=2 -Ptb_sys.JIT=300
start sys03 tb_sys sim/tb_homa_system.v -Ptb_sys.SEED=3 -Ptb_sys.SVC=3000
start sys04 tb_sys sim/tb_homa_system.v -Ptb_sys.SEED=4 -Ptb_sys.MAXOUT=6 -Ptb_sys.GAP=20 -Ptb_sys.NRPC=4
wait
start sys05 tb_sys sim/tb_homa_system.v -Ptb_sys.SEED=5 -Ptb_sys.LOSS=20 -Ptb_sys.STRICT=0
start sys06 tb_sys sim/tb_homa_system.v -Ptb_sys.SEED=6 -Ptb_sys.DUP=30 -Ptb_sys.STRICT=0
start sys07 tb_sys sim/tb_homa_system.v -Ptb_sys.SEED=7 -Ptb_sys.BAD=20 -Ptb_sys.STRICT=0
start sys08 tb_sys sim/tb_homa_system.v -Ptb_sys.SEED=8 -Ptb_sys.LOSS=50 -Ptb_sys.DUP=30 -Ptb_sys.BAD=20 -Ptb_sys.JIT=200 -Ptb_sys.STRICT=0
start sys09 tb_sys sim/tb_homa_system.v -Ptb_sys.SEED=9 -Ptb_sys.K=1 -Ptb_sys.SLV=1 -Ptb_sys.MAXOUT=4
start sys10 tb_sys sim/tb_homa_system.v -Ptb_sys.SEED=10 -Ptb_sys.K=4 -Ptb_sys.SLV=6 -Ptb_sys.MAXOUT=4 -Ptb_sys.APPST=200
start sys11 tb_sys sim/tb_homa_system.v -Ptb_sys.SEED=11 -Ptb_sys.SEG=8 -Ptb_sys.MAXM=128 -Ptb_sys.UNS=32 -Ptb_sys.RTT=32 -Ptb_sys.LOSS=20 -Ptb_sys.STRICT=0
start sys12 tb_sys sim/tb_homa_system.v -Ptb_sys.SEED=12 -Ptb_sys.SEG=256 -Ptb_sys.MAXM=4096 -Ptb_sys.UNS=4096 -Ptb_sys.RTT=512 -Ptb_sys.NREQ=60
wait
fail=0
for f in build/reg_*.log; do
    line=$(grep "^DONE" "$f" | tail -1)
    [ -n "$line" ] || line="NO RESULT"
    printf '%-10s %s\n' "$(basename "$f" .log | sed 's/reg_//')" "$line"
    case "$line" in *PASS) ;; *) fail=1; grep -m 3 "ERROR\|error" "$f" ;; esac
done
exit $fail
