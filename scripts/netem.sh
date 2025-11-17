#!/bin/bash
#
#  netem.sh - Simple wrapper for Linux tc netem/qdisc
#
#  Usage:
#    sudo ./netem.sh <iface> <command> [options...]
#
#  Commands:
#    enable   -- apply network emulation params
#    disable  -- clear all tc settings
#    status   -- show current tc qdisc
#
#  Examples:
#    sudo ./netem.sh eno1 enable  --delay 50ms --loss 1% --rate 10mbit
#    sudo ./netem.sh eno1 disable
#

IFACE=$1
CMD=$2
shift 2

# ----------- Default parameters -------------
DELAY=""
JITTER=""
LOSS=""
RATE=""
CORR=""

# ----------- Parse parameters -------------
while [[ $# -gt 0 ]]; do
  case $1 in
    --delay)
      DELAY="$2"
      shift 2
      ;;
    --jitter)
      JITTER="$2"
      shift 2
      ;;
    --loss)
      LOSS="$2"
      shift 2
      ;;
    --corr)
      CORR="$2"
      shift 2
      ;;
    --rate)
      RATE="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

# ----------- Apply commands -------------
case "$CMD" in
  enable)
    echo "[INFO] Applying tc netem rules on $IFACE..."

    # Delete existing rules
    tc qdisc del dev $IFACE root 2>/dev/null

    # Build netem string
    NETEM=""

    [[ -n "$DELAY" ]] && NETEM+=" delay $DELAY"
    [[ -n "$JITTER" ]] && NETEM+=" $JITTER"
    [[ -n "$CORR" ]]   && NETEM+=" $CORR"
    [[ -n "$LOSS"  ]]  && NETEM+=" loss $LOSS"

    if [[ -n "$RATE" ]]; then
      # Use tbf for bandwidth limit
      echo "[INFO] Rate limit = $RATE"
      tc qdisc add dev $IFACE root handle 1: netem $NETEM
      tc qdisc add dev $IFACE parent 1: handle 10: tbf rate $RATE burst 32kbit latency 400ms
    else
      tc qdisc add dev $IFACE root netem $NETEM
    fi

    echo "[INFO] tc rule applied."
    ;;

  disable)
    echo "[INFO] Removing tc rules from $IFACE..."
    tc qdisc del dev $IFACE root 2>/dev/null
    echo "[INFO] Cleared."
    ;;

  status)
    tc -s qdisc show dev $IFACE
    ;;

  *)
    echo "Usage: sudo ./netem.sh <iface> <enable|disable|status> [--delay X] [--jitter Y] [--loss P] [--rate B]"
    exit 1
    ;;
esac
