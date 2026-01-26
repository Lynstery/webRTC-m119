#!/usr/bin/env bash
set -e

# ===============================
# Environment
# ===============================
PYTHON=/home/zh/miniforge3/envs/plot-env/bin/python3
ANALYSIS=scripts/analysis_compare.py
OUTDIR=expr_logs/compare

# ===============================
# Common experiment arguments
# ===============================

# Mahimahi is disabled for this sweep (pass empty string to run_exp.sh)
mahimahi_args=""

FPS=30

# Video configuration (passed positionally to scripts/run_exp.sh)
VIDEO_NAME="game"
WIDTH=1280
HEIGHT=720
END_INDEX=1000

# 固定：
fec_args="Exp-FECMethod/RS/Exp-FixedFECRatio/10"
force_fieldtrials="--force_fieldtrials=${fec_args}/Exp-RefMode/K-step/Exp-FixedReferenceStep/1/"

# ===============================
# Result collection
# ===============================
RESULT_PATHS=()
DESCS=()
LOG_DIR=logs
mkdir -p "$LOG_DIR"

run() {
  desc="$1"
  shift
  echo "Running: $desc" >&2

  log="$LOG_DIR/${desc//[^a-zA-Z0-9_-]/_}.log"

  if [[ -f "$log" ]]; then
    echo "Log exists, skip run: $log" >&2
  else
    ./scripts/run_exp.sh "$@" >"$log" 2>&1
  fi

  result_path=$(tail -n 1 "$log")

  echo "$desc => $result_path" >&2
  RESULT_PATHS+=("$result_path")
  DESCS+=("$desc")

  sleep 1
}

# ===============================
# Experiments (bitrate sweep)
# NOTE: scripts/run_exp.sh expects bitrate in **Mbps** (it internally converts to kbps).
# ===============================
for ENCODE_BITRATE_Mbps in $(seq 1 1 50); do
  run "bitrate-${ENCODE_BITRATE_Mbps}Mbps" \
    "" 0 \
    "${VIDEO_NAME}" "${WIDTH}" "${HEIGHT}" "${FPS}" "${END_INDEX}" "${ENCODE_BITRATE_Mbps}" \
    "${force_fieldtrials}"
done

# ===============================
# Run analysis
# ===============================

ARGS=("$FPS" "$OUTDIR")
for ((i=0; i<${#RESULT_PATHS[@]}; i++)); do
  ARGS+=("${RESULT_PATHS[$i]}" "${DESCS[$i]}")
done

echo "Running analysis:" >&2
echo "$PYTHON $ANALYSIS ${ARGS[*]}" >&2

$PYTHON $ANALYSIS "${ARGS[@]}"
