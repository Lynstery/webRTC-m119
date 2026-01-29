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

mahimahi_args="mm-delay 50 mm-ge-loss downlink 0.02 0.5 0.005 0.6"

FPS=60
video_args="game3 1280 720 $FPS 1000 10"

fec_args="Exp-FECMethod/RS/Exp-FixedFECRatio/5"

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
# Experiments
# ===============================

run "P-only" \
  "$mahimahi_args" 0 \
  $video_args \
  --force_fieldtrials=$fec_args/Exp-RefMode/K-step/Exp-FixedReferenceStep/1/

run "FireBreak-0(Acked-only)" \
  "$mahimahi_args" 0 \
  $video_args \
  --force_fieldtrials=$fec_args/Exp-RefMode/FireBreak/Exp-FixedFirebreakRefPrevProb/0/

run "FireBreak-50" \
  "$mahimahi_args" 0 \
  $video_args \
  --force_fieldtrials=$fec_args/Exp-RefMode/FireBreak/Exp-FixedFirebreakRefPrevProb/50/

run "FireBreak-80" \
  "$mahimahi_args" 0 \
  $video_args \
  --force_fieldtrials=$fec_args/Exp-RefMode/FireBreak/Exp-FixedFirebreakRefPrevProb/80/

run "FireBreak" \
  "$mahimahi_args" 0 \
  $video_args \
  --force_fieldtrials=$fec_args/Exp-RefMode/FireBreak/

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
