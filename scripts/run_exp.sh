#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 8 ]; then
  echo "Usage: $0 MAHIMAHI_ARGS DISABLE_ETHERNET VIDEO_NAME WIDTH HEIGHT FPS END_INDEX ENCODE_BITRATE [EXTRA_ARGS...]"
  echo "Example:"
  echo "  $0 \"mm-delay 40 mm-loss downlink 0.05\" 0 game2 1280 720 30 3000 10 --force_fieldtrials=Exp-FECMethod/RS/Exp-RefMod/Dynamic/Exp-FixedReferenceStep/10/"
  echo "  $0 \"\" 1 game2 1280 720 30 3000 10 --force_fieldtrials=Exp-FECMethod/RS/Exp-RefMod/Dynamic/Exp-FixedReferenceStep/10/"
  exit 1
fi

EXP_DURATION=40  # seconds

MAHIMAHI_ARGS="$1"
DISABLE_ETHERNET="$2"
VIDEO_NAME="$3"
WIDTH="$4"
HEIGHT="$5"
FPS="$6"
END_INDEX="$7"
ENCODE_BITRATE="$8"
shift 8
EXTRA_ARGS=("$@")

EXTRA_ARGS_RECEIVER=()

if [[ "${DISABLE_ETHERNET}" == "1" ]]; then
  EXTRA_ARGS_RECEIVER+=("--disable_ethernet")
fi

MM_PREFIX=()
if [[ -n "${MAHIMAHI_ARGS}" ]]; then
  # 注意：这会按空白切分；你传入的例子正好适用
  read -r -a MM_PREFIX <<< "${MAHIMAHI_ARGS}"
fi

SERVER_IP="114.212.83.86"
RECEIVER_HOST="room528-01"
SENDER_HOST="cs528"

WEBRTC_ROOT="/home/zh/workspace/webRTC-m119"
BIN="${WEBRTC_ROOT}/out/Exp0/video_streaming_client_headless"

VIDEO_CONFIG="${WIDTH}x${HEIGHT}@${FPS}"
VIDEO="/data/zh/youtube_videos/${VIDEO_NAME}/${VIDEO_CONFIG}/%06d.yuv"

EXPR_DIR="${WEBRTC_ROOT}/expr"
TRACE_SENDER="${EXPR_DIR}/trace_sender.json"
TRACE_RECEIVER="${EXPR_DIR}/trace_receiver.json"

EXTRA_TAG="-"
if ((${#EXTRA_ARGS[@]})); then
  EXTRA_TAG=$(printf '%s ' "${EXTRA_ARGS[@]}")
  EXTRA_TAG=${EXTRA_TAG//\//-}
  EXTRA_TAG=${EXTRA_TAG// /-}
fi
RECEIVER_EXTRA_TAG="-"
if ((${#EXTRA_ARGS_RECEIVER[@]})); then
  RECEIVER_EXTRA_TAG=$(printf '%s ' "${EXTRA_ARGS_RECEIVER[@]}")
  RECEIVER_EXTRA_TAG=${RECEIVER_EXTRA_TAG//\//-}
  RECEIVER_EXTRA_TAG=${RECEIVER_EXTRA_TAG// /-}
fi
RESULT_NAME="${VIDEO_NAME}-${WIDTH}@${FPS}-${ENCODE_BITRATE}Kbps-${EXTRA_TAG}${RECEIVER_EXTRA_TAG}${MAHIMAHI_ARGS// /-}"

echo "======================================"
echo "MAHIMAHI_ARGS = ${MAHIMAHI_ARGS}"
echo "DISABLE_ETHERNET = ${DISABLE_ETHERNET}"
echo "VIDEO_NAME = ${VIDEO_NAME}"
echo "RESOLUTION = ${WIDTH}x${HEIGHT}"
echo "FPS        = ${FPS}"
echo "END_INDEX  = ${END_INDEX}"
echo "BITRATE    = ${ENCODE_BITRATE}"
echo "RESULT_PATH     = ${RESULT_NAME}"
echo "EXTRA_ARGS = ${EXTRA_ARGS[*]}"
echo "EXTRA_ARGS (receiver) = ${EXTRA_ARGS_RECEIVER[*]}"
echo "======================================"

echo ">>> [1/6] 启动 receiver"

"${MM_PREFIX[@]}" "${BIN}" \
  --server="${SERVER_IP}" \
  --video="${VIDEO}" \
  --end_index="${END_INDEX}" \
  --width="${WIDTH}" \
  --height="${HEIGHT}" \
  --fps="${FPS}" \
  --trace_file="${TRACE_RECEIVER}" \
  "${EXTRA_ARGS_RECEIVER[@]}" \
  "${EXTRA_ARGS[@]}" &

sleep 2

echo ">>> [2/6] 通过 SSH 启动 sender"

ssh "${SENDER_HOST}" bash << EOF &
set -euo pipefail

WEBRTC_ROOT="/home/zh/workspace/webRTC-m119"
BIN="\${WEBRTC_ROOT}/out/Exp0/video_streaming_client_headless"

\${BIN} \
  --server="${SERVER_IP}" \
  --video="${VIDEO}" \
  --end_index="${END_INDEX}" \
  --width="${WIDTH}" \
  --height="${HEIGHT}" \
  --fps="${FPS}" \
  --autocall \
  --trace_file="\${WEBRTC_ROOT}/expr/trace_sender.json" \
  --fixed_encode_bitrate_kbps=${ENCODE_BITRATE}000 \
  --fixed_pacing_bitrate_kbps=100000 \
  "${EXTRA_ARGS[@]}"
EOF

echo ">>> [3/6] 实验运行中"
sleep $EXP_DURATION

echo ">>> [4/6] 停止 sender（cs528）"
ssh "${SENDER_HOST}" bash << 'EOF' || true
pkill -9 -f video_streaming_client_headless || true
EOF
sleep 1

echo ">>> [5/6] 停止 receiver（本机）"
pkill -9 -f video_streaming_client_headless || true

sleep 1

echo ">>> [6/6] 拉取 trace + 分析 + 归档"

rsync-fzf pull "${SENDER_HOST}" \
  "/home/zh/workspace/webRTC-m119/expr/trace_sender.json" \
  "${EXPR_DIR}/"

cd "${WEBRTC_ROOT}"

/home/zh/miniforge3/envs/plot-env/bin/python3 scripts/analysis.py \
  expr/trace_sender.json \
  expr/trace_receiver.json \
  expr

mkdir -p expr_logs
rm -rf "expr_logs/${RESULT_NAME}"
cp -r expr "expr_logs/${RESULT_NAME}"

INFO_FILE="expr_logs/${RESULT_NAME}/meta.txt"

echo ">>> 写入实验元信息: ${INFO_FILE}"

{
  echo "Time:            $(date '+%Y-%m-%d %H:%M:%S')"
  echo "MahiMahi Args:   ${MAHIMAHI_ARGS}"
  echo "Disable Ethernet: ${DISABLE_ETHERNET}"
  echo "Video:           ${VIDEO_NAME}"
  echo "Resolution:      ${WIDTH}x${HEIGHT}"
  echo "FPS:             ${FPS}"
  echo "END_INDEX:       ${END_INDEX}"
  echo "Encode bitrate:  ${ENCODE_BITRATE} Mbps"
  echo "Extra args:      ${EXTRA_ARGS[*]}"
  echo "Git commit:"
  git -C "${WEBRTC_ROOT}" rev-parse HEAD 2>/dev/null || echo "<unknown>"
  echo
  echo "Git status:"
  git -C "${WEBRTC_ROOT}" status --porcelain 2>/dev/null || echo "<unknown>"
} > "${INFO_FILE}"


echo ">>> 实验完成：expr_logs/${RESULT_NAME}"
