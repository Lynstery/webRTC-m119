#!/usr/bin/env bash
set -euo pipefail

REF_DIR=$1
DIST_DIR=$2
WIDTH=$3
HEIGHT=$4
OUTPUT_CSV=$5

PIX_FMT=420
BITDEPTH=8

echo "tracking_id,vmaf" > "$OUTPUT_CSV"

shopt -s nullglob

for dist_path in "$DIST_DIR"/*.yuv; do
  echo "Processing $dist_path ..."
  dist_file="$(basename "$dist_path")"
  dist_file_without_ext="${dist_file%.*}"
  frame_id=$((10#$dist_file_without_ext)) 
  echo "Processing frame ID: $frame_id"
  ref_path="$REF_DIR/$dist_file"

  if [[ ! -f "$ref_path" ]]; then
    echo "[WARN] ref frame not found: $ref_path , skip"
    continue
  fi

  # 临时 json 输出
  tmp_json="$(mktemp)"
  echo "exec: vmaf -r $ref_path -d $dist_path --width $WIDTH --height $HEIGHT --pixel_format $PIX_FMT --bitdepth $BITDEPTH --json --output $tmp_json"

  vmaf \
    -r "$ref_path" \
    -d "$dist_path" \
    --width "$WIDTH" \
    --height "$HEIGHT" \
    --pixel_format "$PIX_FMT" \
    --bitdepth "$BITDEPTH" \
    --json \
    --output "$tmp_json" \
    >/dev/null 2>&1

  # 取 pooled vmaf
  vmaf_score="$(jq -r '.pooled_metrics.vmaf.mean' "$tmp_json")"

  echo "$dist_file, $vmaf_score"
  echo "$dist_file, $vmaf_score" >> "$OUTPUT_CSV"

  rm -f "$tmp_json"
done

echo "Done. Result saved to $OUTPUT_CSV"