#!/bin/bash

workspace_path=/home/zh/workspace/webrtc-video-streaming
exec_path="$workspace_path/out/Exp0"
video_path=/data/zh/video_frames/c001/%06d.png

# 启动 receiver
setsid "$exec_path"/video_streaming_client_headless \
    --server=localhost \
    --video="$video_path" \
    --trace_file="$workspace_path/expr/trace_receiver.json" \
    > >(tee "$workspace_path/expr/log_receiver.txt" ) 2>&1 &

RECEIVER_PID=$!

sleep 1

# 启动 sender
setsid "$exec_path"/video_streaming_client_headless \
    --server=localhost \
    --video="$video_path" \
    --autocall \
    --trace_file="$workspace_path/expr/trace_sender.json" \
    > >(tee "$workspace_path/expr/log_sender.txt" ) 2>&1 &

SENDER_PID=$!

sleep 25

echo "Receiver PID = $RECEIVER_PID"
echo "Sender PID   = $SENDER_PID"

kill -9 -$SENDER_PID
kill -9 -$RECEIVER_PID

sleep 1

# 后处理 JSON
python3 "$workspace_path/scripts/fix_tail_and_expand.py" "$workspace_path/expr/trace_sender.json"
python3 "$workspace_path/scripts/fix_tail_and_expand.py" "$workspace_path/expr/trace_receiver.json"
