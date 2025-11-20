#/bin/bash
python_path=/home/zh/anaconda3/envs/py312_torch260/bin/python3

rsync-fzf pull room528-01 /home/zh/workspace/webrtc-video-streaming/expr/trace_receiver.json expr/
"$python_path" scripts/fix_tail_and_expand.py expr/trace_sender.json
"$python_path" scripts/fix_tail_and_expand.py expr/trace_receiver.json
"$python_path" scripts/analysis.py expr/trace_sender.json expr/trace_receiver.json > expr/result.txt
