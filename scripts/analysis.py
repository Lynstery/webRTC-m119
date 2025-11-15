import sys
import json
import pandas as pd

def load_events(path):
    with open(path) as f:
        data = json.load(f)
    return data["traceEvents"]

def flatten_event(e, source):
    """展开 args，仅保留 name, ts, args.xxx，并增加 from 字段"""

    out = {
        "name": e.get("name"),
        "ts": e.get("ts"),
        "from": source, 
    }

    args = e.get("args", {})
    if isinstance(args, dict):
        for k, v in args.items():
            out[f"args.{k}"] = v

    return out

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"Usage: python {sys.argv[0]} trace_sender.json trace_receiver.json")
        sys.exit(1)

    path_sender = sys.argv[1]
    path_receiver = sys.argv[2]

    # 读取原始数据
    events_sender = load_events(path_sender)
    events_receiver = load_events(path_receiver)

    # 展开 + 加 from 字段
    flat_sender = [flatten_event(e, "sender") for e in events_sender]
    flat_receiver = [flatten_event(e, "receiver") for e in events_receiver]

    # 合并两个列表
    all_events = flat_sender + flat_receiver

    # 转成 Pandas DataFrame
    df = pd.DataFrame(all_events)

    print(df.head())            # 展示前几行
    print("Total events:", len(df))

    # 可选：保存文件
    # df.to_csv("merged_events.csv", index=False)