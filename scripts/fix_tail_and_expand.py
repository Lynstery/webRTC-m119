import json
import sys

def fix_tail(path):
    # 读取全部行
    with open(path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    # 从后向前找到最后一行非空行的索引
    idx = len(lines) - 1
    while idx >= 0 and lines[idx].strip() == "":
        idx -= 1

    # 删除这一行
    if idx >= 0:
        del lines[idx]

    # 写回文件，并追加 ]}
    with open(path, "w", encoding="utf-8") as f:
        for line in lines:
            f.write(line)
        f.write("]}\n")

    print("✅ Last non-empty line removed and appended `]}`")


def expand_embedded_json(obj):
    """Recursively expand 'json' string field in 'args' dicts."""
    if isinstance(obj, dict):
        # If contains args.json, expand it
        if "args" in obj and isinstance(obj["args"], dict) and "json" in obj["args"]:
            j = obj["args"].get("json")
            if isinstance(j, str):
                try:
                    inner = json.loads(j)  # decode inner JSON string
                    obj["args"].update(inner)
                    del obj["args"]["json"]
                except Exception as e:
                    print("Warning: cannot decode:", j, e)

        # Recurse children
        for k, v in obj.items():
            obj[k] = expand_embedded_json(v)

    elif isinstance(obj, list):
        return [expand_embedded_json(i) for i in obj]

    return obj

def dump_trace_json(data, outfile):
    assert "traceEvents" in data and isinstance(data["traceEvents"], list)

    with open(outfile, "w", encoding="utf-8") as f:
        f.write('{ "traceEvents": [\n')
        events = data["traceEvents"]
        for i, ev in enumerate(events):
            prefix = "" if i == 0 else ","
            json_str = json.dumps(ev, ensure_ascii=False)
            f.write(f"{prefix}{json_str}\n")
        f.write("]}")

    print("✅ Embeded json expanded.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: python {sys.argv[0]} input.json")
        sys.exit(1)

    infile = sys.argv[1]

    fix_tail(infile)

    with open(infile, "r") as f:
        data = json.load(f)

    data = expand_embedded_json(data)

    dump_trace_json(data, infile)

    print(f"✅ Output written to {infile}")
