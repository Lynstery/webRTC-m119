import json
from sys import argv
import numpy as np
import pandas as pd
from dataclasses import dataclass, field
from typing import List, Dict, Optional

def fix_tail(path):
    with open(path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    idx = len(lines) - 1
    while idx >= 0 and lines[idx].strip() == "":
        idx -= 1

    if idx >= 0:
        del lines[idx]

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

def fix_and_expand_trace_file(file):
    fix_tail(file)
    with open(file, "r") as f:
        data = json.load(f)
    data = expand_embedded_json(data)
    dump_trace_json(data, file)


def read_and_parse_trace(path_trace_sender, path_trace_receiver):

    fix_and_expand_trace_file(path_trace_sender)
    fix_and_expand_trace_file(path_trace_receiver)

    def load_events(path):
        with open(path) as f:
            data = json.load(f)
        return data["traceEvents"]

    def flatten_event(e, source):
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

    def preprocess_events(events):
        tmp = pd.DataFrame(events)
        args_uint64 = tmp.dtypes[tmp.dtypes == "float64"]
        print(args_uint64.index.tolist())
        for e in events:
            for arg in args_uint64.index.tolist():
                if arg not in e:
                    e[arg] = 0

    events_sender = load_events(path_sender)
    events_receiver = load_events(path_receiver)

    flat_sender = [flatten_event(e, "sender") for e in events_sender]
    flat_receiver = [flatten_event(e, "receiver") for e in events_receiver]

    all_events = flat_sender + flat_receiver
    preprocess_events(all_events)
    df = pd.DataFrame(all_events)
    return df

@dataclass
class PacketInfo:
    seq: Optional[int] = None
    packet_type: Optional[str] = None  # video / fec / rtx
    frame_rtp_ts: Optional[int] = None
    ts_sent: Optional[int] = None
    ts_received: Optional[int] = None
    is_recovered: bool = False # is this packet recovered by FEC/RTX
    extra_info: dict = field(default_factory=dict)


@dataclass
class FrameInfo:
    status: str = "captured"   # dropped_before_encode / dropped_by_encoder / encoded / received / decoded
    rtp_ts_capture: Optional[int] = None

    dropped_reason: Optional[str] = None

    ts_captured: Optional[int] = None
    ts_start_encode: Optional[int] = None
    ts_encoded: Optional[int] = None

    rtp_ts: Optional[int] = None
    frame_type: Optional[str] = None
    frame_size: Optional[int] = None

    num_media_packets: Optional[int] = None
    num_fec_packets: Optional[int] = None

    packet_infos: List[PacketInfo] = field(default_factory=list)

    ts_received_encoded: Optional[int] = None
    picture_id: Optional[int] = None
    refs: Optional[str] = None

    ts_start_decode: Optional[int] = None
    ts_decoded: Optional[int] = None

    encoding_delay: Optional[int] = None
    pacing_delay: Optional[int] = None
    frame_network_delay: Optional[int] = None
    jitter_delay: Optional[int] = None
    decoding_delay: Optional[int] = None
    e2e_delay: Optional[int] = None


def build_event_index(df: pd.DataFrame):

    idx = {}

    def add_index(event_name, key_col):
        sub = df[df["name"] == event_name].to_dict("records")
        idx[event_name] = {}
        for row in sub:
            key = row[key_col]
            idx[event_name].setdefault(key, []).append(row)

    # Frame related
    add_index("Frame:Captured", "args.rtp_ts_capture")
    add_index("Frame:Dropped", "args.rtp_ts_capture")
    add_index("Frame:Start Encode", "args.rtp_ts_capture")
    add_index("Frame:Encoded", "args.rtp_ts_capture")
    add_index("Frame:Packetization", "args.rtp_ts")
    add_index("Frame:Generate FEC", "args.last_rtp_ts")

    # Packet receiving
    add_index("Packet:Receive Media RTP", "args.seq")
    add_index("Packet:Receive Recovered Media RTP", "args.seq")
    add_index("FlexFEC:Receive FEC Packet", "args.fec_seq")
    add_index("Rtx:Receive RTX Packet", "args.rtx_seq")

    # Frame receiving
    add_index("Frame:Received EncodedFrame", "args.rtp_ts")
    add_index("Frame:Start Decode", "args.rtp_ts")
    add_index("Frame:Decoded", "args.rtp_ts")

    return idx


def extract_frames_packets(df: pd.DataFrame, idx) -> Dict[int, FrameInfo]:
    capture_rtp_ts_to_frame: Dict[int, FrameInfo] = {} # rtp_ts_capture -> rtp_ts
    rtp_ts_to_frame: Dict[int, FrameInfo] = {} # rtp_ts -> frame_info

    all_capture_ts = df[df["name"] == "Frame:Captured"]["args.rtp_ts_capture"].unique()
    all_capture_ts.sort()

    for rtp_ts_capture in all_capture_ts:
        frame = FrameInfo(rtp_ts_capture=rtp_ts_capture)

        # --- Capture ---
        evt_cap = idx["Frame:Captured"][rtp_ts_capture][0]
        frame.ts_captured = int(evt_cap["ts"])
        frame.status = "captured"

        # --- Dropped before encoding ---
        evt_drop = idx["Frame:Dropped"].get(rtp_ts_capture, [None])[0]
        if evt_drop:
            frame.status = "dropped_before_encode"
            frame.dropped_reason = evt_drop["args.reason"]
            capture_rtp_ts_to_frame[rtp_ts_capture] = frame
            continue

        # --- Start Encode ---
        evt_start_enc = idx["Frame:Start Encode"].get(rtp_ts_capture, [None])[0]
        if evt_start_enc:
            frame.ts_start_encode = int(evt_start_enc["ts"])
        else:
            frame.status = "dropped_before_encode"
            capture_rtp_ts_to_frame[rtp_ts_capture] = frame
            continue

        # --- Encoded ---
        evt_encoded = idx["Frame:Encoded"].get(rtp_ts_capture, [None])[0]
        if not evt_encoded:
            frame.status = "dropped_by_encoder"
            capture_rtp_ts_to_frame[rtp_ts_capture] = frame
            continue

        frame.status = "encoded"
        frame.ts_encoded = int(evt_encoded["ts"])
        frame.rtp_ts = int(evt_encoded["args.rtp_ts"])
        frame.frame_type = evt_encoded["args.frame_type"]
        frame.frame_size = int(evt_encoded["args.frame_size"])

        frame.encoding_delay = frame.ts_encoded - frame.ts_captured

        evt_frame_packetization = idx["Frame:Packetization"].get(frame.rtp_ts, [None])[0]
        packet_count = evt_frame_packetization["args.packet_count"] if evt_frame_packetization else 0
        evt_frame_generate_fec = idx["Frame:Generate FEC"].get(frame.rtp_ts, [None])[0]
        num_fec_packets = evt_frame_generate_fec["args.num_fec_packets"] if evt_frame_generate_fec else 0

        frame.num_media_packets = packet_count
        frame.num_fec_packets = num_fec_packets

        capture_rtp_ts_to_frame[rtp_ts_capture] = frame
        rtp_ts_to_frame[frame.rtp_ts] = frame

    return capture_rtp_ts_to_frame, rtp_ts_to_frame

def extract_packets(df: pd.DataFrame, idx, rtp_ts_to_frame: Dict[int, FrameInfo]):
    sent_events = df[df["name"] == "Packet:Sent"].sort_values(by="ts")

    for _, row in sent_events.iterrows():
        packet_type = row["args.packet_type"]
        if packet_type not in ["video", "fec", "rtx"]:
            continue

        if packet_type == "fec":
            frame_rtp_ts = row.get("args.protected_frame_rtp_ts", None)
        else:
            frame_rtp_ts = row.get("args.rtp_ts", None)

        frame = rtp_ts_to_frame.get(frame_rtp_ts, None)
        if frame is None:
            continue

        packet = PacketInfo(
            seq=int(row["args.seq"]),
            packet_type=row["args.packet_type"],
            ts_sent=int(row["ts"]),
            frame_rtp_ts=frame_rtp_ts,
        )
        frame.packet_infos.append(packet)

        if packet_type == "video":
            
            evt_recv_list = idx["Packet:Receive Media RTP"].get(packet.seq, [])
            evt_recv = None
            for evt in evt_recv_list:
                if evt["args.rtp_ts"] == frame_rtp_ts:
                    evt_recv = evt
                    break
                
            evt_recover_recv_list = idx["Packet:Receive Recovered Media RTP"].get(packet.seq, [])
            evt_recover_recv = None
            for evt in evt_recover_recv_list:
                if evt["args.rtp_ts"] == frame_rtp_ts:
                    evt_recover_recv = evt
                    break
                
            if evt_recv:
                packet.ts_received = int(evt_recv["ts"])
                packet.is_recovered = False
            elif evt_recover_recv:
                packet.ts_received = int(evt_recover_recv["ts"]) # the received ts means the recovered time
                packet.is_recovered = True # recovered by FEC/RTX
            else:
                packet.ts_received = None
                packet.is_recovered = False

        elif packet_type == "fec":
            evt_recv_list = idx["FlexFEC:Receive FEC Packet"].get(packet.seq, [])
            evt_recv = None
            for evt in evt_recv_list:
                if evt["ts"] > packet.ts_sent and evt["ts"] - packet.ts_sent < 2000000: # within 2s after sent (rtt unlikely > 2s)
                    evt_recv = evt
                    break
                
            if evt_recv:
                packet.ts_received = int(evt_recv["ts"])
                packet.is_recovered = False
                packet.extra_info["protected_seqs"] = evt_recv["args.protected_seqs"]
            else:
                packet.ts_received = None
                packet.is_recovered = False
        else: # packet_type == "rtx"
            evt_recv_list = idx["Rtx:Receive RTX Packet"].get(packet.seq, [])
            evt_recv = None
            for evt in evt_recv_list:
                if evt["ts"] > packet.ts_sent and evt["ts"] - packet.ts_sent < 2000000: # within 2s after sent (rtt unlikely > 2s)
                    evt_recv = evt
                    break
                
            if evt_recv:
                packet.ts_received = int(evt_recv["ts"])
                packet.is_recovered = False
                packet.extra_info["original_seq"] = evt_recv["args.seq"]
            else:
                packet.ts_received = None
                packet.is_recovered = False

def extract_frame_receiving(df: pd.DataFrame, idx, rtp_ts_to_frame: Dict[int, FrameInfo]):

    for frame in rtp_ts_to_frame.values():

        num_media_packets_sent = 0
        ts_last_media_packet_sent = None
        for packet in frame.packet_infos:
            if packet.packet_type == "video":
                num_media_packets_sent += 1
                ts_last_media_packet_sent = packet.ts_sent

        if num_media_packets_sent < frame.num_media_packets:
            continue
        frame.pacing_delay = ts_last_media_packet_sent - frame.ts_encoded

        evt_received_encoded = idx["Frame:Received EncodedFrame"].get(frame.rtp_ts, [None])[0]
        
        if evt_received_encoded is None:
            continue
        frame.status = "received"
        frame.ts_received_encoded = int(evt_received_encoded["ts"])
        frame.picture_id = int(evt_received_encoded["args.picture_id"])
        frame.refs = evt_received_encoded["args.refs"]

        frame.frame_network_delay = frame.ts_received_encoded - ts_last_media_packet_sent

        evt_start_decode = idx["Frame:Start Decode"].get(frame.rtp_ts, [None])[0]
        evt_decoded = idx["Frame:Decoded"].get(frame.rtp_ts, [None])[0]
        if evt_decoded is None:
            continue
        frame.status = "decoded"
        frame.ts_start_decode = int(evt_start_decode["ts"])
        frame.ts_decoded = int(evt_decoded["ts"])
        frame.jitter_delay = frame.ts_start_decode - frame.ts_received_encoded
        frame.decoding_delay = frame.ts_decoded - frame.ts_start_decode
        frame.e2e_delay = frame.ts_decoded - frame.ts_captured

def print_result(capture_rtp_ts_to_frame):
    for frame in capture_rtp_ts_to_frame.values():
        print("--------------------------------")
        print("Frame rtp_ts_capture = ", frame.rtp_ts_capture)
        if frame.status == "dropped_before_encode":
            print("dropped before encode, reason =", frame.dropped_reason)
            continue
        elif frame.status == "dropped_by_encoder":
            print("dropped by encoder")
            continue
        print("rtp_ts =", frame.rtp_ts)
        print("frame_type =", frame.frame_type)
        print("frame_size =", frame.frame_size, "bytes")
        print("num_media_packets =", frame.num_media_packets)
        print("num_fec_packets =", frame.num_fec_packets)
        print("Packets:")
        for packet in frame.packet_infos:
            print(f"    type={packet.packet_type}, seq={packet.seq}")
            print(f"    sent_time_after_encoded={(packet.ts_sent - frame.ts_encoded)/1000} ms", end="")
            if packet.ts_received is not None:
                print(f", received_time_after_encoded={(packet.ts_received - frame.ts_encoded)/1000} ms, recovered={int(packet.is_recovered)}")
                if packet.packet_type == "fec":
                    protected_seqs = packet.extra_info.get("protected_seqs", [])
                    print(f"        protected_seqs={protected_seqs}")
                elif packet.packet_type == "rtx":
                    original_seq = packet.extra_info.get("original_seq", None)
                    print(f"        original_seq={original_seq}")
            else:
                print(", NOT received")

        print("Frame Metrics:")
        print(f"    status={frame.status}")
        if frame.encoding_delay:
            print(f"    encoding_delay={frame.encoding_delay/1000} ms")
        if frame.pacing_delay:
            print(f"    pacing_delay={frame.pacing_delay/1000} ms")
        if frame.frame_network_delay:
            print(f"    frame_network_delay={frame.frame_network_delay/1000} ms")
        if frame.jitter_delay:
            print(f"    jitter_delay={frame.jitter_delay/1000} ms")
        if frame.decoding_delay:
            print(f"    decoding_delay={frame.decoding_delay/1000} ms")
        if frame.e2e_delay:
            print(f"    e2e_delay={frame.e2e_delay/1000} ms")
            
import matplotlib.pyplot as plt

def draw_delays(capture_rtp_ts_to_frame, pdf_path="delays.pdf", max_frames=1000):
    """
    绘制每帧 delay 折线图，保存为 PDF。
    高亮显示：
      - frame.status == 'encoded' → 红色竖线
      - frame.status == 'received' → 黄色竖线
      - 其它非 decoded 状态 → 灰色竖线
    """

    frames = list(capture_rtp_ts_to_frame.values())
    
    if max_frames is not None and len(frames) > max_frames:
        frames = frames[0: max_frames]
        
    frame_indices = list(range(len(frames)))

    delay_fields = [
        "encoding_delay",
        "pacing_delay",
        "frame_network_delay",
        "jitter_delay",
        "decoding_delay",
        "e2e_delay",
    ]

    delay_data = {name: [] for name in delay_fields}

    # 分类的 frame 索引
    red_frames = []      # encoded
    yellow_frames = []   # received
    gray_frames = []     # others (not decoded)

    for idx, frame in enumerate(frames):

        # -- collect delays (convert us → ms) --
        delay_data["encoding_delay"].append(
            frame.encoding_delay / 1000 if frame.encoding_delay is not None else None
        )
        delay_data["pacing_delay"].append(
            frame.pacing_delay / 1000 if frame.pacing_delay is not None else None
        )
        delay_data["frame_network_delay"].append(
            frame.frame_network_delay / 1000 if frame.frame_network_delay is not None else None
        )
        delay_data["jitter_delay"].append(
            frame.jitter_delay / 1000 if frame.jitter_delay is not None else None
        )
        delay_data["decoding_delay"].append(
            frame.decoding_delay / 1000 if frame.decoding_delay is not None else None
        )
        delay_data["e2e_delay"].append(
            frame.e2e_delay / 1000 if frame.e2e_delay is not None else None
        )

        # -- classify frame status --
        status = getattr(frame, "status", None)

        if status == "encoded":
            red_frames.append(idx)
        elif status == "received":
            yellow_frames.append(idx)
        elif status != "decoded":
            gray_frames.append(idx)

    # ---- 绘图 ----
    width = max(14, len(frames) / 30)
    plt.figure(figsize=(width, 7))

    # 延迟曲线
    for name in delay_fields:
        plt.plot(frame_indices, delay_data[name], label=name)

    # ---- 状态高亮 ----
    # encoded → red
    for idx in red_frames:
        plt.axvline(x=idx, color="red", alpha=0.4, linestyle="--")

    # received → yellow
    for idx in yellow_frames:
        plt.axvline(x=idx, color="yellow", alpha=0.4, linestyle="--")

    # others (lost/error) → gray
    for idx in gray_frames:
        plt.axvline(x=idx, color="gray", alpha=0.25, linestyle="--")

    plt.xlabel("Frame Index")
    plt.ylabel("Delay (ms)")
    plt.title("Per-frame Delay Breakdown (Encoded/Received/Other Highlight)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    plt.savefig(pdf_path)
    plt.close()

    print(f"[draw_delays] PDF saved → {pdf_path}")
          

if __name__ == "__main__":
    assert 4 <= len(argv) <= 5, "Usage: python analysis.py <path_sender_trace> <path_receiver_trace> <fig_save_path> [time_diff_ms]"
    path_sender = argv[1]
    path_receiver = argv[2]
    fig_save_path = argv[3]
    time_diff_ms = int(argv[4]) if len(argv) > 4 else 0
    df = read_and_parse_trace(path_sender, path_receiver)
    # Adjust receiver timestamps
    df.loc[df["from"] == "receiver", "ts"] -= time_diff_ms * 1000 
    
    idx = build_event_index(df)
    capture_rtp_ts_to_frame, rtp_ts_to_frame = extract_frames_packets(df, idx)
    print("Total captured frames:", len(capture_rtp_ts_to_frame))
    extract_packets(df, idx, rtp_ts_to_frame)
    print("Total encoded frames:", len(rtp_ts_to_frame))
    extract_frame_receiving(df, idx, rtp_ts_to_frame)
    print("Total decoded frames:", sum(1 for f in rtp_ts_to_frame.values() if f.status in ["decoded"]))
    
    print("================= Result =================")
    print_result(capture_rtp_ts_to_frame)
    draw_delays(capture_rtp_ts_to_frame, pdf_path=f"{fig_save_path}/delays.pdf") 
    
