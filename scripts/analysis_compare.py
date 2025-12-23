import sys
import json
from sys import argv
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from dataclasses import dataclass, field
from typing import List, Dict, Optional

def find_bad_line(path):
    with open(path, "rb") as f:
        for lineno, line in enumerate(f, 1):
            try:
                line.decode("utf-8")
            except UnicodeDecodeError:
                print(f"❌ 非 UTF-8 行号: {lineno}")
                print(line[:200])  # 前 200 字节
                break
            
def fix_tail(path):
    find_bad_line(path)
    with open(path, "r") as f:
        lines = f.readlines()

    idx = len(lines) - 1
    while idx >= 0 and lines[idx].strip() == "":
        idx -= 1

    if idx >= 0:
        del lines[idx]

    with open(path, "w") as f:
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

    events_sender = load_events(path_trace_sender)
    events_receiver = load_events(path_trace_receiver)

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
    tracking_id: Optional[int] = None

    dropped_reason: Optional[str] = None

    ts_captured: Optional[int] = None
    ts_start_encode: Optional[int] = None
    ts_encoded: Optional[int] = None

    rtp_ts: Optional[int] = None
    frame_type: Optional[str] = None
    frame_size: Optional[int] = None
    qp: Optional[int] = None

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
    decode_scheduling_delay: Optional[int] = None
    decoding_delay: Optional[int] = None
    e2e_delay: Optional[int] = None
    
    psnr: Optional[float] = None
    ssim: Optional[float] = None


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
    add_index("Frame:Quality", "args.tracking_id")

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
        frame.tracking_id = int(evt_cap["args.tracking_id"])
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
        frame.qp = int(evt_encoded["args.qp"])

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
        frame.decode_scheduling_delay = frame.ts_start_decode - frame.ts_received_encoded
        frame.decoding_delay = frame.ts_decoded - frame.ts_start_decode
        frame.e2e_delay = frame.ts_decoded - frame.ts_captured
        
        evt_quality = idx["Frame:Quality"].get(frame.tracking_id, [None])[0]
        if evt_quality:
            frame.psnr = float(evt_quality["args.psnr"])
            frame.ssim = float(evt_quality["args.ssim"])

def draw_frame_size_qp_psnr_compare(
    capture_a,
    capture_b,
    pdf_path="frame_size_qp_psnr_compare.pdf",
    label_a="A",
    label_b="B",
    start_frame=50,
    max_frames=1000,
):
    """
    在同一个 PDF 中对比两种方案（A vs B）：
      子图1：Frame Size
      子图2：QP
      子图3：PSNR
    """

    # ====== 统一颜色定义（核心修改点） ======
    COLOR_A = "tab:blue"
    COLOR_B = "tab:orange"
    AVG_ALPHA = 0.7
    # =======================================

    def collect(frames_dict):
        frames = list(frames_dict.values())
        frames = frames[start_frame:]

        if max_frames is not None and len(frames) > max_frames:
            frames = frames[:max_frames]

        idxs, sizes, qps, psnrs = [], [], [], []

        for f in frames:
            if (getattr(f, "frame_size", None) is None or
                getattr(f, "qp", None) is None or
                getattr(f, "psnr", None) is None):
                continue
            idxs.append(f.tracking_id)
            sizes.append(f.frame_size)
            qps.append(f.qp)
            psnrs.append(f.psnr)

        return idxs, sizes, qps, psnrs

    idx_a, size_a, qp_a, psnr_a = collect(capture_a)
    idx_b, size_b, qp_b, psnr_b = collect(capture_b)

    if not idx_a or not idx_b:
        print("[draw_frame_size_qp_psnr_compare] Missing valid data.")
        return

    # -------- 对齐 x 轴（取交集） --------
    common_idx = sorted(set(idx_a) & set(idx_b))

    def align(idxs, vals):
        m = dict(zip(idxs, vals))
        return [m[i] for i in common_idx if i in m]

    size_a = align(idx_a, size_a)
    qp_a   = align(idx_a, qp_a)
    psnr_a = align(idx_a, psnr_a)

    size_b = align(idx_b, size_b)
    qp_b   = align(idx_b, qp_b)
    psnr_b = align(idx_b, psnr_b)

    # -------- 创建子图 --------
    width = max(18, len(common_idx) / 30)
    fig, (ax_size, ax_qp, ax_psnr) = plt.subplots(
        3, 1, figsize=(width, 11), sharex=True
    )

    # ====================================================
    # 1) Frame Size
    # ====================================================
    ax_size.plot(common_idx, size_a, color=COLOR_A, label=label_a)
    ax_size.plot(common_idx, size_b, color=COLOR_B, label=label_b)

    ax_size.axhline(np.mean(size_a), color=COLOR_A,
                    linestyle="--", alpha=AVG_ALPHA)
    ax_size.axhline(np.mean(size_b), color=COLOR_B,
                    linestyle="--", alpha=AVG_ALPHA)

    ax_size.set_ylabel("Frame Size (bytes)")
    ax_size.set_title("Frame Size / QP / PSNR Comparison")
    ax_size.grid(True)
    ax_size.legend()

    # ====================================================
    # 2) QP
    # ====================================================
    ax_qp.plot(common_idx, qp_a, color=COLOR_A, label=label_a)
    ax_qp.plot(common_idx, qp_b, color=COLOR_B, label=label_b)

    ax_qp.axhline(np.mean(qp_a), color=COLOR_A,
                  linestyle="--", alpha=AVG_ALPHA)
    ax_qp.axhline(np.mean(qp_b), color=COLOR_B,
                  linestyle="--", alpha=AVG_ALPHA)

    ax_qp.set_ylabel("QP")
    ax_qp.grid(True, linestyle="--", alpha=0.3)
    ax_qp.legend()

    # ====================================================
    # 3) PSNR
    # ====================================================
    ax_psnr.plot(common_idx, psnr_a, color=COLOR_A, label=label_a)
    ax_psnr.plot(common_idx, psnr_b, color=COLOR_B, label=label_b)

    ax_psnr.axhline(np.mean(psnr_a), color=COLOR_A,
                    linestyle="--", alpha=AVG_ALPHA)
    ax_psnr.axhline(np.mean(psnr_b), color=COLOR_B,
                    linestyle="--", alpha=AVG_ALPHA)

    ax_psnr.set_ylabel("PSNR (dB)")
    ax_psnr.set_xlabel("Frame Index (tracking_id)")
    ax_psnr.grid(True, linestyle="--", alpha=0.3)
    ax_psnr.legend()

    # -------- 统一 x 轴刻度 --------
    ax_psnr.xaxis.set_major_locator(plt.MultipleLocator(20))

    plt.tight_layout()
    plt.savefig(pdf_path)
    plt.close()

    print(f"[draw_frame_size_qp_psnr_compare] PDF saved → {pdf_path}")

def draw_psnr_violin(capture_rtp_ts_to_frame,
                     pdf_path="psnr_violin.pdf",
                     start_frame=50,
                     max_frames=1000):
    """
    绘制 PSNR 的大提琴分布图
    """

    frames = list(capture_rtp_ts_to_frame.values())
    frames = frames[start_frame:]

    if max_frames is not None and len(frames) > max_frames:
        frames = frames[:max_frames]

    psnr_values = []

    for frame in frames:
        psnr = getattr(frame, "psnr", None)
        if psnr is not None:
            psnr_values.append(psnr)

    if not psnr_values:
        print("[draw_psnr_violin] No PSNR data found.")
        return

    avg_psnr = np.mean(psnr_values)

    plt.figure(figsize=(6, 6))

    # 大提琴图
    parts = plt.violinplot(
        psnr_values,
        showmeans=False,
        showmedians=True,
        showextrema=True
    )

    # 美化颜色
    for pc in parts["bodies"]:
        pc.set_facecolor("#4c72b0")
        pc.set_edgecolor("black")
        pc.set_alpha(0.7)

    # 平均值线
    plt.axhline(avg_psnr, color="red", linestyle="--", linewidth=1.5,
                label=f"Avg = {avg_psnr:.2f} dB")

    plt.text(
        1.02, avg_psnr,
        f"{avg_psnr:.2f} dB",
        va="bottom",
        ha="left",
        color="red",
        fontsize=10
    )

    plt.ylabel("PSNR (dB)")
    plt.title("PSNR Distribution (Violin Plot)")
    plt.xticks([1], ["All Frames"])
    plt.grid(True, axis="y", linestyle="--", alpha=0.4)
    plt.legend()
    plt.tight_layout()

    plt.savefig(pdf_path)
    plt.close()

    print(f"[draw_psnr_violin] PDF saved → {pdf_path}") 

    
def draw_delays(capture_rtp_ts_to_frame, pdf_path="delays.pdf", start_frame=50, max_frames=1000):
    """
    绘制每帧 delay 的平直(step)折线图。
    高亮显示：
      - frame.status == 'encoded' → 红色竖线
      - frame.status == 'received' → 黄色竖线
      - 其它非 decoded 状态 → 灰色竖线
    """

    frames = list(capture_rtp_ts_to_frame.values())
    frames = frames[start_frame:] 
    if max_frames is not None and len(frames) > max_frames:
        frames = frames[:max_frames]
        
    frame_indices = [frame.tracking_id for frame in frames]

    delay_fields = [
        "encoding_delay",
        "pacing_delay",
        "frame_network_delay",
        "decode_scheduling_delay",
        "decoding_delay",
        "e2e_delay",
    ]

    delay_data = {name: [] for name in delay_fields}

    # 分类的 frame 索引
    red_frames = []      # encoded
    yellow_frames = []   # received
    gray_frames = []     # others (not decoded)

    for frame in frames:
        idx = frame.tracking_id
        
        # collect delays (us -> ms)
        delay_data["encoding_delay"].append(
            frame.encoding_delay / 1000 if frame.encoding_delay is not None else None
        )
        delay_data["pacing_delay"].append(
            frame.pacing_delay / 1000 if frame.pacing_delay is not None else None
        )
        delay_data["frame_network_delay"].append(
            frame.frame_network_delay / 1000 if frame.frame_network_delay is not None else None
        )
        delay_data["decode_scheduling_delay"].append(
            frame.decode_scheduling_delay / 1000 if frame.decode_scheduling_delay is not None else None
        )
        delay_data["decoding_delay"].append(
            frame.decoding_delay / 1000 if frame.decoding_delay is not None else None
        )
        delay_data["e2e_delay"].append(
            frame.e2e_delay / 1000 if frame.e2e_delay is not None else None
        )

        # classify
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

    
    # 平直折线图（变细）
    for name in delay_fields:
        plt.step(frame_indices, delay_data[name], where="post", label=name, linewidth=0.8)

    # ---- 状态高亮 ----
    for idx in red_frames:
        plt.axvline(x=idx, color="red", alpha=0.4, linestyle="--")
    for idx in yellow_frames:
        plt.axvline(x=idx, color="yellow", alpha=0.4, linestyle="--")
    for idx in gray_frames:
        plt.axvline(x=idx, color="gray", alpha=0.25, linestyle="--")

    # x 轴更密集：
    plt.gca().xaxis.set_major_locator(plt.MultipleLocator(20))  # 每 20 帧一个 tick
    # plt.tick_params(axis='x', labelsize=7)  # 缩小字号避免重叠
    
    plt.xlabel("Frame Index")
    plt.ylabel("Delay (ms)")
    plt.title("Per-frame Delay Breakdown (Step Plot)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    plt.savefig(pdf_path)
    plt.close()

    print(f"[draw_delays] PDF saved → {pdf_path}")

def draw_e2e_delay_cdf(capture_rtp_ts_to_frame, pdf_path="e2e_cdf.pdf",
                       start_frame=50, max_frames=1000):
    """
    绘制 e2e_delay 的 CDF 图。
    e2e_delay 单位: us → 会转换成 ms。
    """
    frames = list(capture_rtp_ts_to_frame.values())
    frames = frames[start_frame:]

    if max_frames is not None and len(frames) > max_frames:
        frames = frames[:max_frames]

    # 提取 e2e delay (转换为 ms), 过滤 None
    delays_ms = [
        frame.e2e_delay / 1000
        for frame in frames
        if frame.e2e_delay is not None
    ]

    if len(delays_ms) == 0:
        print("[draw_e2e_delay_cdf] No valid e2e_delay data.")
        return

    delays = np.array(delays_ms)
    delays_sorted = np.sort(delays)

    # CDF y-values
    cdf = np.linspace(0, 1, len(delays_sorted))

    # ---- 绘图 ----
    plt.figure(figsize=(8, 6))
    plt.plot(delays_sorted, cdf, linewidth=1.2)

    plt.xlabel("e2e Delay (ms)")
    plt.ylabel("CDF")
    plt.title("CDF of e2e_delay")
    plt.grid(True)
    plt.tight_layout()

    plt.savefig(pdf_path)
    plt.close()

    print(f"[draw_e2e_delay_cdf] PDF saved → {pdf_path}")
    

def draw_frame_timeline(capture_rtp_ts_to_frame, pdf_path="timeline.pdf", start_frame=50, max_frames=1000):
    """
    绘制每帧的多阶段时间轴图，横轴时间使用毫秒（ms）。
    阶段：
        captured → start_encode → encoded
        → received_encoded → start_decode → decoded

    背景颜色：
        - 没有 encoded            → 灰色
        - 有 encoded 但未收到     → 红色
        - 收到但未解码           → 黄色
    """

    frames = list(capture_rtp_ts_to_frame.values())
    frames = frames[start_frame:]
    num_frames = len(frames)

    if max_frames is not None and num_frames > max_frames:
        frames = frames[0:max_frames]
        num_frames = len(frames)

    # 阶段顺序
    stages = [
        ("captured", "ts_captured"),
        ("start_encode", "ts_start_encode"),
        ("encoded", "ts_encoded"),
        ("received", "ts_received_encoded"),
        ("start_decode", "ts_start_decode"),
        ("decoded", "ts_decoded"),
    ]

    # 阶段颜色
    colors = {
        "captured": "#4daf4a",        # green
        "start_encode": "#377eb8",    # blue
        "encoded": "#984ea3",         # purple
        "received": "#ff7f00",        # orange
        "start_decode": "#a65628",    # brown
        "decoded": "#e41a1c",         # red
    }

    # 背景颜色
    BG_NO_ENCODED  = "#e0e0e0"   # 灰
    BG_NO_RECEIVED = "#ffcccc"   # 淡红
    BG_NO_DECODED  = "#fff4b2"   # 浅黄

    # 安全取 timestamp（自动转换为 ms）
    def safe_ts_ms(ts):
        return (ts / 1000.0) if ts is not None else np.nan

    # 为每帧构建 [(start_ms, end_ms, stage_name), ...]
    frame_intervals = []

    for frame in frames:
        times_ms = [
            safe_ts_ms(frame.ts_captured),
            safe_ts_ms(frame.ts_start_encode),
            safe_ts_ms(frame.ts_encoded),
            safe_ts_ms(frame.ts_received_encoded),
            safe_ts_ms(frame.ts_start_decode),
            safe_ts_ms(frame.ts_decoded),
        ]

        intervals = []
        for i in range(len(times_ms) - 1):
            t0, t1 = times_ms[i], times_ms[i+1]
            stage_name, _ = stages[i]

            if np.isnan(t0) or np.isnan(t1):
                continue  # 不画 missing 阶段

            intervals.append((t0, t1, stage_name))

        frame_intervals.append(intervals)

    # --- 图尺寸自动扩展（长视频也能看清楚） ---
    width = max(12, num_frames / 20)
    height = max(6, num_frames / 40)
    fig, ax = plt.subplots(figsize=(width, height))

    y_positions = np.arange(num_frames)

    # 先画背景色（整条 y 带）
    for idx, frame in enumerate(frames):
        if frame.ts_encoded is None:
            bg_color = BG_NO_ENCODED
        elif frame.ts_received_encoded is None:
            bg_color = BG_NO_RECEIVED
        elif frame.ts_decoded is None:
            bg_color = BG_NO_DECODED
        else:
            bg_color = None

        if bg_color:
            # 在该帧所在的整条 y 区间上画背景（整个 x 轴范围）
            ax.axhspan(idx - 0.5, idx + 0.5,
                       facecolor=bg_color, alpha=0.6, zorder=0)

    # 再画每帧所有阶段的时间线
    for frame_idx, intervals in enumerate(frame_intervals):
        for (t0, t1, stage) in intervals:
            ax.hlines(
                y=frame_idx,
                xmin=t0,
                xmax=t1,
                colors=colors[stage],
                linewidth=1.0,
                zorder=1,
            )

    # y 轴标签 & 方向
    ax.set_yticks(y_positions)
    ax.set_yticklabels([str(frame.tracking_id) for frame in frames])
    ax.invert_yaxis()  # 让 0 帧在最上面

    # --- 图属性 ---
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Frame Index")
    ax.set_title("Per-frame Processing Timeline (ms)")

    # grid on x axis
    ax.grid(True, axis="x", linestyle="--", alpha=0.3)

    # 手动 legend（阶段）
    legend_lines = []
    legend_labels = []
    for stage_name, color in colors.items():
        legend_lines.append(
            plt.Line2D([0], [0], color=color, lw=4)
        )
        legend_labels.append(stage_name)
    ax.legend(legend_lines, legend_labels, loc="upper right")

    plt.tight_layout()
    plt.savefig(pdf_path)
    plt.close()

    print(f"[draw_frame_timeline] PDF saved → {pdf_path}")


if __name__ == "__main__":
    from sys import argv
    import os

    assert 6 <= len(argv) <= 8, (
        "Usage:\n"
        "  python analysis.py "
        "<sender_a> <receiver_a> "
        "<sender_b> <receiver_b> "
        "<fig_save_path> "
        "[time_diff_ms_a] [time_diff_ms_b]"
    )

    # -------- 参数解析 --------
    path_sender_a   = argv[1]
    path_receiver_a = argv[2]
    path_sender_b   = argv[3]
    path_receiver_b = argv[4]
    fig_save_path   = argv[5]

    time_diff_ms_a = int(argv[6]) if len(argv) > 6 else 0
    time_diff_ms_b = int(argv[7]) if len(argv) > 7 else 0

    os.makedirs(fig_save_path, exist_ok=True)

    # ============================================================
    # A: Baseline
    # ============================================================
    print("========== Parsing trace A ==========")

    df_a = read_and_parse_trace(path_sender_a, path_receiver_a)
    df_a.loc[df_a["from"] == "receiver", "ts"] -= time_diff_ms_a * 1000

    idx_a = build_event_index(df_a)
    capture_rtp_ts_to_frame_a, rtp_ts_to_frame_a = extract_frames_packets(df_a, idx_a)

    extract_packets(df_a, idx_a, rtp_ts_to_frame_a)
    extract_frame_receiving(df_a, idx_a, rtp_ts_to_frame_a)

    print("[A] Captured frames:", len(capture_rtp_ts_to_frame_a))
    print("[A] Encoded frames :", len(rtp_ts_to_frame_a))
    print("[A] Decoded frames :", sum(
        1 for f in rtp_ts_to_frame_a.values() if f.status == "decoded"
    ))

    # ============================================================
    # B: Proposed
    # ============================================================
    print("========== Parsing trace B ==========")

    df_b = read_and_parse_trace(path_sender_b, path_receiver_b)
    df_b.loc[df_b["from"] == "receiver", "ts"] -= time_diff_ms_b * 1000

    idx_b = build_event_index(df_b)
    capture_rtp_ts_to_frame_b, rtp_ts_to_frame_b = extract_frames_packets(df_b, idx_b)

    extract_packets(df_b, idx_b, rtp_ts_to_frame_b)
    extract_frame_receiving(df_b, idx_b, rtp_ts_to_frame_b)

    print("[B] Captured frames:", len(capture_rtp_ts_to_frame_b))
    print("[B] Encoded frames :", len(rtp_ts_to_frame_b))
    print("[B] Decoded frames :", sum(
        1 for f in rtp_ts_to_frame_b.values() if f.status == "decoded"
    ))

    # ============================================================
    # 对比绘图
    # ============================================================
    print("========== Drawing comparison ==========")

    start_frame = 50
    max_frames = 3000

    draw_frame_size_qp_psnr_compare(
        capture_rtp_ts_to_frame_a,
        capture_rtp_ts_to_frame_b,
        pdf_path=f"{fig_save_path}/frame_size_qp_psnr_compare.pdf",
        label_a="A",
        label_b="B",
        start_frame=start_frame,
        max_frames=max_frames,
    )

    print("========== Done ==========")
    
