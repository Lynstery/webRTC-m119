import shutil
import sys
import json
import ast
from sys import argv
import tempfile, subprocess, json, os
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from dataclasses import dataclass, field
from typing import List, Dict, Optional
from matplotlib.ticker import MultipleLocator

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
    ref_distance: Optional[int] = None

    ts_start_decode: Optional[int] = None
    ts_decoded: Optional[int] = None

    encoding_delay: Optional[int] = None
    pacing_delay: Optional[int] = None
    frame_network_delay: Optional[int] = None
    decode_scheduling_delay: Optional[int] = None
    decoding_delay: Optional[int] = None
    e2e_delay: Optional[int] = None

    width: Optional[int] = None
    height: Optional[int] = None
    psnr: Optional[float] = None
    ssim: Optional[float] = None
    vmaf: Optional[float] = None
    ref_filepath: Optional[str] = None
    recv_filepath: Optional[str] = None

# ----------------------------------------

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
        content = f.read()

    if content.endswith("]}\n"):
        print("✅ File already ends with `]}` — no changes made")
        return

    lines = content.splitlines(keepends=True)

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

    events_sender = load_events(path_sender)
    events_receiver = load_events(path_receiver)

    flat_sender = [flatten_event(e, "sender") for e in events_sender]
    flat_receiver = [flatten_event(e, "receiver") for e in events_receiver]

    all_events = flat_sender + flat_receiver
    preprocess_events(all_events)
    df = pd.DataFrame(all_events)
    return df

# ----------------------------------------

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
        rtp_ts_capture = int(rtp_ts_capture)
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
        packet_count = int(evt_frame_packetization["args.packet_count"]) if evt_frame_packetization else 0
        evt_frame_generate_fec = idx["Frame:Generate FEC"].get(frame.rtp_ts, [None])[0]
        num_fec_packets = int(evt_frame_generate_fec["args.num_fec_packets"]) if evt_frame_generate_fec else 0

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
        if frame_rtp_ts is None:
            continue
        frame_rtp_ts = int(frame_rtp_ts)

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

        if frame.num_media_packets is None:
            continue

        num_media_packets_sent = 0
        ts_last_media_packet_sent = None
        for packet in frame.packet_infos:
            if packet.packet_type == "video":
                num_media_packets_sent += 1
                ts_last_media_packet_sent = packet.ts_sent

        if ts_last_media_packet_sent is None:
            continue
        
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
        if type(frame.refs) == str:
            frame.refs = ast.literal_eval(frame.refs)
        frame.ref_distance = frame.picture_id - frame.refs[0] if frame.refs else None

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
            frame.width = int(evt_quality["args.width"])
            frame.height = int(evt_quality["args.height"])
            frame.ref_filepath = evt_quality["args.ref_filepath"]
            frame.recv_filepath = evt_quality["args.recv_filepath"]
            # calculate VMAF later
            # frame.vmaf = calculate_vmaf(frame.ref_filepath, frame.recv_filepath, frame.width, frame.height)
            # print(f"Calculated VMAF for tracking_id={frame.tracking_id}: {frame.vmaf}")

    calculate_vmaf_batch_parallel(capture_rtp_ts_to_frame.values())

def calculate_vmaf(ref_filepath: str, dist_filepath: str, width: int, height: int) -> Optional[float]:

    if not os.path.exists(ref_filepath) or not os.path.exists(dist_filepath):
        print(f"[calculate_vmaf] File not found: {ref_filepath} or {dist_filepath}")
        return None

    with tempfile.NamedTemporaryFile(delete=False, suffix=".json") as tmp_json:
        tmp_json_path = tmp_json.name

    cmd = [
        "vmaf",
        "-r", ref_filepath,
        "-d", dist_filepath,
        "--width", f"{width}",
        "--height", f"{height}",
        "--pixel_format", "420",
        "--bitdepth", "8",
        "--json",
        "--output", tmp_json_path,
    ]

    cmd2 = ["jq", "-r", ".pooled_metrics.vmaf.mean", tmp_json_path]

    try:
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        result = subprocess.run(cmd2, check=True, capture_output=True, text=True)
        vmaf_score = float(result.stdout.strip())
        return vmaf_score
    except Exception as e:
        print(f"[calculate_vmaf] Error calculating VMAF: {e}")
        return None
    finally:
        if os.path.exists(tmp_json_path):
            os.remove(tmp_json_path)

def calculate_vmaf_for_all_frames(capture_rtp_ts_to_frame):
    for frame in capture_rtp_ts_to_frame.values():
        if frame.status != "decoded":
            continue
        if frame.ref_filepath and frame.recv_filepath and frame.width and frame.height:
            frame.vmaf = calculate_vmaf(frame.ref_filepath, frame.recv_filepath, frame.width, frame.height)
            print(f"Calculated VMAF for tracking_id={frame.tracking_id}: {frame.vmaf}")


def concat_yuv_files(file_list, out_path, buf_size=8 * 1024 * 1024):
    """把多个单帧 yuv 文件顺序拼成一个 yuv（流式拷贝更省内存）"""
    with open(out_path, "wb") as out_f:
        for path in file_list:
            with open(path, "rb") as in_f:
                shutil.copyfileobj(in_f, out_f, length=buf_size)


import os, json, tempfile, subprocess
from concurrent.futures import ProcessPoolExecutor, as_completed

VMAF_MODEL = "name=vmaf:path=/home/zh/workspace/vmaf/model/vmaf_v0.6.1.json:motion.motion_force_zero=true"

def _run_vmaf_one_batch(ref_files, dist_files, width, height):
    """子进程执行：拼 yuv + 跑 vmaf，返回 vmaf list（与输入帧一一对应）"""
    with tempfile.TemporaryDirectory() as tmpdir:
        ref_yuv = os.path.join(tmpdir, "ref.yuv")
        dist_yuv = os.path.join(tmpdir, "dist.yuv")
        out_json = os.path.join(tmpdir, "vmaf.json")

        concat_yuv_files(ref_files, ref_yuv)
        concat_yuv_files(dist_files, dist_yuv)

        cmd = [
            "vmaf",
            "-r", ref_yuv,
            "-d", dist_yuv,
            "--width", str(width),
            "--height", str(height),
            "--pixel_format", "420",
            "--bitdepth", "8",
            "--model", VMAF_MODEL,
            "--json",
            "--output", out_json,
        ]

        subprocess.run(
            cmd,
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
        )

        with open(out_json, "r", encoding="utf-8") as f:
            data = json.load(f)

        vmaf_frames = data["frames"]
        return [entry["metrics"]["vmaf"] for entry in vmaf_frames]

def calculate_vmaf_batch_parallel(
    frames,
    batch_size: int = 50,
    workers: int | None = None,
    prefer_ordered_log: bool = True,
):
    """
    并行计算 VMAF：每个 batch 一个进程跑 vmaf
    - workers=None: 默认用 os.cpu_count()//2（更稳，不把机器打满）
    """

    valid_frames = [
        f for f in frames
        if f.status == "decoded" and f.ref_filepath and f.recv_filepath
    ]
    if not valid_frames:
        print("[calculate_vmaf_batch_parallel] No valid decoded frames.")
        return

    width = valid_frames[0].width
    height = valid_frames[0].height

    # 切 batch
    batches = []
    for i in range(0, len(valid_frames), batch_size):
        batch = valid_frames[i:i + batch_size]
        ref_files = [f.ref_filepath for f in batch]
        dist_files = [f.recv_filepath for f in batch]
        batches.append((i, batch, ref_files, dist_files))

    if workers is None:
        cpu = os.cpu_count() or 4
        workers = max(1, cpu // 2)

    # 提交任务
    futures = {}
    with ProcessPoolExecutor(max_workers=workers) as ex:
        for start_i, batch, ref_files, dist_files in batches:
            fut = ex.submit(_run_vmaf_one_batch, ref_files, dist_files, width, height)
            futures[fut] = (start_i, batch)

        # 回收结果（可乱序完成，但写回 frame 没问题）
        done_msgs = []
        for fut in as_completed(futures):
            start_i, batch = futures[fut]
            vmafs = fut.result()

            # 回填
            for frame, vmaf_val in zip(batch, vmafs):
                frame.vmaf = vmaf_val

            msg = f"[calculate_vmaf_batch_parallel] Processed frames {batch[0].tracking_id}–{batch[-1].tracking_id}"
            if prefer_ordered_log:
                done_msgs.append((start_i, msg))
            else:
                print(msg)

        if prefer_ordered_log:
            for _, msg in sorted(done_msgs, key=lambda x: x[0]):
                print(msg)

# =========================
# Save
# =========================

def save_result_to_json(capture_rtp_ts_to_frame, result_save_path: str):
    out_data = {"frames": []}

    for frame in capture_rtp_ts_to_frame.values():
        if frame.status == "decoded" and (frame.ref_filepath is None or frame.recv_filepath is None):
            continue  # skip frames without saved

        frame_dict = frame.__dict__.copy()
        frame_dict["packet_infos"] = [
            packet.__dict__ for packet in frame.packet_infos
        ]
        out_data["frames"].append(frame_dict)

    with open(result_save_path, "w", encoding="utf-8") as f:
        json.dump(out_data, f, indent=4, ensure_ascii=False)

    print(f"[save_result_to_json] Result saved to: {result_save_path}")


# =========================
# Load
# =========================

def read_result_from_json(result_path: str) -> List[FrameInfo]:
    with open(result_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    frames: List[FrameInfo] = []

    for frame_dict in data.get("frames", []):
        packet_dicts = frame_dict.pop("packet_infos", [])
        packet_infos = [
            PacketInfo(**packet_dict)
            for packet_dict in packet_dicts
        ]

        frame = FrameInfo(
            **frame_dict,
            packet_infos=packet_infos,
        )
        frames.append(frame)

    return frames

# additional function to print results in txt format for easier reading
def print_result_txt(capture_rtp_ts_to_frame, result_save_path=None):
    out = open(result_save_path, "w", encoding="utf-8") if result_save_path else sys.stdout

    def p(*args, **kwargs):
        print(*args, file=out, **kwargs)

    for frame in capture_rtp_ts_to_frame.values():
        p("--------------------------------")
        p(f"Frame tracking_id = {frame.tracking_id} rtp_ts_capture = {frame.rtp_ts_capture}")

        if frame.status == "dropped_before_encode":
            p("dropped before encode, reason =", frame.dropped_reason)
            continue
        elif frame.status == "dropped_by_encoder":
            p("dropped by encoder")
            continue

        p("rtp_ts =", frame.rtp_ts)
        p("frame_type =", frame.frame_type)
        p("frame_size =", frame.frame_size, "bytes")
        p("qp =", frame.qp)
        p("num_media_packets =", frame.num_media_packets)
        p("num_fec_packets =", frame.num_fec_packets)

        p("Frame Dependencies:")
        p(f"    picture_id: {frame.picture_id}")
        p(f"    refs: {frame.refs}")
        p(f"    ref_distance: {frame.ref_distance}")

        p("Frame Metrics:")
        p(f"    status={frame.status}")
        if frame.encoding_delay:
            p(f"    encoding_delay={frame.encoding_delay/1000} ms")
        if frame.pacing_delay:
            p(f"    pacing_delay={frame.pacing_delay/1000} ms")
        if frame.frame_network_delay:
            p(f"    frame_network_delay={frame.frame_network_delay/1000} ms")
        if frame.decode_scheduling_delay:
            p(f"    decode_scheduling_delay={frame.decode_scheduling_delay/1000} ms")
        if frame.decoding_delay:
            p(f"    decoding_delay={frame.decoding_delay/1000} ms")
        if frame.e2e_delay:
            p(f"    e2e_delay={frame.e2e_delay/1000} ms")
        if frame.psnr is not None:
            p(f"    psnr={frame.psnr} dB")
        if frame.ssim is not None:
            p(f"    ssim={frame.ssim}")

        p("Packets:")
        for packet in frame.packet_infos:
            p(f"    type={packet.packet_type}, seq={packet.seq}")
            # 注意：end="" → 同样保持行为
            line = f"    sent_time_after_encoded={(packet.ts_sent - frame.ts_encoded)/1000} ms"
            if packet.ts_received is not None:
                line += (
                    f", received_time_after_encoded="
                    f"{(packet.ts_received - frame.ts_encoded)/1000} ms, "
                    f"recovered={int(packet.is_recovered)}"
                )
                p(line)

                if packet.packet_type == "fec":
                    protected_seqs = packet.extra_info.get("protected_seqs", [])
                    p(f"        protected_seqs={protected_seqs}")
                elif packet.packet_type == "rtx":
                    original_seq = packet.extra_info.get("original_seq", None)
                    p(f"        original_seq={original_seq}")
            else:
                p(line + ", NOT received")


    if result_save_path:
        out.close()
        print(f"[print_result] Saved result to: {result_save_path}")

if __name__ == "__main__":
    assert 2 <= len(argv) <= 3, "Usage: python analysis.py <expr_path> [time_diff_ms]"
    expr_path = argv[1]
    if expr_path.endswith("/"):
        expr_path = expr_path[:-1]

    time_diff_ms = int(argv[2]) if len(argv) > 2 else 0
    path_sender = expr_path + "/trace_sender.json"
    path_receiver = expr_path + "/trace_receiver.json"
    fig_save_path = expr_path
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
    save_result_to_json(capture_rtp_ts_to_frame, result_save_path=f"{fig_save_path}/result.json")
    print_result_txt(capture_rtp_ts_to_frame, result_save_path=f"{fig_save_path}/result.txt")
