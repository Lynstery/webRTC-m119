import json
from sys import argv
import numpy as np
import pandas as pd
from dataclasses import dataclass, field
from typing import List, Dict, Optional

def read_and_parse_trace(path_trace_sender, path_trace_receiver):
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



def extract_frames_packets(df: pd.DataFrame) -> Dict[int, FrameInfo]:
    capture_rtp_ts_to_frame: Dict[int, FrameInfo] = {} # rtp_ts_capture -> rtp_ts
    rtp_ts_to_frame: Dict[int, FrameInfo] = {} # rtp_ts -> frame_info

    all_capture_ts = df[df["name"] == "Frame:Captured"]["args.rtp_ts_capture"].unique()
    all_capture_ts.sort()

    evts_frame_captured = df[df["name"] == "Frame:Captured"]
    evts_frame_dropped = df[df["name"] == "Frame:Dropped"]
    evts_frame_start_encode = df[df["name"] == "Frame:Start Encode"]
    evts_frame_encoded = df[df["name"] == "Frame:Encoded"]
    evts_frame_packetization = df[df["name"] == "Frame:Packetization"]
    evts_frame_generate_fec = df[df["name"] == "Frame:Generate FEC"]
    
    for rtp_ts_capture in all_capture_ts:
        frame = FrameInfo(rtp_ts_capture=rtp_ts_capture)

        # --- Capture ---
        evt = evts_frame_captured[evts_frame_captured["args.rtp_ts_capture"] == rtp_ts_capture]
        frame.ts_captured = int(evt["ts"].values[0])
        frame.status = "captured"

        # --- Dropped before encoding ---
        evt_drop = evts_frame_dropped[evts_frame_dropped["args.rtp_ts_capture"] == rtp_ts_capture]
        if not evt_drop.empty:
            frame.status = "dropped_before_encode"
            frame.dropped_reason = evt_drop["args.reason"].values[0]
            capture_rtp_ts_to_frame[rtp_ts_capture] = frame
            continue

        # --- Start Encode ---
        evt_start_enc = evts_frame_start_encode[evts_frame_start_encode["args.rtp_ts_capture"] == rtp_ts_capture]
        if not evt_start_enc.empty:
            frame.ts_start_encode = int(evt_start_enc["ts"].values[0])
        else:
            frame.status = "dropped_before_encode"
            capture_rtp_ts_to_frame[rtp_ts_capture] = frame
            continue

        # --- Encoded ---
        evt_encoded = evts_frame_encoded[evts_frame_encoded["args.rtp_ts_capture"] == rtp_ts_capture]
        if evt_encoded.empty:
            frame.status = "dropped_by_encoder"
            capture_rtp_ts_to_frame[rtp_ts_capture] = frame
            continue
        
        frame.status = "encoded"
        frame.ts_encoded = int(evt_encoded["ts"].values[0])
        frame.rtp_ts = int(evt_encoded["args.rtp_ts"].values[0])
        frame.frame_type = evt_encoded["args.frame_type"].values[0]
        frame.frame_size = int(evt_encoded["args.frame_size"].values[0])

        frame.encoding_delay = frame.ts_encoded - frame.ts_captured
        
        event_frame_packetization = evts_frame_packetization[evts_frame_packetization["args.rtp_ts"] == frame.rtp_ts]
        packet_count = event_frame_packetization["args.packet_count"].values[0]
        event_frame_generate_fec = evts_frame_generate_fec[evts_frame_generate_fec["args.last_rtp_ts"] == frame.rtp_ts]
        num_fec_packets = event_frame_generate_fec["args.num_fec_packets"].values[0] if not event_frame_generate_fec.empty else 0
        
        frame.num_media_packets = packet_count
        frame.num_fec_packets = num_fec_packets

        capture_rtp_ts_to_frame[rtp_ts_capture] = frame
        rtp_ts_to_frame[frame.rtp_ts] = frame

    return capture_rtp_ts_to_frame, rtp_ts_to_frame

def extract_packets(df: pd.DataFrame, rtp_ts_to_frame: Dict[int, FrameInfo]):
    sent_events = df[df["name"] == "Packet:Sent"].sort_values(by="ts")
    evts_recv_media = df[df["name"] == "Packet:Receive Media RTP"]
    evts_recv_recover_media = df[df["name"] == "Packet:Receive Recovered Media RTP"]
    evts_recv_fec = df[df["name"] == "FlexFEC:Receive FEC Packet"]
    evts_recv_rtx = df[df["name"] == "Rtx:Receive RTX Packet"]

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
        
        # 查找接收事件
        if packet_type == "video":
            evt_recv = evts_recv_media[evts_recv_media["args.seq"] == packet.seq]
            evt_recover_recv = evts_recv_recover_media[evts_recv_recover_media["args.seq"] == packet.seq]
            if not evt_recv.empty:
                packet.ts_received = int(evt_recv["ts"].values[0])
                packet.is_recovered = False
            elif not evt_recover_recv.empty:
                packet.ts_received = int(evt_recover_recv["ts"].values[0]) # the received ts means the recovered time
                packet.is_recovered = True # recovered by FEC/RTX
            else:
                packet.ts_received = None
                packet.is_recovered = False
                
        elif packet_type == "fec":
            evt_recv = evts_recv_fec[evts_recv_fec["args.fec_seq"] == packet.seq]
            if not evt_recv.empty:
                packet.ts_received = int(evt_recv["ts"].values[0])
                packet.is_recovered = False
                packet.extra_info["protected_seqs"] = evt_recv["args.protected_seqs"].values[0]
            else:
                packet.ts_received = None
                packet.is_recovered = False
        else: # packet_type == "rtx"
            evt_recv = evts_recv_rtx[evts_recv_rtx["args.rtx_seq"] == packet.seq]
            if not evt_recv.empty:
                packet.ts_received = int(evt_recv["ts"].values[0])
                packet.is_recovered = False
                packet.extra_info["original_seq"] = evt_recv["args.seq"].values[0]
            else:
                packet.ts_received = None
                packet.is_recovered = False

def extract_frame_receiving(rtp_ts_to_frame: Dict[int, FrameInfo], df: pd.DataFrame):
    evts_frame_received_encoded = df[df["name"] == "Frame:Received EncodedFrame"]
    evts_frame_start_decode = df[df["name"] == "Frame:Start Decode"]
    evts_frame_decoded = df[df["name"] == "Frame:Decoded"]
    
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
        
        evt_received_encoded = evts_frame_received_encoded[evts_frame_received_encoded["args.rtp_ts"] == frame.rtp_ts]
        if evt_received_encoded.empty:
            continue
        frame.status = "received"
        frame.ts_received_encoded = int(evt_received_encoded["ts"].values[0])
        frame.picture_id = int(evt_received_encoded["args.picture_id"].values[0])
        frame.refs = evt_received_encoded["args.refs"].values[0]
        
        frame.frame_network_delay = frame.ts_received_encoded - ts_last_media_packet_sent
        
        evt_start_decode = evts_frame_start_decode[evts_frame_start_decode["args.rtp_ts"] == frame.rtp_ts]
        evt_decoded = evts_frame_decoded[evts_frame_decoded["args.rtp_ts"] == frame.rtp_ts]
        if evt_decoded.empty:
            continue
        frame.status = "decoded"
        frame.ts_start_decode = int(evt_start_decode["ts"].values[0])
        frame.ts_decoded = int(evt_decoded["ts"].values[0])
        frame.jitter_delay = frame.ts_start_decode - frame.ts_received_encoded
        frame.decoding_delay = frame.ts_decoded - frame.ts_start_decode
        frame.e2e_delay = frame.ts_decoded - frame.ts_captured
        

if __name__ == "__main__":
    path_sender = argv[1]
    path_receiver = argv[2]
    df = read_and_parse_trace(path_sender, path_receiver)
    
    capture_rtp_ts_to_frame, rtp_ts_to_frame = extract_frames_packets(df)
    print("Total captured frames:", len(capture_rtp_ts_to_frame))
    extract_packets(df, rtp_ts_to_frame)
    print("Total encoded frames:", len(rtp_ts_to_frame))
    extract_frame_receiving(rtp_ts_to_frame, df) 
    print("Total decoded frames:", sum(1 for f in rtp_ts_to_frame.values() if f.status in ["decoded"]))
    
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