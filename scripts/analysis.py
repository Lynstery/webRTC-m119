import json
import pandas as pd
import numpy as np

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

def preprocess_events(events):
    tmp = pd.DataFrame(events)
    args_uint64 = tmp.dtypes[tmp.dtypes == "float64"]
    print(args_uint64.index.tolist())
    for e in events:
        for arg in args_uint64.index.tolist():
            if arg not in e:
                e[arg] = 0

path_sender = "../expr/trace_sender.json"
path_receiver = "../expr/trace_receiver.json"

# 读取原始数据
events_sender = load_events(path_sender)
events_receiver = load_events(path_receiver)

# 展开 + 加 from 字段
flat_sender = [flatten_event(e, "sender") for e in events_sender]
flat_receiver = [flatten_event(e, "receiver") for e in events_receiver]

# 合并两个列表
all_events = flat_sender + flat_receiver

preprocess_events(all_events)

df = pd.DataFrame(all_events)

print(df.dtypes)

print(df["name"].unique().tolist())

all_frame_rtp_ts_capture = df[df["name"] == "Frame:Captured"]["args.rtp_ts_capture"].unique()
all_frame_rtp_ts_capture.sort()

for rtp_ts_capture in all_frame_rtp_ts_capture:
    print("--------------------------------")
    print("Frame rtp_ts_capture =", rtp_ts_capture)
    event_frame_captured = df[(df["name"] == "Frame:Captured") & (df["args.rtp_ts_capture"] == rtp_ts_capture)]
    event_frame_start_encode = df[(df["name"] == "Frame:Start Encode") & (df["args.rtp_ts_capture"] == rtp_ts_capture)]
    
    if event_frame_start_encode.empty:
        event_frame_dropped = df[(df["name"] == "Frame:Dropped") & (df["args.rtp_ts_capture"] == rtp_ts_capture)]
        reason = event_frame_dropped["args.reason"].values[0] 
        print(f"    dropped before encoding, reason={reason}!")
        continue
    else:
        event_frame_encoded = df[(df["name"] == "Frame:Encoded") & (df["args.rtp_ts_capture"] == rtp_ts_capture)]
        
        if event_frame_encoded.empty:
            print(f"    dropped by encoder!")
            continue
        
        ts_captured = event_frame_captured["ts"].values[0]
        ts_encoded = event_frame_encoded["ts"].values[0]
        rtp_ts = event_frame_encoded["args.rtp_ts"].values[0]
        frame_type = event_frame_encoded["args.frame_type"].values[0]
        frame_size = event_frame_encoded["args.frame_size"].values[0]
        event_frame_packetization = df[(df["name"] == "Frame:Packetization") & (df["args.rtp_ts"] == rtp_ts)]
        packet_count = event_frame_packetization["args.packet_count"].values[0]
        event_frame_generate_fec = df[(df["name"] == "Frame:Generate FEC") & (df["args.last_rtp_ts"] == rtp_ts)]
        num_media_packets = event_frame_generate_fec["args.num_media_packets"].values[0] if not event_frame_generate_fec.empty else 0 
        num_fec_packets = event_frame_generate_fec["args.num_fec_packets"].values[0] if not event_frame_generate_fec.empty else 0

        events_frame_all_packet_sent = df[(df["name"] == "Packet:Sent") & ((df["args.rtp_ts"] == rtp_ts) | (df["args.protected_frame_rtp_ts"] == rtp_ts))]
        events_frame_media_packet_sent = events_frame_all_packet_sent[events_frame_all_packet_sent["args.packet_type"] == "video"]
        
        
        events_frame_all_packet_sent = events_frame_all_packet_sent.sort_values(by="ts")
        events_frame_media_packet_sent = events_frame_media_packet_sent.sort_values(by="ts") 
        
        ts_first_packet_sent = events_frame_all_packet_sent["ts"].values[0] if not events_frame_all_packet_sent.empty else None 
        ts_last_media_packet_sent = events_frame_media_packet_sent["ts"].values[-1] if not events_frame_media_packet_sent.empty else None
        
        encoding_delay = ts_encoded - ts_captured
        pacing_delay = ts_last_media_packet_sent - ts_encoded if ts_last_media_packet_sent is not None else 0

        print(f"rtp_ts={rtp_ts}, frame_type={frame_type}, frame_size={frame_size} bytes, packet_count={packet_count}")
        
        print("all related packets sent:") 
        for _, event_packet_sent in events_frame_all_packet_sent.iterrows():
            seq = event_packet_sent["args.seq"]
            packet_type = event_packet_sent["args.packet_type"]
            transport_seq = event_packet_sent["args.transport_seq"]
            ts_packet_sent = event_packet_sent["ts"]
            print(f"    Packet seq={seq}, packet_type={packet_type}, transport_seq={transport_seq}, delay_since_capture={(ts_packet_sent - ts_captured)/1000} ms")
            
            if packet_type == "video": 
                event_packet_received = df[(df["name"] == "Packet:Received RTP") & (df["args.seq"] == seq) & (df["from"] == "receiver")]
                if not event_packet_received.empty:
                    ts_received = event_packet_received["ts"].values[0]
                    is_recovered = event_packet_received["args.is_recovered"].values[0]
                    print(f"    received at receiver, on-way-delay={(ts_received - ts_packet_sent)/1000} ms, is_recovered={is_recovered}")
                else:
                    print(f"    NOT received at receiver")
            elif packet_type == "fec":
                event_fec_packet_received = df[(df["name"] == "FlexFEC:Receive FEC Packet") & (df["args.fec_seq"] == seq) & (df["from"] == "receiver")]
                if not event_fec_packet_received.empty:
                    ts_received = event_fec_packet_received["ts"].values[0]
                    protected_seqs = event_fec_packet_received["args.protected_seqs"].values[0]
                    print(f"    recovered at receiver, on-way-delay={(ts_received - ts_packet_sent)/1000} ms")
                else:
                    print(f"    NOT received at receiver")
            elif packet_type == "rtx":
                event_rtx_packet_received = df[(df["name"] == "RtxReceiveStream:OnRtpPacket") & (df["args.rtx_seq"] == seq) & (df["from"] == "receiver")]
                if not event_rtx_packet_received.empty:
                    ts_received = event_rtx_packet_received["ts"].values[0]
                    protected_seq = event_rtx_packet_received["args.seq"].values[0]
                    print(f"    received at receiver, on-way-delay={(ts_received - ts_packet_sent)/1000} ms, retrans seq={protected_seq}")
                else:
                    print(f"    NOT received at receiver")
             
            
        if ts_last_media_packet_sent is None:
            print(f"    Not start sending!")
            continue 
        
        print(f"num_media_packets={num_media_packets}, num_fec_packets={num_fec_packets}")
        print(f"encoding_delay = {encoding_delay/1000} ms, pacing delay = {pacing_delay/1000} ms")
         
        event_frame_received_encoded = df[(df["name"] == "Frame:Received EncodedFrame") & (df["args.rtp_ts"] == rtp_ts)]
        if event_frame_received_encoded.empty:
            print(f"    Not received at receiver!")
            continue
        
        picture_id = event_frame_received_encoded["args.picture_id"].values[0]
        refs = event_frame_received_encoded["args.refs"].values[0]
        ts_received_encoded = event_frame_received_encoded["ts"].values[0]
        
        print(f"received encoded frame")
        
        event_frame_ready_to_decode = df[(df["name"] == "Frame:ReadytoDecode") & (df["args.rtp_ts"] == rtp_ts)] 
        event_frame_start_decode = df[(df["name"] == "Frame:Start Decode") & (df["args.rtp_ts"] == rtp_ts)]
        event_frame_decoded = df[(df["name"] == "Frame:Decoded") & (df["args.rtp_ts"] == rtp_ts)]
        
        if event_frame_decoded.empty:
            print(f"    Not decoded at receiver!")
            continue 
         
        ts_ready_to_decode = event_frame_ready_to_decode["ts"].values[0]
        ts_start_decode = event_frame_start_decode["ts"].values[0]
        ts_decoded = event_frame_decoded["ts"].values[0]
        
        frame_network_delay = ts_received_encoded - ts_last_media_packet_sent
        jitter_delay = ts_start_decode - ts_received_encoded
        decoding_delay = ts_decoded - ts_start_decode
        e2e_delay = ts_decoded - ts_captured
        print(f"picture_id={picture_id}, refs={refs}")
        
        print(f"frame_network_delay={frame_network_delay/1000} ms")
        print(f"jitter_delay={jitter_delay/1000} ms")
        print(f"decoding_delay={decoding_delay/1000} ms")
        print(f"e2e_delay={e2e_delay/1000} ms")
