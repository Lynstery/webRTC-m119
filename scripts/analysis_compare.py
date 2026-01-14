from os import makedirs
import os
import sys
import json
from sys import argv
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from dataclasses import dataclass, field
from typing import List, Dict, Optional
from matplotlib.ticker import MultipleLocator
from parse import FrameInfo, PacketInfo, read_result_from_json

deadline_ms = 100  # end-to-end deadline is 100 ms 
fps = 30  # 30 FPS
target_interval_ms = 1000.0 / fps  # 33.33 ms
stall_perception_ms = 80  # > 100 ms 才算 stall

name_list = []
def trace_label(i):
    return name_list[i]

TRACE_COLORS = [
    "#4C72B0",  # blue
    "#DD8452",  # orange
    "#55A868",  # green
    "#C44E52",  # red
    "#8172B2",  # purple
    "#937860",  # brown
    "#DA8BC3",  # pink
    "#8C8C8C",  # gray
]

def calculate_and_save_metrics_comp(expr_path_list, frames_info_list, save_path="metrics.json"):
    """
    对多个 trace 计算 QoE / QoS 指标，并统一保存到一个 JSON 文件
    """

    def safe_list(vals):
        return [v for v in vals if v is not None]

    def calc_single_trace_metrics(frames_info):
        metrics = {}

        # ---------------- basic frame stats ----------------
        num_captured_frames = len(frames_info)
        num_encoded_frames = sum(
            1 for f in frames_info
            if f.status in ("encoded", "decoded", "received")
        )
        num_rendered_frames = sum(1 for f in frames_info if f.status == "decoded")
        num_dropped_frames = num_encoded_frames - num_rendered_frames

        num_deadline_missed_frames_received = sum(
            1 for f in frames_info
            if f.e2e_delay is not None and (f.e2e_delay / 1000 > deadline_ms)
        )
        num_deadline_missed_frames_total = (
            num_deadline_missed_frames_received + num_dropped_frames
        )

        frame_sizes = safe_list([f.frame_size for f in frames_info])
        qps = safe_list([f.qp for f in frames_info])

        metrics["frame_stats"] = {
            "sent_frames_num": num_encoded_frames,
            "rendered_frames_num": num_rendered_frames,
            "key_frames_num": sum(1 for f in frames_info if f.frame_type == "key"),
            "drop_rate": num_dropped_frames / num_encoded_frames
            if num_encoded_frames > 0 else None,
            "media_bitrate_kbps":
                float(np.mean(frame_sizes)) * 8 * fps / 1000
                if frame_sizes else None,
            "frame_size_bytes": {
                "avg": float(np.mean(frame_sizes)) if frame_sizes else None,
                "min": float(np.min(frame_sizes)) if frame_sizes else None,
                "max": float(np.max(frame_sizes)) if frame_sizes else None,
            },
            "avg_qp": float(np.mean(qps)) if qps else None,
        }

        # ---------------- e2e delay ----------------
        e2e_delays_ms = safe_list([
            f.e2e_delay / 1000
            for f in frames_info
            if f.e2e_delay is not None
        ])

        if e2e_delays_ms:
            arr = np.array(e2e_delays_ms)
            metrics["e2e_delay_ms"] = {
                "avg": float(np.mean(arr)),
                "p50": float(np.percentile(arr, 50)),
                "p90": float(np.percentile(arr, 90)),
                "p95": float(np.percentile(arr, 95)),
                "p99": float(np.percentile(arr, 99)),
                "max": float(np.max(arr)),
                "deadline_ms": deadline_ms,
                "deadline_miss_rate":
                    num_deadline_missed_frames_total / num_encoded_frames
                    if num_encoded_frames > 0 else None,
            }
        else:
            metrics["e2e_delay_ms"] = None

        # ---------------- stall ----------------
        render_ts_ms = [
            f.ts_decoded / 1000
            for f in frames_info
            if f.ts_decoded is not None
        ]

        stalls = []
        for i in range(1, len(render_ts_ms)):
            delta = render_ts_ms[i] - render_ts_ms[i - 1]
            stall_duration = max(0.0, delta - target_interval_ms)
            if stall_duration > stall_perception_ms:
                stalls.append(stall_duration)

        if stalls:
            arr = np.array(stalls)
            metrics["stall_ms"] = {
                "stall_perception_ms": stall_perception_ms,
                "avg": float(np.mean(arr)),
                "p50": float(np.percentile(arr, 50)),
                "p90": float(np.percentile(arr, 90)),
                "p95": float(np.percentile(arr, 95)),
                "p99": float(np.percentile(arr, 99)),
                "max": float(np.max(arr)),
                "stall_ratio":
                    float(np.sum(arr) / (num_encoded_frames * target_interval_ms))
                    if num_encoded_frames > 0 else None,
            }
        else:
            metrics["stall_ms"] = None

        # ---------------- quality ----------------
        def quality_block(vals, extra=None):
            if not vals:
                return None
            arr = np.array(vals)
            block = {
                "avg": float(np.mean(arr)),
                "p10": float(np.percentile(arr, 10)),
                "p50": float(np.percentile(arr, 50)),
                "min": float(np.min(arr)),
            }
            if extra:
                block.update(extra)
            return block

        metrics["psnr"] = quality_block(safe_list([f.psnr for f in frames_info]))
        metrics["ssim"] = quality_block(safe_list([f.ssim for f in frames_info]))
        metrics["vmaf"] = quality_block(
            safe_list([f.vmaf for f in frames_info]),
            extra={
                "low_quality_threshold": 70,
                "low_quality_ratio":
                    float(np.mean(np.array([f.vmaf for f in frames_info if f.vmaf is not None]) < 70))
                    if any(f.vmaf is not None for f in frames_info) else None,
            }
        )

        return metrics

    # ==================== main ====================
    all_metrics = {
        "meta": {
            "fps": fps,
            "deadline_ms": deadline_ms,
            "stall_perception_ms": stall_perception_ms,
            "target_interval_ms": target_interval_ms,
        },
        "traces": {}
    }

    for i, frames_info in enumerate(frames_info_list):
        all_metrics["traces"][f"trace{i}"] = calc_single_trace_metrics(frames_info)
        all_metrics["traces"][f"trace{i}"]["expr_path"] = expr_path_list[i]

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    with open(save_path, "w", encoding="utf-8") as f:
        json.dump(all_metrics, f, indent=4)

    print(f"[calculate_and_save_metrics_comp] Metrics saved → {save_path}")

def draw_stall_cdf_comp(
    frames_infos_list,
    pdf_path="stall_cdf_comp.pdf",
    start_frame=50,
    max_frames=1000,
    tail_start=0.9,
):
    plt.figure(figsize=(7, 5))

    for i, frames_info in enumerate(frames_infos_list):
        frames = frames_info[start_frame:]
        if max_frames is not None:
            frames = frames[:max_frames]

        render_ts_ms = [
            frame.ts_decoded / 1000.0
            for frame in frames
            if getattr(frame, "ts_decoded", None) is not None
        ]

        if len(render_ts_ms) < 2:
            continue

        stalls = np.diff(render_ts_ms)
        stalls = np.maximum(0.0, stalls - target_interval_ms)
        stalls_sorted = np.sort(stalls)
        cdf = np.arange(1, len(stalls_sorted) + 1) / len(stalls_sorted)

        mask = cdf >= tail_start
        if not np.any(mask):
            continue

        plt.plot(
            stalls_sorted[mask],
            cdf[mask],
            linewidth=2.0,
            color=TRACE_COLORS[i % len(TRACE_COLORS)],
            label=trace_label(i),
        )
    plt.xlim(left=0)
    plt.ylim(tail_start, 1.0)
    plt.xlabel("Stall Duration (ms)")
    plt.ylabel("CDF")
    plt.title(f"Stall Duration CDF (Tail ≥ {int(tail_start*100)}%)")
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.legend()
    plt.tight_layout()
    plt.savefig(pdf_path)
    plt.close()
    print(f"[draw_stall_cdf_comp] PDF saved → {pdf_path}")

def draw_psnr_violin_comp(
    frames_infos_list,
    pdf_path="psnr_violin_comp.pdf",
    start_frame=50,
    max_frames=1000,
):
    data = []
    for frames_info in frames_infos_list:
        frames = frames_info[start_frame:]
        if max_frames is not None:
            frames = frames[:max_frames]
        psnrs = [f.psnr for f in frames if getattr(f, "psnr", None) is not None]
        data.append(psnrs)

    plt.figure(figsize=(7, 6))
    parts = plt.violinplot(
        data,
        showmeans=False,
        showmedians=True,
        showextrema=True,
    )

    for i, pc in enumerate(parts["bodies"]):
        pc.set_facecolor(TRACE_COLORS[i % len(TRACE_COLORS)])
        pc.set_alpha(0.7)

    plt.xticks(
        range(1, len(data) + 1),
        [trace_label(i) for i in range(len(data))]
    )
    plt.ylabel("PSNR (dB)")
    plt.title("PSNR Distribution")
    plt.grid(True, axis="y", linestyle="--", alpha=0.4)
    plt.tight_layout()
    plt.savefig(pdf_path)
    plt.close()
    print(f"[draw_frame_psnr] PDF saved → {pdf_path}")

def draw_frame_vmaf(frames_info, pdf_path="frame_vmaf.pdf",
                    start_frame=50, max_frames=1000):
    """
    绘制每帧 VMAF 折线图（frame.vmaf）
    """

    frames = list(frames_info)
    frames = frames[start_frame:]

    # 限制最大帧数
    if max_frames is not None and len(frames) > max_frames:
        frames = frames[:max_frames]

    frame_indices = []
    frame_vmaf = []

    # 收集 VMAF 数据
    for frame in frames:
        idx = frame.tracking_id

        vmaf = getattr(frame, "vmaf", None)
        if vmaf is None:
            continue  # 跳过没有 VMAF 的帧

        frame_indices.append(idx)
        frame_vmaf.append(vmaf)

    if not frame_indices:
        print("[draw_frame_vmaf] No VMAF data found.")
        return

    # —— 绘图 ——
    width = max(14, len(frame_indices) / 30)
    plt.figure(figsize=(width, 6))

    # 折线
    plt.plot(frame_indices, frame_vmaf,
             color="green", linewidth=1.2, label="VMAF")

    # 散点
    plt.scatter(frame_indices, frame_vmaf,
                s=10, color="black")

    avg_vmaf = np.mean(frame_vmaf)
    plt.axhline(avg_vmaf, linestyle="--", color="gray",
                linewidth=1.5, label=f"Avg = {avg_vmaf:.2f}")

    plt.text(
        frame_indices[-1],
        avg_vmaf,
        f"Avg = {avg_vmaf:.2f}",
        va="bottom",
        ha="right",
        fontsize=10,
        color="gray",
        backgroundcolor="white"
    )

    # x 轴更密集
    plt.gca().xaxis.set_major_locator(plt.MultipleLocator(20))  # 每 20 帧一个 tick

    plt.xlabel("Frame Index (tracking_id)")
    plt.ylabel("VMAF")
    plt.title("Per-frame VMAF over Time")
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.legend()
    plt.tight_layout()

    plt.savefig(pdf_path)
    plt.close()

    print(f"[draw_frame_vmaf] PDF saved → {pdf_path}")
    
def draw_frame_size_qp_psnr_vmaf_comp(
    frames_infos_list,
    pdf_path,
    start_frame=50,
    max_frames=1000,
):
    fig, axes = plt.subplots(4, 1, figsize=(18, 12), sharex=True)
    ax_size, ax_qp, ax_psnr, ax_vmaf = axes

    for i, frames_info in enumerate(frames_infos_list):
        frames = frames_info[start_frame:]
        if max_frames is not None:
            frames = frames[:max_frames]

        idx, size, qp, psnr, vmaf = [], [], [], [], []
        for f in frames:
            if None in (
                getattr(f, "frame_size", None),
                getattr(f, "qp", None),
                getattr(f, "psnr", None),
                getattr(f, "vmaf", None),
            ):
                continue
            idx.append(f.tracking_id)
            size.append(f.frame_size)
            qp.append(f.qp)
            psnr.append(f.psnr)
            vmaf.append(f.vmaf)

        color = TRACE_COLORS[i % len(TRACE_COLORS)]
        label = trace_label(i)

        ax_size.plot(idx, size, color=color, label=label)
        ax_qp.plot(idx, qp, color=color, label=label)
        ax_psnr.plot(idx, psnr, color=color, label=label)
        ax_vmaf.plot(idx, vmaf, color=color, label=label)

    ax_size.set_ylabel("Frame Size (B)")
    ax_qp.set_ylabel("QP")
    ax_psnr.set_ylabel("PSNR (dB)")
    ax_vmaf.set_ylabel("VMAF")
    ax_vmaf.set_xlabel("Frame Index")

    for ax in axes:
        ax.grid(True, linestyle="--", alpha=0.3)
        ax.legend()

    plt.tight_layout()
    plt.savefig(pdf_path)
    plt.close()
    print(f"[draw_frame_size_qp_psnr_vmaf_comp] PDF saved → {pdf_path}")

def draw_vmaf_violin_comp(
    frames_infos_list,
    pdf_path="vmaf_violin_comp.pdf",
    start_frame=50,
    max_frames=1000,
):
    """
    绘制多条 trace 的 VMAF 分布（violin plot）
    """

    data = []

    for frames_info in frames_infos_list:
        frames = list(frames_info)[start_frame:]
        if max_frames is not None:
            frames = frames[:max_frames]

        vmaf_values = [
            frame.vmaf
            for frame in frames
            if getattr(frame, "vmaf", None) is not None
        ]
        data.append(vmaf_values)

    if not any(len(d) > 0 for d in data):
        print("[draw_vmaf_violin_comp] No VMAF data found.")
        return

    plt.figure(figsize=(7, 6))

    parts = plt.violinplot(
        data,
        showmeans=False,
        showmedians=True,
        showextrema=True,
    )

    for i, pc in enumerate(parts["bodies"]):
        pc.set_facecolor(TRACE_COLORS[i % len(TRACE_COLORS)])
        pc.set_edgecolor("black")
        pc.set_alpha(0.7)

    plt.xticks(
        range(1, len(data) + 1),
        [trace_label(i) for i in range(len(data))],
    )

    plt.ylabel("VMAF")
    plt.title("VMAF Distribution")
    plt.grid(True, axis="y", linestyle="--", alpha=0.4)
    plt.tight_layout()
    plt.savefig(pdf_path)
    plt.close()

    print(f"[draw_vmaf_violin_comp] PDF saved → {pdf_path}")

def draw_frame_size_violin_comp(
    frames_infos_list,
    pdf_path="frame_size_violin_comp.pdf",
    start_frame=50,
    max_frames=1000,
):
    """
    绘制多条 trace 的 frame_size 分布（violin plot）
    """

    data = []

    for frames_info in frames_infos_list:
        frames = list(frames_info)[start_frame:]
        if max_frames is not None:
            frames = frames[:max_frames]

        frame_sizes = [
            frame.frame_size
            for frame in frames
            if getattr(frame, "frame_size", None) is not None
        ]
        data.append(frame_sizes)

    if not any(len(d) > 0 for d in data):
        print("[draw_frame_size_violin_comp] No frame_size data found.")
        return

    plt.figure(figsize=(7, 6))

    parts = plt.violinplot(
        data,
        showmeans=False,
        showmedians=True,
        showextrema=True,
    )

    # 颜色：按 trace
    for i, pc in enumerate(parts["bodies"]):
        pc.set_facecolor(TRACE_COLORS[i % len(TRACE_COLORS)])
        pc.set_edgecolor("black")
        pc.set_alpha(0.7)

    plt.xticks(
        range(1, len(data) + 1),
        [trace_label(i) for i in range(len(data))],
    )

    plt.ylabel("Frame Size (bytes)")
    plt.title("Frame Size Distribution")
    plt.grid(True, axis="y", linestyle="--", alpha=0.4)
    plt.tight_layout()
    plt.savefig(pdf_path)
    plt.close()

    print(f"[draw_frame_size_violin_comp] PDF saved → {pdf_path}")

def draw_e2e_delay_cdf_comp(
    frames_infos_list,
    pdf_path,
    start_frame=50,
    max_frames=1000,
):
    plt.figure(figsize=(8, 6))

    for i, frames_info in enumerate(frames_infos_list):
        frames = frames_info[start_frame:]
        if max_frames is not None:
            frames = frames[:max_frames]

        delays = [
            f.e2e_delay / 1000
            for f in frames
            if getattr(f, "e2e_delay", None) is not None
        ]
        if not delays:
            continue

        delays = np.sort(delays)
        cdf = np.linspace(0, 1, len(delays))
        plt.plot(delays, cdf,
                 color=TRACE_COLORS[i % len(TRACE_COLORS)],
                 label=trace_label(i))

    plt.xlabel("e2e Delay (ms)")
    plt.ylabel("CDF")
    plt.title("CDF of e2e Delay")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(pdf_path)
    plt.close()
    print(f"[draw_e2e_delay_cdf_comp] PDF saved → {pdf_path}")


if __name__ == "__main__":
    print(f"len(argv) = {len(argv)}, argv = {argv}")
    assert (4 <= len(argv) and (len(argv) % 2 == 0)), "Usage: python analysis_and_draw_comp.py <save_path> <expr1_path> <name1> <expr2_path> <name2> ..."
    save_path = argv[1]
    if save_path.endswith("/"):
        save_path = save_path[:-1]
        
    metrics_save_path = f"{save_path}/metrics.json" 
    fig_save_path = f"{save_path}/figures" 
    makedirs(fig_save_path, exist_ok=True)
    
    argv_list = argv[2:]
    expr_paths_list = argv_list[::2]
    name_list = argv_list[1::2]
    
    for i in range(len(expr_paths_list)):
        expr_path = expr_paths_list[i]
        if expr_path.endswith("/"):
            expr_path = expr_path[:-1]
        expr_paths_list[i] = expr_path
    
    assert len(expr_paths_list) == len(name_list)
    for i in range(len(name_list)):
        print(f"Trace {i}: {name_list[i]} : {expr_paths_list[i]}")
            
    frame_infos_list = [read_result_from_json(f"{expr_path}/result.json") for expr_path in expr_paths_list]
    
    start_frame = 50
    max_frames = 1000
    # ====== draw ======
    calculate_and_save_metrics_comp(expr_paths_list, frame_infos_list, save_path=metrics_save_path)
    draw_e2e_delay_cdf_comp(frame_infos_list, pdf_path=f"{fig_save_path}/e2e_cdf_comp.pdf", start_frame=start_frame, max_frames=max_frames)
    draw_stall_cdf_comp(frame_infos_list, pdf_path=f"{fig_save_path}/stall_cdf_comp.pdf", start_frame=start_frame, max_frames=max_frames)
    draw_frame_size_qp_psnr_vmaf_comp(frame_infos_list, pdf_path=f"{fig_save_path}/frame_size_qp_psnr_vmaf_comp.pdf", start_frame=start_frame, max_frames=max_frames)
    
    draw_frame_size_violin_comp(frame_infos_list, pdf_path=f"{fig_save_path}/frame_size_distribution_comp.pdf", start_frame=start_frame, max_frames=max_frames)
    draw_psnr_violin_comp(frame_infos_list, pdf_path=f"{fig_save_path}/psnr_distribution_comp.pdf", start_frame=start_frame, max_frames=max_frames) 
    draw_vmaf_violin_comp(frame_infos_list, pdf_path=f"{fig_save_path}/vmaf_distribution_comp.pdf", start_frame=start_frame, max_frames=max_frames)