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

def calculate_and_save_metrics(frames_info, save_path="metrics.json"):
    """
    计算并保存关键 QoE / QoS 指标到 JSON
    """
    
    def safe_list(vals):
        return [v for v in vals if v is not None]

    metrics = {}

    # basic metrics    
    
    # dropped_before_encode / dropped_by_encoder / encoded / received / decoded
    num_captured_frames = len(frames_info)
    num_encoded_frames = sum(1 for f in frames_info if f.status == "encoded" or f.status == "decoded" or f.status == "received") 
    num_rendered_frames = sum(1 for f in frames_info if f.status == "decoded")
    num_dropped_frames = num_encoded_frames - num_rendered_frames
    num_deadline_missed_frames_received = sum(1 for f in frames_info if f.e2e_delay is not None and (f.e2e_delay / 1000 > deadline_ms))
    num_deadline_missed_frames_total = num_deadline_missed_frames_received + num_dropped_frames
    
    e2e_delays_ms = safe_list([
        f.e2e_delay / 1000
        for f in frames_info
        if f.e2e_delay is not None
    ])

    metrics["frame_stats"] = {
        "sent_frames_num": num_encoded_frames,
        "rendered_frames_num": num_rendered_frames,
        "key_frames_num": sum(1 for f in frames_info if f.frame_type == "key"),
        "drop_rate": num_dropped_frames / num_encoded_frames,
        "media_bitrate_kbps": float(np.mean([f.frame_size for f in frames_info if f.frame_size is not None])) * 8 * fps / 1000,
        "frame_size_bytes": {
            "avg": float(np.mean([f.frame_size for f in frames_info if f.frame_size is not None])),
            "min": float(np.min([f.frame_size for f in frames_info if f.frame_size is not None])),
            "max": float(np.max([f.frame_size for f in frames_info if f.frame_size is not None])),
        },
        "avg_qp": float(np.mean([f.qp for f in frames_info if f.qp is not None])),
    }

    # delay metrics    
    
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
            "deadline_miss_rate": num_deadline_missed_frames_total / num_encoded_frames,
        }
    else:
        metrics["e2e_delay_ms"] = None

    # stall metrics    
    
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

    stalls = np.array(stalls)

    if len(stalls) > 0:
        stall_positive = stalls[stalls > 0]
        metrics["stall_ms"] = {
            "stall_perception_ms": stall_perception_ms,
            "avg": float(np.mean(stalls)),
            "stall_ratio": float(np.sum(stalls) / num_encoded_frames * target_interval_ms),
            "p50": float(np.percentile(stalls, 50)),
            "p90": float(np.percentile(stalls, 90)),
            "p95": float(np.percentile(stalls, 95)),
            "p99": float(np.percentile(stalls, 99)),
            "max": float(np.max(stalls)),
        }
    else:
        metrics["stall_ms"] = None

    # quality metrics 
    psnr_vals = safe_list([f.psnr for f in frames_info])
    ssim_vals = safe_list([f.ssim for f in frames_info])
    vmaf_vals = safe_list([f.vmaf for f in frames_info])

    if psnr_vals:
        arr = np.array(psnr_vals)
        metrics["psnr"] = {
            "avg": float(np.mean(arr)),
            "p10": float(np.percentile(arr, 10)),
            "p50": float(np.percentile(arr, 50)),
            "min": float(np.min(arr)),
        }
    else:
        metrics["psnr"] = None

    if ssim_vals:
        arr = np.array(ssim_vals)
        metrics["ssim"] = {
            "avg": float(np.mean(arr)),
            "p10": float(np.percentile(arr, 10)),
            "min": float(np.min(arr)),
        }
    else:
        metrics["ssim"] = None

    if vmaf_vals:
        arr = np.array(vmaf_vals)
        metrics["vmaf"] = {
            "avg": float(np.mean(arr)),
            "p10": float(np.percentile(arr, 10)),
            "p50": float(np.percentile(arr, 50)),
            "min": float(np.min(arr)),
            "low_quality_threshold": 70,
            "low_quality_ratio": float(np.mean(arr < 70)),
        }
    else:
        metrics["vmaf"] = None

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    with open(save_path, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=4)

    print(f"[calculate_and_save_metrics] Metrics saved → {save_path}")

def draw_frame_sizes(frames_info, pdf_path="frame_sizes.pdf", start_frame=50, max_frames=1000):
    """
    绘制每帧 frame_size 折线图，并标出帧类型
    """
    frames = frames_info[start_frame:]

    # 只取前 max_frames 帧
    if max_frames is not None and len(frames) > max_frames:
        frames = frames[:max_frames]

    frame_indices = []
    frame_sizes = []
    frame_types = []

    # 收集数据
    for frame in frames:
        idx = frame.tracking_id
        # 跳过 dropped 帧：没有 frame_size
        if getattr(frame, "frame_size", None) is None:
            continue
        frame_indices.append(idx)
        frame_sizes.append(frame.frame_size)
        frame_types.append(frame.frame_type if frame.frame_type else "unknown")

    # —— 映射帧类型到颜色 ——
    color_map = {
        "key": "red",
        "delta": "blue",
    }
    point_colors = [color_map.get(ft, "black") for ft in frame_types]

    # —— 绘图 ——
    width = max(14, len(frame_indices) / 30)
    plt.figure(figsize=(width, 7))

    # 画折线（帧大小）
    plt.plot(frame_indices, frame_sizes, label="frame_size (bytes)")

    # 标记帧类型
    plt.scatter(frame_indices, frame_sizes, c=point_colors, s=12, label="Frame Types")
    
    # -------- 新增：平均值 --------
    avg_size = np.mean(frame_sizes)
    plt.axhline(avg_size, linestyle="--", color="gray",
                linewidth=1.5, label=f"Avg = {avg_size:.1f} B")

    # 平均值文字标注（右上角）
    plt.text(
        frame_indices[-1],
        avg_size,
        f"Avg = {avg_size:.1f} B",
        va="bottom",
        ha="right",
        fontsize=10,
        color="gray",
        backgroundcolor="white"
    )
    # -------------------------------- 
     
    # x 轴更密集：
    plt.gca().xaxis.set_major_locator(plt.MultipleLocator(20))  # 每 20 帧一个 tick
    # plt.tick_params(axis='x', labelsize=7)  # 缩小字号避免重叠

    # 添加图例说明（颜色 → 帧类型）
    for ft, color in color_map.items():
        plt.scatter([], [], c=color, label=f"{ft}-frame")

    plt.xlabel("Frame Index")
    plt.ylabel("Frame Size (bytes)")
    plt.title("Frame Size over Time with Frame Type Annotation")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(pdf_path)
    plt.close()

    print(f"[draw_frame_sizes] PDF saved → {pdf_path}")

def draw_stall_cdf(
    frames_info,
    pdf_path="stall_cdf.pdf",
    start_frame=50,
    max_frames=1000,
    tail_start=0.9,
):

    frames = frames_info[start_frame:]

    if max_frames is not None and len(frames) > max_frames:
        frames = frames[:max_frames]

    # ---------- 收集 render 时间 ----------
    render_ts_ms = []
    for frame in frames:
        ts_render = getattr(frame, "ts_decoded", None)
        if ts_render is not None:
            render_ts_ms.append(ts_render / 1000.0)

    if len(render_ts_ms) < 2:
        print("[draw_stall_cdf] Not enough rendered frames.")
        return

    # ---------- 计算 stall ----------
    stalls = []
    for i in range(1, len(render_ts_ms)):
        delta = render_ts_ms[i] - render_ts_ms[i - 1]
        stalls.append(delta)

    stalls = np.array(stalls)
    if len(stalls) == 0:
        print("[draw_stall_cdf] No stall samples.")
        return

    # ---------- 排序 & CDF ----------
    stalls_sorted = np.sort(stalls)
    cdf = np.arange(1, len(stalls_sorted) + 1) / len(stalls_sorted)

    # ---------- 分位点 ----------
    percentiles = {
        "p60": np.percentile(stalls_sorted, 60),
        "p80": np.percentile(stalls_sorted, 80),
        "p90": np.percentile(stalls_sorted, 90),
        "p95": np.percentile(stalls_sorted, 95),
        "p97": np.percentile(stalls_sorted, 97),
        "p99": np.percentile(stalls_sorted, 99),
    }

    # ---------- tail ----------
    mask = cdf >= tail_start
    stalls_tail = stalls_sorted[mask]
    cdf_tail = cdf[mask]

    if len(stalls_tail) == 0:
        print("[draw_stall_cdf] No tail samples.")
        return

    # ---------- 绘图 ----------
    plt.figure(figsize=(7, 5))

    plt.plot(
        stalls_tail,
        cdf_tail,
        linewidth=2.0,
        label="Stall CDF",
    )

    # 分位线
    for label, val in percentiles.items():
        if val >= stalls_tail[0]:
            plt.axvline(
                val,
                linestyle="--",
                linewidth=1.0,
                alpha=0.6,
                label=f"{label} = {val:.1f} ms",
            )
    if stall_perception_ms >= stalls_tail[0]:
        plt.axvline(
            stall_perception_ms,
            color="red",
            linestyle="-",
            linewidth=2.0,
            alpha=0.9,
            label=f"Perceptual Threshold = {stall_perception_ms:.0f} ms",
        )

    plt.ylim(tail_start, 1.0)

    plt.xlabel("Stall Duration (ms)")
    plt.ylabel("CDF")
    plt.title("Stall Duration CDF (Tail ≥ {:.0f}%)".format(tail_start * 100))

    plt.grid(True, linestyle="--", alpha=0.4)
    plt.legend(loc="lower right")
    plt.tight_layout()
    plt.savefig(pdf_path)
    plt.close()

    print(f"[draw_stall_cdf] Tail CDF PDF saved → {pdf_path}")
    
def draw_frame_rendered_timeline(
    frames_info,
    pdf_path="frame_rendered_timeline.pdf",
    start_frame=50,
    max_frames=1000,
):
    """
    绘制帧渲染时间轴：
      - x 轴：时间（ms）
      - 每个点表示一帧的渲染时间
      - 标注帧编号（tracking_id）
      - deadline miss 的帧用红色标出
    """

    frames = list(frames_info)
    frames = frames[start_frame:]

    if max_frames is not None and len(frames) > max_frames:
        frames = frames[:max_frames]

    frame_indices = []
    frame_ts_render_ms = []
    frame_deadline_met = []

    for frame in frames:
        idx = frame.tracking_id

        ts_render = getattr(frame, "ts_decoded", None)
        if ts_render is None:
            continue

        e2e_delay = getattr(frame, "e2e_delay", None)
        if e2e_delay is None:
            continue

        # us → ms
        frame_indices.append(idx)
        frame_ts_render_ms.append(ts_render / 1000.0)

        deadline_met = e2e_delay <= 100_000  # 100 ms
        frame_deadline_met.append(deadline_met)

    if not frame_ts_render_ms:
        print("[draw_frame_rendered_timeline] No rendered frame data.")
        return

    # ---------- 绘图 ----------
    width = max(60, len(frames) / 30)
    plt.figure(figsize=(width, 3))
    
    # 时间轴（y=0）
    y_base = 0
    plt.hlines(
        y=y_base,
        xmin=min(frame_ts_render_ms),
        xmax=max(frame_ts_render_ms),
        color="gray",        # 灰色
        linestyle="--",      # 虚线
        linewidth=0.2,       # 稍微细一点
        alpha=0.5,           # 略微透明
        zorder=1,
    )

    # 分颜色画点
    for idx, t, ok in zip(frame_indices, frame_ts_render_ms, frame_deadline_met):
        color = "green" if ok else "red"
        marker = "." if ok else "."

        plt.scatter(
            t,
            y_base,
            color=color,
            marker=marker,
            s=5,
            zorder=2,
        )

        # 标注帧编号（稍微往上偏移）
        plt.text(
            t,
            y_base + 0.01,
            str(idx),
            ha="center",
            va="bottom",
            fontsize=1,
            color=color,
        )
    
    # ---------- 图形修饰 ----------
    plt.xlabel("Render Time (ms)")
    plt.yticks([])  # 不显示 y 轴刻度
    plt.title("Frame Rendered Timeline (Deadline Miss Highlighted)")

    ax = plt.gca()

    # 每 100 ms 一个主刻度（只用于 grid，不显示数字）
    ax.xaxis.set_major_locator(MultipleLocator(100))

    # ★ 关键：关闭 x 轴刻度文字
    ax.tick_params(axis="x", which="both", labelbottom=False)

    # 仍然画网格线
    ax.grid(True, axis="x", which="major", linestyle="--", alpha=0.4)
    ax.grid(True, axis="x", which="minor", linestyle=":", alpha=0.2)

    # legend（手动）
    plt.scatter([], [], color="green", marker=".", label="Deadline met")
    plt.scatter([], [], color="red", marker=".", label="Deadline missed")
    plt.legend(loc="upper right")

    plt.tight_layout()
    plt.savefig(pdf_path)
    plt.close() 


    print(f"[draw_frame_rendered_timeline] PDF saved → {pdf_path}")
    
def draw_frame_qp(frames_info, pdf_path="frame_qp.pdf",
                  start_frame=50, max_frames=1000):
    """
    绘制每帧 QP 折线图（frame.qp）
    """

    frames = list(frames_info)
    frames = frames[start_frame:]

    # 限制最大帧数
    if max_frames is not None and len(frames) > max_frames:
        frames = frames[:max_frames]

    frame_indices = []
    frame_qps = []

    # 收集 QP 数据
    for frame in frames:
        idx = frame.tracking_id

        qp = getattr(frame, "qp", None)
        if qp is None:
            continue  # 跳过没有 QP 的帧

        frame_indices.append(idx)
        frame_qps.append(qp)

    if not frame_indices:
        print("[draw_frame_qp] No QP data found.")
        return

    # —— 绘图 ——
    width = max(14, len(frame_indices) / 30)
    plt.figure(figsize=(width, 6))

    # 折线
    plt.plot(frame_indices, frame_qps,
             color="purple", linewidth=1.2, label="QP")

    # 散点
    plt.scatter(frame_indices, frame_qps,
                s=10, color="black")

    # 平均 QP
    avg_qp = np.mean(frame_qps)
    plt.axhline(avg_qp, linestyle="--", color="gray",
                linewidth=1.5, label=f"Avg = {avg_qp:.2f}")

    plt.text(
        frame_indices[-1],
        avg_qp,
        f"Avg = {avg_qp:.2f}",
        va="bottom",
        ha="right",
        fontsize=10,
        color="gray",
        backgroundcolor="white"
    )

    # x 轴更密集
    plt.gca().xaxis.set_major_locator(plt.MultipleLocator(20))  # 每 20 帧一个 tick

    plt.xlabel("Frame Index (tracking_id)")
    plt.ylabel("QP")
    plt.title("Per-frame QP over Time")
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.legend()
    plt.tight_layout()

    plt.savefig(pdf_path)
    plt.close()

    print(f"[draw_frame_qp] PDF saved → {pdf_path}")

def draw_frame_psnr(frames_info, pdf_path="frame_psnr.pdf",
                    start_frame=50, max_frames=1000):
    """
    绘制每帧 PSNR 折线图（frame.psnr）
    """

    frames = list(frames_info)
    frames = frames[start_frame:]

    # 限制最大帧数
    if max_frames is not None and len(frames) > max_frames:
        frames = frames[:max_frames]

    frame_indices = []
    frame_psnr = []

    # 收集 PSNR 数据
    for frame in frames:
        idx = frame.tracking_id

        psnr = getattr(frame, "psnr", None)
        if psnr is None:
            continue  # 跳过没有 PSNR 的帧

        frame_indices.append(idx)
        frame_psnr.append(psnr)

    if not frame_indices:
        print("[draw_frame_psnr] No PSNR data found.")
        return

    # —— 绘图 ——
    width = max(14, len(frame_indices) / 30)
    plt.figure(figsize=(width, 6))

    # 折线
    plt.plot(frame_indices, frame_psnr, color="blue", linewidth=1.2, label="PSNR (dB)")

    # 散点
    plt.scatter(frame_indices, frame_psnr, s=10, color="red")

    avg_psnr = np.mean(frame_psnr)
    plt.axhline(avg_psnr, linestyle="--", color="gray",
                linewidth=1.5, label=f"Avg = {avg_psnr:.2f} dB")

    plt.text(
        frame_indices[-1],
        avg_psnr,
        f"Avg = {avg_psnr:.2f} dB",
        va="bottom",
        ha="right",
        fontsize=10,
        color="gray",
        backgroundcolor="white"
    )

    # x 轴更密集
    plt.gca().xaxis.set_major_locator(plt.MultipleLocator(20))  # 每 20 帧一个tick

    plt.xlabel("Frame Index (tracking_id)")
    plt.ylabel("PSNR (dB)")
    plt.title("Per-frame PSNR over Time")
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.legend()
    plt.tight_layout()

    plt.savefig(pdf_path)
    plt.close()

    print(f"[draw_frame_psnr] PDF saved → {pdf_path}")

def draw_psnr_violin(frames_info,
                     pdf_path="psnr_violin.pdf",
                     start_frame=50,
                     max_frames=1000):
    """
    绘制 PSNR 的大提琴分布图
    """

    frames = list(frames_info)
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
    
def draw_vmaf_violin(frames_info,
                     pdf_path="vmaf_violin.pdf",
                     start_frame=50,
                     max_frames=1000):
    """
    绘制 VMAF 的大提琴分布图
    """

    frames = list(frames_info)
    frames = frames[start_frame:]

    if max_frames is not None and len(frames) > max_frames:
        frames = frames[:max_frames]

    vmaf_values = []

    for frame in frames:
        vmaf = getattr(frame, "vmaf", None)
        if vmaf is not None:
            vmaf_values.append(vmaf)

    if not vmaf_values:
        print("[draw_vmaf_violin] No VMAF data found.")
        return

    avg_vmaf = np.mean(vmaf_values)

    plt.figure(figsize=(6, 6))

    # 大提琴图
    parts = plt.violinplot(
        vmaf_values,
        showmeans=False,
        showmedians=True,
        showextrema=True
    )

    # 美化颜色（与 PSNR 区分）
    for pc in parts["bodies"]:
        pc.set_facecolor("#55a868")   # 绿色系
        pc.set_edgecolor("black")
        pc.set_alpha(0.7)

    # 平均值线
    plt.axhline(avg_vmaf, color="red", linestyle="--", linewidth=1.5,
                label=f"Avg = {avg_vmaf:.2f}")

    plt.text(
        1.02, avg_vmaf,
        f"{avg_vmaf:.2f}",
        va="bottom",
        ha="left",
        color="red",
        fontsize=10
    )

    plt.ylabel("VMAF")
    plt.title("VMAF Distribution (Violin Plot)")
    plt.xticks([1], ["All Frames"])
    plt.grid(True, axis="y", linestyle="--", alpha=0.4)
    plt.legend()
    plt.tight_layout()

    plt.savefig(pdf_path)
    plt.close()

    print(f"[draw_vmaf_violin] PDF saved → {pdf_path}") 
    
def draw_frame_size_qp_psnr_vmaf_refdis(
    frames_info,
    pdf_path="frame_size_qp_psnr_vmaf_refdis.pdf",
    start_frame=50,
    max_frames=1000,
):
    """
    在同一个 PDF 中绘制 4 个对齐子图（共享 x 轴）：
      1) Frame Size
      2) QP
      3) PSNR
      4) VMAF
      5) Reference Distance
    """

    frames = list(frames_info)
    frames = frames[start_frame:]

    if max_frames is not None and len(frames) > max_frames:
        frames = frames[:max_frames]

    frame_indices = []
    frame_sizes = []
    frame_qps = []
    frame_psnrs = []
    frame_vmafs = []
    frame_types = []
    frame_refdis = []
    # -------- 收集数据（一次遍历，保证 x 对齐） --------
    for frame in frames:
        idx = frame.tracking_id

        if (getattr(frame, "frame_size", None) is None or
            getattr(frame, "qp", None) is None or
            getattr(frame, "psnr", None) is None) or getattr(frame, "vmaf", None) is None or getattr(frame, "ref_distance", None) is None:
            continue

        frame_indices.append(idx)
        frame_sizes.append(frame.frame_size)
        frame_qps.append(frame.qp)
        frame_psnrs.append(frame.psnr)
        frame_vmafs.append(frame.vmaf)
        frame_refdis.append(frame.ref_distance)
        frame_types.append(frame.frame_type if frame.frame_type else "unknown")

    if not frame_indices:
        print("[draw_frame_size_qp_psnr_vmaf_refdis] No valid frame data.")
        return

    # -------- 颜色（frame type） --------
    color_map = {"key": "red", "delta": "blue"}
    point_colors = [color_map.get(ft, "black") for ft in frame_types]

    # -------- 创建子图（共享 x 轴） --------
    width = max(16, len(frame_indices) / 30)
    fig, axes = plt.subplots(
        5, 1,
        figsize=(width, 12),
        sharex=True
    )

    ax_size, ax_qp, ax_psnr, ax_vmaf, ax_refdis = axes
    # ====================================================
    # 1) Frame Size
    # ====================================================
    ax_size.plot(frame_indices, frame_sizes, label="Frame Size (bytes)")
    ax_size.scatter(frame_indices, frame_sizes, c=point_colors, s=12)

    avg_size = np.mean(frame_sizes)
    ax_size.axhline(avg_size, linestyle="--", color="gray",
                    linewidth=1.5, label=f"Avg = {avg_size:.1f} B")

    ax_size.text(
        frame_indices[-1], avg_size,
        f"Avg = {avg_size:.1f} B",
        ha="right", va="bottom",
        fontsize=9, color="gray",
        backgroundcolor="white"
    )

    ax_size.set_ylabel("Frame Size (bytes)")
    ax_size.set_title("Frame Size / QP / PSNR / VMAF / Reference Distance over Time")
    ax_size.grid(True)
    ax_size.legend(loc="upper right")

    # ====================================================
    # 2) QP
    # ====================================================
    ax_qp.plot(frame_indices, frame_qps,
               color="purple", linewidth=1.2, label="QP")
    ax_qp.scatter(frame_indices, frame_qps, s=10, color="black")

    avg_qp = np.mean(frame_qps)
    ax_qp.axhline(avg_qp, linestyle="--", color="gray",
                  linewidth=1.5, label=f"Avg = {avg_qp:.2f}")

    ax_qp.text(
        frame_indices[-1], avg_qp,
        f"Avg = {avg_qp:.2f}",
        ha="right", va="bottom",
        fontsize=9, color="gray",
        backgroundcolor="white"
    )

    ax_qp.set_ylabel("QP")
    ax_qp.grid(True, linestyle="--", alpha=0.3)
    ax_qp.legend(loc="upper right")

    # ====================================================
    # 3) PSNR
    # ====================================================
    ax_psnr.plot(frame_indices, frame_psnrs,
                 color="blue", linewidth=1.2, label="PSNR (dB)")
    ax_psnr.scatter(frame_indices, frame_psnrs, s=10, color="red")

    avg_psnr = np.mean(frame_psnrs)
    ax_psnr.axhline(avg_psnr, linestyle="--", color="gray",
                    linewidth=1.5, label=f"Avg = {avg_psnr:.2f} dB")

    ax_psnr.text(
        frame_indices[-1], avg_psnr,
        f"Avg = {avg_psnr:.2f} dB",
        ha="right", va="bottom",
        fontsize=9, color="gray",
        backgroundcolor="white"
    )

    ax_psnr.set_ylabel("PSNR (dB)")
    ax_psnr.set_xlabel("Frame Index (tracking_id)")
    ax_psnr.grid(True, linestyle="--", alpha=0.3)
    ax_psnr.legend(loc="upper right")
    # ====================================================
    # 4) VMAF
    # ====================================================
    ax_vmaf.plot(frame_indices, frame_vmafs,
                 color="green", linewidth=1.2, label="VMAF")
    ax_vmaf.scatter(frame_indices, frame_vmafs, s=10, color="black")
    avg_vmaf = np.mean(frame_vmafs)
    ax_vmaf.axhline(avg_vmaf, linestyle="--", color="gray",
                    linewidth=1.5, label=f"Avg = {avg_vmaf:.2f}")
    ax_vmaf.text(
        frame_indices[-1], avg_vmaf,
        f"Avg = {avg_vmaf:.2f}",
        ha="right", va="bottom",
        fontsize=9, color="gray",
        backgroundcolor="white"
    )
    ax_vmaf.set_ylabel("VMAF")
    ax_vmaf.set_xlabel("Frame Index (tracking_id)")
    ax_vmaf.grid(True, linestyle="--", alpha=0.3)
    ax_vmaf.legend(loc="upper right")
    # ====================================================
    # 5) Reference Distance
    # ====================================================
    ax_refdis.plot(frame_indices, frame_refdis,
                   color="orange", linewidth=1.2, label="Reference Distance")
    ax_refdis.scatter(frame_indices, frame_refdis, s=10, color="black")
    avg_refdis = np.mean(frame_refdis)
    ax_refdis.axhline(avg_refdis, linestyle="--", color="gray",
                      linewidth=1.5, label=f"Avg = {avg_refdis:.2f}")
    ax_refdis.text(
        frame_indices[-1], avg_refdis,
        f"Avg = {avg_refdis:.2f}",
        ha="right", va="bottom",
        fontsize=9, color="gray",
        backgroundcolor="white"
    )
    ax_refdis.set_ylabel("Reference Distance")
    ax_refdis.set_xlabel("Frame Index (tracking_id)")
    ax_refdis.grid(True, linestyle="--", alpha=0.3)
    ax_refdis.legend(loc="upper right")
    
    # -------- 统一 x 轴刻度 --------
    ax_psnr.xaxis.set_major_locator(plt.MultipleLocator(20))

    plt.tight_layout()
    plt.savefig(pdf_path)
    plt.close()

    print(f"[draw_frame_size_qp_psnr_vmaf] PDF saved → {pdf_path}")

def draw_frame_size_violin(frames_info,
                           pdf_path="frame_size_violin.pdf",
                           start_frame=50,
                           max_frames=1000):
    """
    绘制 frame_size 的大提琴分布图
    """

    frames = list(frames_info)
    frames = frames[start_frame:]

    if max_frames is not None and len(frames) > max_frames:
        frames = frames[:max_frames]

    frame_sizes = []

    for frame in frames:
        size = getattr(frame, "frame_size", None)
        if size is not None:
            frame_sizes.append(size)

    if not frame_sizes:
        print("[draw_frame_size_violin] No frame_size data found.")
        return

    avg_size = np.mean(frame_sizes)

    plt.figure(figsize=(6, 6))

    parts = plt.violinplot(
        frame_sizes,
        showmeans=False,
        showmedians=True,
        showextrema=True
    )

    for pc in parts["bodies"]:
        pc.set_facecolor("#55a868")
        pc.set_edgecolor("black")
        pc.set_alpha(0.7)

    plt.axhline(avg_size, color="red", linestyle="--", linewidth=1.5,
                label=f"Avg = {avg_size:.1f} B")

    plt.text(
        1.02, avg_size,
        f"{avg_size:.1f} B",
        va="bottom",
        ha="left",
        color="red",
        fontsize=10
    )

    plt.ylabel("Frame Size (bytes)")
    plt.title("Frame Size Distribution (Violin Plot)")
    plt.xticks([1], ["All Frames"])
    plt.grid(True, axis="y", linestyle="--", alpha=0.4)
    plt.legend()
    plt.tight_layout()

    plt.savefig(pdf_path)
    plt.close()

    print(f"[draw_frame_size_violin] PDF saved → {pdf_path}")  
    
def draw_delays_timeline(frames_info, pdf_path="delays_timeline.pdf", start_frame=50, max_frames=1000):
    """
    绘制每帧 delay 的平直(step)折线图。
    高亮显示：
      - frame.status == 'encoded' → 红色竖线
      - frame.status == 'received' → 黄色竖线
      - 其它非 decoded 状态 → 灰色竖线
    """

    frames = list(frames_info)
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

def draw_e2e_delay_cdf(frames_info, pdf_path="e2e_cdf.pdf",
                       start_frame=50, max_frames=1000):
    """
    绘制 e2e_delay 的 CDF 图。
    e2e_delay 单位: us → 会转换成 ms。
    """
    frames = list(frames_info)
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

def draw_firebreak_timeline(frames_info, start_frame=50, max_frames=1000, pdf_path="firebreak_timeline.pdf"):
    """
    Frame-level timeline visualization including:
    - frame.g
    - frame.g_aimd_
    - frame.g_avg_
    - frame.ref_prev_prob
    - frame.ref_distance
    - packet loss shown as red vertical spans
    """

    frames = list(frames_info)
    frames = frames[start_frame:]

    if max_frames is not None and len(frames) > max_frames:
        frames = frames[:max_frames]

    frame_indices = []
    frame_g_values = []
    frame_g_aimd_values = []
    frame_g_avg_values = []
    frame_ref_prev_probs = []
    frame_ref_distances = []
    loss_frames = []

    for frame in frames:
        idx = getattr(frame, "tracking_id", None)
        status = getattr(frame, "status", None)
        if idx is None or status is None:
            continue

        frame_indices.append(idx)
        frame_g_values.append(getattr(frame, "g", None))
        frame_g_aimd_values.append(getattr(frame, "g_aimd_", None))
        frame_g_avg_values.append(getattr(frame, "g_avg_", None))
        frame_ref_prev_probs.append(getattr(frame, "ref_prev_prob", None))
        frame_ref_distances.append(getattr(frame, "ref_distance", None))

        if status != "decoded":
            loss_frames.append(idx)

    if not frame_indices:
        print("[draw_firebreak_timeline] No frame data.")
        return

    # ---------- Figure ----------
    width = max(14, len(frame_indices) / 25)
    fig, ax = plt.subplots(figsize=(width, 4))

    # ---------- g / g_aimd / g_avg ----------
    ax.plot(frame_indices, frame_g_values,
            label="g", linewidth=1.8)
    ax.plot(frame_indices, frame_g_aimd_values,
            label="g_aimd", linestyle="--", linewidth=1.4)
    ax.plot(frame_indices, frame_g_avg_values,
            label="g_avg", linestyle=":", linewidth=2.0)

    ax.set_xlabel("Frame Index (tracking_id)")
    ax.set_ylabel("Control Variables (g)")
    ax.grid(True, linestyle="--", alpha=0.4)

    # ---------- Loss markers ----------
    for idx in loss_frames:
        ax.axvspan(idx - 0.5, idx + 0.5,
                   color="red", alpha=0.18, linewidth=0)

    # ---------- ref_prev_prob (right y-axis) ----------
    ax2 = ax.twinx()
    ax2.plot(frame_indices, frame_ref_prev_probs,
             color="purple", linestyle="-.", linewidth=1.6,
             label="ref_prev_prob")
    ax2.set_ylabel("Ref Prev Probability")

    # ---------- ref_distance (second right y-axis, offset) ----------
    ax3 = ax.twinx()
    ax3.spines["right"].set_position(("outward", 55))
    ax3.step(frame_indices, frame_ref_distances,
             where="post", color="brown", linewidth=1.6,
             label="ref_distance")
    ax3.set_ylabel("Reference Distance")

    # ---------- Legend (merge all axes) ----------
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    lines3, labels3 = ax3.get_legend_handles_labels()
    ax.legend(lines1 + lines2 + lines3,
              labels1 + labels2 + labels3,
              loc="upper right",
              frameon=True)

    # ---------- Title ----------
    ax.set_title(
        "Firebreak Timeline: Loss, Reference Distance, and Control Dynamics",
        fontsize=12
    )

    plt.tight_layout()
    plt.savefig(pdf_path)
    plt.close()

    print(f"[draw_firebreak_timeline] PDF saved → {pdf_path}")

if __name__ == "__main__":
    assert 3 == len(argv), "Usage: python analysis_and_draw.py <fps> <expr_path>"
    fps = float(argv[1])
    target_interval_ms = 1000.0 / fps 
    expr_path = argv[2]
    if expr_path.endswith("/"):
        expr_path = expr_path[:-1]
    
    metrics_save_path = f"{expr_path}/metrics.json" 
    fig_save_path = f"{expr_path}/figures" 
    makedirs(fig_save_path, exist_ok=True)
    
    frame_infos = read_result_from_json(f"{expr_path}/result.json") 
    
    calculate_and_save_metrics(frame_infos, metrics_save_path)
    
    start_frame = 50
    max_frames = 1000
    draw_delays_timeline(frame_infos, pdf_path=f"{fig_save_path}/delays_timeline.pdf", start_frame=start_frame, max_frames=max_frames)
    draw_e2e_delay_cdf(frame_infos, pdf_path=f"{fig_save_path}/e2e_cdf.pdf", start_frame=start_frame, max_frames=max_frames)
    draw_stall_cdf(frame_infos, pdf_path=f"{fig_save_path}/stall_cdf.pdf", start_frame=start_frame, max_frames=max_frames)
    
    draw_frame_size_qp_psnr_vmaf_refdis(frame_infos, pdf_path=f"{fig_save_path}/frame_size_qp_psnr_vmaf_refdis.pdf", start_frame=start_frame, max_frames=max_frames)
    
    draw_frame_size_violin(frame_infos, pdf_path=f"{fig_save_path}/frame_size_distribution.pdf", start_frame=start_frame, max_frames=max_frames)
    draw_psnr_violin(frame_infos, pdf_path=f"{fig_save_path}/psnr_distribution.pdf", start_frame=start_frame, max_frames=max_frames) 
    draw_vmaf_violin(frame_infos, pdf_path=f"{fig_save_path}/vmaf_distribution.pdf", start_frame=start_frame, max_frames=max_frames)
    
    draw_firebreak_timeline(frame_infos, pdf_path=f"{fig_save_path}/firebreak_timeline.pdf", start_frame=start_frame, max_frames=max_frames)
    
    # draw_frame_rendered_timeline(frame_infos, pdf_path=f"{fig_save_path}/frame_rendered_timeline.pdf", start_frame=start_frame, max_frames=max_frames)
    
