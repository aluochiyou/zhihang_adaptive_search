#!/usr/bin/env python3
"""Verify that the selected Python runtime can load and execute the real YOLO model."""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any, Dict, List

REQUIRED_CLASSES = [
    "prius_hybrid",
    "prius_hybrid_camo",
    "car_lexus",
    "car_opel",
    "fire_truck",
    "person_white",
    "suv_camo",
    "person_red",
]


def normalize_names(names: Any) -> Dict[int, str]:
    if isinstance(names, dict):
        return {int(k): str(v) for k, v in names.items()}
    if isinstance(names, (list, tuple)):
        return {i: str(v) for i, v in enumerate(names)}
    return {}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--quantize", type=int, default=16, choices=[0, 16, 32])
    parser.add_argument("--warmup", type=int, default=2)
    parser.add_argument("--iterations", type=int, default=8)
    parser.add_argument("--minimum-fps", type=float, default=10.0)
    parser.add_argument("--allow-cpu", action="store_true")
    parser.add_argument("--skip-class-check", action="store_true")
    parser.add_argument("--required-classes", default=",".join(REQUIRED_CLASSES))
    parser.add_argument("--report")
    args = parser.parse_args()

    import cv2
    import numpy as np
    import torch
    import ultralytics
    from ultralytics import YOLO

    model_path = Path(args.model).expanduser().resolve()
    if not model_path.is_file():
        raise SystemExit(f"[ERROR] model not found: {model_path}")

    requested = str(args.device).strip()
    if requested == "auto":
        device = "0" if torch.cuda.is_available() else "cpu"
    else:
        device = requested
    if device != "cpu" and not torch.cuda.is_available():
        raise SystemExit("[ERROR] CUDA requested but torch.cuda.is_available() is false")
    if device == "cpu" and not args.allow_cpu:
        raise SystemExit(
            "[ERROR] CPU runtime is disabled for formal readiness. "
            "Use --allow-cpu only for validation or after a real 3-stream >=10 Hz test."
        )

    quantize = args.quantize if args.quantize > 0 else None
    if device == "cpu" and quantize == 16:
        quantize = 32

    model = YOLO(str(model_path))
    names = normalize_names(getattr(model, "names", {}))
    if not names:
        try:
            names = normalize_names(model.model.names)
        except Exception:
            names = {}

    required = [x.strip() for x in args.required_classes.split(",") if x.strip()]
    available = set(names.values())
    missing = [name for name in required if name not in available]

    image = np.zeros((720, 1280, 3), dtype=np.uint8)
    predict_kwargs = {
        "source": image,
        "imgsz": args.imgsz,
        "conf": 0.25,
        "iou": 0.70,
        "device": device,
        "verbose": False,
    }
    quantize_supported = True
    if quantize is not None:
        predict_kwargs["quantize"] = quantize

    def predict_once():
        nonlocal quantize_supported
        try:
            return model.predict(**predict_kwargs)
        except (TypeError, ValueError) as exc:
            if "quantize" not in predict_kwargs:
                raise
            quantize_supported = False
            predict_kwargs.pop("quantize", None)
            print(f"[WARN] runtime does not accept quantize={quantize}; retrying without it: {exc}")
            return model.predict(**predict_kwargs)

    for _ in range(max(1, args.warmup)):
        predict_once()
    if torch.cuda.is_available():
        torch.cuda.synchronize()

    start = time.perf_counter()
    for _ in range(max(1, args.iterations)):
        predict_once()
    if torch.cuda.is_available():
        torch.cuda.synchronize()
    elapsed = time.perf_counter() - start
    fps = max(1, args.iterations) / max(elapsed, 1e-9)

    report: Dict[str, Any] = {
        "ok": True,
        "model": str(model_path),
        "python_runtime": __import__("sys").executable,
        "torch": torch.__version__,
        "ultralytics": ultralytics.__version__,
        "opencv": cv2.__version__,
        "cuda_available": bool(torch.cuda.is_available()),
        "cuda_version": getattr(torch.version, "cuda", None),
        "device": device,
        "gpu_name": (
            torch.cuda.get_device_name(int(device))
            if device != "cpu" and str(device).isdigit()
            else None
        ),
        "imgsz": args.imgsz,
        "quantize_requested": args.quantize,
        "quantize_used": quantize if quantize_supported else None,
        "quantize_supported": quantize_supported,
        "iterations": args.iterations,
        "elapsed_seconds": elapsed,
        "single_worker_synthetic_fps": fps,
        "minimum_fps": args.minimum_fps,
        "single_worker_fps_pass": fps >= args.minimum_fps,
        "class_names": names,
        "missing_required_classes": missing,
    }

    if missing and not args.skip_class_check:
        report["ok"] = False
    if fps < args.minimum_fps:
        report["ok"] = False

    if args.report:
        report_path = Path(args.report).expanduser()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"[OK] model loaded: {model_path}")
    print(f"[OK] runtime: {report['python_runtime']}")
    print(f"[OK] torch={torch.__version__} cuda={torch.cuda.is_available()} device={device}")
    if report["gpu_name"]:
        print(f"[OK] gpu={report['gpu_name']}")
    print(f"[OK] ultralytics={ultralytics.__version__} opencv={cv2.__version__}")
    print(f"[CHECK] class_count={len(names)} missing_required={missing}")
    print(f"[CHECK] synthetic single-worker fps={fps:.2f} requirement={args.minimum_fps:.2f}")
    print(
        "[NOTE] Formal launch still requires three real camera/perception pipelines "
        "to sustain >=10 Hz before the mission start barrier."
    )
    if not report["ok"]:
        if missing and not args.skip_class_check:
            print(f"[ERROR] required model classes missing: {missing}")
        if fps < args.minimum_fps:
            print("[ERROR] single-worker benchmark did not meet the configured minimum FPS")
        return 2
    print("[OK] YOLO runtime/model verification passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
