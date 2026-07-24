#!/usr/bin/env python3
"""Generate a portable machine profile for Zhihang V6.7.19.

The script uses only the Python standard library. It never edits PX4, Gazebo,
XTDrone communication files or the installed ROS package.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import platform
import shlex
import shutil
import subprocess
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


BUNDLE = Path(__file__).resolve().parent
HOME = Path.home()


def first_existing(candidates: Iterable[Path], *, file: bool = False) -> Optional[Path]:
    for candidate in candidates:
        if not str(candidate):
            continue
        candidate = candidate.expanduser()
        if (candidate.is_file() if file else candidate.is_dir()):
            return candidate.resolve()
    return None


def command_path(name: str) -> Optional[str]:
    return shutil.which(name)


def run(args: List[str], timeout: float = 20.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=False,
    )


def detect_conda() -> Optional[Path]:
    candidates = [
        Path(os.environ.get("CONDA_EXE", "")),
        HOME / "miniconda3/bin/conda",
        HOME / "anaconda3/bin/conda",
        HOME / "mambaforge/bin/conda",
        HOME / "miniforge3/bin/conda",
    ]
    found = command_path("conda")
    if found:
        candidates.insert(0, Path(found))
    return first_existing(candidates, file=True)


def conda_envs(conda: Optional[Path]) -> Dict[str, Path]:
    if not conda:
        return {}
    result = run([str(conda), "env", "list", "--json"])
    if result.returncode:
        return {}
    try:
        data = json.loads(result.stdout)
    except json.JSONDecodeError:
        return {}
    output: Dict[str, Path] = {}
    for raw in data.get("envs", []):
        path = Path(raw)
        output[path.name] = path
    return output


def probe_python(python_cmd: List[str]) -> Dict[str, Any]:
    code = r'''
import json, platform, sys
out = {
    "ok": True,
    "python": sys.executable,
    "python_version": platform.python_version(),
}
for name in ("numpy", "cv2", "torch", "ultralytics"):
    try:
        module = __import__(name)
        out[name] = getattr(module, "__version__", "unknown")
    except Exception as exc:
        out["ok"] = False
        out[name] = None
        out[name + "_error"] = f"{type(exc).__name__}: {exc}"
try:
    import torch
    out["cuda_available"] = bool(torch.cuda.is_available())
    out["cuda_device_count"] = int(torch.cuda.device_count())
    out["cuda_version"] = getattr(torch.version, "cuda", None)
    out["devices"] = []
    for idx in range(torch.cuda.device_count()):
        props = torch.cuda.get_device_properties(idx)
        out["devices"].append({
            "index": idx,
            "name": torch.cuda.get_device_name(idx),
            "total_memory_gb": round(props.total_memory / (1024 ** 3), 2),
        })
except Exception as exc:
    out["cuda_available"] = False
    out["cuda_probe_error"] = f"{type(exc).__name__}: {exc}"
print(json.dumps(out, ensure_ascii=False))
'''
    result = run(python_cmd + ["-c", code], timeout=45.0)
    if result.returncode:
        return {
            "ok": False,
            "command": python_cmd,
            "error": result.stderr.strip() or result.stdout.strip(),
        }
    try:
        return json.loads(result.stdout.strip().splitlines()[-1])
    except Exception as exc:
        return {
            "ok": False,
            "command": python_cmd,
            "error": f"probe parse failure: {exc}; stdout={result.stdout[-500:]}",
        }


def detect_terminal_backend(preferred: str) -> str:
    if preferred != "auto":
        return preferred
    for name in ("gnome-terminal", "xterm", "konsole", "tmux"):
        if command_path(name):
            return name
    return "none"


def find_qgc(explicit: Optional[str]) -> Optional[Path]:
    candidates: List[Path] = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    candidates.extend(
        [
            HOME / "software/QGC/QGroundControl(1).AppImage",
            HOME / "software/QGC/QGroundControl.AppImage",
            HOME / "QGroundControl.AppImage",
            HOME / "Downloads/QGroundControl.AppImage",
            HOME / "下载/QGroundControl.AppImage",
            Path("/opt/QGroundControl.AppImage"),
        ]
    )
    for pattern in (
        str(HOME / "software/QGC/*.AppImage"),
        str(HOME / "Downloads/*QGroundControl*.AppImage"),
        str(HOME / "下载/*QGroundControl*.AppImage"),
    ):
        candidates.extend(Path(p) for p in glob.glob(pattern))
    return first_existing(candidates, file=True)


def find_model(explicit: Optional[str]) -> Optional[Path]:
    candidates: List[Path] = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    candidates.extend(
        [
            HOME / "yolo_models/best.pt",
            HOME / "runs/detect/train/weights/best.pt",
            HOME / "runs/train/weights/best.pt",
        ]
    )
    return first_existing(candidates, file=True)


def env_line(name: str, value: Any) -> str:
    return f"export {name}={shlex.quote(str(value))}"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Auto-detect paths and generate machine_profile.env"
    )
    parser.add_argument("--workspace")
    parser.add_argument("--ros-setup", default="/opt/ros/noetic/setup.bash")
    parser.add_argument("--underlay")
    parser.add_argument("--message-ws-setup")
    parser.add_argument("--xtdrone-root")
    parser.add_argument("--px4-root")
    parser.add_argument("--px4-package", default="px4")
    parser.add_argument("--px4-launch", default="auto")
    parser.add_argument("--px4-env-script", default="")
    parser.add_argument("--qgc")
    parser.add_argument("--terminal-backend", default="auto",
                        choices=["auto", "gnome-terminal", "xterm", "konsole", "tmux"])
    parser.add_argument("--runtime", default="auto",
                        choices=["auto", "conda", "venv", "system", "direct"])
    parser.add_argument("--conda-env", default="yolo26")
    parser.add_argument("--venv", default=str(HOME / ".venvs/yolo26"))
    parser.add_argument("--python")
    parser.add_argument("--model")
    parser.add_argument("--allow-cpu", action="store_true")
    parser.add_argument("--devices", default="auto,auto,auto")
    parser.add_argument("--record-bag", action="store_true")
    parser.add_argument("--no-qgc", action="store_true")
    parser.add_argument(
        "--output",
        default=str(BUNDLE / "machine_profile.env"),
        help="profile output path",
    )
    parser.add_argument(
        "--report",
        default=str(BUNDLE / "machine_profile_report.json"),
        help="JSON detection report",
    )
    parser.add_argument("--install-user-profile", action="store_true")
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()

    workspace = (
        Path(args.workspace).expanduser().resolve()
        if args.workspace
        else first_existing(
            [
                Path(os.environ.get("ZHIHANG_WS", "")),
                HOME / "xtdrone_competition_ws",
                HOME / "catkin_ws",
            ]
        )
    )
    xtdrone = (
        Path(args.xtdrone_root).expanduser().resolve()
        if args.xtdrone_root
        else first_existing(
            [
                Path(os.environ.get("ZHIHANG_XTDRONE_ROOT", "")),
                HOME / "XTDrone",
                HOME / "xtdrone",
            ]
        )
    )
    px4_root = (
        Path(args.px4_root).expanduser().resolve()
        if args.px4_root
        else first_existing(
            [
                Path(os.environ.get("ZHIHANG_PX4_ROOT", "")),
                HOME / "PX4_Firmware",
                HOME / "PX4-Autopilot",
            ]
        )
    )
    ros_setup = Path(args.ros_setup).expanduser()
    underlay = (
        Path(args.underlay).expanduser().resolve()
        if args.underlay
        else first_existing(
            [
                HOME / "catkin_ws/devel/setup.bash",
                HOME / "xtdrone_ws/devel/setup.bash",
            ],
            file=True,
        )
    )
    message_ws_setup = (
        Path(args.message_ws_setup).expanduser().resolve()
        if args.message_ws_setup
        else first_existing(
            [HOME / "zhihang_ws/devel/setup.bash"], file=True)
    )
    qgc = find_qgc(args.qgc)
    model = find_model(args.model)
    conda = detect_conda()
    envs = conda_envs(conda)
    venv = Path(args.venv).expanduser().resolve()
    terminal_backend = detect_terminal_backend(args.terminal_backend)

    runtime = args.runtime
    python_probe: Dict[str, Any] = {}
    python_cmd: Optional[List[str]] = None
    runtime_detail = ""

    if runtime in ("auto", "direct") and args.python:
        candidate = Path(args.python).expanduser()
        if candidate.is_file():
            runtime = "direct"
            python_cmd = [str(candidate.resolve())]
            runtime_detail = str(candidate.resolve())
    if runtime in ("auto", "conda") and args.conda_env in envs and conda:
        runtime = "conda"
        python_cmd = [
            str(conda), "run", "--no-capture-output",
            "-n", args.conda_env, "python"
        ]
        runtime_detail = args.conda_env
    if runtime in ("auto", "venv") and (venv / "bin/python").is_file():
        runtime = "venv"
        python_cmd = [str(venv / "bin/python")]
        runtime_detail = str(venv)
    if runtime in ("auto", "system") and command_path("python3"):
        runtime = "system"
        python_cmd = [command_path("python3") or "python3"]
        runtime_detail = python_cmd[0]
    if runtime == "auto":
        runtime = "conda" if conda else "venv"

    if python_cmd:
        python_probe = probe_python(python_cmd)
    else:
        python_probe = {
            "ok": False,
            "error": (
                "selected runtime does not yet have an executable Python; "
                "run setup_yolo_runtime.sh"
            ),
        }

    cuda_available = bool(python_probe.get("cuda_available"))
    devices = args.devices
    if devices == "auto,auto,auto" and cuda_available:
        count = int(python_probe.get("cuda_device_count", 0))
        if count >= 3:
            devices = "0,1,2"
        elif count == 2:
            devices = "0,1,1"
        else:
            devices = "0,0,0"
    elif devices == "auto,auto,auto" and not cuda_available:
        devices = "cpu,cpu,cpu"

    base_xt = xtdrone or (HOME / "XTDrone")
    profile_values: Dict[str, Any] = {
        "ZHIHANG_WS": workspace or (HOME / "xtdrone_competition_ws"),
        "ZHIHANG_ROS_SETUP": ros_setup,
        "ZHIHANG_OPTIONAL_UNDERLAY": underlay or "",
        "ZHIHANG_MESSAGE_WS_SETUP": message_ws_setup or (HOME / "zhihang_ws/devel/setup.bash"),
        "ZHIHANG_XTDRONE_ROOT": base_xt,
        "ZHIHANG_PX4_ROOT": px4_root or (HOME / "PX4_Firmware"),
        "ZHIHANG_PX4_ROS_PACKAGE": args.px4_package,
        "ZHIHANG_PX4_LAUNCH_FILE": args.px4_launch,
        "ZHIHANG_PX4_LAUNCH_ARGS": "",
        "ZHIHANG_PX4_ENV_SCRIPT": args.px4_env_script,
        "ZHIHANG_GUARD_ROS_PACKAGE": "zhihang_xtdrone_guard",
        "ZHIHANG_GUARD_LAUNCH_FILE": "guard_standard_vtol.launch",
        "ZHIHANG_COMMUNICATION_DIR": base_xt / "communication",
        "ZHIHANG_POSE_SCRIPT": (
            base_xt / "sensing/pose_ground_truth/get_multi_vehcle_local_pose.sh"
        ),
        "ZHIHANG_MODEL_STATE_SCRIPT": base_xt / "zhihang2026/model_state.py",
        "ZHIHANG_COMPETITION_DATA_DIR": base_xt / "zhihang2026",
        "ZHIHANG_QGC_EXECUTABLE": qgc or "",
        "ZHIHANG_QGC_ARGS": "",
        "ZHIHANG_START_QGC": 0 if args.no_qgc else 1,
        "ZHIHANG_TERMINAL_BACKEND": terminal_backend,
        "ZHIHANG_TMUX_SESSION": "zhihang_v6_7_19",
        "ZHIHANG_HEADLESS": 1 if terminal_backend == "tmux" else 0,
        "ZHIHANG_YOLO_RUNTIME": runtime,
        "ZHIHANG_YOLO_ENV": args.conda_env,
        "ZHIHANG_YOLO_VENV": venv,
        "ZHIHANG_YOLO_PYTHON": (
            Path(args.python).expanduser().resolve() if args.python else ""
        ),
        "ZHIHANG_YOLO_MODEL": model or (HOME / "yolo_models/best.pt"),
        "ZHIHANG_YOLO_REQUIRE_CUDA": 0 if args.allow_cpu else 1,
        "ZHIHANG_YOLO_DEVICES": devices,
        "ZHIHANG_YOLO_QUANTIZE": 16 if cuda_available else 32,
        "ZHIHANG_YOLO_ADAPTIVE": 1,
        "ZHIHANG_YOLO_ADAPTIVE_SIZES": "640,512,416",
        "ZHIHANG_YOLO_MINIMUM_FPS": 10.0,
        "ZHIHANG_YOLO_CONFIDENCE": 0.25,
        "ZHIHANG_YOLO_IOU": 0.70,
        "ZHIHANG_YOLO_WARMUP_ITERATIONS": 3,
        "ZHIHANG_YOLO_START_LOCAL_WORKERS": 1,
        "ZHIHANG_YOLO_HOSTS": "127.0.0.1,127.0.0.1,127.0.0.1",
        "ZHIHANG_YOLO_BIND_HOST": "127.0.0.1",
        "ZHIHANG_YOLO_PORT_BASE": 17771,
        "ZHIHANG_DEFAULT_SCENE": "scene_001_adaptive_v6_7_19_formal",
        "ZHIHANG_DEFAULT_SEED": -1,
        "ZHIHANG_RECORD_BAG": 1 if args.record_bag else 0,
    }

    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Auto-generated Zhihang V6.7.19 portable machine profile",
        "# Generated by configure_portable_machine.py",
        "# Review this file before formal competition use.",
        "",
    ]
    sections = [
        ("ROS / workspace", [
            "ZHIHANG_WS", "ZHIHANG_ROS_SETUP", "ZHIHANG_OPTIONAL_UNDERLAY",
            "ZHIHANG_MESSAGE_WS_SETUP"
        ]),
        ("PX4 / XTDrone", [
            "ZHIHANG_XTDRONE_ROOT", "ZHIHANG_PX4_ROOT",
            "ZHIHANG_PX4_ROS_PACKAGE", "ZHIHANG_PX4_LAUNCH_FILE",
            "ZHIHANG_PX4_LAUNCH_ARGS", "ZHIHANG_PX4_ENV_SCRIPT",
            "ZHIHANG_GUARD_ROS_PACKAGE", "ZHIHANG_GUARD_LAUNCH_FILE",
            "ZHIHANG_COMMUNICATION_DIR", "ZHIHANG_POSE_SCRIPT",
            "ZHIHANG_MODEL_STATE_SCRIPT", "ZHIHANG_COMPETITION_DATA_DIR",
        ]),
        ("Terminal / QGroundControl", [
            "ZHIHANG_QGC_EXECUTABLE", "ZHIHANG_QGC_ARGS",
            "ZHIHANG_START_QGC", "ZHIHANG_TERMINAL_BACKEND",
            "ZHIHANG_TMUX_SESSION", "ZHIHANG_HEADLESS",
        ]),
        ("YOLO runtime", [
            "ZHIHANG_YOLO_RUNTIME", "ZHIHANG_YOLO_ENV",
            "ZHIHANG_YOLO_VENV", "ZHIHANG_YOLO_PYTHON",
            "ZHIHANG_YOLO_MODEL", "ZHIHANG_YOLO_REQUIRE_CUDA",
            "ZHIHANG_YOLO_DEVICES", "ZHIHANG_YOLO_QUANTIZE",
            "ZHIHANG_YOLO_ADAPTIVE", "ZHIHANG_YOLO_ADAPTIVE_SIZES",
            "ZHIHANG_YOLO_MINIMUM_FPS", "ZHIHANG_YOLO_CONFIDENCE",
            "ZHIHANG_YOLO_IOU", "ZHIHANG_YOLO_WARMUP_ITERATIONS",
            "ZHIHANG_YOLO_START_LOCAL_WORKERS", "ZHIHANG_YOLO_HOSTS",
            "ZHIHANG_YOLO_BIND_HOST", "ZHIHANG_YOLO_PORT_BASE",
        ]),
        ("Mission defaults", [
            "ZHIHANG_DEFAULT_SCENE", "ZHIHANG_DEFAULT_SEED",
            "ZHIHANG_RECORD_BAG",
        ]),
    ]
    for title, keys in sections:
        lines.append(f"# ---------- {title} ----------")
        lines.extend(env_line(key, profile_values[key]) for key in keys)
        lines.append("")
    output.write_text("\n".join(lines), encoding="utf-8")

    report = {
        "version": "6.7.19",
        "platform": {
            "system": platform.system(),
            "release": platform.release(),
            "machine": platform.machine(),
            "python": platform.python_version(),
        },
        "detected": {
            "workspace": str(workspace) if workspace else None,
            "ros_setup": str(ros_setup),
            "underlay": str(underlay) if underlay else None,
            "message_ws_setup": str(message_ws_setup) if message_ws_setup else None,
            "xtdrone_root": str(xtdrone) if xtdrone else None,
            "px4_root": str(px4_root) if px4_root else None,
            "qgc": str(qgc) if qgc else None,
            "model": str(model) if model else None,
            "conda": str(conda) if conda else None,
            "conda_envs": {k: str(v) for k, v in envs.items()},
            "terminal_backend": terminal_backend,
            "runtime": runtime,
            "runtime_detail": runtime_detail,
            "python_probe": python_probe,
        },
        "profile": {k: str(v) for k, v in profile_values.items()},
        "warnings": [],
    }

    required = {
        "workspace": workspace,
        "ROS Noetic setup": ros_setup if ros_setup.is_file() else None,
        "XTDrone root": xtdrone,
        "competition message setup": message_ws_setup,
        "model_state.py": (
            (xtdrone / "zhihang2026/model_state.py")
            if xtdrone and (xtdrone / "zhihang2026/model_state.py").is_file()
            else None
        ),
        "YOLO model": model,
    }
    for name, value in required.items():
        if not value:
            report["warnings"].append(f"not detected: {name}")
    if terminal_backend == "none":
        report["warnings"].append("no supported terminal backend detected")
    if not python_probe.get("ok"):
        report["warnings"].append("YOLO Python stack is not ready")
    if not cuda_available and not args.allow_cpu:
        report["warnings"].append("CUDA unavailable but formal profile requires CUDA")

    report_path = Path(args.report).expanduser()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    if args.install_user_profile:
        user_path = (
            Path(os.environ.get("XDG_CONFIG_HOME", HOME / ".config"))
            / "zhihang_v6_7_19/machine.env"
        )
        user_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(output, user_path)
        print(f"[OK] user profile: {user_path}")

    print(f"[OK] profile generated: {output}")
    print(f"[OK] report generated: {report_path}")
    print(f"[DETECT] workspace={workspace or '<missing>'}")
    print(f"[DETECT] xtdrone={xtdrone or '<missing>'}")
    print(f"[DETECT] model={model or '<missing>'}")
    print(f"[DETECT] runtime={runtime} detail={runtime_detail or '<not ready>'}")
    print(f"[DETECT] cuda={cuda_available} devices={devices}")
    print(f"[DETECT] terminal={terminal_backend}")
    for warning in report["warnings"]:
        print(f"[WARN] {warning}")

    if args.strict and report["warnings"]:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
