#!/usr/bin/env python3
from __future__ import annotations

import os
import subprocess
import tempfile
from pathlib import Path


B = Path(__file__).resolve().parents[1]

with tempfile.TemporaryDirectory() as tmp:
    root = Path(tmp)
    home = root / "home"
    ws = home / "other_ws"
    xt = home / "OtherXTDrone"
    qgc = home / "apps/QGroundControl.AppImage"
    model = home / "models/best.pt"
    (ws / "src").mkdir(parents=True)
    (xt / "communication").mkdir(parents=True)
    (xt / "sensing/pose_ground_truth").mkdir(parents=True)
    (xt / "zhihang2026").mkdir(parents=True)
    qgc.parent.mkdir(parents=True)
    qgc.write_text("fake", encoding="utf-8")
    qgc.chmod(0o755)
    model.parent.mkdir(parents=True)
    model.write_bytes(b"fake-model")
    pose = xt / "sensing/pose_ground_truth/get_multi_vehcle_local_pose.sh"
    pose.write_text("#!/bin/sh\n", encoding="utf-8")
    pose.chmod(0o755)
    (xt / "zhihang2026/model_state.py").write_text("print('x')\n", encoding="utf-8")
    comm = xt / "communication/multi_vehicle_communication.sh"
    comm.write_text("#!/bin/sh\n", encoding="utf-8")
    comm.chmod(0o755)

    profile = root / "machine.env"
    report = root / "report.json"
    env = dict(os.environ)
    env["HOME"] = str(home)
    cmd = [
        "python3", str(B / "configure_portable_machine.py"),
        "--workspace", str(ws),
        "--xtdrone-root", str(xt),
        "--qgc", str(qgc),
        "--model", str(model),
        "--runtime", "system",
        "--allow-cpu",
        "--terminal-backend", "tmux",
        "--output", str(profile),
        "--report", str(report),
    ]
    result = subprocess.run(cmd, env=env, text=True, capture_output=True)
    assert result.returncode == 0, result.stderr + result.stdout
    text = profile.read_text(encoding="utf-8")
    assert f"export ZHIHANG_WS={ws}" in text
    assert f"export ZHIHANG_XTDRONE_ROOT={xt}" in text
    assert "export ZHIHANG_MESSAGE_WS_SETUP=" in text
    assert "export ZHIHANG_PX4_LAUNCH_FILE=auto" in text
    assert f"export ZHIHANG_YOLO_MODEL={model}" in text
    assert "export ZHIHANG_YOLO_RUNTIME=system" in text
    assert "export ZHIHANG_YOLO_REQUIRE_CUDA=0" in text
    assert "export ZHIHANG_TERMINAL_BACKEND=tmux" in text
    assert report.is_file()

print("V6.7.19 PORTABLE PROFILE TEST PASSED")
