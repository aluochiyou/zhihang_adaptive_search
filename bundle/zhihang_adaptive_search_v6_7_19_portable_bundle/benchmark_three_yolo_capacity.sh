#!/usr/bin/env bash
# Concurrent synthetic capacity test for the three formal YOLO workers.
# This is a preflight estimate; the formal real-camera >=10 Hz barrier remains authoritative.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILE=""
MODEL=""
ITERATIONS=20
MIN_FPS=""
while (($#)); do
  case "$1" in
    --profile) PROFILE="${2:?missing value}"; shift ;;
    --model) MODEL="${2:?missing value}"; shift ;;
    --iterations) ITERATIONS="${2:?missing value}"; shift ;;
    --minimum-fps) MIN_FPS="${2:?missing value}"; shift ;;
    *) echo "[ERROR] unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done
[[ -n "${PROFILE}" ]] && export ZHIHANG_PROFILE_FILE="${PROFILE}"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/shell_arg_utils.sh"
source "${ROOT}/yolo_runtime_common.sh"
MODEL="${MODEL:-${ZHIHANG_YOLO_MODEL}}"
MODEL="$(zh_expand_user_path "${MODEL}")"
MIN_FPS="${MIN_FPS:-${ZHIHANG_YOLO_MINIMUM_FPS}}"
[[ -f "${MODEL}" ]] || { echo "[ERROR] model not found: ${MODEL}" >&2; exit 1; }
zh_resolve_yolo_runtime >/dev/null

OUT="${ROOT}/three_worker_benchmark_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${OUT}"
pids=()
for vid in 0 1 2; do
  device="$(zh_resolve_device_for_vehicle "${vid}")"
  args=(
    --model "${MODEL}"
    --device "${device}"
    --imgsz 640
    --quantize "${ZHIHANG_YOLO_QUANTIZE}"
    --warmup 3
    --iterations "${ITERATIONS}"
    --minimum-fps "${MIN_FPS}"
    --report "${OUT}/vehicle_${vid}.json"
  )
  [[ "${ZHIHANG_YOLO_REQUIRE_CUDA}" == "0" ]] && args+=(--allow-cpu)
  echo "[START] v${vid} device=${device}"
  (zh_yolo_run "${ROOT}/verify_yolo_runtime.py" "${args[@]}" \
    > "${OUT}/vehicle_${vid}.log" 2>&1) &
  pids+=("$!")
done
rc=0
for i in 0 1 2; do
  if wait "${pids[$i]}"; then
    echo "[PASS] v${i} concurrent worker"
  else
    echo "[FAIL] v${i} concurrent worker; inspect ${OUT}/vehicle_${i}.log" >&2
    rc=2
  fi
done
python3 - "${OUT}" "${MIN_FPS}" <<'PY'
import json, pathlib, sys
root = pathlib.Path(sys.argv[1]); required=float(sys.argv[2])
rows=[]
for i in range(3):
    p=root/f"vehicle_{i}.json"
    if not p.is_file():
        rows.append({"vehicle_id":i,"ok":False,"error":"missing report"}); continue
    d=json.loads(p.read_text())
    rows.append({"vehicle_id":i,"ok":bool(d.get("ok")),"device":d.get("device"),
                 "gpu_name":d.get("gpu_name"),"fps":d.get("single_worker_synthetic_fps")})
summary={"note":"Concurrent synthetic benchmark; formal real-camera barrier is authoritative.",
         "required_fps_per_worker":required,"workers":rows,
         "pass":len(rows)==3 and all(r.get("ok") and float(r.get("fps") or 0)>=required for r in rows)}
(root/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2))
print(json.dumps(summary,ensure_ascii=False,indent=2))
raise SystemExit(0 if summary["pass"] else 2)
PY
pyrc=$?
if ((pyrc != 0)); then rc=2; fi
echo "[RESULT] ${OUT}/summary.json"
exit "${rc}"
