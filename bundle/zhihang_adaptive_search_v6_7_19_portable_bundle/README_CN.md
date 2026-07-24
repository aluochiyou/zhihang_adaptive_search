# 智航三机自主搜索 V6.7.19 Portable 完整版

V6.7.19以V6.7.18任务算法为模板，增加跨电脑机器配置层、YOLO运行时适配、三路并发性能检查、赛事自定义消息工作空间接入和最终官方话题桥接。核心搜索、动态跟踪、重捕获、静态候选管理、精准定位、HOLD恢复和返航逻辑保持不变。

## 最快部署路径

```bash
cd ~/下载/zhihang_adaptive_search_v6_7_19_portable_bundle

# 1. 生成本机配置。路径可按本机实际情况显式传入。
bash configure_portable_machine.sh \
  --workspace "$HOME/xtdrone_competition_ws" \
  --xtdrone-root "$HOME/XTDrone" \
  --px4-root "$HOME/PX4_Firmware" \
  --model "$HOME/yolo_models/best.pt" \
  --runtime auto \
  --record-bag \
  --install-user-profile

# 2. 已有YOLO环境时先检查；没有时创建并安装。
bash setup_yolo_runtime.sh --create --install \
  --runtime conda --env yolo26 \
  --model "$HOME/yolo_models/best.pt"

# 3. 安装前自检、备份、安装、编译。
bash install_portable.sh

# 4. 赛前诊断与三进程容量测试。
bash doctor_portable.sh
bash benchmark_three_yolo_capacity.sh --iterations 20

# 5. 正式一键启动。
bash launch_portable_formal_one_click.sh \
  --scene scene_001_adaptive_v6_7_19_formal \
  --model "$HOME/yolo_models/best.pt" \
  --runtime-env yolo26 \
  --seed -1 \
  --record-bag
```

正式启动器遵守以下屏障：PX4/Gazebo、XTDrone通信和位姿先启动；管理器、三机飞行/感知和三个真实YOLO进程随后启动；三路处理链连续达到10Hz后，先启动`score1.bag`，再启动`model_state.py`，最后授权无人机解锁执行任务。

## 关键文档

- `V6.7.19_PORTABLE_COMPETITION_MANUAL_CN.md`：完整部署、算法、流程和故障说明书。
- `COMPETITION_COMPLIANCE_MATRIX.md`：赛事要求到代码证据的对应关系。
- `machine_profile.example.env`：所有跨电脑配置项。
- `SOURCE_CODE_INDEX.md`：完整源代码索引。
- `VALIDATION_RESULT.txt`：交付前静态与单元测试结果。

## 不修改的官方文件

- `~/XTDrone/communication/vtol_communication.py`
- `~/XTDrone/communication/multi_vehicle_communication.sh`
- `~/XTDrone/communication/multi_vehicle_commonication.sh`
- 赛方PX4模型、Gazebo模型、world、launch、`model_state.py`和`zhihang_ws`

安装器会先备份已有算法ROS包，并对上述三项通信文件进行安装前后SHA-256一致性检查。
