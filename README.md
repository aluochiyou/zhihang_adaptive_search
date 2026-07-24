# 智航自适应搜索系统 V6.7.19

三无人机协同目标检测、跟踪与定位系统，用于 2026 智航竞赛。

## 项目概述

本项目实现了基于 ROS Noetic 的三架 VTOL 无人机协同搜索系统，具备以下能力：

- **动态目标检测与跟踪**: 实时检测并跟踪移动目标（suv_camo, person_red）
- **静态目标检测与定位**: 检测并精确定位静态目标（6 类）
- **自适应搜索策略**: 基于检测结果动态调整搜索路线
- **配对引导搜索**: 利用已检测目标引导搜索未检测配对目标
- **精确悬停定位**: 通过悬停和多帧融合提高定位精度

## 系统架构

### 核心组件

1. **Mission Manager** (`mission_manager.py`)
   - 任务规划与分配
   - 目标状态管理
   - 搜索策略优化

2. **Flight Agent** (`vehicle_flight_agent.py`)
   - 飞行控制（固定翼/多旋翼切换）
   - 航点跟踪
   - 返航管理

3. **Perception Agent** (`vehicle_perception_agent.py`)
   - YOLO 检测结果处理
   - 目标检测验证
   - 多帧融合

4. **Target State Estimator** (`vision_target_state_estimator.py`)
   - 动态目标状态估计
   - 轨迹预测
   - 重识别

5. **YOLO Workers** (`yolo26_single_worker.py`)
   - 实时目标检测
   - 高性能推理（>10Hz）
   - 结果发布

### 搜索策略

1. **初始搜索阶段**: 三机分区覆盖搜索
2. **动态目标跟踪**: 检测到动态目标后立即跟踪
3. **静态目标确认**: 悬停精确定位已检测静态目标
4. **配对引导搜索**: 利用已检测目标引导搜索配对目标
5. **残差搜索**: 搜索未覆盖区域

## 部署

### 环境要求

- Ubuntu 20.04
- ROS Noetic
- PX4 Firmware
- Gazebo 11
- Python 3.8+
- CUDA 11.x (用于 YOLO)

### 快速部署

```bash
cd bundle/zhihang_adaptive_search_v6_7_19_portable_bundle

# 1. 配置本机环境
bash configure_portable_machine.sh \
  --workspace "$HOME/zhihang2026_ws" \
  --xtdrone-root "$HOME/XTDrone" \
  --px4-root "$HOME/PX4_Firmware" \
  --model "$HOME/zhihang_yolo26/weights/best.pt" \
  --runtime auto \
  --record-bag \
  --install-user-profile

# 2. 设置 YOLO 运行时
bash setup_yolo_runtime.sh --create --install \
  --runtime conda --env yolo26 \
  --model "$HOME/zhihang_yolo26/weights/best.pt"

# 3. 安装到工作空间
bash install_portable.sh

# 4. 运行诊断
bash doctor_portable.sh
bash benchmark_three_yolo_capacity.sh --iterations 20

# 5. 启动任务
bash launch_portable_formal_one_click.sh \
  --scene scene_001_adaptive_v6_7_19_formal \
  --model "$HOME/zhihang_yolo26/weights/best.pt" \
  --runtime-env yolo26 \
  --seed -1 \
  --record-bag
```

详细部署说明请参阅 [bundle/README_CN.md](bundle/zhihang_adaptive_search_v6_7_19_portable_bundle/README_CN.md)

## 测试结果

### 2026-07-24 正式测试

**任务时长**: 37.7 分钟  
**场景**: scene_001_adaptive_v6_7_19_formal

#### 动态目标 ✅ 完全成功

| 目标 | 起点 | 终点 | 跟踪次数 |
|------|------|------|----------|
| suv_camo | (599.8, -2.9) | (597.6, -0.8) | 481 |
| person_red | (1414.3, -0.3) | (1414.6, -0.4) | 2000 |

#### 静态目标 ⚠️ 部分成功

- **检测到**: 5/6 目标
- **已确认**: 0/6 目标
- **未检测到**: prius_hybrid_camo

详细测试结果请参阅 [docs/TEST_REPORT.md](docs/TEST_REPORT.md)

## 项目结构

```
zhihang_adaptive_search/
├── README.md                          # 本文件
├── bundle/                            # V6.7.19 便携式部署包
│   └── zhihang_adaptive_search_v6_7_19_portable_bundle/
│       ├── README_CN.md               # 中文部署说明
│       ├── configure_portable_machine.sh
│       ├── setup_yolo_runtime.sh
│       ├── install_portable.sh
│       ├── doctor_portable.sh
│       ├── launch_portable_formal_one_click.sh
│       └── zhihang_adaptive_search_v6/
│           ├── scripts/               # 核心脚本
│           ├── launch/                # ROS launch 文件
│           └── config/                # 配置文件
├── docs/                              # 文档
│   └── TEST_REPORT.md                 # 测试报告
├── test_results/                      # 测试结果
│   ├── competition_final_results.json
│   ├── evaluation.json
│   └── flight_results.json
└── scripts/                           # 辅助脚本
```

## 关键特性

### 动态目标跟踪

- **固定翼跟踪**: 高速跟踪移动目标
- **多旋翼精确定位**: 切换到多旋翼进行精确定位
- **目标重捕获**: 丢失目标后执行方形螺旋搜索重新捕获
- **轨迹预测**: 基于历史轨迹预测目标位置

### 静态目标定位

- **多帧融合**: 融合连续 3 帧检测结果提高置信度
- **射线-平面 intersection**: 基于相机姿态和地面平面计算目标位置
- **悬停精确定位**: 在目标上方悬停进行多次测量
- **空间一致性验证**: 验证多次测量结果的空间一致性

### 自适应搜索

- **配对引导**: 利用已检测目标引导搜索配对目标
- **残差路线**: 搜索未覆盖区域
- **动态调整**: 根据检测结果动态调整搜索路线

## 性能指标

- **YOLO 检测频率**: >10 Hz (每架无人机)
- **动态目标检测延迟**: <1 秒
- **静态目标定位精度**: <5m (水平)
- **任务执行时间**: ~35 分钟

## 已知问题

1. **静态目标确认失败**: 检测到但未确认静态目标
2. **任务超时**: 37.7 分钟超出 35 分钟限制
3. **漏检目标**: prius_hybrid_camo 未被检测到

## 改进方向

1. 优化静态目标确认逻辑
2. 提高搜索效率
3. 增强目标检测能力
4. 优化任务时间管理

## 许可证

本项目仅供学习和研究使用。

## 联系方式

如有问题或建议，请联系项目维护者。
