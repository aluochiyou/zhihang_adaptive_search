# V6.7.19 Portable 竞赛要求符合性矩阵

本矩阵依据用户提供的《2026年度中国青年科技创新“揭榜挂帅”擂台赛——XH-202632 无人机集群自主协同识别与态势融合参赛选手手册》整理。赛事官方文件、模型、world、通信脚本和消息工作空间仍以赛方原始版本为准，本包不复制或覆盖这些文件。

| 官方要求 | V6.7.19实现 | 运行证据/检查入口 |
|---|---|---|
| Ubuntu 20.04、Gazebo、ROS1 | 以ROS Noetic/catkin为目标平台；机器路径通过profile适配 | `doctor_portable.sh`、`source_zhihang_ros_env.sh` |
| 3架无人机全程自主 | 1个管理器、3个飞行代理、3个感知代理、3个独立YOLO进程 | `manager.launch`、`vehicle_terminal.launch` |
| 不随意修改官方文件和启动步骤 | 安装器只安装算法ROS包；对三项XTDrone通信文件做前后SHA-256校验 | `install_to_workspace.sh`、`NOTICE.md` |
| 目标运动代码启动后才能开始任务 | 管理器和三路YOLO就绪后才启动`model_state.py`并发布启动授权 | `launch_mission_formal_one_click.sh` |
| 起飞前开始记录`score1.bag` | 三路处理链就绪后先开bag，再开目标运动、再授权解锁 | `terminal_commands/06_score1_bag.sh` |
| bag必须含官方9个话题 | 脚本逐项固定记录三机state、Gazebo model_states、三机cmd和两类结果话题 | `competition_readiness_check.py` |
| 静目标结果：时间戳、类别、x、y；最多20条；首次消息为准 | 任务结束只生成一次最终静态列表；精确定位优先、可用检测定位保底；每个目标以`static_target`类别发布 | `competition_result_publisher.py`、`competition_final_results.json` |
| 动目标结果：模型名、起止时间与位置；最多20条；首次消息为准 | 每个动态目标提取首末有效轨迹点，一次发布 | 同上 |
| 官方自定义消息由`zhihang_ws`提供 | profile支持独立`ZHIHANG_MESSAGE_WS_SETUP`；桥接器按字段结构发现消息类型 | `machine_profile.example.env`、`competition_result_publisher.py` |
| 静态定位误差门限10m | 精准定位控制目标为30m高度、XY≤5m、Z≤1m、yaw=0、稳定3s | `adaptive_search_formal.yaml`、`vehicle_flight_agent.py` |
| 完成时间：首架armed至末架disarmed，门限35min | 32min搜索预算+3min返航预留；超时仍继续安全降落并告警 | `mission_manager.py`、`return_strategy`配置 |
| 协同效率：静目标50m覆盖区域 | 三机差异化航线、航迹覆盖采样、补漏和高风险区优先 | `common.py`、`mission_manager.py` |
| 科目二动态轨迹与全景态势 | 持续保存动态轨迹、静态精确结果、时间戳和定位报告，供态势图生成 | `final_results.json`、`evaluation.json`、分析脚本 |
| 实时性与时延分析 | 三个独立YOLO进程、GPU分配、640/512/416自适应、真实处理链≥10Hz屏障 | `benchmark_three_yolo_capacity.sh`、`wait_manager_application_ready.py` |
| 源代码、可执行程序可部署运行 | 完整ROS包、安装器、配置器、测试、诊断和启动器均在交付包中 | `validate_package.py`、`install_portable.sh` |

## 重要边界

1. 赛方自定义消息的**具体包名和消息类型名称**未在选手手册正文中给出，因此本包采用字段结构自动发现；也可在正式配置的`competition_output/outer_message_type`中显式填写。
2. 静态/动态官方话题由本包在任务结束时只发布一次最终结果，以避免“系统以第一次接收消息为准”导致早期粗定位覆盖最终精定位。
3. `/gazebo/model_states`仅按官方要求进入`score1.bag`。正式任务管理、目标定位、任务分配和飞行控制不以该目标真值作为输入。
4. 合规矩阵能验证代码与配置，不替代赛方环境中的实际飞行、GPU吞吐、消息类型和评分系统联调。
