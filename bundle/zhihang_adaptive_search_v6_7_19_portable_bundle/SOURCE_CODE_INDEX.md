# V6.7.19 Portable 完整源代码索引

交付包共包含 **123** 个代码、配置、测试、文档、图示与验证文件（不含本索引和最终哈希清单）。归档包即为“全部代码”交付；安装器只会将其中的ROS算法包复制到用户工作空间。

| 路径 | 大小(B) | 作用 |
|---|---:|---|
| `CHANGELOG.md` | 2407 | 版本变化记录。 |
| `COMPETITION_COMPLIANCE_MATRIX.md` | 3900 | 赛事要求与代码实现证据矩阵。 |
| `COMPETITION_READINESS_RESULT.json` | 3891 | 机器可读竞赛静态检查结果。 |
| `CORE_ALGORITHM_SCOPE.txt` | 1918 | 修改边界与V6.7.18算法保留范围。 |
| `NOTICE.md` | 1337 | 官方文件保护、真值防火墙和交付边界。 |
| `README_CN.md` | 2458 | 快速部署入口和交付摘要。 |
| `V6.7.19_PORTABLE_COMPETITION_MANUAL_CN.md` | 33383 | 完整跨电脑部署、YOLO真实运行、任务流程和故障说明书。 |
| `VALIDATION_RESULT.txt` | 1970 | 最终静态、单元和竞赛参数验证结论及实机验证边界。 |
| `VERSION` | 7 | 交付版本号。 |
| `abort_all.sh` | 210 | 运行、安装、检查或维护Shell脚本。 |
| `analyze_latest_run.sh` | 413 | 运行、安装、检查或维护Shell脚本。 |
| `apply_v6_7_18_hotfix.sh` | 256 | 运行、安装、检查或维护Shell脚本。 |
| `apply_v6_7_19_portable.sh` | 136 | 运行、安装、检查或维护Shell脚本。 |
| `authorize_manager_start.py` | 1624 | 运行、检查或辅助Python程序。 |
| `benchmark_three_yolo_capacity.sh` | 2937 | 三模型并发推理容量和显存/FPS评估。 |
| `check_topic_rate_once.py` | 2931 | 运行、检查或辅助Python程序。 |
| `check_yolo26_env.sh` | 1344 | 运行、安装、检查或维护Shell脚本。 |
| `competition_readiness_check.py` | 10011 | 赛事目标、计时、真值隔离、bag和官方结果桥检查。 |
| `competition_readiness_check.sh` | 568 | 运行、安装、检查或维护Shell脚本。 |
| `configure_portable_machine.py` | 18686 | 自动检测路径、终端、YOLO和GPU并生成机器profile。 |
| `configure_portable_machine.sh` | 248 | 运行、安装、检查或维护Shell脚本。 |
| `diagnose_formal_environment.sh` | 2034 | 运行、安装、检查或维护Shell脚本。 |
| `docs/V6.7.19_Portable_竞赛部署与任务说明书.docx` | 1076009 | 最终可编辑DOCX交付说明书。 |
| `docs/V6.7.19_Portable_竞赛部署与任务说明书.pdf` | 1311887 | 最终PDF交付说明书。 |
| `docs/images/architecture.dot` | 3406 | 说明书架构/流程图源文件或渲染图片。 |
| `docs/images/architecture.png` | 703475 | 说明书架构/流程图源文件或渲染图片。 |
| `docs/images/mission_flow.dot` | 2075 | 说明书架构/流程图源文件或渲染图片。 |
| `docs/images/mission_flow.png` | 370734 | 说明书架构/流程图源文件或渲染图片。 |
| `doctor_portable.sh` | 3794 | 目标电脑路径、ROS、消息、模型、GPU、端口和实时话题诊断。 |
| `export_yolo_runtime_manifest.sh` | 2988 | 运行、安装、检查或维护Shell脚本。 |
| `install_portable.sh` | 1789 | 运行、安装、检查或维护Shell脚本。 |
| `install_terminal_layout_dependency.sh` | 436 | 运行、安装、检查或维护Shell脚本。 |
| `install_to_workspace.sh` | 2013 | 运行、安装、检查或维护Shell脚本。 |
| `launch_base_environment_one_click.sh` | 2113 | 运行、安装、检查或维护Shell脚本。 |
| `launch_everything_one_click.sh` | 213 | 运行、安装、检查或维护Shell脚本。 |
| `launch_mission_formal_one_click.sh` | 5175 | 运行、安装、检查或维护Shell脚本。 |
| `launch_mission_four_terminals.sh` | 2935 | 运行、安装、检查或维护Shell脚本。 |
| `launch_mission_one_click.sh` | 829 | 运行、安装、检查或维护Shell脚本。 |
| `launch_portable_formal_one_click.sh` | 2050 | 跨电脑正式比赛总入口。 |
| `load_machine_profile.sh` | 5305 | 所有脚本共享的机器profile加载器。 |
| `machine_profile.example.env` | 2880 | 依赖、配置模板或文本说明。 |
| `preflight.sh` | 1085 | 运行、安装、检查或维护Shell脚本。 |
| `preflight_launcher.sh` | 3057 | 运行、安装、检查或维护Shell脚本。 |
| `preflight_manager.sh` | 758 | 运行、安装、检查或维护Shell脚本。 |
| `preflight_vehicle.sh` | 2026 | 运行、安装、检查或维护Shell脚本。 |
| `prepare_yolo26_model.sh` | 1264 | 运行、安装、检查或维护Shell脚本。 |
| `requirements_yolo26.txt` | 236 | 依赖、配置模板或文本说明。 |
| `requirements_yolo26_core.txt` | 50 | 依赖、配置模板或文本说明。 |
| `run_all_ros_nodes.sh` | 488 | 运行、安装、检查或维护Shell脚本。 |
| `run_detached_yolo_worker.sh` | 1174 | 运行、安装、检查或维护Shell脚本。 |
| `run_manager.sh` | 677 | 运行、安装、检查或维护Shell脚本。 |
| `run_manager_formal.sh` | 900 | 运行、安装、检查或维护Shell脚本。 |
| `run_plan_only.sh` | 1205 | 运行、安装、检查或维护Shell脚本。 |
| `run_vehicle_terminal.sh` | 5272 | 运行、安装、检查或维护Shell脚本。 |
| `run_vehicle_terminal_formal.sh` | 1091 | 运行、安装、检查或维护Shell脚本。 |
| `safe_cleanup_previous_run.sh` | 1437 | 运行、安装、检查或维护Shell脚本。 |
| `setup_yolo_runtime.sh` | 4072 | 创建、安装并验证conda/venv/system/direct YOLO运行环境。 |
| `shell_arg_utils.sh` | 1036 | 运行、安装、检查或维护Shell脚本。 |
| `source_zhihang_ros_env.sh` | 6592 | Noetic、underlay、zhihang_ws和算法工作空间确定性加载器。 |
| `start_yolo26_workers.sh` | 2034 | 运行、安装、检查或维护Shell脚本。 |
| `status_one_click.sh` | 753 | 运行、安装、检查或维护Shell脚本。 |
| `status_yolo26_workers.sh` | 586 | 运行、安装、检查或维护Shell脚本。 |
| `stop_yolo26_workers.sh` | 517 | 运行、安装、检查或维护Shell脚本。 |
| `terminal_commands/01_px4_gazebo.sh` | 2098 | 独立终端命令：01_px4_gazebo。 |
| `terminal_commands/02_guard.sh` | 246 | 独立终端命令：02_guard。 |
| `terminal_commands/03_xtdrone_communication.sh` | 1615 | 独立终端命令：03_xtdrone_communication。 |
| `terminal_commands/04_pose_ground_truth.sh` | 654 | 独立终端命令：04_pose_ground_truth。 |
| `terminal_commands/05_model_state.sh` | 747 | 独立终端命令：05_model_state。 |
| `terminal_commands/06_score1_bag.sh` | 699 | 独立终端命令：06_score1_bag。 |
| `terminal_commands/07_qgroundcontrol.sh` | 578 | 独立终端命令：07_qgroundcontrol。 |
| `terminal_commands/08_manager.sh` | 427 | 独立终端命令：08_manager。 |
| `terminal_commands/08_manager_formal.sh` | 426 | 独立终端命令：08_manager_formal。 |
| `terminal_commands/09_vehicle_0.sh` | 731 | 独立终端命令：09_vehicle_0。 |
| `terminal_commands/09_vehicle_0_formal.sh` | 745 | 独立终端命令：09_vehicle_0_formal。 |
| `terminal_commands/10_vehicle_1.sh` | 732 | 独立终端命令：10_vehicle_1。 |
| `terminal_commands/10_vehicle_1_formal.sh` | 746 | 独立终端命令：10_vehicle_1_formal。 |
| `terminal_commands/11_vehicle_2.sh` | 732 | 独立终端命令：11_vehicle_2。 |
| `terminal_commands/11_vehicle_2_formal.sh` | 746 | 独立终端命令：11_vehicle_2_formal。 |
| `terminal_launcher_common.sh` | 5665 | 运行、安装、检查或维护Shell脚本。 |
| `tests/test_detection_event_validator.py` | 4983 | 回归测试：test_detection_event_validator.py。 |
| `tests/test_formal_environment_loader.sh` | 4490 | 回归测试：test_formal_environment_loader.sh。 |
| `tests/test_portable_profile.py` | 2324 | 回归测试：test_portable_profile.py。 |
| `tests/test_portable_terminal_backend.sh` | 560 | 回归测试：test_portable_terminal_backend.sh。 |
| `tests/test_role_policy_and_maneuver_route.py` | 1479 | 回归测试：test_role_policy_and_maneuver_route.py。 |
| `tests/test_static_route_commitment.py` | 9262 | 回归测试：test_static_route_commitment.py。 |
| `tests/test_target_localization_and_report_validation.py` | 6427 | 回归测试：test_target_localization_and_report_validation.py。 |
| `tests/test_tracking_recovery_and_static_verify.py` | 5429 | 回归测试：test_tracking_recovery_and_static_verify.py。 |
| `tests/test_v6_7_15_motion_spiral_and_role_rollback.py` | 3324 | 回归测试：test_v6_7_15_motion_spiral_and_role_rollback.py。 |
| `tests/test_v6_7_16_dynamic_transition_gate.py` | 940 | 回归测试：test_v6_7_16_dynamic_transition_gate.py。 |
| `tests/test_v6_7_19_launcher_args.sh` | 838 | 回归测试：test_v6_7_19_launcher_args.sh。 |
| `tests/test_v6_7_19_safety_candidates.py` | 3213 | 回归测试：test_v6_7_19_safety_candidates.py。 |
| `tests/test_yolo_runtime_common.sh` | 909 | 回归测试：test_yolo_runtime_common.sh。 |
| `validate_package.py` | 9242 | 完整离线语法、单元和回归验证入口。 |
| `verify_install.py` | 4481 | 运行、检查或辅助Python程序。 |
| `verify_yolo_runtime.py` | 6562 | 运行、检查或辅助Python程序。 |
| `wait_application_terminal_ready.sh` | 1650 | 运行、安装、检查或维护Shell脚本。 |
| `wait_base_ready.sh` | 1680 | 运行、安装、检查或维护Shell脚本。 |
| `wait_manager_application_ready.py` | 2194 | 运行、检查或辅助Python程序。 |
| `wait_manager_start.py` | 2669 | 运行、检查或辅助Python程序。 |
| `wait_target_motion_ready.sh` | 725 | 运行、安装、检查或维护Shell脚本。 |
| `wait_xtdrone_communication_ready.sh` | 857 | 运行、安装、检查或维护Shell脚本。 |
| `yolo_runtime_common.sh` | 5117 | 运行、安装、检查或维护Shell脚本。 |
| `zhihang_adaptive_search_v6/CMakeLists.txt` | 746 | ROS包构建/安装元数据。 |
| `zhihang_adaptive_search_v6/config/adaptive_search.yaml` | 18621 | ROS任务参数配置：adaptive_search.yaml。 |
| `zhihang_adaptive_search_v6/config/adaptive_search_formal.yaml` | 18625 | ROS任务参数配置：adaptive_search_formal.yaml。 |
| `zhihang_adaptive_search_v6/launch/all_in_one.launch` | 2258 | ROS launch：all_in_one.launch。 |
| `zhihang_adaptive_search_v6/launch/manager.launch` | 1301 | ROS launch：manager.launch。 |
| `zhihang_adaptive_search_v6/launch/vehicle_terminal.launch` | 827 | ROS launch：vehicle_terminal.launch。 |
| `zhihang_adaptive_search_v6/package.xml` | 1720 | ROS包构建/安装元数据。 |
| `zhihang_adaptive_search_v6/scripts/competition_result_publisher.py` | 7799 | 最终官方自定义消息静态/动态话题桥。 |
| `zhihang_adaptive_search_v6/scripts/mission_manager.py` | 165858 | 任务规划、角色分配、候选管理、计时和最终官方结果汇总。 |
| `zhihang_adaptive_search_v6/scripts/summarize_adaptive_run.py` | 1816 | 运行、检查或辅助Python程序。 |
| `zhihang_adaptive_search_v6/scripts/summarize_target_localization.py` | 1813 | 运行、检查或辅助Python程序。 |
| `zhihang_adaptive_search_v6/scripts/validation_target_state_relay.py` | 1862 | 验证模式真值旁路，不参与正式控制。 |
| `zhihang_adaptive_search_v6/scripts/vehicle_flight_agent.py` | 131308 | 单机Offboard/VTOL/搜索/动态跟踪/精准定位/返航控制。 |
| `zhihang_adaptive_search_v6/scripts/vehicle_perception_agent.py` | 62276 | 单机图像、YOLO通信、定位、显示和证据保存。 |
| `zhihang_adaptive_search_v6/scripts/vision_target_state_estimator.py` | 10644 | 正式模式视觉动态状态融合。 |
| `zhihang_adaptive_search_v6/scripts/yolo26_single_worker.py` | 11157 | 独立YOLO推理TCP服务。 |
| `zhihang_adaptive_search_v6/setup.py` | 248 | ROS包构建/安装元数据。 |
| `zhihang_adaptive_search_v6/src/zhihang_adaptive_search_v6/__init__.py` | 48 | 运行、检查或辅助Python程序。 |
| `zhihang_adaptive_search_v6/src/zhihang_adaptive_search_v6/common.py` | 67058 | 任务规划与通用算法共享库。 |
| `zhihang_adaptive_search_v6/src/zhihang_adaptive_search_v6/target_localization.py` | 42974 | 相机几何、多帧一致性和目标定位共享库。 |
| `zhihang_adaptive_search_v6/src/zhihang_adaptive_search_v6/tracking_recovery.py` | 17465 | 动态跟踪滤波、预测与方形螺旋重捕获共享库。 |

## 受保护且不在包内的官方文件

- `XTDrone/communication/vtol_communication.py`
- `XTDrone/communication/multi_vehicle_communication.sh`
- `XTDrone/communication/multi_vehicle_commonication.sh`
- 赛事PX4/Gazebo模型、world、launch、`model_state.py`和`zhihang_ws`

每个包内文件的最终SHA-256见`SHA256SUMS.txt`。
