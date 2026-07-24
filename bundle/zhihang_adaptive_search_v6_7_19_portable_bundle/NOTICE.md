# 使用边界与第三方环境说明

本包包含项目自编的任务管理、三机飞行节点、三路感知节点、独立YOLO进程、视觉目标状态估计、官方结果话题桥接、验证工具、安装与启动脚本。

正式模式中，目标检测、目标定位、动态状态估计、任务分配和飞行控制使用YOLO结果及飞行器自身状态。`/gazebo/model_states`按赛事要求记录到`score1.bag`，但不作为正式规划、定位、分配或控制的目标真值输入。验证模式可启用独立真值relay进行状态机验证，两种数据源互斥。

本包不包含也不修改PX4、Gazebo、XTDrone、赛事world/模型、`model_state.py`、赛事`zhihang_ws`以及以下通信文件：

- `~/XTDrone/communication/vtol_communication.py`
- `~/XTDrone/communication/multi_vehicle_communication.sh`
- `~/XTDrone/communication/multi_vehicle_commonication.sh`

安装脚本覆盖算法ROS包前会建立时间戳备份，并对三项通信文件执行安装前后SHA-256一致性校验。赛事官方文件如需安装或替换，应严格按赛方手册执行，不应由本算法包代替。

YOLO权重、PyTorch、Ultralytics、OpenCV、ROS、PX4、Gazebo、XTDrone和QGroundControl分别受其原许可证约束。使用者需自行确认模型权重和训练数据的授权状态。
