# Changelog

## V6.7.19 Portable

### 跨电脑部署
- 新增`machine_profile.env`配置层，统一管理ROS工作空间、PX4、XTDrone、赛事消息工作空间、QGroundControl、终端、YOLO环境、GPU和端口。
- 支持`gnome-terminal`、`xterm`、`konsole`和`tmux`，可用于桌面或无头电脑。
- PX4赛事launch支持`auto`发现，优先官方`zhihang2026.launch`，兼容旧环境`zhihang20267.launch`。
- ROS环境按“Noetic基础→可选underlay→赛事zhihang_ws→算法工作空间”顺序加载。

### YOLO真实运行
- 支持conda、venv、system Python和直接Python解释器。
- 支持单GPU、多GPU、外部YOLO端点、CPU验证和每机独立设备配置。
- 三个YOLO进程分别服务三架无人机，支持640→512→416自适应输入尺寸。
- 新增真实模型类别检查、单进程验证、三进程并发容量测试和运行时环境清单导出。
- 正式任务继续要求三路真实相机/检测/定位链连续达到至少10Hz后才能开始。

### 竞赛输出与合规
- 新增`competition_result_publisher.py`，按赛事消息字段结构自动发现`zhihang_ws`消息类型。
- 任务结束只发布一次最终静目标和动目标官方话题，避免首次粗结果被评分系统锁定。
- 官方输出增加部分任务保底：静态精确定位优先、检测定位保底，动态轨迹不足时使用可靠跨机/最近视觉定位；单一分量为空不阻断另一分量。
- 静目标输出类别`static_target`；动目标输出`suv_camo`和`person_red`的起止点。
- 新增赛事符合性矩阵和官方9话题bag检查。

### 保留的V6.7.18核心能力
- 30m动态固定翼转旋翼门限；40m重捕获高度保持；90m等距方形螺旋、6m/s重捕获。
- 动态跟踪滤波、航向保护、目标丢失继续接近、外部可靠信息注入。
- suv_camo五秒运动核验、prius_hybrid_camo混淆隔离与角色回滚。
- 静态多候选聚类、逐组精准定位、错误候选原图保存、最终最高置信度保留。
- 静态精准定位30m、yaw=0、XY≤5m、Z≤1m、稳定3s，person_white向西偏移10m。
- 固定翼HOLD强制恢复、无效零速度setpoint保护、32+3分钟时间策略和独立返航降落。

## V6.7.18
- 修复启动参数前后空格和被引号保护的`~/...`模型路径。
- 在打开三架无人机终端前统一归一化并验证模型绝对路径。
