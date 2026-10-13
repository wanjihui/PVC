> 2026-10-08最新状态：已按用户要求恢复赛道循迹，APP_IMU_VERIFY_ENABLE=0，陀螺阻尼初调0.08；IMU、图像和日志独立上传，K1/K2仅控制运动。下文保留先前IMU验证过程与修复记录，当前运行见[赛道采集说明](视觉循迹实跑采集.md)。

# IMU验证过程与独立图像上传记录

2026-10-08。以本会话用户要求为准，在已有WiFi异步DMA和500ms起步改动基础上完成验证入口，不按其他智能体的回退建议撤销。

## 此前IMU验证时的烧录行为

config.h的APP_IMU_VERIFY_ENABLE=1。main仍调用app_init/app_poll，app.c已经补齐真实验证分支，不再是只改三个文件的半成品。

初始化：600MHz时钟、UART、电机PWM=0、IMU静止校准、GPT1连续微秒计时、WiFi；不初始化相机/编码器/视觉/按键/速度环，不启动PIT。主循环以GPT实际经过时间每约5ms调用imu_update，随后调用wifi_poll_imu_verify和wifi_poll_log。WiFi状态机需要的eDMA完成中断保留；app_tick_isr与app_keys_isr在此模式为空，避免重复采样和误发车。

5ms是主循环采样目标而非硬中断保证；初始化/重新入网仍同步，会影响采样间隔。缺采样不突发补读，imu_update按实际dt解算并记录迟采样。APP_IMU_VERIFY_TEXT_LOG_HZ=0，避免周期UART文本阻塞采样；初始化及异常日志保留。需要文本时可设1，在8082看到gz_raw、gyro_control、az与采样间隔，但同步printf会带来间隔抖动。

IMU初始化正常后，数据自动以50ms更新候选，无需K1，K2也不停止IMU传输。未就绪只显示初始化错误日志，不持续制造NaN参数包。8081仍为148字节兼容包，前36字节为三角度、三轴去偏角速度和三轴加速度；samples改为IMU采样计数，enc/target/speed/duty/视觉字段为0，WiFi耗时字段保留。不读取未初始化编码器/PID，不挪用原字段含义。8082继续补发初始化/异常日志。此模式没有调用摄像头，因此8083没有画面。

## 怎么核对符号

1. 编译/下载project/mdk/rt1064.uvprojx的nor_sdram_zf_dtcm。上电校准保持静止，确认日志MCAR_IMU_VERIFY、IMU OK、WIFI OK和READY。
2. 打开现有WiFiSPI_Monitor，01选择“综合验证（IMU + 编码器）”，监听8081参数与8082日志。02选gz_dps；03可看姿态模型。编码器字段为0是当前模式预期。
3. 整车放平，分别手动车头左转和右转，记录gz_dps正负；同时记录静止az_g是约+1还是-1，并说明IMU是否重新安装。只转车轮而车身不动时gz近0不能判断转向符号。
4. 8081的gz_dps是传感器原始轴方向，不乘IMU_MOUNT_YAW_SIGN；控制用imu_get_gyro_z_dps才乘安装符号并低通。应确认控制用角速度满足车头左转正、右转负，再开启陀螺阻尼。

## 此前纠偏与起步任务状态

原正常模式实际纠偏是视觉→角速度/横移指令→麦轮运动学→四轮编码器PI。IMU虽一直采集，ctl_gyro_kd=0，未参与电机纠偏。它不是四轮编码器速度反馈的来源。

已保留CAR_START_RAMP_MS=500：K1进入RUNNING，全程速度闭环，control_tick_isr每5ms推进vx，默认每拍3cm/s，第100拍150cm/s。新图像主循环只缓存视觉指令，5ms任务读取最新IMU做角速度阻尼，再分配四轮目标并执行PID；斜坡不依赖相机/WiFi刷新。停止清PWM、缓存和斜坡计时，下次K1重新从0开始。

当前验证模式不调用这些控制函数；ctl_gyro_kd仍为0，2026-10-08已按实测把IMU_MOUNT_YAW_SIGN改为-1，控制用角速度左正右负。ctl_gyro_kd仍为0，尚未开启陀螺纠偏；当前只验证IMU，不切回循迹或自行指定阻尼系数。

## 摄像头上传规则（已修订）

正常模式主循环只要持续capture/vis_deal产生新帧，就始终调用wifi_poll(now,new_frame)。WiFi对新帧取原始灰度+最终边界/中线/补线快照，仍每500ms最多启动一帧，17片异步DMA上传；K1/K2均不取消图像，停车时也能观察。图像与电机是否RUNNING无关。

正常参数仍只在RUNNING发送。wifi_cancel_data仅取消正常参数，wifi_cancel_camera仅用于显式关闭摄像头/通信，不再在K2停车时调用。图像整帧600ms期限和失败取消仍有效；停止调用相机不会持续重复制造旧帧，最多完成已经提交的快照。IMU验证入口未初始化相机，所以此次只传IMU；后续若恢复/增加主循环摄像头调用即可上传，不需要发车。

## 验证与恢复

应用模拟检查覆盖IMU正常/初始化失败/WiFi首次失败后重试，验证只由主循环采样、无PIT重复、保持IDLE/PWM禁止。实际wifi/异步驱动检查验证停车IMU包、初始化日志、原始gz、零轮速字段；实际vision/wifi/异步驱动检查验证停车图像、K2不取消在途图像、正常参数停止、超时及不可变17片快照。实际control/pid检查验证500ms斜坡、重启复位、停止立即清输出和非零陀螺反馈的反向纠偏行为（测试显式设系数，固件仍0）。这些是软件验证，不等同于实车。

最新固件构建日志build_imu_mount_full.log；仅编译，未自动下载。恢复循迹入口为APP_IMU_VERIFY_ENABLE=0，但应先完成符号确认及陀螺反馈配置。起步改前备份_startup_ramp_before_20261008_200541，验证开始前另有_imu_verify_before_20261008_201307。后者只含当时其他智能体备份的部分文件，不能视为当前所有文件的完整回退点。

## 2026-10-08收不到周期数据的修复

上位机实际日志已收到20:29:00构建的MCAR_IMU_VERIFY、IMU OK与WIFI OK，确认启动日志的WiFi链条至少已工作。明确代码缺陷是app验证入口只调用timer_init/timer_clear，没有timer_start。逐飞timer_init内部GPT_Init不设置EN位，timer_clear仅在已经启动时停/开，不能启动一个停止的GPT；所以GPT1计数保持0，verify_ms也为0，5ms IMU采样和50ms参数候选都不触发，初始化日志却可由无周期限制的flush请求发送。

已在app.c补timer_start(GPT_TIM_1)，同样补齐vision_init的GPT启动，恢复正常模式视觉/通信耗时统计。WiFi驱动、协议、网络目标与电机状态均未修改；不是把收到日志当成所有DMA功能均已验证。

此前模拟时钟自动递增，掩盖了漏启动。回归检查现区分初始化停止/显式启动/停止时不计数，应用和真实vision/wifi检查通过。Keil最新编译0错误0警告；修复前备份_imu_timer_fix_before_20261008_203226。尚需下载确认8081数据到达，若仍异常再按阶段日志排查，不宣称已经实车恢复。


## 2026-10-08空UDP包修正

20:37日志的IMU FAIL stage=3与20:43日志的IMU OK是两次不同启动，不能把后者“包长0”继续归因于校准。前者NaN是未就绪标记，后者则是有效IMU初始化后的发送问题。

原异步流程在DATA_SEND后的几个主循环才执行显式UDP_SEND，而模块已经在数据收到2ms后自动发包，迟到的命令会重复发送空UDP。驱动现在使用模块自动发送，在DMA结束后通过DWT实际时间保留发送/socket锁3ms，不延时等待、不发送重复命令；任何状态仍需继续调用wifi_poll_log推进。

IMU正常自动传输，不受K1/K2控制；当前模式app_keys_isr为空、Speed_PID_Enable=0、car_state=CAR_IDLE，所有PWM保持0。编译0错误0警告、真实驱动模拟包含2ms自动发包及20ms主循环停顿；未自动下载，需下载当前nor_sdram_zf_dtcm并核对新的built时间，观察samples持续递增、gz_dps响应以及日志不再出现“包长0”。


## 2026-10-08安装符号实测与处理

用户实测：传感器整个翻转安装，背面朝天、正面朝地；水平静止roll约-180°、pitch约-1°、yaw约-0.1°。原始gz左转负、右转正；gy抬头正、低头负，pitch变化相反；gx左倾负、右倾正，roll左倾降低、右倾升高。

姿态内核imu_attitude_reset使用roll=atan2(ay,az)，倒装时az朝负，初始化roll接近±180°符合代码。ZYX欧拉角的pitch变化率为gy*cos(roll)-gz*sin(roll)，roll接近±180°时约为-gy，所以pitch与gy相反并不意味着解算错误。roll变化率在小俯仰时接近gx，二者同向也与实测一致；roll跨±180°会出现数值跳变。

config.h中的IMU_MOUNT_YAW_SIGN已由+1改为-1。只在imu_update中的控制角速度低通分支使用，imu_get_gyro_z_dps供控制反馈，左转正、右转负；上传的gyro_dps[2]/gz_dps仍是传感器原始方向，三轴姿态、加速度和角速度没有单独反号，保持一套右手传感器坐标。未把传感器姿态变成车体姿态，因此静止roll仍约-180°，不是要求调到0°。

当前APP_IMU_VERIFY_ENABLE=1、文本日志频率0、ctl_gyro_kd=0；IMU自动上传、不受K1/K2控制，电机保持停止。工具review_imu_mount.py使用真实imu.c与模拟SPI输入，验证倒装初始化、原始gz保持不变及控制gz反号；姿态内核21项检查通过，包含正装/倒装时正gy对应不同pitch变化方向。硬件SPI与实车控制反馈效果未在软件模拟中验证。

同时修正完整重编译时的两个SDK警告：fsl_debug_console.c的浮点scanf分支比较HUGE_VAL，而原编译器默认浮点选项假定有限值。仅在MDK的MiscControls补-fno-finite-math-only，保留优化级别/FPU配置及SDK源码，不用关闭警告掩盖检查。单文件复现修改前2警告、修改后0警告；完整Rebuild日志build_imu_mount_full.log为0错误0警告，Code=54376，RO=7228，RW=100，ZI=49576。先前0警告是增量构建，未重编译这个SDK文件。

改前备份docs/_imu_mount_before_20261008_210333。当前AXF已生成，未自动下载。下载后原始gz仍左负右正属于预期；控制内部符号才改为左正右负。
