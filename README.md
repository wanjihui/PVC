> 当前（2026-10-08 晚）已恢复赛道循迹：APP_IMU_VERIFY_ENABLE=0，K1发车/K2停车，500ms加速至150cm/s并启用弯道降速，转向环参数全部集中在 `code/config/config.h` 的 `CTL_*` 段；图像上传已提速（周期100ms + 突发4片/20ms + 主循环有界补发）。IMU、图像与日志独立上传。见[实跑采集说明](docs/视觉循迹实跑采集.md)。

# PVC · RT1064 四电机麦轮循迹小车

McarPVC 是基于 NXP i.MX RT1064 的四电机麦克纳姆轮循迹工程，使用总钻风 MT9V03X 摄像头获取赛道图像，通过麦轮运动学分配四个车轮的运动指令，并采集 ICM20602 姿态与角速度供实跑分析（已按实测安装符号启用初调陀螺阻尼）。

底层基于逐飞科技 RT1064 开源库，视觉算法移植自个人先前的 `summer-smartcar` 双电机加舵机项目。当前模块联调已通过，开始使用统一的正常运行框架。

## 当前版本

**2026-10-08当前入口：** main持续调用app_poll，APP_VISION_TRACKING_ENABLE=1恢复视觉循迹。车体vx在500ms内升到150cm/s，弯道按 `|err|` 自动降速（最多降45%）；固定前瞻55行；视觉转向经麦轮运动学分配四轮Target，5ms中断执行独立PI。上电等待K1，K2在10ms扫描中断消抖后直接清PWM，不依赖WiFi返回。

**发送时机：** 上电连通即提交初始化日志；K1/K2只控制运动；IMU正常就持续上传148字节参数包，停车目标/PWM为0；摄像头只要主循环处理新图像就上传，不受K1/K2门控，必要事件日志独立发送。图像每100ms最多启动一帧（目标10fps），由异步状态机逐片上传，并允许一圈内补发多片；LPSPI1使用eDMA TX30/RX31，运行期端口切换和UDP提交不原地等待。画面为原始灰度加最终边界、中线及补线，固件和新版上位机配套使用。详见[视觉循迹实跑采集](docs/视觉循迹实跑采集.md)。

**运行参数（全部在 `code/config/config.h`）：**

- 转向环：`CTL_OMEGA_KP1=1.60`、`CTL_OMEGA_KP2=0.012`、`CTL_OMEGA_KD=0.30`、死区 2px、限幅 `CTL_OMEGA_MAX_DPS=240`、陀螺阻尼 `CTL_GYRO_KD=0.20`
- 弯道降速：`CTL_CORNER_ERR_PX=100`、`CTL_CORNER_SLOW_RATIO=0.45`
- 四轮速度环：Kp=0.20、Ki=0.0030、Kd=0、前馈 Kff=0.02
- 巡航：`CAR_TARGET_SPEED_CMS=150`、`CAR_START_RAMP_MS=500`
- 符号：`IMU_MOUNT_YAW_SIGN=-1`（控制角速度左正右负，上传的原始 gz 仍左负右正）、`CONTROL_IMAGE_TURN_SIGN=-1`

调参顺序与"症状→参数"对照见下方 [转向与速度调参](#转向与速度调参) 一节。

**滚子安装方向：** 用户确认俯视且车头朝上，FL/RR为左上—右下，FR/RL为左下—右上，与当前逆运动学一致；四轮电机前进方向补偿不变。

用户提供L=20cm、W=18cm、周长19.5cm。编码器已回退逐飞原库一倍频（2026-10-07），按1024线、68/30齿传动计算119.02906 count/cm，速度cm/s、距离cm；原始累计值仍是整数计数。换算接口为 `encoder_set_counts_per_cm()` / `encoder_get_counts_per_cm()`，不再使用每米参数。2026-10-07手动转轮确认原FL/FR对应反了，已交换前轮硬件通道：FL=C2/C24，FR=C0/C1；方向补偿随通道交换为1/0/1/0。电机输出映射未改。

当前视觉处理和四轮轮速内环均已启用。

正常循迹入口统一调用应用层初始化和调度。上电等待，KEY1发车、KEY2清PWM停车；IMU和编码器在5ms PIT采样，主循环处理视觉、控制决策和WiFi。接收端口为8081参数、8082必要日志、8083图像。详见[正式运行框架说明](docs/正式运行框架说明.md)。

编码器采样与控制策略分开配置：四路计数一直更新。UDP参数包扩展为148字节，兼容原108字节前缀，追加视觉偏差、车体指令与处理/通信耗时。新版上位机选择“视觉循迹”预设。

| 配置 | 默认值 | 当前行为 |
|---|---|---|
| `APP_VISION_TRACKING_ENABLE` | `1` | app_poll处理新帧并更新视觉外环，0可退回独立轮速调试 |
| `APP_IMU_VERIFY_ENABLE` | `0` | 1=只跑 IMU+WiFi 的符号验证固件（不初始化相机/编码器/按键，不需 K1） |
| `APP_DRIVE_ENABLE` | `1` | 初始化通过后允许KEY1发车，上电仍保持停止 |
| `ENCODER_CLOSED_LOOP_ENABLE` | `1` | 四轮独立速度PI每5ms执行 |
| `MOTOR_OPEN_LOOP_GAIN` / `_MAX` | `0.10f` / `20` | 开环调试映射与 ±20% 限制（闭环路径输出上限是 `MOTOR_DUTY_MAX`） |
| `WIFI_DATA_PERIOD_MS` | `50` | UDP参数上传目标20Hz |
| `WIFI_IMAGE_PERIOD_MS` | `100` | 图像上传目标10fps（原 500 = 2fps） |
| `WIFI_CAMERA_BURST_CHUNKS` / `_MS` | `4` / `20` | 图像分片突发：连续多发几片再让路给参数包 |
| `WIFI_CAMERA_DRAIN_STEPS` / `_US` | `4` / `8000` | 主循环一圈内最多补发几步分片（1=关闭） |
| `APP_MAIN_LOOP_DELAY_MS` | `1` | 主循环末尾延时；0=全速（图像上传更快） |

目标150cm/s仍需实测响应验证；轮速闭环不等于车身航向闭环（转向由视觉外环 + 陀螺阻尼共同产生）。停车操作会立即撤掉PWM。

## 硬件与接口

| 硬件 | 用途 |
|---|---|
| RT1064 主板 | 主控，程序初始化为 600 MHz |
| 总钻风 MT9V03X | CSI 并口摄像头，188 × 120 灰度图像 |
| 两块双路 DRV8701E | 四路有刷电机 DIR + PWM 控制，PWM 频率 17 kHz |
| ICM20602 | SPI4 六轴 IMU，当前控制主要使用 Z 轴角速度 |
| 四路正交编码器 | 一倍频采样、厘米换算；四轮速度闭环已启用（Kp/Ki/前馈在 config.h） |
| USB 转 TTL 串口模块 | 接收UART1备份事件日志，可选 |
| 四个按键 | 当前使用 KEY_1 发车、KEY_2 停车 |

四轮编号和接口如下，轮子顺序在控制、运动学和遥测中保持一致：

```text
           车头
    M1 左前     M2 右前
    M3 左后     M4 右后
```

| 车轮 | DIR | PWM |
|---|---|---|
| M1 / FL 左前 | C9 | C8 |
| M2 / FR 右前 | C7 | C6 |
| M3 / RL 左后 | D2 | D3 |
| M4 / RR 右后 | C10 | C11 |

ICM20602 使用 SCK=C23、MOSI=C22、MISO=C21、CS=C20；该接口由库中的 `zf_device_icm20602.h` 定义。电机和编码器的方向、底盘尺寸及控制参数在 `code/config/config.h` 中配置。

## 编译与下载

1. 使用 Keil MDK 5 打开 `project/mdk/rt1064.uvprojx`。
2. 选择工程目标 `nor_sdram_zf_dtcm`，使用 ARM Compiler 6。本机已使用 ARMCLANG V6.24 完成编译；工程引用的设备包为 `NXP.MIMXRT1064_DFP.15.0.0`。
3. 确认设备为 `MIMXRT1064DVL6A`，执行 Build；工程已包含 SDK、逐飞库、业务源码和分散加载文件。
4. 在 Debug / Utilities 中选择与 RT1064 匹配的 ARM CMSIS-DAP 或 J-Link，确认调试连接和 Flash 下载配置后烧录。

`libraries` 中的 `.a`、`.lib` 文件属于工程依赖，需要保留。`Objects`、`Listings` 等编译结果不纳入版本管理，编译时会重新生成。

## 控制流程

```text
摄像头灰度图像
    ↓
Otsu 阈值、二值化、连通滤波、边界搜索、补线
    ↓
赛道中线与视觉偏差 err
    ↓
转向环 ω = −(KP1·e + KP2·e|e| + KD·Δe)     ← 视觉外环，约 50 Hz，随出帧更新
    ↓
5 ms 合成：vx 弯道降速 → ω 叠加陀螺阻尼并限幅 → 麦轮逆运动学
    ↓
FL / FR / RL / RR 四轮目标速度
    ↓
四轮独立速度 PI + 前馈 → PWM（17 kHz，DIR + PWM）
```

| 任务 | 周期 | 主要工作 |
|---|---|---|
| 主循环视觉与决策 | 随摄像头出帧，约 50 Hz | 图像处理、运行状态推进、运动指令和遥测 |
| PIT_CH0 | 5 ms / 200 Hz | IMU 更新、电机输出；四路计数持续采样，闭环启用后执行轮速 PID |
| PIT_CH1 | 10 ms / 100 Hz | 按键扫描 |

三个时间尺度分工：**视觉外环（~50 Hz）决定往哪转**；**5 ms 合成决定转多少（限幅/陀螺阻尼/弯道降速）**；**5 ms 轮速环决定四个轮子各自多快**。调参时先动外环增益（`CTL_OMEGA_KP1`），再动限幅与降速（`CTL_OMEGA_MAX_DPS` / `CTL_CORNER_*`），最后才动轮速环（`SPEED_*`）。

车体坐标约定为 `vx > 0` 前进、`vy > 0` 左移、`omega > 0` 俯视逆时针左转；`gyro_z` 也统一为左转时为正。

视觉偏差定义为“赛道中线列坐标减去画面中心”。正常未镜像图像中，正偏差表示赛道位于画面右侧，应右转，因此默认 `CONTROL_IMAGE_TURN_SIGN=-1`。横移增益当前默认为 0，先使用前进和自转完成循迹。

KEY_1 发车，KEY_2 停车。连续出界达到 3 帧，或检测到第 2 次斑马线上升沿，也会触发停车；每次重新发车会重置本次运行的视觉停车计数。

当前IMU统一采集六轴数据并进行四元数姿态解算；固定三轴校准用于姿态和上位机，控制Z轴保留静止零偏跟踪与低通滤波。yaw是上电相对角，不是绝对航向。

## 上位机通信

新版上位机同时监听8081参数、8082事件日志和8083摄像头图像。当前参数包为148字节小端格式，包含姿态、三轴角速度/加速度、四轮累计计数/目标/速度/占空比以及视觉指令和耗时；摄像头使用MCAM v1。WiFi目标、端口及上传周期在 `code/config/config.h` 配置。UART1 B12/B13、115200、8N1仅保留启动与事件日志备份，旧20通道FireWater输出已移除。

实跑数据用 `tools/analyze_capture.py`（本地工具，按 .gitignore 不入库）判读：`python tools/analyze_capture.py` 会自动切出"电机有输出"的实跑段，打印 `|err|` 趋势、ω 顶限幅比例、转向达成率（真实 yaw 速率 / 指令）、四轮跟踪误差、占空比峰值，以及视觉单帧耗时与 WiFi 服务耗时。

## 目录结构

```text
PVC/
├── code/
│   ├── config/       硬件开关、引脚、底盘尺寸、转向/速度/相机等全部可调参数
│   ├── vision/       图像处理、搜线与视觉事件（err 来源）
│   ├── motor/        电机驱动与麦轮逆运动学
│   ├── encoder/      四路编码器与 cm/s 换算
│   ├── imu/          ICM20602 初始化、标定、四元数姿态与角速度滤波
│   ├── pid/          四轮增量式速度 PI + 前馈
│   ├── control/      状态机、转向环、弯道降速、停车与遥测
│   ├── buttons/      按键扫描与命令
│   └── wifi/         UDP参数、事件与图像上传（分片/突发/补发调度）
├── libraries/        逐飞库、NXP SDK 与第三方组件
├── project/
│   ├── mdk/          Keil 工程、下载配置与分散加载文件
│   └── user/         主入口 main.c、应用调度 app.c、中断服务 isr.c
├── .gitignore
└── README.md
```

本地 `tool`、`tools`、`docs` 目录不上传。使用说明集中在此 README，业务配置以源码中的宏和参数为准。

## 后续标定与闭环

四路采样、WiFi 与四轮速度闭环已经常态接入（`ENCODER_CLOSED_LOOP_ENABLE=1`）。
仍需补齐的标定：每轮起转/最小维持占空比与电池电压—速度曲线（流程模板见 `docs/四轮起转占空比测试.md`）、
陀螺零偏与噪声、六面加速度校准。控制参数按实跑数据迭代，判读方法见上一节的 `tools/analyze_capture.py`。

## 来源与验证状态

- 底层驱动和工程基座：逐飞科技 RT1064 开源库，参考编码器、DRV8701E、ICM20602、MT9V03X 与 printf 官方示例。
- 视觉算法：个人 `summer-smartcar` 项目，适配到 RT1064 与麦轮控制流程。
- 底层库许可证见 [libraries/LICENSE](libraries/LICENSE)，SDK 等第三方组件以各文件原有声明为准。

截至 2026-10-08，当前配置已在本机使用 Keil ARMCLANG V6.24 完成编译，结果为 **0 errors、0 warnings**（Code=82536）。编译结果不代表完成实车标定。

2026-10-08 WiFiSPI参考CYT4BB7_Air迁移为RT1064异步eDMA发送，说明、资源占用及实车检查步骤见[WiFiSPI异步DMA优化说明](docs/WiFiSPI异步DMA优化说明.md)。同日晚的实跑（`数据记录_20261008_212130`）显示 `|err|` 由 55px 增到 86px 并出界、转向达成率仅 73%、峰值占空比仅 21%，据此完成了一轮转向参数整定与相机上传提速。

## 转向与速度调参

转向环公式（`code/control/control.c` 的 `omega_calc` + `control_tick_isr`）：

```text
e      = 赛道中线 − 画面中心 (px)，e>0 表示赛道在画面右侧
ω_vis  = TURN_SIGN * ( KP1*e + KP2*e*|e| + KD*(e − e上次) )        TURN_SIGN = −1
ω_out  = clamp( ω_vis − GYRO_KD*gyro_z , ±OMEGA_MAX )              gyro_z 已归一化为左转正
rot[cm/s] = 0.3316 * ω[deg/s]        FL/RL = vx − rot，FR/RR = vx + rot
转弯半径 R = v/ω：150cm/s 时 120°/s → 72cm，240°/s → 36cm（弯道降到 82cm/s 则为 20cm）
```

| 症状 | 先改 | 再改 | 说明 |
|---|---|---|---|
| 直道画龙/左右摆 | `CTL_GYRO_KD` ↑ | `CTL_OMEGA_KD` ↑、`CTL_OMEGA_KP1` ↓ | 先加阻尼再压增益 |
| 入弯冲出去、转不过来 | `CTL_OMEGA_MAX_DPS` ↑ | `CTL_CORNER_SLOW_RATIO` ↑、`CAR_TARGET_SPEED_CMS` ↓ | R = v/ω，两条路都有效 |
| 打了方向但车头不跟 | `SPEED_KP_*` / `SPEED_KI_*` ↑ | `SPEED_FF` ↑ | 差速没落到轮上 |
| 小偏差不修、贴不住线 | `CTL_OMEGA_KP1` ↑ | `CTL_DEADBAND_PX` ↓（2→1） | 注意 1 计数/5ms = 1.68cm/s 的量化地板 |
| 出弯过冲 | `CTL_OMEGA_KD` ↑ | `CTL_OMEGA_KP2` ↓ | 微分抑制超调 |
| 轮速乱抖/啸叫 | `SPEED_KP_*` ↓ | `SPEED_KI_*` ↓ | 内环振荡 |
| 起步打滑、起步画龙 | `CAR_START_RAMP_MS` ↑ | `SPEED_FF` ↓ | 起步斜坡 |

**调参纪律**：一次只改一个参数 → 跑 2~3 圈 → `python tools/analyze_capture.py` 看五个指标
（`|err|` 均值与趋势、ω 顶限幅比例、转向达成率、四轮跟踪误差、占空比峰值）→ 记录改动与效果。

**安全约束**：符号类开关（`CONTROL_IMAGE_TURN_SIGN`、`IMU_MOUNT_YAW_SIGN`、`MECANUM_Y_SIGN`、`MECANUM_W_SIGN`）只允许 ±1，改动前先把车轮架空验证；**不要用 `MOTORx_DIR_INVERT` / `ENCODERx_INVERT` 去"修"转向**——那会同时破坏前进与自转的一致性，编码器符号接反会让速度环变成正反馈（飞车）。
