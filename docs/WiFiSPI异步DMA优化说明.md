> 2026-10-09更新：本文保留此前阶段说明，当前直弯同速、实时调参与收发调度以[本轮修订](实时调参与修订验证.md)和[控制原理](控制原理与实时调参说明.md)为准。

> 最新赛道入口（2026-10-08）：APP_IMU_VERIFY_ENABLE=0；IMU正常就发送兼容参数，摄像头新帧独立上传，K1/K2控制运动，停车仍有IMU和零目标/PWM。发送时序仍按下文自动发包修正。

> 2026-10-08发送时序已修正：逐飞模块收到SPI数据后2ms自动发UDP；异步驱动不再跨主循环补发UDP_SEND，避免产生空包。详见下文和IMU验证文档。后续门控已更新：摄像头只要主循环处理新帧就上传，不受K1/K2控制；当前入口为IMU专用验证。本文的最初K2取消图像策略已被替换，见[当前验证与独立图像说明](IMU符号验证与独立图像上传.md)。

# WiFiSPI异步eDMA优化说明

2026-10-08。针对赛道运行时WiFi同步握手/端口切换拖延视觉外环的问题，迁移CYT4BB7_Air的提交、poll、DMA思路。保持150cm/s直弯同速、现有视觉/PID、K1发车、K2中断停车、148字节参数包及MCAM格式2。

## 参考证据与适配范围

参考[CYT4BB7_Air national-2026驱动](https://github.com/ZhangStudyLife/CYT4BB7_Air/blob/national-2026/libraries/zf_device/zf_device_wifi_spi.c)。本机副本HEAD f4154ef；远端同路径原始文件与本机文件逐字节一致，留存于reference_wifi_spi_20261008/zf_device_wifi_spi.c。

参考驱动先复制发送数据，然后主循环wifi_spi_send_poll分步握手、等待DMA与检查应答，CYT4BB7使用PDMA DW1/SCB。其poll仍有wait_idle(1)，socket切换及UDP提交仍同步，DMA启动长度上限256字节。McarPVC使用RT1064 LPSPI1/eDMA，不能复制其寄存器、队列大小、数据包或耗时声明。本次进一步把端口切换和UDP提交也改为异步。

## 调用链与代码

- app.c：保持先按键、再视觉/目标；正常模式任何状态调用wifi_poll准备图像，IMU正常就同时准备参数；任何状态调用wifi_poll_log推进发送。可选IMU验证分支调用wifi_poll_imu_verify，IMU正常就上传，不受K1/K2控制。
- code/wifi/wifi.c：单在途UDP事务；最新148字节候选槽、不可变23040字节图像快照、8192字节日志缓存。日志优先，其次最新参数，最后图像分片。参数每50ms更新，图像每500ms最多启动一帧。
- libraries/zf_device/zf_device_wifi_spi_async.h：新异步接口，提交返回0只表示已复制，不表示电脑收到。
- libraries/zf_device/zf_device_wifi_spi_async.inc：由原驱动在mutex定义后包含，复用原协议/FIFO；不修改原同步API含义，不新增MDK源文件注册。

一次事务：若需换端口/上次中止，则CLOSE_SEND→CLOSE_REPLY→CONNECT_SEND→CONNECT_REPLY；随后DATA_SEND→可选DATA_DRAIN→AUTO_SEND_WAIT。只有应答为DATA_START才继续读模块收到的指令；不再额外发送UDP_SEND。每一步先看INT；未就绪立即返回。就绪时只启动一次LPSPI_MasterTransferEDMA，主循环继续执行。DMA完成回调拉高GPIO CS、记录数据传输完成时的DWT周期数并置完成标志；下一圈检查应答、推进状态。没有运行期wait_idle、阻塞SPI发送、延时轮询或同步socket切换。

模块原库明确规定：UDP模式收到数据后，2ms没有新数据就自动发包。原异步实现把显式UDP_SEND排到后续多个主循环，可能在自动发包之后再发送空UDP，这与上位机“包长0”一致。现在数据DMA结束后保留socket与发送锁至少3ms（包含1ms余量），期间poll立即返回；用DWT实际周期数计算，不用粗粒度PIT毫秒数，不重置与IMU共享的计数器。窗口未结束不能提交新数据或换端口，避免两包合并及当前缓存被丢弃/改发到其他端口。主循环晚回来直接结束窗口，不补发空命令。

TX/RX同时DMA：SPI发送本来同时产生接收字节，RX通道防止FIFO溢出。一次可接受1~1424字节UDP负载，覆盖1400字节图像分片+24字节头；短包按原库补够32字节接收空间，实际有效长度仍由头部记录。单包未完成不提交下一包、不改变目标端口。

初始化入网/初始socket以及停车状态入网重试仍使用原同步库；必要低频事件日志仍同步printf到UART。主循环原有1ms延时保留。图像快照/CRC、数据复制及状态检查仍占CPU，所以不能声称所有通信耗时为0或视觉必定50Hz。

## DMA资源与内存

LPSPI1 TX使用eDMA通道30，RX通道31，IRQ优先级3，源为kDmaRequestMuxLPSPI1Tx/Rx。只初始化/中止这两个通道，不调用EDMA_Init重置共享控制器。当前CSI相机使用自身CSI传输机制；FlexIO库另有通道0，本次不占它。以后若新增DMA外设需检查通道冲突。

DMA发送/接收数组和SDK句柄均32字节对齐，位于现有非缓存SDRAM段，避免默认DTCM不可访问以及缓存不一致问题。实际链接映射：TX句柄0x81E00000，RX句柄0x81E00020，LPSPI句柄0x81E00040，RX数组0x81E00100，TX数组0x81E006A0。SDK句柄内的软件TCD/命令字同样会被DMA读取，不能只迁移数组而留句柄在DTCM。

## 超时、门控与失败处理

单阶段期限200ms、总事务期限500ms，均通过板上时间跨圈检查，不在函数里等待。INT一直低、DMA不完成、连续DATA_START/非法应答会退出事务，关闭CS、释放mutex并记录错误；上层退避1秒，下一次发送先异步关闭/重建socket，以清除未提交的旧数据。

K2继续由原按键ISR清PWM。正常模式主循环保持IMU参数和摄像头独立上传；当前IMU验证模式不调用按键、不启动电机，参数也独立上传。初始化、STOP、错误日志独立发送。已提交到无线网络的包无法撤回，电脑可能晚到一包。

日志在数据SPI提交成功且3ms自动发包窗口结束后从缓存删除；这不是网络发送确认或电脑端ACK，UDP仍可能丢包。失败保留日志，恢复后补发；日志按最多1000字节分包，文本可能跨UDP边界，上位机现有逐行缓存处理它。参数候选忙时覆盖旧候选，seq在生成时递增，因此缺号也可能是主动覆盖/发送失败，不应全部算无线丢包。图像分片只有成功完成事务才增加编号，失败取消半帧；整帧超过600ms取消，电脑800ms未组齐丢弃。一次只保存最新候选与一帧图像，避免积压旧状态。

## 软件验证与实车边界

首次优化构建：Keil nor_sdram_zf_dtcm编译0错误、0警告，日志build_wifi_async.log；Code=81624，RO=7672，RW=248，ZI=142372。链接映射确认DMA对象位于0x81E…非缓存SDRAM。

- tools/review_wifi_async.py使用真实异步驱动、模拟SDK/GPIO/模块应答：1424字节、提交后原缓冲改写、忙INT/DMA立即返回、异步切端口/自动发包窗口、取消清理、非法接收长度、DMA启动失败/超时/中止和恢复。
- tools/review_vision_wifi.py使用真实vision/wifi/异步驱动，验证原始灰度同帧处理、补线标记、17片不可变图像、K2保留图像DMA但停止正常参数、整帧期限、停车发送初始化日志、日志失败保留/恢复成功才出缓存、最新参数候选覆盖。
- tools/test_framework.ps1覆盖20种应用配置/失败场景及编码器、真实按键、IMU、协议检查，全部通过。
- 上位机tests/test_protocols.py 13项通过，包含解析真实C生成的图像分片、CRC和补线颜色；本次无需更换上位机EXE。

这些是编译和模拟检查，没有自动下载、没有验证实际模块握手时序、无线吞吐或赛道改善。本次备份为_wifi_async_before_20261008_191402，含修改前驱动/应用、测试、文档和已编译AXF。

此前IMU验证模式：上电IMU OK、WIFI OK后自动上传8081参数，无需K1；K2不停止IMU上传，电机始终为0。当前正常循迹模式由K1/K2控制电机，参数/图像独立上传。若WIFI stage=5表示DMA配置失败，若UDP async result=2/3/4分别表示超时/协议/DMA错误，请保留日志。确认三路正常后再跑同一段赛道并保存CSV，比较vis_interval_us、vis_time_us、wifi_time_us和wifi_peak_us。图像目标仍2fps，先验证消除同步等待，再决定是否提高图像频率。


## 空包修正验证（2026-10-08）

更新tools/review_wifi_async.py的模块模拟器：真实模拟收到payload后2ms自动发包，延迟的显式UDP_SEND可发出空包。修改驱动前，原1424字节单包测试因实际产生两包而失败；修改后每包只产生一次数据，empty_datagrams保持0。新增5ms PIT跳变但实际仅1ms经过、主循环20ms停顿、跨端口锁定和接收FIFO检查，全部通过。真实wifi/IMU验证及vision/wifi模拟也通过，保持148字节协议与17片图像。

当前IMU验证构建：docs/build_imu_udp_fix.log，0错误0警告；Code=54336，RO=7228，RW=100，ZI=49576。备份docs/_imu_udp_fix_before_20261008_204434。未自动下载；上述均为软件验证，实际模块空包消除和IMU曲线仍需新版烧录后确认。
