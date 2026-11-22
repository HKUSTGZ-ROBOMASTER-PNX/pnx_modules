# 模块接入

含后台线程的完整服务统一放在 `modules`。当前目录保持 IMU 的采集、解码、姿态算法及
温控实现集中组织；裁判系统的组帧、协议定义和接收线程集中在 `referee`。
`devices` 不创建后台线程，模块单向依赖 `devices`、BSP 和基础库。

根工程链接 `modules`；其他应用可使用：

```cmake
target_link_libraries(my_app PRIVATE modules)
```

此 target 提供模块头文件和生成的 `user_config.hpp`。配置脚本只生成配置，不另建转发
target。各服务由应用显式调用对应启动入口，链接模块不会自动创建线程。

| 模块 | 启动入口 | 输出 |
| --- | --- | --- |
| BMI088 姿态 | `imu::start_bmi088(output)` | 应用提供的姿态 topic，使用 `msg` 订阅 |
| DMIMU 姿态 | `imu::start_dmimu(output, mutex)` | 同上；请求模式必须传 CAN 互斥量 |
| BMI270 姿态 | `imu::start_bmi270(output)` | 姿态 topic、反馈与校准快照 |
| BMI088 温控 | BMI088 服务负责启动；独立使用时调用 `imu::bmi088_heater::init()` | `imu::bmi088_heater::read()` |
| 裁判接收 | `referee::init(config)` | `referee::read(sample)` 按类型读取，`referee::read()` 获取链路诊断 |

`imu::attitude::output` 是 topic 类型，应用为每个 IMU 创建独立对象（如
`static imu::attitude::output output;`）并传入启动接口，
三个服务可同时运行，各自创建一个采集线程。启动调用须串行，topic 须存活到固件结束。
DMIMU 请求模式使用的共享互斥量也须存活到固件结束。
BMI088 在主板运行 ESKF；DMIMU、BMI270 使用外置板解算结果。配置集中在 `configs/user_config.json`。

CAN 服务须在应用初始化总线前启动以注册接收。BMI270 直接发送控制帧并消费总线错误信号量。
应用负责 CAN 初始化和故障恢复。

启动成功后在读取任务中订阅并保留订阅者，循环读取：

```cpp
auto subscriber = output.subscribe();
imu::attitude::state data{};
const auto result = subscriber.read(data);
```

订阅失败可重试；读取成功后检查 `online`、`stage`、`sequence` 和 `received_ms`，topic 只保留最新值。
角度为 rad、角速度为 rad/s、加速度为 m/s²（包含重力）。BMI088 / BMI270 四元数为 NWU Body-to-World、
WXYZ；DMIMU 沿用设备坐标约定。

温控固定使用板载 BMI088 加热资源；独立启动时传入温控参数、线程配置与静态线程栈：
`imu::bmi088_heater::init(thermal, config, stack, sizeof(stack))`。
两块板的保温 PID 分别在 `boards.<板型>.bmi088_thermal` 中配置 `kp`、`ki`、`kd` 和
`max_out`

H723 的实现在 `bmi088_heater.cpp` 内部独占 TIM3、TIM6 / DMA2 Stream6，实现断点加热保护。
启动加热时装载一次 DMA，正常温控更新仅刷新 250 ms 硬件倒计时；CPU 停在断点或线程停止
更新时，TIM6 触发 DMA 将 TIM3 的 CCER 清零，直接关闭加热输出。后续占空比写入不会
重新打开输出。恢复运行后丢弃旧温度，收到新温度后才重新启动加热；初始化或启动失败
时停止加热。调试器须保留初始化时解除的 TIM6/TIM3 调试冻结位；
`read()` 是线程发布的状态快照，CPU 暂停期间不会随硬件关断更新。
实际断点关断时序仍需上板用示波器验证。

F407 保留原有软件温控超时行为，不接入本次 H723 的硬件保护。当前 JSON 默认关闭 F407 加热。

## BMI270

通过 [bmi270.hpp](imu/bmi270/include/bmi270.hpp) 接入；`boards.<板型>.bmi270_can` 选择总线，
`bmi270` 配置协议参数，`ahrs` 配置线程优先级、栈和等待周期。
使用固定 1 Mbps Classic CAN 标准帧：`command_id=0x270`、`target_id=0x271`；
`0x271～0x274` 依次反馈 gyro、四元数、accel、Euler，`0x275` 为控制回复。

`bmi270.receive_gyro / receive_quaternion / receive_accel / receive_euler` 选择外置板发送的反馈，
禁用帧不参与在线判断。至少选择一类反馈，启动流程完成且所选反馈全部有效后 topic 才在线。
`imu::bmi270_read_feedback(sample)` 读取各帧最新缓存，使用时检查对应的 `*_valid`、`*_received_ms`
和 `*_updates`，各帧不保证同步；仅选 gyro / accel 时在线不代表姿态有效。

- `calibrate_on_start=true`：静止标定本次 RAM gyro 偏置，accel 偏差清零。
- `calibrate_on_start=false`：整组下发 `gyro_offset`（rad/s，每轴 ±0.15）和
  `accel_offset`（m/s²，每轴 ±100），外置板在融合前执行 gyro 减偏差、accel 加偏差，确认后运行。

两种方式均不写 Flash，也不在主板重复补偿；主板与外置板固件须配套更新。
在应用线程中用 `imu::bmi270_request()` 请求 `imu::bmi270::command::calibrate` 或 `set_offsets`，
通过 `bmi270_read_control(control)` / `bmi270_read_calibration(calibration)` 读取控制与最近完整状态
（命名空间均为 `imu`）。
命令应答超时可用 `bmi270_retry()` 保留原 token 重试；已知外置板重启后调用 `bmi270_reset()`。
这些接口和快照读取会等待 BMI270 工作线程处理队列请求。

## 裁判接收

`boards.<板型>.referee_uart` 选择接收串口；`referee` 配置接收线程的优先级、周期、
离线超时和栈大小。`period_ticks` 使用 ThreadX tick，`offline_timeout_ms` 使用毫秒，
`stack_size` 使用字节。服务独占 UART 接收，不能和遥控接收共用同一 UART。

接收流程：串口中断写入 FIFO → 接收线程组帧并校验 CRC → 按类型解析 → 更新缓存 →
调用该类型的可选回调。解析器独立维护帧长度和超时状态；线程负责有界消费、链路状态发布
和串口故障恢复。队列溢出或接收重启时丢弃未完成帧，保留已有缓存供应用判断新鲜度。

```cpp
#include "user_config.hpp"

pnxtypes::status start_referee()
{
    return referee::init(user_config::referee);
}

void control_step()
{
    referee::sample<GameRobotStatus_t> robot{};
    if (referee::read(robot))
    {
        // robot.data 是解析后的机器人状态。
        // 按业务要求检查 robot.received_ms 的新鲜度；sequence 可判断更新。
    }
}
```

线程自动解析并缓存协议头中已定义的 19 类固定长度数据和机器人交互数据。
用户以 `sample<协议类型>` 选择读取，首次收到该类型前返回 false，且不修改目标；
读取不会消费缓存，可供多个线程独立读取。每类数据分别记录接收时间和更新序号，
断线后保留最近值，不能仅凭其他包保持在线就认定所需数据仍然新鲜。
缓存只保留最新一包，不保证交付所有射击、受伤或交互事件。

需要逐帧处理时，在 `callbacks` 中只填写所需类型；回调直接接收解析后的数据，
不需要判断命令 ID、检查长度或复制协议结构：

```cpp
void on_shoot(const referee::sample<ShootData_t>& shot)
{
    // shot.data 是本次射击数据，直接处理业务。
}

pnxtypes::status start_referee_with_events()
{
    referee::callbacks events{};
    events.shoot_data = referee::receive_callback<ShootData_t>::bind<&on_shoot>();
    return referee::init(user_config::referee, events);
}
```

上述两种启动方式选其一。回调配置由服务复制，绑定的对象须保持固件生命周期。
回调在缓存更新后于接收线程执行，且已退出缓存临界区，不可阻塞；参数引用只在本次调用有效。
`referee.stack_size` 配置整个接收线程的栈空间（字节），默认 2048；自定义回调也使用这块栈，
需要按实际回调调用链调整并上板检查余量。
未注册回调的类型仍然解析和缓存。没有结构定义的命令跳过业务解析，不触发回调。
机器人交互数据缓存为 `sample<referee::robot_interaction_data>`，保留交互头及实际长度的载荷；
自定义交互内容仍由应用按自己的协议解释。已知类型长度错误的包不会覆盖缓存或调用回调。
服务独占所配置 UART 的接收，应用不能另行启动同一 UART 的接收或终止该线程。
`referee::read()` 无参数版本返回在线状态、接收时间及错误计数。
初始化结果锁存，重复调用不会重建线程或更换回调。

## RFID

应用在 ThreadX 初始化钩子（如 BSP 调用的 `app_start()`）或任务中显式调用
`rfid::init()`，并检查返回状态。仅链接模块不会自动启动 RFID。重复调用不会重复创建线程，
调用方须串行调用。串口操作在工作线程内执行，启动入口不等待读卡器或卡片。
设备目录只保留无后台线程的协议和蜂鸣器接口。

服务使用旧工程的 UART 读卡协议：9600 baud、8N1、默认地址 32，A3 命令读取块 4 的
16 字节 data，使用读卡器保存的 KEYA。只支持读数据，不提供 UID 或写卡功能。
默认 F407 使用 USART1（PA9 TX / PB7 RX）及 TIM4 CH3，H723 使用 USART10 / TIM12 CH2；必须按实际接线
修改 `boards.<板型>.rfid_uart / buzzer_timer / buzzer_channel`。串口独占收发，蜂鸣器
独占定时器，不可与遥控、裁判、UI 或温控冲突。F407 的 UI 与裁判串口默认设为 none，
避免占用 RFID 的 USART1。配置和资源校验由生成器统一完成。

`rfid` 配置组集中设置地址、块号、串口速率、线程优先级/栈和各时间参数。
接收中断只复制字节；服务线程组帧、检查地址/长度/成功状态/校验和，处理成功 data：

- 全 `0x01`：调用设备层 `remotes::set_enabled(false)` 并鸣叫一声。
- 全 `0x00`：恢复设备层遥控输出并鸣叫两声。
- 连续贴卡且状态相同时不重复鸣叫；首次有效全零也提示两声。
- 读块失败、应答超时或收发异常会重新允许下一次刷卡提示；同一张卡拿开再刷也会重新鸣叫，移卡本身不改变遥控权限。
- 混合内容、无卡、坏帧、溢出或超时保持已有权限；上电默认允许遥控。

DR16、VT03、PS2 的解码出口共用设备层权限。禁用期间即使传入坏帧，也返回中立输出，
摇杆、鼠标、按键清零，拨杆置 low；重新允许后使用后续解码数据。应用无需自行检查
RFID 权限。该机制约束后续解码输出，不能撤回应用之前复制的数据或已发出的电机目标。

`rfid::read()` 返回线程安全快照，包含 data、接收毫秒时间、序号、在线状态、收发和
蜂鸣状态；序号为零表示没有成功读取。缓存保留上次成功数据，使用时检查在线状态与
时间。线程、DMA 缓冲和回调存活到固件结束，无运行时停止接口。初始化失败会重试，
接收错误或超时后丢弃本次帧并等待静默恢复间隔。协议没有事务 ID，无法识别任意延迟
到下一请求期间的旧回复。蜂鸣错误记录在状态中，不阻止遥控权限更新或继续读卡。

主机 test 和应用 example 已删除。两种板型的默认构建验证单独报告；实际接线、读卡、
发声和机构动作仍需上板验证。

F407 已通过 CMSIS-DAP 完成烧录校验和在线断点验证：RFID 初始化成功，DMA 接收启动，
USART1 的工作模式查询收到有效应答。`__tx_ts_wait` 是所有线程暂时阻塞时的正常空闲循环，
不代表死锁。模块明确返回的读块失败记录为 error；没有应答才记录 timeout。
已在实卡块 4 上读到连续 16 字节 0x01，并确认底层遥控权限变为 false。
全零块 4 的恢复切换和实际发声仍待验证；此前全零读取来自块 1，不能替代该项验证。

读卡器未连接、没有放卡或卡被移开时，不阻塞系统启动或其他线程。启动入口只创建线程，
不等待模块应答；工作线程使用 RX DMA 和毫秒截止时间等待数据，每轮休眠一个 tick。
明确的读块失败结束本次请求，按 poll_ms 重试；无应答在 timeout_ms 后放弃，按
recovery_ms 等待恢复。缺席期间 online 为 false，缓存序号不增加，遥控权限保持原值，
不会把未收到数据的零缓冲区当成恢复卡。无 TX DMA 的 UART 仅允许一次 8 字节请求的
有界阻塞发送；默认 F407 USART1 使用 TX DMA。
