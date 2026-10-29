# 模块接入

含后台线程的完整服务统一放在 `modules`。当前目录保持 IMU 的采集、解码、姿态算法及
温控实现集中组织；裁判系统的组帧、协议定义和接收线程集中在 `referee`。
`devices` 不创建后台线程，模块单向依赖 `devices`、BSP 和基础库。

根工程链接 `modules`；其他应用可使用：

```cmake
target_link_libraries(my_app PRIVATE modules)
```

此 target 提供模块头文件和生成的 `user_config.hpp`。配置脚本只生成配置，不另建转发
target。所有服务都需要应用在 CubeMX 初始化后、ThreadX 初始化钩子或线程中显式启动。

| 模块 | 启动入口 | 输出 |
| --- | --- | --- |
| BMI088 姿态 | `imu::start_bmi088()` | `imu::attitude::output`，使用 `msg` 订阅 |
| DMIMU 姿态 | `imu::start_dmimu()`；请求模式传共享 CAN 互斥量 | 同一姿态 topic |
| BMI088 温控 | BMI088 服务负责启动；独立使用时调用 `imu::bmi088_heater::init()` | `imu::bmi088_heater::read()` |
| 裁判接收 | `referee::init(config)` | `referee::read(sample)` 按类型读取，`referee::read()` 获取链路诊断 |

IMU 通过 `boards.<板型>.ahrs_device` 选择，CMake 仅编译一个姿态服务，选择 `none` 时不编译
姿态服务或温控线程。BMI088 选择额外链接 CMSIS-DSP；DMIMU 不运行 ESKF。
使用方式、资源所有权和错误恢复详见 [IMU 文档](imu/README.md)。

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
