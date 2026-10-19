# IMU 姿态服务

在 `configs/user_config.json` 的 `boards.<板型>.ahrs_device` 选择 `bmi088`、`dmimu` 或
`none`。CMake 只编译选中的设备服务，BMI088 额外链接 CMSIS-DSP。服务不会自动启动；
在应用自己的 `app_start()` 或线程初始化流程中调用 `imu::start_bmi088()` 或
`imu::start_dmimu()`，没有统一启动包装。共用消息和 ESKF 位于 `attitude/include`，
ESKF 本身不访问硬件、ThreadX 或 `msg`。
服务与底层驱动集中在 `modules/imu`，应用链接 `modules` 即可使用这些头文件。
`ahrs` 和 `bmi088_heater` 的线程栈参数以字节计，调度周期中的 `ticks` 以 ThreadX tick 计；
温控与恢复配置中的 `ms` 以毫秒计，不能假定一个 tick 恒等于一毫秒。
选择 `none` 时不编译 AHRS 实现，应用也应移除对应调用，或用 `USER_AHRS_DEVICE != 0`
条件编译。以下示例用于已启用 AHRS 的配置。

```cpp
#include "attitude.hpp"
#include "user_config.hpp"

msg::subscriber<imu::attitude::state> attitude;

extern "C" void app_start()
{
#if USER_AHRS_DEVICE == 1
    const auto started = imu::start_bmi088();
#elif USER_AHRS_DEVICE == 2
    const auto started = imu::start_dmimu(); // 主动上报模式。
#else
    return;
#endif
#if USER_AHRS_DEVICE != 0
    if (started != pnxtypes::status::ok)
    {
        return; // 按应用要求处理启动失败。
    }
    attitude = imu::attitude::output.subscribe();
    if (!attitude)
    {
        return;
    }
#if USER_AHRS_DEVICE == 2
    // 先注册所有共享 CAN 订阅者，再由应用统一启动总线；检查返回状态。
    const auto result = bsp::can::init(user_config::dmimu_can);
    if (result != pnxtypes::status::ok)
    {
        return;
    }
#endif
#endif
}

// 在控制线程中调用；订阅对象持续保留，不要每轮重新订阅。
void control_step()
{
    imu::attitude::state sample{};
    if (attitude.read(sample) == pnxtypes::status::ok && sample.online &&
        sample.stage == imu::attitude::phase::running)
    {
        // 使用 sample.quaternion、sample.gyro 等。
    }
}
```

启动成功只表示工作线程启动，初始化结果会锁存，后续调用不会重新绑定资源。
`imu::attitude::output` 只允许选中的设备服务发布，应用负责订阅，不应同时发布其他来源。
`output` 为最新值主题，不保证逐帧交付；读取失败时不要继续使用未验证的新数据。
缓存旧输出的控制逻辑还须按 `received_ms` 检查新鲜度。`sequence` 只在有效姿态更新时递增；
错误和离线状态也会发布。线程与资源保持固件生命周期，不支持外部直接终止线程。

## BMI088

底层接口仍可单独使用：包含 `bmi088.hpp`，以 `bmi088_config` 提供 SPI 和两个片选，
构造 `imu::bmi088` 后调用 `configure()`，检查返回状态后读取 `read_acc()`、`read_gyro()`
和 `read_temperature()`。单位依次为 m/s²、rad/s 和摄氏度。所有操作由同一线程串行调用；
启动姿态服务后，不可再用另一个实例访问同一传感器。

手动校准时，每轮将新采样传给 `calibrate()` 一次；返回 `busy` 表示继续采集，
`ok` 或 `error` 表示本轮结束。采集期间保持 `gyro_offset` 不变，不传入安装补偿后的数据。
`shutdown()` 不负责关闭独立温控服务，停止采样前应先禁用加热并确认其停止。

`bmi088/src/bmi088_service.cpp` 独占 SPI 采样，先配置芯片，启用温控时等待温度进入稳定带，再进行
可选的启动静止校准，最后使用 ESKF 解算。读取失败、预热超时或采样间隔超过 `max_dt`
会禁用加热并发布故障，等待 `retry_period_ms` 后重新配置。静止校准失败也走此恢复流程，
不会把运动数据当作零偏。恢复后 ESKF 重新对齐，连续航向从新的起点累计。

ESKF 输入为 rad/s 和 m/s²，使用 DWT 实际时间间隔。输出四元数为 WXYZ、传感器坐标到
NWU 世界坐标；欧拉角和累计航向单位为弧度。六轴滤波没有绝对航向观测，不能保证长期航向
不漂移。`gyro_bias` 是 ESKF 估计的剩余零偏，发布的 `gyro` 已扣除它。

`ahrs.eskf` 集中配置噪声、静止检测、冲击抑制和初始协方差参数；`gyro_offset`、
`accel_offset` 是传感器静态偏置，默认保留原驱动数值，须按实际传感器校准。
`calibrate_on_start` 默认关闭；开启时会在预热后累计 4000 次读取。调度周期使用 ThreadX
tick，不等于固定的 1 ms，需根据板级 tick 频率配置，不能超过芯片输出频率重复累计样本。

温控实现位于 `modules/imu/bmi088/src/bmi088_heater.cpp`，独立线程仅管理 PWM，不访问 SPI。
AHRS 提交新温度，设备服务负责 PID、超时停止和稳定判定，参数在 `bmi088_heater` 和各板
`bmi088_thermal` 中。温控线程有独立优先级和栈；传感器或 AHRS 恢复不重建温控线程。

## DMIMU

底层接口位于 `dmimu.hpp`。`configure()` 只保存本地配置，不启动 CAN 或修改设备参数。
应用将中断收到的 CAN 帧及接收时间复制到队列，在所属线程调用 `receive()`，再用
`take_sample()` 取出完整数据；组帧需要加速度、角速度、欧拉角和四元数四类帧。
队列丢帧后调用 `clear()`，避免拼接不连续数据。使用姿态服务时，这些工作由服务完成。
`read_register()` 和 `write_register()` 返回发送状态，设备确认需通过 `take_reply()`
另行检查；不能将发送成功当作设备配置成功。

`dmimu/src/dmimu_service.cpp` 从 CAN 中断复制目标帧到有界队列，在线程内组装、检查时效并发布。
它直接使用设备层提供的四元数和设备上报的欧拉角，不运行 ESKF，也不重复计算欧拉角。
四元数检查和归一化由设备层在组帧完成时处理。
队列溢出会丢弃未完成组装并重新同步，`dropped_frames` 统计丢帧。

主动模式要求设备预先配置为输出加速度、角速度、欧拉角及四元数四类帧。请求模式每轮发送
一个寄存器读取请求，避免瞬间耗尽 F407 的发送邮箱；四个请求和应答需落在组装时间窗内。
设备设置及 ACK 验证由应用负责，AHRS 不自动写入或保存设备参数。

先调用 `imu::start_dmimu()` 注册回调，再由应用初始化共享 CAN。服务不停止、重启或重新配置
共享总线；总线故障恢复由应用统一协调。主动模式不发送 CAN 命令。
请求模式须先创建保持固件生命周期的 `TX_MUTEX`，并传给 `imu::start_dmimu(&can_mutex)`；
同一总线的其他发送者和生命周期操作必须持有同一个互斥量。AHRS 以不等待方式尝试获取锁，
锁忙时留到下一轮发送；请求模式不提供互斥量会返回 `invalid_arg`。上方示例适用于主动模式。
离线恢复或队列溢出后重新开始累计航向，不跨数据缺口猜测旋转圈数。

DMIMU 保留设备自身的坐标和旋转约定，不能未经安装方向及坐标校验就与 BMI088 输出互换。
两条路径均发布传感器坐标下的数据，不执行安装方向或参考点补偿。原 `ahrs.imu_offset`
已迁至 `application.imu_offset`，生成到 `user_config::application::imu_offset`，设备服务
不会读取它。应用若需要将加速度换算到机器人参考点，可在收到有效消息后执行：

```cpp
const auto centripetal_accel = math::cross(sample.gyro,
    math::cross(sample.gyro, user_config::application::imu_offset));
const auto reference_accel = math::subtract(sample.accel, centripetal_accel);
```

这里位移为参考点指向 IMU 的向量，单位米，必须与角速度处于同一坐标系；上述公式仅补偿
向心加速度，未包含角加速度项。BMI088 的 ESKF 现在使用传感器测得的加速度，非零安装位移
不再影响设备内部解算。实际采样频率、坐标方向、温控参数、线程栈余量和 CAN 时序仍需上板验证。
