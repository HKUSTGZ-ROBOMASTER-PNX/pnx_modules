# pnx_modules

业务服务模块：AHRS、遥控器、裁判系统、UI、USB 网关、消息定义等。

## 子目录

| 路径 | 说明 |
|------|------|
| `messages/` | `MsgAhrs`、`MsgVisionRx` 等话题结构体 |
| `ahrs/` | IMU + EKF，发布 `ahrs` |
| `remoter/` | DR16 / VT03 遥控器 |
| `referee/` | 裁判系统串口 |
| `ui/` | 客户端 UI |
| `usb/` | 通用 USB CDC 网关（`pnx_usb` 目标） |

## 依赖

- `pnx_os`、`pnx_bsp`、`pnx_devices`、`pnx_libraries`、`pnx_platform`
- `pnx_configs`

## CMake 目标

- `pnx_modules`（静态库）
- `pnx_usb`（`usb/` 子目录，静态库）

## Submodule

```bash
git submodule add <url> pnx_modules
```

## 入口

`modules.hpp` 聚合各服务类头文件；启动编排见 `pnx_tasks`。
