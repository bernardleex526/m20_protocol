# m20_protocol

云深处 M20 / M20 Pro `basic_server` 协议的共享实现：帧编解码、TCP 流式解码、运动/模式命令构造。导航、定位和错误码表不包含在内。纯 Python，无 ROS 依赖；同时是 `ament_python` 包，可直接放进 colcon 工作区。

此前同一协议在 `m20_chase`、`deamon`、`deamon2`、`2.5d_ccplan`、`mechanical-dog-control` 中各写了一份，且存在不一致（见下）。各仓库现以 git submodule 引用本包。

## 帧格式

| 偏移 | 内容 |
|---|---|
| 0–3 | 同步字 `EB 91 EB 90` |
| 4–5 | ASDU 长度（uint16 LE） |
| 6–7 | 消息 ID（uint16 LE） |
| 8 | 格式，`0x01` = JSON |
| 9–15 | 保留，全 0 |

ASDU：`{"PatrolDevice": {"Type", "Command", "Time", "Items"}}`。

## 用法

```python
from m20_protocol import StreamDecoder, decode
from m20_protocol import commands as C

sock.send(C.heartbeat(msg_id))
sock.send(C.axis_normalized(*C.normalize_velocity((vx, vy, wz))))  # Cmd=21，常规/辅助模式
sock.send(C.velocity_si(vx, vy, wz))                               # Cmd=25，导航模式
```

## 已知的跨仓库差异

- **同步字**：`2.5d_ccplan` 原实现为 `EB 90 EB 90`（来自执行计划附录，计划中列为未决项 P15）；其余实现与实际可用的厂商桌面客户端均为 `EB 91 EB 90`。本包采用后者。
- **步态 ID 与固件相关**：软件 ≥ V1.1.7 为 `4097/4099/12290/12291`（`GAIT_TABLE_V117`），手册 V0.0.4-0 为 `GaitParam 1/14`。`set_gait` 不限制取值，由调用方按机器固件选择。
- `DEFAULT_AXIS_MAX`（Cmd=21 满量程）取文档最大值，**需在实机上标定**。

## 测试

```bash
python -m pytest test
```

本包只覆盖协议层的单元测试，不代表任何实机运动验收。
