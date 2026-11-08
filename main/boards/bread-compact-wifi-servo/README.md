# Bread Compact Wi-Fi + 舵机 (面包板)

在 [bread-compact-wifi](../bread-compact-wifi/) 的基础上增加一个 SG90 舵机，其余接线（麦克风、功放、OLED、按键、LED、灯）完全相同。

## 舵机接线

| 舵机线 | 连接 |
| --- | --- |
| 黄色（信号） | GPIO17 |
| 红色（电源） | 外部 5V |
| 棕色（地） | 外部电源负极，并与开发板 GND 相连（必须共地） |

## MCP 工具

| 工具 | 说明 |
| --- | --- |
| `self.servo.set_angle` | 转到指定角度，参数 `angle` 范围 0~180，90 为中间位置 |
| `self.servo.get_angle` | 读取当前角度 |

开机后舵机会先转到 90 度。脉宽按 SG90 的 0.5ms~2.5ms 对应 0~180 度，如果你的舵机转不到两端或在两端抖动，调整 `servo_controller.h` 中的 `kMinPulseUs` / `kMaxPulseUs`。

## 编译

`idf.py menuconfig` → Xiaozhi Assistant → Board Type 选择 `Bread Compact Wi-Fi + Servo (面包板)`，或使用：

```sh
python scripts/build.py bread-compact-wifi-servo
```
