# Bread Compact Wi-Fi + 双轴舵机 (面包板)

在 [bread-compact-wifi](../bread-compact-wifi/) 上增加两个 SG90，做头部左右（pan）和上下（tilt）。麦克风、功放、OLED、按键、LED、灯的接线不变。

OLED 继续用 **SDA = GPIO41、SCL = GPIO42**。不要把屏幕改到 GPIO6 / GPIO7：这两脚是麦克风 DIN 和喇叭 DOUT。

## 引脚

| 功能 | 连接 |
| --- | --- |
| 俯仰 tilt（上下点头） | **GPIO10**，LEDC 通道 0 |
| 偏航 pan（左右摇头） | **GPIO11**，LEDC 通道 1 |
| OLED SDA | GPIO41 |
| OLED SCL | GPIO42 |
| 麦克风 DIN | GPIO6 |
| 喇叭 DOUT | GPIO7 |
| 舵机红线（电源） | **外部 5V** |
| 舵机棕线（地） | 外部电源负极，并与开发板 GND 相连（必须共地） |
| 舵机黄线（信号） | 上表对应的 GPIO |

两个 SG90 堵转时电流很大，不要从开发板 3.3V 取电，也不要指望 USB 5V 经板载稳压同时带动两只舵机。用独立 5V，和开发板共地。信号是 3.3V、50Hz PWM（脉宽 500–2500µs 对应 0–180°），SG90 可以直接接。电源旁边加一只 100–470µF 电容可以减少舵机动作时的复位。

状态灯是 WS2812（RMT），灯控是普通 GPIO，不用 LEDC。舵机占用 LEDC 定时器 0 的通道 0（tilt）和通道 1（pan），14 位，50Hz。不要再在这两个通道上挂背光 PWM。

## 角度

装舵机臂之前先上电，让两轴停在 90°，再拧紧摇臂，头部才是正的。

| 轴 | 安全范围 | 中位 | 方向 |
| --- | --- | --- | --- |
| pan | 50–130 | 90 | 小角度朝左，大角度朝右 |
| tilt | 55–125 | 90 | 小角度抬头，大角度低头 |

固件会把角度夹在这个范围内。如果实际方向反了，把舵机臂拆下转 180° 再装回中位，不要改麦克风或屏幕引脚。

## 动作

聊天协议里的 `emotion`（`display->SetEmotion`）会同时换 OLED 表情并排队一个头部动作，不堵住音频和主循环。新表情会从当前角度缓动过去，盖掉还没做完的旧动作。

| 表情 | 头部 |
| --- | --- |
| neutral | 停下脚本，空闲时缓慢呼吸（俯仰约 ±2.5°），偶尔瞥一眼 |
| happy / laughing / funny | 快速点头，略微抬头 |
| loving / kissy | 探身（pan + tilt）再轻轻点头 |
| embarrassed | 低头看向一侧，再回到中位 |
| sad / crying | 低头并慢慢左右摆，结束后保持低头，直到下一个表情 |
| angry | 短促用力摇头 |
| surprised / shocked | 快速抬头并停一下 |
| thinking / confused | 头歪向一侧（pan + tilt），再回中 |
| sleepy / relaxed | 慢呼吸并略微低头，结束后保持困倦姿态 |
| winking / silly / cool / confident / delicious | 侧目或得意地点一下头 |

设备状态也会驱动空闲动作（不打断正在播放的表情）：

- 唤醒 / 连接中 / 聆听：抬头看过来
- 说话 / 提示音：轻轻点头；如果刚才的表情是难过或困倦，就继续保持那个姿态
- 待机且 idle 打开：呼吸；idle 关掉则回中停住

上电先输出中位脉宽，再播放一段开机张望（抬头、左右看、点头、回中）。开机后大约 3 秒内，待机流程发出的 `neutral` 不会把这段动作掐断。`robot_2` 这类非表情图标不会带动头部。

## MCP 工具

| 工具 | 说明 |
| --- | --- |
| `self.servo.get_pan` / `set_pan` | 读取或缓动到 pan，`angle` 50–130 |
| `self.servo.get_tilt` / `set_tilt` | 读取或缓动到 tilt，`angle` 55–125 |
| `self.servo.play_action` | `action`：`center`、`shake`、`nod`、`curious`、`dizzy`、`look_left`、`look_right`、`alive`、`idle`、`idle_off` |
| `self.servo.set_idle` | `enabled` 为 true/false，开关待机呼吸 |

工具只负责入队，马上返回。平时聊天不用调这些工具，表情会自己带动头部。

脉宽仍是 0.5ms–2.5ms 对应 0–180°。如果舵机在安全角度两端顶死或发抖，改 `dual_servo_gestures.h` 里的 `kMinPulseUs` / `kMaxPulseUs`，不要把范围放大到机械限位外面。

## 编译

`idf.py menuconfig` → Xiaozhi Assistant → Board Type 选择 `Bread Compact Wi-Fi + Servo (面包板)`，或使用：

```sh
python scripts/build.py bread-compact-wifi-servo
```

## 上板看动作

1. 共地并接好外部 5V 后上电。头部应先在 90°，接着抬头、向左、向右、点头，再回到中间，然后轻微呼吸。
2. 唤醒后说话。聆听时头抬起；回答时如果没有更强的表情，会轻轻点头。
3. 用一轮情绪比较密的对话看表情，例如：「讲个笑话」「假装很生气」「吓我一跳」「我有点难过」「我喜欢你」「你在想什么」「装酷」「你困了吗」。OLED 表情应照常切换，头部做对应动作，说话声音不应被卡住。
4. 需要单动时用 MCP：`set_pan` 60、`set_tilt` 70、`play_action` `shake`，再用 `set_idle` false 关掉呼吸。
