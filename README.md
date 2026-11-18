# STM32 CAN 多节点温控与电机闭环控制系统

基于 STM32F407 主节点和 STM32F103 从节点的 CAN 多节点控制项目。系统通过 CAN 总线采集温度、下发电机控制指令，并在从节点完成编码器测速与速度/位置闭环控制。

## 项目功能

- STM32F407 主节点通过 DS18B20 采集温度
- 主节点通过 CAN 向从节点发送温度状态、故障和控制数据
- 主节点按键支持调速和温度查询
- STM32F103 从节点接收控制指令并控制 TB6612 驱动电机
- 编码器实时反馈转速与位置
- 支持速度环、位置环、位置-速度双环控制和急停模式
- CAN 报文和数据结构由 `Shared_Inc/can_protocol.h` 统一定义
- 支持远程请求电机状态反馈

## 系统拓扑

```text
                ┌────────────────────┐
DS18B20 ──────> │ STM32F407 主节点    │
                │ 温度采集 / 按键控制  │
                └─────────┬──────────┘
                          │ CAN H / CAN L
             ┌────────────┴────────────┐
             │                         │
    ┌────────▼────────┐       ┌────────▼────────┐
    │ STM32F103 从节点1 │       │ STM32F103 从节点2 │
    │ 编码器 + TB6612   │       │ 编码器 + TB6612   │
    │ PID 速度/位置闭环  │       │ PID 速度/位置闭环  │
    └──────────────────┘       └──────────────────┘
```

仓库中预留了多个 F103 从节点目录，也可按实际节点数量扩展为节点 1、节点 2、节点 3。

## 控制流程

1. F407 读取 DS18B20 温度，温度放大 10 倍后通过 CAN 发送。
2. F103 接收温度状态帧，并根据温度状态切换电机运行参数。
3. F407 可发送目标速度、控制模式或急停指令。
4. F103 从编码器获取实时转速和位置，通过 PID 计算后输出 PWM。
5. TB6612 根据 PID 输出控制电机正转、反转或刹车。
6. F407 可发送远程帧请求 F103 上报实时电机状态。

温度帧中的温度示例：

- `255` 表示温度 `25.5℃`
- `300` 表示温度阈值 `30.0℃`
- `fan_pwm_duty` 表示风扇占空比
- `is_over_temp` 为 `1` 时表示温度过高

## 节点功能说明

### F407 主节点

- `DS18B20`：采集环境温度，未触发转换时返回上次有效温度
- `KEY0`：短按发送电机速度控制指令，目标速度为 50 RPM
- `KEY1`：短按读取温度并通过 USART1 输出 `TEMP=xxxx`
- `KEY2`：预留扩展按键接口，可用于调速、换向或急停
- `CAN1`：发送温度状态帧和电机控制帧
- `USART1`：串口温度调试输出

当前按键逻辑支持短按和长按识别：短按时间不少于 20 ms，长按时间不少于 2 s。

### F103 从节点

- CAN 接收主节点控制指令
- 编码器实时获取转速和位置
- 速度环工作时，目标转速按 ±500 RPM 限幅
- PWM 输出按 TB6612 驱动逻辑控制正反转和刹车
- 停止模式下清除 PID 累积量并设置刹车输出
- 单位置环和位置-速度双环模式由统一控制循环切换

> 代码中的温度联动示例：若收到温度标志值为 10，则目标速度设为 50 RPM；若标志值为 100，则目标速度设为 500 RPM。实际量产工程建议改为根据真实温度区间和阈值动态映射速度，避免使用固定哨兵值。

## CAN 通信协议

公共协议定义位于：

```text
Shared_Inc/can_protocol.h
```

所有节点都应包含该头文件，并使用标准帧、11 位 CAN 标识符进行通信。

### 标准帧列表

| CAN ID | 名称 | 方向 | 说明 |
|---|---|---|---|
| `0x001` | 急停 | F407 → F103 | 切换停止模式，清除 PID，电机刹车 |
| `0x101` | 电机换向 | F407 → F103 | 将目标速度取反 |
| `0x102` | P 参数微调 | F407 → F103 | 动态设置比例系数 |
| `0x103` | I 参数微调 | F407 → F103 | 动态设置积分系数 |
| `0x104` | D 参数微调 | F407 → F103 | 动态设置微分系数 |
| `0x105` | 电机控制指令 | F407 → F103 | 控制模式、目标速度、目标位置 |
| `0x108` | 目标速度 | F407 → F103 | 16 位有符号 RPM |
| `0x109` | 目标位置 | F407 → F103 | 32 位编码器位置 |
| `0x10A` | 速度环 | F407 → F103 | 切换为速度闭环 |
| `0x10B` | 位置环 | F407 → F103 | 切换为位置闭环 |
| `0x10C` | 双环控制 | F407 → F103 | 切换为位置-速度串级闭环 |
| `0x201` | 电机状态反馈 | F103 → F407 | 实时模式、速度、位置和故障码 |
| `0x301` | 温度状态反馈 | F407/F103 节点 | 温度、阈值、风扇占空比和过温标志 |

### 数据结构

协议中主要使用以下结构体：

```c
typedef struct {
    uint8_t  ctrl_mode;
    int16_t  target_speed;
    int32_t  target_pos;
    uint8_t  cmd_seq;
} CanMsg_MotorCmd_t;

typedef struct {
    uint8_t  current_mode;
    int16_t  actual_speed;
    int32_t  actual_pos;
    uint8_t  error_code;
} CanMsg_MotorFb_t;

typedef struct {
    int16_t  current_temp;
    int16_t  threshold_temp;
    uint8_t  fan_pwm_duty;
    uint8_t  is_over_temp;
} CanMsg_TempStatus_t;
```

结构体统一使用 1 字节对齐，并通过小端方式在整数和字节数组之间拷贝。CAN 单帧数据长度不得超过 8 字节。

### 控制模式

| 值 | 模式 | 说明 |
|---|---|---|
| `0x00` | 停止/急停 | 清除 PID 累积量，PWM 为 0 或进入刹车 |
| `0x01` | 速度闭环 | 编码器速度跟踪目标 RPM |
| `0x02` | 位置闭环 | 编码器位置跟踪目标位置 |
| `0x03` | 位置-速度双环 | 位置环输出作为速度环给定 |

## 硬件连接

### F407 主节点

- CAN1_RX：PI9
- CAN1_TX：PB9
- DS18B20：依据 BSP 初始化工程中的单总线引脚配置
- KEY0/KEY1/KEY2：GPIOE 引脚 PE2/PE3/PE4
- USART1：工程配置对应的串口 TX/RX

### F103 从节点

- CAN_RX：PB8（CAN1 重映射）
- CAN_TX：PB9（CAN1 重映射）
- TIM 编码器接口：读取电机编码器
- TIM2_CH2：TB6612 PWM 输入
- AIN1/AIN2：TB6612 电机方向控制
- STBY：TB6612 驱动使能

所有节点接入同一条 CAN 总线时应满足：

- CAN_H 与 CAN_H 相连
- CAN_L 与 CAN_L 相连
- CAN 与 CAN 公地相连
- 总线两端各接约 120 Ω 终端电阻
- CAN 收发器供电和通信信号共地

## 编译与下载

项目使用 STM32CubeMX 生成的 CMake 工程，推荐使用 VS Code + CMake Tools + Cortex-Debug。

### F407 主节点

```powershell
cd "F407_Master"
cmake --preset Release
cmake --build build/Release
```

生成文件位于：

```text
F407_Master/build/Release/Master.hex
F407_Master/build/Release/Master.bin
```

### F103 从节点

```powershell
cd "F103_Salve1"
cmake --preset Release
cmake --build build/Release
```

生成文件位于：

```text
F103_Salve1/build/Release/CAN_Send.hex
F103_Salve1/build/Release/CAN_Send.bin
```

编译前确认各节点已经包含公共协议目录：

```cmake
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Shared_Inc
)
```

## 调试建议

1. 先单独测试 DS18B20 温度读取和串口输出。
2. 再测试 F103 编码器的方向、转速和位置计数。
3. 分离正反转测试 TB6612 的 PWM 和方向引脚。
4. 单独验证速度环，再叠加温度联动逻辑。
5. 接入 CAN 时先在示波器或 CAN 分析仪中确认 0x001、0x101、0x301 等报文。
6. 急停优先级最高，异常时应立即切入停止模式并清除 PID 状态。
7. 确认波特率、时钟预分频、CAN 采样点和滤波器配置在所有节点一致。

## 团队协作与分支

```text
云端
├── Stable_version  稳定版本
├── develop         公共集成分支
├── dev_gmf         F407 主节点开发分支
├── dev_lc          F103 从节点 1 开发分支
├── dev_dxl         F103 从节点 2 开发分支
└── dev_ls          F103 从节点 3 开发分支

本地节点目录
├── F407_Master     STM32F407 主节点
├── F103_Salve1     STM32F103 从节点 1
├── F103_Salve2     STM32F103 从节点 2
├── F103_Salve3     STM32F103 从节点 3
└── Shared_Inc      所有节点共享协议
```

开发流程建议：

1. 从最新 `develop` 拉取代码。
2. 在个人 `dev_*` 分支中修改自己负责的节点。
3. 修改公共协议时同步更新 `Shared_Inc/can_protocol.h`。
4. 向个人远端分支推送代码并发起合并请求。
5. 管理员确认无冲突后合入 `develop`。
6. 不要直接在个人本地分支合并公共 `develop`，以免污染独立开发环境。

## 目录结构

```text
CAN Project/
├── README.md
├── Shared_Inc/
│   └── can_protocol.h
├── F407_Master/
│   ├── Core/                  # CubeMX 外设初始化与中断
│   ├── User/BSP/              # 按键、LED、DS18B20、单总线
│   ├── User/APP/              # 应用任务和按键逻辑
│   └── User/Middleware/       # 通用延时等中间件
├── F103_Salve1/
│   ├── Core/                  # CubeMX 外设初始化
│   └── APP/                   # CAN、PID、编码器和电机控制
├── F103_Salve2/
└── F103_Salve3/
```

## 注意事项

- 上电时先保持驱动使能安全，必须在初始化后确认电机处于停止状态。
- 急停后应立即清除 PID 的积分项和历史输出，防止重新启动时速度突增。
- 对速度目标、PWM 输出和位置反馈应设置合理限幅。
- CAN 收到异常、超时或堵转时应设置故障码并进入安全状态。
- 公共协议修改后，F407 与 F103 必须同时更新、重新编译和下载。