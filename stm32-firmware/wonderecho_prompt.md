# 提示词任务文档：基于 WonderEcho 语音交互模块的车辆控制开发

## 任务背景与目标

我们需要在当前的 **STM32H743 + FreeRTOS + HAL 库** 的多功能小车工程中，接入并集成 **WonderEcho 语音交互模块**。该模块搭载了 CI1302 语音芯片，固件已完成烧录，可通过软件 I2C 接口与 MCU 进行通信。

你的任务是：
1. **底层驱动开发**：将原工程中空的 `bsp_Voice_Recognition.c` 与 `.h` 重命名为 `WonderEcho.c` 与 `.h`，实现软件 I2C 驱动与 WonderEcho 交互逻辑。
2. **应用层开发**：在 `Middware/App/` 下新建 `app_Voice_Recognition.c` 与 `.h`，编写语音控制应用层逻辑。
3. **底盘任务集成**：**不新建独立的 FreeRTOS 任务，将语音交互与控制全部放在现有的底盘任务（`chassis_task`）中运行**，以统一底盘速度控制并简化系统调度。
4. **代码风格模仿**：严格模仿现有工程风格，使用清晰的英文变量命名，并对硬件初始化和关键时序编写**详尽的中文注释**。

---

## 硬件与协议配置参考

### 1. I2C 引脚定义
在 `Core/Inc/main.h` 中已经有如下软件 I2C 的宏定义，请直接使用：
```c
#define SOFT_I2C_SDA_Pin GPIO_PIN_1
#define SOFT_I2C_SDA_GPIO_Port GPIOB
#define SOFT_I2C_SCL_Pin GPIO_PIN_0
#define SOFT_I2C_SCL_GPIO_Port GPIOB
```

### 2. 语音模块协议配置
* **设备 7 位 I2C 地址**：`0x34` (写地址为 `0x68`，读地址为 `0x69`)
* **识别结果寄存器**：`0x64` (读取该寄存器获得识别到的词 ID)
* **播报指令寄存器**：`0x6E` (向该寄存器写入播报数据)
* **核心控制命令词及其 ID 对照表**：
  * **前进** (ID: `0x01`)
  * **后退** (ID: `0x02`)
  * **左转** (ID: `0x03`)
  * **右转** (ID: `0x04`)
  * **停止/停下** (ID: `0x09`)
  * **加速** (ID: `0x0D`)
  * **减速** (ID: `0x0E`)

---

## 详细开发步骤

### 第一步：重命名与工程维护 [MODIFY]
1. 将 `Drivers/Hardware/bsp_Voice_Recognition.c` 和 `bsp_Voice_Recognition.h` 重命名为 `WonderEcho.c` 和 `WonderEcho.h`。
2. 修改 Keil MDK 工程配置文件 `STM32H743.uvprojx`，更新这两个文件的文件名引用。

### 第二步：底层 I2C 驱动实现 (`WonderEcho.c` 和 `WonderEcho.h`)
1. **参考工程现有风格**：参考 `Drivers/User/Src/touch_iic.c` 的软件 I2C 实现，保持命名风格一致。
2. **开漏模式优化**：SCL (`PB0`) 和 SDA (`PB1`) 引脚必须初始化为 **开漏输出带上拉** 模式 (`GPIO_MODE_OUTPUT_OD`, `GPIO_PULLUP`)。
   > [!IMPORTANT]
   > 由于是开漏模式，SDA 释放时只需输出 `1` 即可直接读取引脚电平，**严禁使用 `HAL_GPIO_Init` 切换 SDA 的输入/输出方向**，以提高 I2C 传输效率并防范时序混乱。
3. **接口要求**：
   * `void WonderEcho_I2C_Init(void)`: 初始化 GPIOB 及软件 I2C 引脚（必须有清晰的 GPIO 配置中文注释）。
   * `uint8_t WonderEcho_GetResult(void)`: 读取 `0x64` 寄存器的语音识别结果。每次 I2C 读取时必须包含超时计数器，如果模块没有应答或掉电，不可进入死循环。
   * `void WonderEcho_Speak(uint8_t cmd, uint8_t idNum)`: 写入播报地址（`0x6E`）。

### 第三步：新增语音应用层 (`app_Voice_Recognition.c` 和 `app_Voice_Recognition.h`) [NEW]
1. 在 `Middware/App/` 文件夹下新建这两个文件。
2. **定义控制状态**：
   * 基础语音控制速度 `voice_speed`（默认为 `20.0f`，限制范围 `10.0f` ~ `50.0f`，加减速步长 `10.0f`）。
3. **编写底盘任务更新接口**：
   * 声明并实现函数：`void App_Voice_Recognition_Update(chassis_move_t *chassis)`。
   * 该函数将由底盘任务以固定的频率（建议每 50ms 轮询一次）调用。
   * 函数内部逻辑：调用 `WonderEcho_GetResult()` 读取识别 ID，若返回有效命令，控制底盘执行相应动作：
     * **`0x01` 前进**：设置 `chassis->mode = CAR_MODE_VOICE`，且 `Vx_set = voice_speed`, `Vy_set = 0.0f`, `Wz_set = 0.0f`。
     * **`0x02` 后退**：设置 `chassis->mode = CAR_MODE_VOICE`，且 `Vx_set = -voice_speed`, `Vy_set = 0.0f`, `Wz_set = 0.0f`。
     * **`0x03` 左转**：设置 `chassis->mode = CAR_MODE_VOICE`，且 `Vx_set = 0.0f`, `Vy_set = 0.0f`, `Wz_set = -20.0f` (负值代表自转左转)。
     * **`0x04` 右转**：设置 `chassis->mode = CAR_MODE_VOICE`，且 `Vx_set = 0.0f`, `Vy_set = 0.0f`, `Wz_set = 20.0f` (正值代表自转右转)。
     * **`0x09` 停止/停下**：设置底盘目标速度 `Vx_set = 0.0f`, `Vy_set = 0.0f`, `Wz_set = 0.0f`。
     * **`0x0D` 加速**：执行 `voice_speed += 10.0f`，上限 `50.0f`；如果当前正在前进或后退，需实时更新 `Vx_set`。
     * **`0x0E` 减速**：执行 `voice_speed -= 10.0f`，下限 `10.0f`；如果当前正在前进或后退，实时更新 `Vx_set`。

### 第四步：底盘任务集成与模式对接 [MODIFY]
1. **模式枚举定义**：
   在 `Middware/App/app_chassis_board.h` 中，在 `CarMode_t` 枚举内添加 `CAR_MODE_VOICE`。
2. **底盘任务初始化**：
   在 `app_chassis_board.c` 的 `chassis_init(chassis_move_t *chassis)` 中，包含 `WonderEcho.h` 并调用 `WonderEcho_I2C_Init()` 完成底层软件 I2C 的初始化。
3. **底盘控制策略适配**：
   在 `app_chassis_board.c` 的 `chassis_set_control(chassis_move_t *chassis)` 中，接入语音控制模式（优先级低于蓝牙、微信小程序和 ROS 导航，但高于默认的纯 GPS 导航）：
   ```c
   // ... 前面已有的蓝牙、小程序、ROS、GPS融合导航等高优先级判断 ...
   
   /* 5. 语音识别控制 (CAR_MODE_VOICE) */
   else if (chassis->mode == CAR_MODE_VOICE)
   {
       // 语音控制速度直接由底盘任务调用的 App_Voice_Recognition_Update 写入 Vx_set/Vy_set/Wz_set，在此处保持现状即可
   }
   /* 6. 默认：纯 GPS 导航 */
   else
   {
       chassis->mode = CAR_MODE_GPS;
       Navigation_Update_Loop(chassis);
   }
   ```
4. **底盘主循环轮询（100Hz 任务内）**：
   语音读取无需过于频繁（每 10ms 读取一次会占用不必要的 I2C 通信时间），因此在底盘任务的主循环 `chassis_task` 中，**利用时间戳或周期计数器，以 50ms（即每 5 个周期）的间隔轮询调用语音应用层更新函数**：
   ```c
   // 在 chassis_task 的 while(1) 循环中调用：
   static uint32_t voice_tick = 0;
   if (HAL_GetTick() - voice_tick >= 50)
   {
       voice_tick = HAL_GetTick();
       App_Voice_Recognition_Update(&chassis_move);
   }
   ```
   > [!NOTE]
   > 这样设计将整个语音控制完全包含在现有的底盘任务上下文中，既避免了多线程写入 `Vx_set` 的资源竞争，又确保了底层控制的单一责任制。

---

## 代码风格与质量要求

1. **代码风格模仿**：
   * 严格模仿现有工程驱动（如 `touch_iic.c`）和底盘层（如 `app_chassis_board.c`）的 C 代码设计模式。
   * 使用有意义的英文变量与函数命名。
2. **详尽的中文注释**：
   * 在所有的引脚初始化配置、I2C 底层时序、寄存器读写、以及运动状态映射逻辑部分，**必须编写清晰、易读的中文注释**，便于后续维护与调试。
3. **通信防卡死与启动保护**：
   * **通信防卡死**：在 I2C 读取时，必须有超时退出机制，绝对不能因为物理接线断开或传感器异常而无限循环导致底盘控制任务卡死挂起。
   * **启动保护**：建议在底盘初始化时，延时 `200ms` 后再发送 I2C 初始化与握手指令，给 WonderEcho 模块足够的上电就绪时间。
4. **严禁改动 GUI 部分**：不用管任何 LVGL 相关的代码和任务，保持界面部分的隔离。
