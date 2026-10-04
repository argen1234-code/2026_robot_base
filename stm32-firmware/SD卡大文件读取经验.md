# SD 卡大文件读取经验

## 最终方案

在 `disk_read` / `disk_write` 中，调用 `BSP_SD_ReadBlocks` 前后关全局中断：

```c
__disable_irq();
int32_t ret = BSP_SD_ReadBlocks(SD_Instance, (uint32_t *)buff, sector, count);
__enable_irq();
```

## 根因

SDMMC polling 模式下，FreeRTOS / LVGL / 电机 PWM 等中断会打断多扇区传输，导致 `FR_DISK_ERR`。

大文件需要几百到几千个扇区连续读取，小文件几十个扇区瞬间读完来不及被打断。

## 走过的弯路

| 尝试 | 为什么没用 |
|---|---|
| FAT32 分块 64 扇区读取 | 块小了但仍可能被打断 |
| 失败后重试 | 中断持续存在，重试也白费 |
| 绕过 FAT32 直接读原始扇区 | 底层还是同一个 polling 读 |
| 换 TF 卡 | 卡没问题 |
| 增大 SDMMC 超时 | 不是超时问题 |
| D-Cache 刷新 | 必要但不解决根本问题 |

## 启发来源

正点原子 STM32F4 例程 `sdio_sdcard.c` 中的注释：

> 关掉总中断(POLLING模式,严禁中断打断SDIO读写!!!)

## 适用场景

- STM32 + SDMMC polling 模式
- FreeRTOS 或其他 RTOS 环境
- 需要连续读取 >250KB 数据

## 副作用

关中断期间系统 tick 停止计数，lv_timer_handler 暂停。对于 1.8MB 文件（~3600 扇区, ~200ms 读取时间），影响可接受。
