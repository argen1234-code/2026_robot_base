# 多功能小车 — STM32H743 四轮全向

2026 嵌入式竞赛。FreeRTOS + LVGL + TF卡 + QSPI Flash。

## 硬件

- MCU: STM32H743IITx (Cortex-M7, 480MHz)
- LCD: 1024×600 LTDC RGB, 33MHz
- SDRAM: 32MB FMC
- Touch: GT911 电容
- TF 卡: SDMMC1 4-bit 20MHz
- QSPI Flash: W25Q64 8MB (120MHz)
- 4 路编码器电机 + PID 闭环

## 快速开始

1. Keil MDK 打开 `MDK-ARM/STM32H743.uvprojx`
2. 勾选 **Use MicroLIB** (Target 页)
3. Rebuild all → Download (F8)
4. 串口 PA9/PA10, 115200

## UI 界面

```
Login(qmq) → Screenmain (4按钮)
              ├─ [数据监控] → DataDisplay ⇄ DataDetail (GPS/Speed/IMU/Mode)
              ├─ [文件管理] → FileManager (TF卡浏览器, 可进文件夹)
              ├─ [设置页面] → Screensettings (bg大图)
              └─ [返回登录] → Login (清空密码)

子页面点击/滑动 → 回 Screenmain
60s 无操作 → 自动回 Login
```

## TF 卡文件

| 文件 | 说明 | 
|---|---|
| `icon.bin` | 主界面图标 |
| `bg.bin` | 设置页背景 1024×600 |
| `data.bin` | 数据监控图标 |
| `setbtn.bin` | 设置按钮图标 |
| `lock.bin` | 返回登录图标 |

**文件名限制**: FAT32 无 LFN, 必须 ≤8 字符 (8.3 格式)

## SquareLine 导出后处理

1. `perl strip_image_data.pl <img.c>` — 图片数据缩为占位符
2. `perl cn_to_utf8_escapes.pl <file.c>` — 中文 → UTF-8 转义
3. `ui.h`: `LV_IMG_DECLARE` → `extern lv_img_dsc_t`, 移除 `lv_i18n.h`, `lvgl/lvgl.h`→`lvgl.h`
4. 图片 `.c`: `const lv_img_dsc_t` → `lv_img_dsc_t`
5. Python 提取 `.bin` → TF 卡 (文件名 ≤8 字符)

## 待完成

- [ ] FATFS-on-QSPI (f_mkfs 卡死, 底层读写 OK)
- [ ] L610 4G 模块
- [ ] Modbus RTU
- [ ] LvglFont 4bpp 字体

详见 [工程状态.md](工程状态.md)
