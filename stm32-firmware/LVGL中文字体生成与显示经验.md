# LVGL 中文字体生成与显示经验

本文记录 STM32H743、LVGL 8.3.6、Keil ARMCC5 工程中生成中文字体并正确显示中文的实际做法。

## 1. 中文显示需要同时解决两个问题

1. C 源码必须能被 ARMCC5 正确编译。
2. LVGL 使用的字体必须包含对应汉字字形。

仅把英文字符串改成中文并不够。如果字体没有该汉字，屏幕通常会显示方框或空白。

## 2. ARMCC5 不直接使用 UTF-8 中文字符串

本工程 ARMCC5 对直接写入源码的 UTF-8 中文字符串兼容性不好，例如：

```c
lv_label_set_text(label, "未开始识别");
```

可能在编译时出现 `missing closing quote`。工程中的可靠写法是把中文转换为 UTF-8 字节转义：

```c
lv_label_set_text(label,
                  "\xE6\x9C\xAA\xE5\xBC\x80\xE5\xA7\x8B"
                  "\xE8\xAF\x86\xE5\x88\xAB");
```

LVGL 仍然会把这些字节当作正常 UTF-8 文本解析。

当前目录的 `cn_to_utf8_escapes.pl` 可以完成这种转换，但运行环境需要安装 Perl：

```powershell
perl .\cn_to_utf8_escapes.pl input.c output.c
```

如果系统没有 Perl，可以用 PowerShell 按相同规则机械转换。转换时只处理双引号字符串中的非 ASCII 字符，不应修改注释、变量名和代码结构。

## 3. 使用 lv_font_conv 生成精简字库

本次使用 Windows 字体 `C:\Windows\Fonts\STZHONGS.TTF`，通过 Node.js 的 `lv_font_conv` 生成字体：

```powershell
npx --yes lv_font_conv `
  --bpp 1 `
  --size 14 `
  --font "C:\Windows\Fonts\STZHONGS.TTF" `
  -r 0x20-0x7f `
  --symbols "需要显示的汉字" `
  --format lvgl `
  --no-compress `
  --no-prefilter `
  --lv-font-name ui_font_CN14 `
  -o ui_font_CN14.c
```

不要直接加入完整 CJK 字库。只收集界面实际使用的字符，可以明显减少 Flash 占用和编译时间。

本工程按照原界面字号生成了三套字体：

- `ui_font_CN14`：替代原 Montserrat 14，用于状态、按钮、页签和参数文字。
- `ui_font_Road`：32 像素，用于原来的路面大字显示和罗盘角度。
- `ui_font_CN64`：替代原 64 像素标题字体，用于“多功能小车”“车辆数据监控”等标题。

这样只替换字体和文字，不修改控件坐标、尺寸、颜色、对齐方式和页面风格。

## 4. 适配本工程的 include 路径

`lv_font_conv` 默认生成的文件可能包含：

```c
#include "lvgl/lvgl.h"
```

本工程的 Keil include 路径使用：

```c
#include "lvgl.h"
```

生成后必须检查并改成工程实际可用的 include，否则会报：

```text
cannot open source input file "lvgl/lvgl.h"
```

## 5. 把字体加入 LVGL 和 Keil 工程

在 `ui.h` 中声明：

```c
LV_FONT_DECLARE(ui_font_CN14);
LV_FONT_DECLARE(ui_font_Road);
LV_FONT_DECLARE(ui_font_CN64);
```

随后把对应 `.c` 文件加入 `STM32H743.uvprojx`，否则即使源码引用正确，链接阶段也会找不到字体对象。

控件使用方式：

```c
lv_obj_set_style_text_font(label, &ui_font_CN14, 0);
```

TabView 的页签文字需要给页签按钮矩阵设置字体：

```c
lv_obj_set_style_text_font(lv_tabview_get_tab_btns(tabview),
                           &ui_font_CN14,
                           0);
```

## 6. 常见问题

### 中文字符串可以编译，但屏幕显示方框

字体的 `--symbols` 中缺少该字符。中文标点如 `，`、`。`、`！` 也必须单独加入。

### 字体文件已经生成，但链接失败

检查字体 `.c` 是否真正加入 Keil 工程，以及 `LV_FONT_DECLARE()` 名称是否和 `--lv-font-name` 一致。

### 更换中文后布局发生变化

不要随意改变字号。应按照原控件字体大小生成对应中文字体，只替换字体对象和字符串。长文本需要确认原控件宽度是否足够，但不要先修改坐标和风格。

### 隐藏按钮仍需要可点击

可以保留按钮对象、尺寸、坐标和事件回调，只把按钮 Label 设置为空字符串。不要删除按钮对象，否则触摸切换功能也会消失。

## 7. 验证步骤

1. 执行 `git diff --check`，检查空白和补丁问题。
2. 用 PowerShell XML 解析检查 `STM32H743.uvprojx`。
3. 先做 Keil 增量编译，解决字体名、include 和源码编码问题。
4. 再执行完整重编，确认所有页面、字体文件和后端状态字符串从头编译。
5. 最后烧录实机，检查字形是否完整、控件是否溢出、隐藏按钮触摸区域是否正常。

编译成功只证明字体已经正确参与编译和链接，LCD 上的实际显示效果仍需要上板确认。
