# Ubuntu 22.04 适配修改记录

> 基线（只读，未改动）：`F:\datasets\db\windows_version\2d`
> 适配副本（本目录）：`F:\datasets\db\ubuntu_version\2d`
> 原则：基线零改动；所有改动仅作用于副本，逐条可回退至 Windows 写法。

## 适配标签
- `[ADAPT-UBU-02]` 跨平台等宽字体
- `[ADAPT-UBU-04]` 硬编码盘符路径 → 脚本相对路径

> 说明：基线 `main.py` **不含** Windows High-DPI 属性设置（X11/Wayland 下 Qt5 自动处理 DPI），故无 `[ADAPT-UBU-01]` 改动。`core/car_indicator.py` 已内置 `Noto Color Emoji`（Linux 优先）+ `Segoe UI Emoji` 回退 + QPainter 矢量车形回退，本就跨平台，无需改动。

---

## 改动总览

| 编号 | 文件 | 类型 | 标签 |
|------|------|------|------|
| N-01 | `core/ui_font.py` | **新增**：跨平台等宽字体工具模块 | ADAPT-UBU-02 |
| C-01 | `main.py` | 字体改用 `mono_family()`；stylesheet 改 f-string | ADAPT-UBU-02 |
| C-02 | `ui/log_panel.py` | 5 处 `QFont('Courier New', N)` → `mono_font(N)` + 导入 | ADAPT-UBU-02 |
| C-03 | `ui/sidebar.py` | 7 处 → `mono_font(N)` + 导入 | ADAPT-UBU-02 |
| C-04 | `ui/pic_overlay.py` | 6 处 → `mono_font(N)` + 导入 | ADAPT-UBU-02 |
| C-05 | `ui/pic_hover_preview.py` | 2 处 → `mono_font(N)` + 导入 | ADAPT-UBU-02 |
| C-06 | `ui/map_view.py` | 2 处 → `mono_font(N)` + 导入 | ADAPT-UBU-02 |
| C-07 | `ui/main_window.py` | 1 处 → `mono_font(N)` + 导入 | ADAPT-UBU-02 |
| C-08 | `_qa_batch3.py` | 5 处 `F:/datasets/db/2d` → 脚本相对路径 | ADAPT-UBU-04 |
| C-09 | `_qa_batch4_test.py` | 2 处盘符路径 → 脚本相对路径 | ADAPT-UBU-04 |
| C-10 | `_selfcheck_batch4.py` | 3 处 `F:\datasets\db\2d` → 脚本相对路径 | ADAPT-UBU-04 |
| C-11 | `core/pic_player.py` | 文档字符串示例路径去盘符（注释，无功能影响） | ADAPT-UBU-04 |

> 新增依赖文件：`requirements.txt`、`ubuntu_environment_requirements.md`、本文件。

---

## 详细说明

### N-01 — 新增 `core/ui_font.py`

跨平台等宽字体探测模块。提供两个函数：

- `mono_family() -> str`：运行时按优先级返回首个可用等宽字体族（结果缓存，仅探测一次）。优先级：`Courier New → Ubuntu Mono → DejaVu Sans Mono → Liberation Mono → Noto Sans Mono → Monospace`。
- `mono_font(size) -> QFont`：构造对应字号的等宽 `QFont`。

惰性缓存确保 `QFontDatabase` 在 `QApplication` 创建之后才被查询（所有调用点均在 widget 构造期，必晚于 `QApplication`）。

**回退到 Windows**：删除本文件，并把各调用点改回 `QFont('Courier New', N)`（见下）。

### C-01 — `main.py`

- `QFont('Courier New', 9)` → `fam = mono_family(); QFont(fam, 9)`
- `setStyleSheet("""...'Courier New'...""")` → f-string，字体族注入 `'{fam}'`，字面量大括号转义为 `{{` `}}`。
- 新增 `from core.ui_font import mono_family`。

**回退**：删除 import 与 `fam=` 行，字体名改回 `'Courier New'`，stylesheet 还原为普通字符串（`f"""`→`"""`，`{{`/`}}`→`{`/`}`）。

### C-02 ~ C-07 — UI 模块字体替换

各文件内所有 `QFont('Courier New', N)` 统一替换为 `mono_font(N)`，并在文件首个 import 前加入
`from core.ui_font import mono_font`。替换处数见上表，逻辑/字号/粗体设置均未改变。

**回退**：删除该 import 行，把 `mono_font(N)` 全部改回 `QFont('Courier New', N)`。

### C-08 ~ C-10 — 自检脚本路径可移植化

根目录开发自检脚本中写死的 `F:\datasets\db\2d` / `F:/datasets/db/2d` 全部替换为
`os.path.dirname(os.path.abspath(__file__))` 派生的脚本相对路径，使其在 Ubuntu 下也能运行。
（这些为开发期 QA 脚本，非主程序运行路径。）

**回退**：把相对路径表达式改回原 Windows 绝对路径字符串。

### C-11 — `core/pic_player.py`

仅修改 docstring 中的示例路径注释 `F:\\...\\data\\...` → `.../data/...`，无任何功能代码改动。

---

## 验证结果

- 全部 `*.py` 通过 `python3 -m py_compile`（语法 / 缩进无误）。
- 所有使用 `mono_font` 的模块均已正确 `import`；功能代码中无残留 `QFont('Courier New', ...)`。
- 副本全树无任何盘符硬编码路径。
- 基线 `windows_version/2d` 全程只读，未发生任何改动。

> 运行期完整验证（实际拉起 GUI）需在装好第二节依赖的 Ubuntu 22.04 桌面环境执行，
> 沙箱无 Qt 系统库，无法做 GUI 级冒烟测试。
