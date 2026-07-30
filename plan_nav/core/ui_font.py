# core/ui_font.py
# [ADAPT-UBU-02] 跨平台等宽字体工具
# Windows 基线硬编码 'Courier New'，但 Ubuntu 22.04 默认不含该字体，
# 直接使用会触发 Qt 字体回退、破坏等宽排版。此模块在运行时按优先级
# 探测系统可用的等宽字体族，全局复用，保证 Windows / Ubuntu 一致显示。
#
# 优先级：Courier New(若装了 msttcorefonts) -> Ubuntu Mono -> DejaVu Sans Mono
#         -> Liberation Mono -> Noto Sans Mono -> Monospace(Qt 通用兜底)
#
# 注意：QFontDatabase 必须在 QApplication 创建之后才能查询字体表，
# 因此这里使用惰性缓存，首次调用（widget 构造时，必在 QApplication 之后）才探测。

from PyQt5.QtGui import QFont, QFontDatabase

_MONO_FAMILY_CACHE = None

# 候选等宽字体族，按优先级排列
_MONO_CANDIDATES = [
    'Courier New',        # Windows 自带 / Ubuntu 装了 ttf-mscorefonts-installer 后可用
    'Ubuntu Mono',        # Ubuntu 推荐等宽字体（fonts-ubuntu）
    'DejaVu Sans Mono',   # Ubuntu 几乎必装（fonts-dejavu）
    'Liberation Mono',    # 备选（fonts-liberation）
    'Noto Sans Mono',     # 备选（fonts-noto）
    'Monospace',          # Qt 通用等宽别名，最终兜底
]


def mono_family() -> str:
    """返回当前系统可用的首个等宽字体族名（结果缓存，仅探测一次）。"""
    global _MONO_FAMILY_CACHE
    if _MONO_FAMILY_CACHE is None:
        try:
            available = set(QFontDatabase().families())
        except Exception:
            available = set()
        _MONO_FAMILY_CACHE = next(
            (name for name in _MONO_CANDIDATES if name in available),
            'Monospace',
        )
    return _MONO_FAMILY_CACHE


def mono_font(size: int) -> QFont:
    """构造指定字号的等宽 QFont，跨平台自动选择字体族。"""
    return QFont(mono_family(), size)
