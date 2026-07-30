#!/usr/bin/env python3
"""Underground Map Editor — 地下空间 SLAM 地图编辑与路径规划工具"""

import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QFont
from ui.main_window import MainWindow
from core.ui_font import mono_family  # [ADAPT-UBU-02] 跨平台等宽字体


def main():
    app = QApplication(sys.argv)

    # [ADAPT-UBU-02] 全局等宽字体（Ubuntu 无 Courier New 时自动回退）
    fam = mono_family()
    font = QFont(fam, 9)
    app.setFont(font)

    # [ADAPT-UBU-02] 全局样式：字体族用运行时探测结果（保留 monospace 兜底）
    app.setStyleSheet(f"""
        QMainWindow {{ background: #f4f2ed; }}
        QLabel {{ color: #2c2c2a; }}
        QPushButton {{ font-family: '{fam}', monospace; }}
        QStatusBar {{ font-family: '{fam}', monospace; }}
    """)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
