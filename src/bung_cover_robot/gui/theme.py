"""Dark HMI theme — a single QSS stylesheet applied app-wide.

Colors are exposed as constants so tabs can tint status text/pills consistently.
"""

from __future__ import annotations

# Palette
BG = "#1b1f27"          # window
PANEL = "#232833"       # group boxes / cards
INPUT = "#2b313d"       # fields, buttons
BORDER = "#363d4a"
TEXT = "#d6dae1"
TEXT_DIM = "#8b93a3"
ACCENT = "#3d8bfd"      # primary blue
ACCENT_HOVER = "#5a9dfd"
SUCCESS = "#3fb950"
DANGER = "#f85149"
WARN = "#d29922"
INFO = "#58a6ff"

# Status pill backgrounds (subtle, for the main screen)
PILL = {
    "ok": ("#16351f", SUCCESS),
    "bad": ("#3a1d1d", DANGER),
    "info": ("#16273d", INFO),
    "warn": ("#332a12", WARN),
    "idle": ("#262b34", TEXT_DIM),
}


def pill_style(kind: str) -> str:
    bg, fg = PILL.get(kind, PILL["idle"])
    return (
        f"background:{bg}; color:{fg}; border-radius:10px; "
        f"padding:4px 12px; font-weight:600;"
    )


DARK_QSS = f"""
* {{ font-family:'Segoe UI','DejaVu Sans',sans-serif; font-size:13px; }}
QWidget {{ background-color:{BG}; color:{TEXT}; }}
QMainWindow, QDialog {{ background-color:{BG}; }}
QToolTip {{ background:{INPUT}; color:{TEXT}; border:1px solid {BORDER}; }}

/* Tabs */
QTabWidget::pane {{ border:1px solid {BORDER}; border-radius:8px; top:-1px; background:{PANEL}; }}
QTabBar::tab {{
    background:transparent; color:{TEXT_DIM}; padding:9px 20px; margin-right:2px;
    border:1px solid transparent; border-top-left-radius:6px; border-top-right-radius:6px;
    font-weight:600;
}}
QTabBar::tab:selected {{ background:{PANEL}; color:#ffffff; border:1px solid {BORDER};
    border-bottom:2px solid {ACCENT}; }}
QTabBar::tab:hover:!selected {{ color:{TEXT}; }}

/* Group boxes */
QGroupBox {{ background:{PANEL}; border:1px solid {BORDER}; border-radius:8px;
    margin-top:16px; padding:12px 12px 10px 12px; font-weight:600; }}
QGroupBox::title {{ subcontrol-origin:margin; left:12px; padding:0 6px; color:{TEXT_DIM}; }}

/* Buttons */
QPushButton {{ background:{INPUT}; border:1px solid #3a4150; border-radius:6px;
    padding:7px 15px; color:{TEXT}; }}
QPushButton:hover {{ background:#333b48; border-color:#4a5262; }}
QPushButton:pressed {{ background:#3a4150; }}
QPushButton:checked {{ background:{ACCENT}; border-color:{ACCENT}; color:#ffffff; }}
QPushButton:disabled {{ background:#242a33; color:#5b6472; border-color:#2e353f; }}
QPushButton[accent="primary"] {{ background:{ACCENT}; border-color:{ACCENT}; color:#ffffff; }}
QPushButton[accent="primary"]:hover {{ background:{ACCENT_HOVER}; }}
QPushButton[accent="danger"] {{ background:#3a1d1d; border-color:{DANGER}; color:{DANGER}; }}
QPushButton[accent="danger"]:hover {{ background:{DANGER}; color:#ffffff; }}

/* Inputs */
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{ background:{INPUT}; border:1px solid #3a4150;
    border-radius:5px; padding:5px 7px; color:{TEXT}; selection-background-color:{ACCENT}; }}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{ border-color:{ACCENT}; }}
QComboBox::drop-down {{ border:none; width:18px; }}
QComboBox QAbstractItemView {{ background:{INPUT}; border:1px solid {BORDER};
    selection-background-color:{ACCENT}; outline:none; }}
QSpinBox::up-button, QDoubleSpinBox::up-button, QSpinBox::down-button,
QDoubleSpinBox::down-button {{ background:#333b48; border:none; width:16px; }}

/* Tables */
QTableWidget, QTableView {{ background:#20242b; alternate-background-color:#242a33;
    gridline-color:#333b48; border:1px solid {BORDER}; border-radius:6px; outline:none; }}
QHeaderView::section {{ background:{INPUT}; color:{TEXT_DIM}; padding:7px 8px; border:none;
    border-right:1px solid {BORDER}; border-bottom:1px solid {BORDER}; font-weight:600; }}
QTableWidget::item {{ padding:2px 4px; }}
QTableWidget::item:selected {{ background:#2d3a52; color:#ffffff; }}

/* Sliders */
QSlider::groove:horizontal {{ height:5px; background:#3a4150; border-radius:2px; }}
QSlider::sub-page:horizontal {{ background:{ACCENT}; border-radius:2px; }}
QSlider::handle:horizontal {{ background:#eaeef5; width:14px; margin:-6px 0; border-radius:7px; }}
QSlider::handle:horizontal:hover {{ background:#ffffff; }}

/* Scrollbars */
QScrollBar:vertical {{ background:transparent; width:12px; margin:2px; }}
QScrollBar::handle:vertical {{ background:#3a4150; border-radius:6px; min-height:28px; }}
QScrollBar::handle:vertical:hover {{ background:#4a5262; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height:0; }}
QScrollBar:horizontal {{ background:transparent; height:12px; margin:2px; }}
QScrollBar::handle:horizontal {{ background:#3a4150; border-radius:6px; min-width:28px; }}
QScrollArea {{ border:none; }}

QCheckBox {{ spacing:8px; }}
QCheckBox::indicator {{ width:16px; height:16px; border:1px solid #3a4150; border-radius:4px;
    background:{INPUT}; }}
QCheckBox::indicator:checked {{ background:{ACCENT}; border-color:{ACCENT}; }}
"""


def apply_theme(app) -> None:
    """Apply the dark theme to a QApplication."""
    app.setStyleSheet(DARK_QSS)
