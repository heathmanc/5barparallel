"""ISA-101 high-performance HMI theme — one QSS stylesheet applied app-wide.

Follows ANSI/ISA-101.01 coloration principles:

  * Light-gray, low-saturation chrome. Color NEVER decorates; it carries
    meaning, so an abnormal condition is the most salient thing on screen.
  * NORMAL states carry NO color — dark-gray text on gray. The absence of
    color is the "everything is fine" signal (no green "running" lamps).
  * RED is reserved exclusively for alarms/faults (DANGER). It appears
    nowhere else, so red anywhere = act now.
  * AMBER is reserved for warnings/abnormal-but-not-alarm (WARN): rejected
    commands, failed validation, attention needed.
  * Muted BLUE marks operator interaction only: selection, focus, the
    checked state of a control (ACCENT). Low salience by design.

Colors are exposed as constants so tabs tint status text consistently. The
semantic names are kept from the old theme (SUCCESS/DANGER/WARN/INFO), but
note SUCCESS is now a neutral dark gray — per ISA-101 a normal/OK state is
shown without color.
"""

from __future__ import annotations

# Palette (ISA-101 gray family)
BG = "#e4e6e6"          # window
PANEL = "#dbdddd"       # group boxes / cards
INPUT = "#f4f5f5"       # fields, light surfaces
BORDER = "#acb1b4"
TEXT = "#22282b"        # primary text (near-black)
TEXT_DIM = "#5d6467"    # secondary text
ACCENT = "#33689c"      # operator interaction: selection / focus / checked
ACCENT_HOVER = "#3f7ab3"
SUCCESS = "#333b3f"     # NORMAL/OK: neutral dark — normal carries no color
DANGER = "#c62828"      # ALARM only (faults). Reserved: appears nowhere else.
WARN = "#b97b00"        # WARNING only (abnormal, attention). Amber, reserved.
INFO = "#3e5c70"        # informational messages: muted blue-gray, low salience
VIDEO_BG = "#c9ccce"    # camera/image view surround (neutral, non-fatiguing)

# Status pill backgrounds. Alarm pills are the ONLY high-chroma elements on
# screen (red/amber field), so they dominate visual salience as ISA-101 intends.
PILL = {
    "ok": ("#cbd0d2", SUCCESS),      # normal: neutral gray, no color
    "bad": (DANGER, "#ffffff"),      # alarm: solid red field, white text
    "warn": ("#e8ae1b", "#22282b"),  # warning: amber field, black text
    "info": ("#c9d3da", INFO),
    "idle": ("#d4d7d8", TEXT_DIM),
}


def pill_style(kind: str) -> str:
    bg, fg = PILL.get(kind, PILL["idle"])
    return (
        f"background:{bg}; color:{fg}; border-radius:10px; "
        f"padding:4px 12px; font-weight:600;"
    )


ISA_QSS = f"""
* {{ font-family:'Segoe UI','DejaVu Sans',sans-serif; font-size:13px; }}
QWidget {{ background-color:{BG}; color:{TEXT}; }}
QMainWindow, QDialog {{ background-color:{BG}; }}
QToolTip {{ background:{INPUT}; color:{TEXT}; border:1px solid {BORDER}; }}

/* Tabs */
QTabWidget::pane {{ border:1px solid {BORDER}; border-radius:6px; top:-1px; background:{PANEL}; }}
QTabBar::tab {{
    background:transparent; color:{TEXT_DIM}; padding:9px 20px; margin-right:2px;
    border:1px solid transparent; border-top-left-radius:5px; border-top-right-radius:5px;
    font-weight:600;
}}
QTabBar::tab:selected {{ background:{PANEL}; color:{TEXT}; border:1px solid {BORDER};
    border-bottom:2px solid {ACCENT}; }}
QTabBar::tab:hover:!selected {{ color:{TEXT}; }}

/* Group boxes */
QGroupBox {{ background:{PANEL}; border:1px solid {BORDER}; border-radius:6px;
    margin-top:16px; padding:12px 12px 10px 12px; font-weight:600; }}
QGroupBox::title {{ subcontrol-origin:margin; left:12px; padding:0 6px; color:{TEXT_DIM}; }}

/* Buttons — flat gray; the label, not color, says what they do */
QPushButton {{ background:#e9eaea; border:1px solid {BORDER}; border-radius:5px;
    padding:7px 15px; color:{TEXT}; }}
QPushButton:hover {{ background:#dfe1e1; border-color:#8f9598; }}
QPushButton:pressed {{ background:#d3d6d6; }}
QPushButton:checked {{ background:{ACCENT}; border-color:{ACCENT}; color:#ffffff; }}
QPushButton:disabled {{ background:#dddfdf; color:#9aa0a3; border-color:#c4c8ca; }}
QPushButton[accent="primary"] {{ background:{ACCENT}; border-color:{ACCENT}; color:#ffffff; }}
QPushButton[accent="primary"]:hover {{ background:{ACCENT_HOVER}; }}
/* Abort/stop controls: neutral field, dark-red text+border. Not a red lamp —
   solid red stays reserved for active alarms. */
QPushButton[accent="danger"] {{ background:#e9eaea; border:1px solid #8c1f1f; color:#8c1f1f;
    font-weight:600; }}
QPushButton[accent="danger"]:hover {{ background:#f2dede; }}

/* Inputs */
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{ background:#fbfbfb; border:1px solid {BORDER};
    border-radius:4px; padding:5px 7px; color:{TEXT}; selection-background-color:{ACCENT};
    selection-color:#ffffff; }}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{ border-color:{ACCENT}; }}
QComboBox::drop-down {{ border:none; width:18px; }}
QComboBox QAbstractItemView {{ background:#fbfbfb; border:1px solid {BORDER};
    selection-background-color:{ACCENT}; selection-color:#ffffff; outline:none; }}
QSpinBox::up-button, QDoubleSpinBox::up-button, QSpinBox::down-button,
QDoubleSpinBox::down-button {{ background:#dfe1e1; border:none; width:16px; }}

/* Tables */
QTableWidget, QTableView {{ background:#fbfbfb; alternate-background-color:#eff1f1;
    gridline-color:#d3d6d6; border:1px solid {BORDER}; border-radius:5px; outline:none; }}
QHeaderView::section {{ background:#dfe1e1; color:{TEXT_DIM}; padding:7px 8px; border:none;
    border-right:1px solid {BORDER}; border-bottom:1px solid {BORDER}; font-weight:600; }}
QTableWidget::item {{ padding:2px 4px; }}
QTableWidget::item:selected {{ background:{ACCENT}; color:#ffffff; }}

/* Sliders */
QSlider::groove:horizontal {{ height:5px; background:#c3c7c9; border-radius:2px; }}
QSlider::sub-page:horizontal {{ background:{ACCENT}; border-radius:2px; }}
QSlider::handle:horizontal {{ background:#ffffff; border:1px solid {BORDER};
    width:14px; margin:-6px 0; border-radius:7px; }}
QSlider::handle:horizontal:hover {{ border-color:{ACCENT}; }}

/* Scrollbars */
QScrollBar:vertical {{ background:transparent; width:12px; margin:2px; }}
QScrollBar::handle:vertical {{ background:#bfc3c5; border-radius:6px; min-height:28px; }}
QScrollBar::handle:vertical:hover {{ background:#a9adb0; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height:0; }}
QScrollBar:horizontal {{ background:transparent; height:12px; margin:2px; }}
QScrollBar::handle:horizontal {{ background:#bfc3c5; border-radius:6px; min-width:28px; }}
QScrollArea {{ border:none; }}

QCheckBox {{ spacing:8px; }}
QCheckBox::indicator {{ width:16px; height:16px; border:1px solid {BORDER}; border-radius:3px;
    background:#fbfbfb; }}
QCheckBox::indicator:checked {{ background:{ACCENT}; border-color:{ACCENT}; }}
"""

# Backwards-compatible alias (pre-ISA name).
DARK_QSS = ISA_QSS


def apply_theme(app) -> None:
    """Apply the ISA-101 theme to a QApplication."""
    app.setStyleSheet(ISA_QSS)
