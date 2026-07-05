"""PLC tab: the tag contract the Studio 5000 program must implement.

Read-only reference generated from plc.tags.TAG_SPECS (the single source of
truth), so it never drifts from what the Python side actually reads/writes.
Includes an Export CSV button for handing the list to the controls engineer.
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QFileDialog,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ..plc import tags as T

_COLUMNS = ["Group", "Tag", "Type", "Direction", "Description"]
_CMD_TINT = QColor("#e8f0fe")   # PC → PLC
_STATUS_TINT = QColor("#e9f7ec")  # PLC → PC


class PlcTab(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        specs = T.TAG_SPECS
        n_cmd = sum(1 for s in specs if s.direction == T.PC_TO_PLC)
        n_status = len(specs) - n_cmd

        root = QVBoxLayout(self)
        root.addWidget(
            QLabel(
                "PLC tag contract — the Studio 5000 program must implement every "
                "tag below (base name <b>VisionRobot</b>).\n"
                f"{len(specs)} tags: {n_cmd} commands/targets (PC → PLC, Python "
                f"writes) and {n_status} status (PLC → PC, Python reads)."
            )
        )

        self.table = self._build_table(specs)
        root.addWidget(self.table)

        row = QHBoxLayout()
        export = QPushButton("Export CSV…")
        export.clicked.connect(self._on_export)
        row.addStretch(1)
        row.addWidget(export)
        root.addLayout(row)

    def _build_table(self, specs) -> QTableWidget:
        table = QTableWidget(len(specs), len(_COLUMNS))
        table.setHorizontalHeaderLabels(_COLUMNS)
        table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(True)

        for r, s in enumerate(specs):
            tint = _CMD_TINT if s.direction == T.PC_TO_PLC else _STATUS_TINT
            for c, value in enumerate(
                (s.group, s.name, s.dtype, s.direction, s.description)
            ):
                item = QTableWidgetItem(value)
                item.setBackground(tint)
                table.setItem(r, c, item)

        header = table.horizontalHeader()
        for c in range(len(_COLUMNS) - 1):
            header.setSectionResizeMode(c, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(
            len(_COLUMNS) - 1, QHeaderView.ResizeMode.Stretch
        )
        return table

    def _on_export(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "Export PLC tag list", "plc_tags.csv", "CSV files (*.csv)"
        )
        if path:
            with open(path, "w", newline="") as fh:
                fh.write(T.tag_table_csv())
