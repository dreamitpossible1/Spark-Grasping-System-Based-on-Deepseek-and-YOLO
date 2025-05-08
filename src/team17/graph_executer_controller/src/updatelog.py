# coding=utf-8

import os, time
from src.team17.graph_executer_controller.utils.general import find_dir_path
from PySide6.QtWidgets import QDialog
from src.team17.graph_executer_controller.ui.ui_updatelog import Ui_Dialog
from PySide6.QtGui import QStandardItemModel, QStandardItem, QIcon

class UpdateLog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.setWindowIcon(QIcon(os.path.join(os.getcwd(), "settings", "myicon.png")))
        self.ui_updatelog = Ui_Dialog()
        self.ui_updatelog.setupUi(self)

