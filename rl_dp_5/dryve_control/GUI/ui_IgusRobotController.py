# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'IgusRobotControllerRcftNP.ui'
##
## Created by: Qt User Interface Compiler version 5.14.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import (QCoreApplication, QMetaObject, QObject, QPoint,
    QRect, QSize, QUrl, Qt)
from PySide2.QtGui import (QBrush, QColor, QConicalGradient, QCursor, QFont,
    QFontDatabase, QIcon, QLinearGradient, QPalette, QPainter, QPixmap,
    QRadialGradient)
from PySide2.QtWidgets import *


class Ui_IgusRobotControl(object):
    def setupUi(self, IgusRobotControl):
        if IgusRobotControl.objectName():
            IgusRobotControl.setObjectName(u"IgusRobotControl")
        IgusRobotControl.resize(826, 600)
        self.centralwidget = QWidget(IgusRobotControl)
        self.centralwidget.setObjectName(u"centralwidget")
        self.horizontalLayoutWidget = QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setObjectName(u"horizontalLayoutWidget")
        self.horizontalLayoutWidget.setGeometry(QRect(0, 0, 791, 51))
        self.horizontalLayout = QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.label_2 = QLabel(self.horizontalLayoutWidget)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setAlignment(Qt.AlignCenter)

        self.horizontalLayout.addWidget(self.label_2)

        self.label = QLabel(self.horizontalLayoutWidget)
        self.label.setObjectName(u"label")
        self.label.setAlignment(Qt.AlignCenter)

        self.horizontalLayout.addWidget(self.label)

        self.label_4 = QLabel(self.horizontalLayoutWidget)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setAlignment(Qt.AlignCenter)

        self.horizontalLayout.addWidget(self.label_4)

        self.label_3 = QLabel(self.horizontalLayoutWidget)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setAlignment(Qt.AlignCenter)

        self.horizontalLayout.addWidget(self.label_3)

        self.label_5 = QLabel(self.horizontalLayoutWidget)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setAlignment(Qt.AlignCenter)

        self.horizontalLayout.addWidget(self.label_5)

        self.horizontalLayoutWidget_2 = QWidget(self.centralwidget)
        self.horizontalLayoutWidget_2.setObjectName(u"horizontalLayoutWidget_2")
        self.horizontalLayoutWidget_2.setGeometry(QRect(0, 50, 791, 41))
        self.horizontalLayout_3 = QHBoxLayout(self.horizontalLayoutWidget_2)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.label_6 = QLabel(self.horizontalLayoutWidget_2)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_3.addWidget(self.label_6)

        self.j1_cur_position = QLabel(self.horizontalLayoutWidget_2)
        self.j1_cur_position.setObjectName(u"j1_cur_position")
        self.j1_cur_position.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_3.addWidget(self.j1_cur_position)

        self.j1_target_position = QLineEdit(self.horizontalLayoutWidget_2)
        self.j1_target_position.setObjectName(u"j1_target_position")
        self.j1_target_position.setMaximumSize(QSize(150, 16777215))
        self.j1_target_position.setInputMethodHints(Qt.ImhPreferNumbers)

        self.horizontalLayout_3.addWidget(self.j1_target_position)

        self.j1_target = QPushButton(self.horizontalLayoutWidget_2)
        self.j1_target.setObjectName(u"j1_target")

        self.horizontalLayout_3.addWidget(self.j1_target)

        self.j1_home = QPushButton(self.horizontalLayoutWidget_2)
        self.j1_home.setObjectName(u"j1_home")

        self.horizontalLayout_3.addWidget(self.j1_home)

        IgusRobotControl.setCentralWidget(self.centralwidget)
        self.statusbar = QStatusBar(IgusRobotControl)
        self.statusbar.setObjectName(u"statusbar")
        IgusRobotControl.setStatusBar(self.statusbar)

        self.retranslateUi(IgusRobotControl)

        QMetaObject.connectSlotsByName(IgusRobotControl)
    # setupUi

    def retranslateUi(self, IgusRobotControl):
        IgusRobotControl.setWindowTitle(QCoreApplication.translate("IgusRobotControl", u"MainWindow", None))
        self.label_2.setText(QCoreApplication.translate("IgusRobotControl", u"Joint Name", None))
        self.label.setText(QCoreApplication.translate("IgusRobotControl", u"Current Position", None))
        self.label_4.setText(QCoreApplication.translate("IgusRobotControl", u"Taget Position", None))
        self.label_3.setText(QCoreApplication.translate("IgusRobotControl", u"Targeting", None))
        self.label_5.setText(QCoreApplication.translate("IgusRobotControl", u"Homing", None))
        self.label_6.setText(QCoreApplication.translate("IgusRobotControl", u"Joint 1", None))
        self.j1_cur_position.setText(QCoreApplication.translate("IgusRobotControl", u"0", None))
        self.j1_target.setText(QCoreApplication.translate("IgusRobotControl", u"Target", None))
        self.j1_home.setText(QCoreApplication.translate("IgusRobotControl", u"Home", None))
    # retranslateUi

