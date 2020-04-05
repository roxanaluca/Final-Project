# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gui/gui.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(768, 410)
        font = QtGui.QFont()
        font.setFamily("Bitstream Vera Sans Mono")
        font.setPointSize(9)
        font.setBold(False)
        font.setWeight(50)
        MainWindow.setFont(font)
        MainWindow.setDocumentMode(False)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setMaximumSize(QtCore.QSize(768, 450))
        self.centralwidget.setObjectName("centralwidget")
        self.label_img0 = QtWidgets.QLabel(self.centralwidget)
        self.label_img0.setGeometry(QtCore.QRect(0, 0, 384, 384))
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.label_img0.setFont(font)
        self.label_img0.setCursor(QtGui.QCursor(QtCore.Qt.CrossCursor))
        self.label_img0.setMouseTracking(True)
        self.label_img0.setFrameShape(QtWidgets.QFrame.Box)
        self.label_img0.setFrameShadow(QtWidgets.QFrame.Plain)
        self.label_img0.setScaledContents(True)
        self.label_img0.setAlignment(QtCore.Qt.AlignCenter)
        self.label_img0.setObjectName("label_img0")
        self.label_img1 = QtWidgets.QLabel(self.centralwidget)
        self.label_img1.setGeometry(QtCore.QRect(385, 0, 384, 384))
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.label_img1.setFont(font)
        self.label_img1.setCursor(QtGui.QCursor(QtCore.Qt.CrossCursor))
        self.label_img1.setMouseTracking(True)
        self.label_img1.setFrameShape(QtWidgets.QFrame.Box)
        self.label_img1.setScaledContents(True)
        self.label_img1.setAlignment(QtCore.Qt.AlignCenter)
        self.label_img1.setObjectName("label_img1")
        self.count = LabelQml(self.centralwidget)
        self.count.setGeometry(QtCore.QRect(360, 129, 100, 100))
        font = QtGui.QFont()
        font.setPointSize(72)
        font.setBold(True)
        font.setWeight(75)
        self.count.setFont(font)
        self.count.setText("")
        self.count.setObjectName("count")
        MainWindow.setCentralWidget(self.centralwidget)
        self.calibrate = QtWidgets.QPushButton(MainWindow)
        self.calibrate.setGeometry(QtCore.QRect(0, 386, 100, 20))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.calibrate.setFont(font)
        self.calibrate.setObjectName("calibrate")

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "EyeCameras application"))
        self.label_img0.setText(_translate("MainWindow", "IMG0"))
        self.label_img1.setText(_translate("MainWindow", "IMG1"))
        self.calibrate.setText(_translate("MainWindow", " Calibrate "))

from widgets.LabelQml import LabelQml
