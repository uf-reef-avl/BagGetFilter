# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'playBagWindow.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(1048, 539)
        self.gridLayout = QtWidgets.QGridLayout(Dialog)
        self.gridLayout.setObjectName("gridLayout")
        self.groupBox = QtWidgets.QGroupBox(Dialog)
        self.groupBox.setObjectName("groupBox")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.groupBox)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.gridLayout_2 = QtWidgets.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.spinFactor = QtWidgets.QSpinBox(self.groupBox)
        self.spinFactor.setObjectName("spinFactor")
        self.gridLayout_2.addWidget(self.spinFactor, 2, 1, 1, 1)
        self.checkFrequency = QtWidgets.QCheckBox(self.groupBox)
        self.checkFrequency.setObjectName("checkFrequency")
        self.gridLayout_2.addWidget(self.checkFrequency, 0, 0, 1, 1)
        self.checkSkip = QtWidgets.QCheckBox(self.groupBox)
        self.checkSkip.setObjectName("checkSkip")
        self.gridLayout_2.addWidget(self.checkSkip, 2, 2, 1, 1)
        self.checkStart = QtWidgets.QCheckBox(self.groupBox)
        self.checkStart.setObjectName("checkStart")
        self.gridLayout_2.addWidget(self.checkStart, 0, 2, 1, 1)
        self.checkSleep = QtWidgets.QCheckBox(self.groupBox)
        self.checkSleep.setObjectName("checkSleep")
        self.gridLayout_2.addWidget(self.checkSleep, 1, 0, 1, 1)
        self.spinPartTime = QtWidgets.QSpinBox(self.groupBox)
        self.spinPartTime.setObjectName("spinPartTime")
        self.gridLayout_2.addWidget(self.spinPartTime, 1, 3, 1, 1)
        self.spinSleep = QtWidgets.QSpinBox(self.groupBox)
        self.spinSleep.setObjectName("spinSleep")
        self.gridLayout_2.addWidget(self.spinSleep, 1, 1, 1, 1)
        self.checkFactor = QtWidgets.QCheckBox(self.groupBox)
        self.checkFactor.setObjectName("checkFactor")
        self.gridLayout_2.addWidget(self.checkFactor, 2, 0, 1, 1)
        self.spinStart = QtWidgets.QSpinBox(self.groupBox)
        self.spinStart.setObjectName("spinStart")
        self.gridLayout_2.addWidget(self.spinStart, 0, 3, 1, 1)
        self.checkPartTime = QtWidgets.QCheckBox(self.groupBox)
        self.checkPartTime.setObjectName("checkPartTime")
        self.gridLayout_2.addWidget(self.checkPartTime, 1, 2, 1, 1)
        self.spinFrequency = QtWidgets.QSpinBox(self.groupBox)
        self.spinFrequency.setObjectName("spinFrequency")
        self.gridLayout_2.addWidget(self.spinFrequency, 0, 1, 1, 1)
        self.spinSkip = QtWidgets.QSpinBox(self.groupBox)
        self.spinSkip.setObjectName("spinSkip")
        self.gridLayout_2.addWidget(self.spinSkip, 2, 3, 1, 1)
        self.checkLoop = QtWidgets.QCheckBox(self.groupBox)
        self.checkLoop.setObjectName("checkLoop")
        self.gridLayout_2.addWidget(self.checkLoop, 3, 0, 1, 1)
        self.gridLayout_3.addLayout(self.gridLayout_2, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.groupBox, 3, 0, 1, 1)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.terminalPlainText = QtWidgets.QPlainTextEdit(Dialog)
        self.terminalPlainText.setObjectName("terminalPlainText")
        self.verticalLayout.addWidget(self.terminalPlainText)
        self.progressBar = QtWidgets.QProgressBar(Dialog)
        self.progressBar.setProperty("value", 24)
        self.progressBar.setObjectName("progressBar")
        self.verticalLayout.addWidget(self.progressBar)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.pushButton = QtWidgets.QPushButton(Dialog)
        self.pushButton.setObjectName("pushButton")
        self.horizontalLayout.addWidget(self.pushButton)
        self.buttonPause = QtWidgets.QPushButton(Dialog)
        self.buttonPause.setObjectName("buttonPause")
        self.horizontalLayout.addWidget(self.buttonPause)
        self.buttonStep = QtWidgets.QPushButton(Dialog)
        self.buttonStep.setObjectName("buttonStep")
        self.horizontalLayout.addWidget(self.buttonStep)
        self.buttonStop = QtWidgets.QPushButton(Dialog)
        self.buttonStop.setObjectName("buttonStop")
        self.horizontalLayout.addWidget(self.buttonStop)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.gridLayout.addLayout(self.verticalLayout, 1, 0, 1, 1)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Play bag window"))
        self.groupBox.setTitle(_translate("Dialog", "Play options"))
        self.checkFrequency.setText(_translate("Dialog", "Publish clock time at frequency HZ (default =100)"))
        self.checkSkip.setText(_translate("Dialog", "Skip regions in the bag with no messages for more than SEC seconds. "))
        self.checkStart.setText(_translate("Dialog", "Start SEC seconds into the bags. "))
        self.checkSleep.setText(_translate("Dialog", "Sleep SEC seconds after every advertise call  "))
        self.checkFactor.setText(_translate("Dialog", "Multiply the publish rate by FACTOR. "))
        self.checkPartTime.setText(_translate("Dialog", "Play only SEC seconds from the bag files. "))
        self.checkLoop.setText(_translate("Dialog", "Loop playback. "))
        self.pushButton.setText(_translate("Dialog", "Play"))
        self.buttonPause.setText(_translate("Dialog", "Pause"))
        self.buttonStep.setText(_translate("Dialog", "Step"))
        self.buttonStop.setText(_translate("Dialog", "Stop"))

