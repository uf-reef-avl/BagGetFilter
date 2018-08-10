# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'bag_filter.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_dialog(object):
    def setupUi(self, dialog):
        dialog.setObjectName("dialog")
        dialog.resize(1096, 657)
        self.gridLayout = QtWidgets.QGridLayout(dialog)
        self.gridLayout.setObjectName("gridLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(dialog)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.lineBagFile = QtWidgets.QLineEdit(dialog)
        self.lineBagFile.setEnabled(False)
        self.lineBagFile.setObjectName("lineBagFile")
        self.horizontalLayout.addWidget(self.lineBagFile)
        self.buttonLoadBagPath = QtWidgets.QPushButton(dialog)
        self.buttonLoadBagPath.setObjectName("buttonLoadBagPath")
        self.horizontalLayout.addWidget(self.buttonLoadBagPath)
        self.buttonClipboard = QtWidgets.QPushButton(dialog)
        self.buttonClipboard.setObjectName("buttonClipboard")
        self.horizontalLayout.addWidget(self.buttonClipboard)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.textClipboard = QtWidgets.QPlainTextEdit(dialog)
        self.textClipboard.setObjectName("textClipboard")
        self.verticalLayout.addWidget(self.textClipboard)
        self.gridLayout.addLayout(self.verticalLayout, 0, 0, 1, 1)
        self.progressBar = QtWidgets.QProgressBar(dialog)
        self.progressBar.setProperty("value", 24)
        self.progressBar.setObjectName("progressBar")
        self.gridLayout.addWidget(self.progressBar, 7, 0, 1, 1)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.buttonPlayBag = QtWidgets.QPushButton(dialog)
        self.buttonPlayBag.setObjectName("buttonPlayBag")
        self.horizontalLayout_3.addWidget(self.buttonPlayBag)
        self.buttonQuit = QtWidgets.QPushButton(dialog)
        self.buttonQuit.setObjectName("buttonQuit")
        self.horizontalLayout_3.addWidget(self.buttonQuit)
        self.buttonSaveBag = QtWidgets.QPushButton(dialog)
        self.buttonSaveBag.setObjectName("buttonSaveBag")
        self.horizontalLayout_3.addWidget(self.buttonSaveBag)
        self.gridLayout.addLayout(self.horizontalLayout_3, 8, 0, 1, 1)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.treeSelectedTopics = QtWidgets.QTreeWidget(dialog)
        self.treeSelectedTopics.setObjectName("treeSelectedTopics")
        self.horizontalLayout_2.addWidget(self.treeSelectedTopics)
        self.gridLayout.addLayout(self.horizontalLayout_2, 5, 0, 1, 1)
        self.labelProgress = QtWidgets.QLabel(dialog)
        self.labelProgress.setObjectName("labelProgress")
        self.gridLayout.addWidget(self.labelProgress, 6, 0, 1, 1)

        self.retranslateUi(dialog)
        QtCore.QMetaObject.connectSlotsByName(dialog)

    def retranslateUi(self, dialog):
        _translate = QtCore.QCoreApplication.translate
        dialog.setWindowTitle(_translate("dialog", "Bag Filter"))
        self.label.setText(_translate("dialog", "Specify the path of your bag"))
        self.buttonLoadBagPath.setText(_translate("dialog", "Select your file"))
        self.buttonClipboard.setText(_translate("dialog", "Show Clipboard"))
        self.buttonPlayBag.setText(_translate("dialog", "Play selected topics"))
        self.buttonQuit.setText(_translate("dialog", "Save selected topics to csv files"))
        self.buttonSaveBag.setText(_translate("dialog", "Save selected topics in a filtered bag"))
        self.treeSelectedTopics.headerItem().setText(0, _translate("dialog", "Bags"))
        self.treeSelectedTopics.headerItem().setText(1, _translate("dialog", "Topics"))
        self.treeSelectedTopics.headerItem().setText(2, _translate("dialog", "TF Frames"))
        self.labelProgress.setText(_translate("dialog", "TextLabel"))

