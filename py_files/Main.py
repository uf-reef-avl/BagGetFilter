#!/usr/bin/python2.7
#
# File: Main.py
# Author: Paul Buzaud
#
# Created:
#

from PyQt5 import QtCore, QtGui, QtWidgets
import BagFilterDesign
from PlayBagWindow import BagPlay
import os
import sys
import rosbag, sys, csv
import time
import string
import tf2_ros



class fileBrowser(QtWidgets.QFileDialog):

    # Initializes and defines the Multilaunch window
    def __init__(self):
        QtWidgets.QFileDialog.__init__(self)
        buttonBox = self.findChild(QtWidgets.QDialogButtonBox)
        lineEdit = self.findChild(QtWidgets.QLineEdit)
        label = self.findChildren(QtWidgets.QLabel)
        combo = self.findChildren(QtWidgets.QComboBox)
        label[1].hide()
        label[2].hide()
        combo[1].hide()
        lineEdit.hide()
        buttonBox.hide()
        self.setNameFilter("bag (*.bag *.)")


    def accept(self):
        self.show()


# This class creates the main window of the application
class BagFilter(QtWidgets.QDialog, BagFilterDesign.Ui_dialog):

    # Initializes and defines the Multilaunch window
    def __init__(self):
        super(self.__class__, self).__init__()
        self.setupUi(self)

        self.labelProgress.hide()
        self.progressBar.hide()
        self.textClipboard.hide()
        self.treeSelectedTopics.setSelectionMode(QtWidgets.QAbstractItemView.MultiSelection)
        self.setTreeSize()


        self.buttonPlayBag.setText("Play the bag")
        # self.fileDialog = QtWidgets.QFileDialog()
        self.fileDialog = fileBrowser()
        self.horizontalLayout_2.insertWidget(0, self.fileDialog, 0)
        self.horizontalLayout_2.setStretch(0, 1)
        self.horizontalLayout_2.setStretch(1, 2)
        self.fileDialog.show()

        self.playBagWindow = BagPlay()

        # Paring buttons to functions
        # self.fileDialog.mouseDoubleClickEvent.connect(self.doNothing)
        self.buttonLoadBagPath.clicked.connect(self.loadBagClick)
        self.buttonSaveBag.clicked.connect(self.saveNewBag)
        self.buttonQuit.clicked.connect(self.saveCsvFile)
        self.buttonPlayBag.clicked.connect(self.playBag)
        self.buttonClipboard.clicked.connect(self.showClipboard)
        self.setAcceptDrops(True)

        self.listOfTopics = []
        self.dragDropEnable = True

        self.bagSize = 0
        self.tfDict = {}
    #
    # def doNothing(self):
    #     print "hoho"
    #Corrects the column header sizes
    def setTreeSize(self):
        self.treeSelectedTopics.setColumnWidth(0, self.width() / 3.3)
        self.treeSelectedTopics.setColumnWidth(1, self.width() / 3.3)
        self.treeSelectedTopics.setColumnWidth(2, self.width() / 3.3)



    def showClipboard(self):
        if self.textClipboard.isVisible():
            self.textClipboard.hide()
        else:
            self.textClipboard.show()

    def loadBagClick(self):
        self.loadBag("")

    def enableDisableButton(self, available):
        self.buttonPlayBag.setEnabled(available)
        self.buttonSaveBag.setEnabled(available)
        self.buttonLoadBagPath.setEnabled(available)
        self.buttonQuit.setEnabled(available)
        self.dragDropEnable = available

    # Runs the Edit Robot Dialog window if there are no other threads running
    def loadBag(self, filename):
        self.tfDict = {}
        self.enableDisableButton(False)
        if filename == "":

            filePath = QtWidgets.QFileDialog.getOpenFileName(self, "Find your bag file", filter="bag (*.bag *.)")

            filePath = filePath[0]
        else:
            filePath = filename

        if filePath[-4:] == ".bag":

            self.treeSelectedTopics.clear()
            bagItem = QtWidgets.QTreeWidgetItem()
            bagItem.setText(0,filePath)


            bag = rosbag.Bag(filePath)
            bagContents = bag.read_messages()
            bagName = bag.filename
            self.listOfTopics = []
            self.labelProgress.setText("Loading Bag "+ filePath)
            self.labelProgress.show()
            self.progressBar.show()
            self.progressBar.setValue(0.)
            self.bagSize = bag.get_message_count()

            i = 0.
            self.tfDict[filePath] = [[],[]]
            for topic, msg, t in bagContents:
                self.progressBar.setValue(int(float(i) / self.bagSize * 100))
                i += 1
                if topic not in self.listOfTopics:
                    item = QtWidgets.QTreeWidgetItem()
                    item.setText(1,topic)
                    bagItem.addChild(item)
                    self.listOfTopics.append(topic)
                    if topic == "/tf":
                        tf_item  = item

                if topic ==  "/tf":
                    try:
                        for index in range(len(msg.transforms)):
                            msg_string = "frame_id : "+str(msg.transforms[index].header.frame_id)+" | child_frame_id : "+str(msg.transforms[index].child_frame_id)
                            stringId = self.tfDict[filePath][0].index(msg_string) if msg_string in self.tfDict[filePath][0] else -1
                            if msg_string not in self.tfDict[filePath][0]:
                                self.tfDict[filePath][0].append(msg_string)
                                self.tfDict[filePath][1].append([index])
                                sub_item = QtWidgets.QTreeWidgetItem()
                                sub_item.setText(2,msg_string)
                                tf_item.addChild(sub_item)
                            elif stringId != -1 and index not in self.tfDict[filePath][1][stringId]:
                                self.tfDict[filePath][1][stringId].append(index)
                    except:
                        pass
            print str(self.tfDict)
            bag.close()

            self.treeSelectedTopics.addTopLevelItem(bagItem)
            self.treeSelectedTopics.expandAll()
            self.progressBar.hide()
            self.labelProgress.hide()
            self.lineBagFile.setText(filePath)
            self.textClipboard.clear()
            self.textClipboard.appendPlainText('<node pkg="rosbag" type="play" name="player" output="screen" args="--clock '+filePath+'"/>')
            lastSlashIndex= filePath.rindex('/')
            self.initialBagDirectory = self.lineBagFile.text()[:lastSlashIndex]
        elif filePath != "":
            QtWidgets.QMessageBox.warning(self, "Warning", "It's not a bag file")

        self.enableDisableButton(True)

    def saveCsvFile(self):
        self.enableDisableButton(False)
        if str(self.lineBagFile.text())[-4:] == ".bag" and len(self.treeSelectedTopics.selectedItems()) != 0:
            folder = QtWidgets.QFileDialog.getExistingDirectory(self, "Choose a folder to store your csv topics", self.initialBagDirectory)
            if folder:
                bag = rosbag.Bag(self.lineBagFile.text())
                bagContents = bag.read_messages()
                bagName = bag.filename

                self.labelProgress.show()
                self.progressBar.show()
                self.progressBar.setValue(0.)
                fileCreated = False
                numberOftTopicsSelected = len(self.treeSelectedTopics.selectedItems())
                for index, topicName in enumerate(self.listOfTopics):

                    topicSize = bag.get_message_count(topicName)
                    currentItem = self.treeSelectedTopics.topLevelItem(0).child(index)
                    if currentItem.text(1) != "/tf":
                        if (currentItem in self.treeSelectedTopics.selectedItems()):
                            # Create a new CSV file for each topic
                            filename = folder + '/' + string.replace(topicName, '/', '_slash_') + '.csv'
                            self.labelProgress.setText("Save csv file " + filename)
                            with open(filename, 'w+') as csvfile:
                                fileCreated = True
                                filewriter = csv.writer(csvfile, delimiter=',')
                                firstIteration = True  # allows header row
                                indice = 0
                                for subtopic, msg, t in bag.read_messages(
                                        topicName):  # for each instant in time that has data for topicName
                                    # parse data from this instant, which is of the form of multiple lines of "Name: value\n"
                                    #	- put it in the form of a list of 2-element lists
                                    indice += 1
                                    self.progressBar.setValue(int((float(
                                        indice) / topicSize * 100)))
                                    msgString = str(msg)
                                    msgList = string.split(msgString, '\n')
                                    instantaneousListOfData = []
                                    for nameValuePair in msgList:
                                        splitPair = string.split(nameValuePair, ':')
                                        for i in range(len(splitPair)):  # should be 0 to 1
                                            splitPair[i] = string.strip(splitPair[i])
                                        if len(splitPair) == 2:
                                            instantaneousListOfData.append(splitPair)
                                    # write the first row from the first element of each pair
                                    if firstIteration:  # header
                                        headers = ["rosbagTimestamp"]  # first column header
                                        for pair in instantaneousListOfData:
                                            headers.append(pair[0])
                                        filewriter.writerow(headers)
                                        firstIteration = False
                                    # write the value from each pair to the file
                                    values = [str(t)]  # first column will have rosbag timestamp
                                    for pair in instantaneousListOfData:
                                        values.append(pair[1])
                                    filewriter.writerow(values)
                    else:

                        for i in range(currentItem.childCount()):
                            tfItem = currentItem.child(i)
                            if (tfItem in self.treeSelectedTopics.selectedItems()):

                                frameTfTopicString = str(tfItem.text(2)).split('|')
                                frameId = frameTfTopicString[0].split(':')[1].strip()
                                childFrameId = frameTfTopicString[1].split(':')[1].strip()
                                filename = folder + '/tf_frame_id' + frameId.replace('/', '_slash_')+"_child_frame_id"+ childFrameId.replace('/', '_slash_')+ '.csv'
                                self.labelProgress.setText("Save csv file " + filename)
                                with open(filename, 'w+') as csvfile:
                                    fileCreated = True
                                    filewriter = csv.writer(csvfile, delimiter=',')
                                    firstIteration = True  # allows header row
                                    indiceTf = 0
                                    for subtopic, msg, t in bag.read_messages(
                                            topicName):  # for each instant in time that has data for topicName
                                        # parse data from this instant, which is of the form of multiple lines of "Name: value\n"
                                        #	- put it in the form of a list of 2-element lists
                                        indiceTf += 1
                                        self.progressBar.setValue(int((float(
                                            indiceTf) / topicSize) * 100 ))
                                        msgString = str(msg)
                                        transformList = string.split(msgString, '  - \n')
                                        transformList = transformList[1:]


                                        for indexTransform, transformString in enumerate(transformList):
                                            msgList = string.split(transformString, '\n')
                                            instantaneousListOfData = []
                                            for nameValuePair in msgList:
                                                splitPair = string.split(nameValuePair, ':')
                                                for i in range(len(splitPair)):  # should be 0 to 1
                                                    splitPair[i] = string.strip(splitPair[i])
                                                if len(splitPair) == 2:
                                                    instantaneousListOfData.append(splitPair)

                                            # write the first row from the first element of each pair
                                            if firstIteration:  # header
                                                headers = ["rosbagTimestamp"]  # first column header
                                                for pair in instantaneousListOfData:
                                                    headers.append(pair[0])
                                                filewriter.writerow(headers)
                                                firstIteration = False

                                            if instantaneousListOfData[5][1][1:-1] == frameId and instantaneousListOfData[6][1][1:-1] == childFrameId:
                                                # write the value from each pair to the file
                                                values = [str(t)]  # first column will have rosbag timestamp
                                                # print instantaneousListOfData

                                                for pair in instantaneousListOfData:
                                                        values.append(pair[1])
                                                filewriter.writerow(values)


                bag.close()
                self.labelProgress.hide()
                self.progressBar.hide()
                if fileCreated:
                    QtWidgets.QMessageBox.information(self, "Information", "The csv files have been successfully created")
                else:
                    QtWidgets.QMessageBox.warning(self, "Warning", "No valid topic selected")


        elif len(self.treeSelectedTopics.selectedItems()) == 0:
            QtWidgets.QMessageBox.warning(self, "Warning", "No topics selected")
        elif str(self.lineBagFile.text())[-4:] != ".bag":
            QtWidgets.QMessageBox.warning(self, "Warning", "No bag selected")

        self.enableDisableButton(True)


    def saveNewBag(self):
        self.enableDisableButton(False)
        if len(self.treeSelectedTopics.selectedItems()) != 0:
            filePath = QtWidgets.QFileDialog.getSaveFileName(self, "Choose a name for your file", self.initialBagDirectory,
                                                             filter="bag (*.bag *.)")
            if filePath[0]:
                if filePath[0][-4:] != ".bag":
                    filename = filePath[0] + '.bag'
                    # commandString = 'rosbag filter ' + self.lineBagFile.text().replace(' ', '\ ') + ' ' + filePath[0].replace(' ', '\ ') + '.bag "'
                else:
                    filename = filePath[0]
                    # commandString = 'rosbag filter ' + self.lineBagFile.text().replace(' ', '\ ') + ' ' + filePath[0].replace(' ', '\ ') + ' "'

                # tfTopicSelected = False
                # tfCommand = ""
                #
                # for currentItem in self.treeSelectedTopics.selectedItems():
                #     if (currentItem.parent() != None) and (currentItem.text(1) != "/tf") and (currentItem.parent().text(1) != "/tf"):
                #         commandString += "topic =='" + currentItem.text(1) + "' or "
                #
                #     if (currentItem.parent() != None) and (currentItem.parent().text(1) == "/tf"):
                #         frameTfTopicString = str(currentItem.text(2)).split('|')
                #         frameId = frameTfTopicString[0].split(':')[1].strip()
                #         childFrameId = frameTfTopicString[1].split(':')[1].strip()
                #
                #         childId = self.tfDict[str(self.lineBagFile.text())][0].index(str(currentItem.text(2)))
                #         indexList = self.tfDict[str(self.lineBagFile.text())][1][childId]
                #
                #         for i in indexList:
                #             tfCommand += "(topic =='/tf' and ("+str(i)+" < len(m.transforms)) and m.transforms[" + str(
                #                 i) + "].header.frame_id == '" + frameId + "' and m.transforms[" + str(
                #                 i) + "].child_frame_id == '" + childFrameId + "') or "
                #         tfTopicSelected = True
                #
                #
                # if tfTopicSelected:
                #     commandString += tfCommand
                #
                #
                # if commandString[-4:] == " or ":
                #     commandString = commandString[:-4]
                #     commandString += '"'
                #
                #     print commandString
                #     os.system(commandString)
                #     informationString = "The new bag has been successfully created. \n"
                #     if tfTopicSelected:
                #         commandString += " Warning : The tf topics have been filtered but as eacg tf message can handle multiples bodyframes definition, there may be some residual tf topics "
                #
                #     QtWidgets.QMessageBox.information(self, "Information", informationString)
                #     self.loadBag(filename)

                tfTopicSelected = False
                tfSelection = []
                topicSelection = []


                for currentItem in self.treeSelectedTopics.selectedItems():
                    tfTopicSelected = True
                    if (currentItem.parent() != None) and (currentItem.text(1) != "/tf") and (
                            currentItem.parent().text(1) != "/tf"):
                        topicSelection.append(currentItem.text(1))

                    if (currentItem.parent() != None) and (currentItem.parent().text(1) == "/tf"):
                        frameTfTopicString = str(currentItem.text(2)).split('|')
                        frameId = frameTfTopicString[0].split(':')[1].strip()
                        childFrameId = frameTfTopicString[1].split(':')[1].strip()

                        childId = self.tfDict[str(self.lineBagFile.text())][0].index(str(currentItem.text(2)))
                        indexList = self.tfDict[str(self.lineBagFile.text())][1][childId]
                        tfSelection.append([frameId,childFrameId,indexList])

                self.labelProgress.show()
                self.progressBar.show()
                i = 0
                self.labelProgress.setText("Creating new bag "+filename)
                with rosbag.Bag(filename, 'w') as outbag:
                    for topic, msg, t in rosbag.Bag(self.lineBagFile.text()).read_messages():
                        i += 1
                        self.progressBar.setValue(int((float(
                            i) / self.bagSize) * 100))
                        # This also replaces tf timestamps under the assumption
                        # that all transforms in the message share the same timestamp
                        if topic == "/tf" and len(msg.transforms) == 1:
                            for tfMsg in tfSelection:
                                if tfMsg[0] == msg.transforms[0].header.frame_id and tfMsg[1] == msg.transforms[0].child_frame_id:
                                    outbag.write(topic, msg, t)
                        elif topic == "/tf" and len(msg.transforms) > 1:
                            inSelection = False
                            temp_msg = tf2_ros.TFMessage()

                            for tfMsg in tfSelection:
                                for index in tfMsg[2]:
                                    try:
                                        if tfMsg[0] == msg.transforms[index].header.frame_id and tfMsg[1] == msg.transforms[index].child_frame_id:
                                            temp_msg.transforms.append(msg.transforms[index])
                                            inSelection = True
                                    except:
                                        pass

                            if inSelection:
                                outbag.write(topic, temp_msg, t)
                        else:
                            if topic in topicSelection:
                                outbag.write(topic, msg, t)
                self.progressBar.hide()
                self.progressBar.hide()
                if tfTopicSelected:
                    QtWidgets.QMessageBox.information(self, "Information", "The new bag has been successfully created. \n")
                    self.loadBag(filename)
                else:
                    QtWidgets.QMessageBox.warning(self, "Warning", "No valid topic selected")
        else:
            QtWidgets.QMessageBox.warning(self, "Warning", "No topic selected")

        self.enableDisableButton(True)

    def playBag(self):
        if str(self.lineBagFile.text())[-4:] == ".bag":

            self.playBagWindow.bagPath = str(self.lineBagFile.text())
            self.playBagWindow.show()

        else:
            QtWidgets.QMessageBox.warning(self, "Warning", "No bag selected")

    def closeEvent(self, event):

        event.accept()  # let the window close

    def dragEnterEvent(self, e):
        if e.mimeData().hasText() and self.dragDropEnable:
            e.accept()
        else:
            e.ignore()

    def dropEvent(self, e):
        urls = e.mimeData().urls()
        url = urls[0]
        filename = url.path()
        self.loadBag(filename)


# Creates and runs the Main Window
def main():
    app = QtWidgets.QApplication(sys.argv)
    form = BagFilter()
    form.show()
    app.exec_()


# Calls main
if __name__ == '__main__':
    main()
