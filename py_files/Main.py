#!/usr/bin/python3
#
# File: Main.py
# Author: Paul Buzaud
#
# Created: Summer 2018
#
# Copyright 2018 FIRSTNAME LASTNAME
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>

from PyQt5 import QtCore, QtGui, QtWidgets
from PlayBagWindow import BagPlay
import BagFilterDesign
import rosbag, sys, csv, os, rospy
import subprocess,yaml
from tf2_ros import TFMessage
from std_msgs.msg import String
from MplCanvas import MplCanvas
from Workers import CSV_Worker, BagFilter_Worker, plot_loader
import pandas as pd



class fileBrowser(QtWidgets.QFileDialog):
    """This class inherits from the QFileDialog class and defines the browser part of the main window"""

    def __init__(self):
        """Initialization of the class and hiding of the useless elements"""
        QtWidgets.QFileDialog.__init__(self)
        self.setOptions(QtWidgets.QFileDialog.DontUseNativeDialog)
        #Find the irrelevant parts of the QFileDialog
        buttonBox = self.findChild(QtWidgets.QDialogButtonBox)
        lineEdit = self.findChild(QtWidgets.QLineEdit)
        label = self.findChildren(QtWidgets.QLabel)
        combo = self.findChildren(QtWidgets.QComboBox)
        tree = self.findChildren(QtWidgets.QTreeView)

        #Change the selection mode of the tree widget
        file_tree = tree[0]
        file_tree.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)

        #Hide the irrelevant part of the QFileDialog
        label[1].hide()
        label[2].hide()
        combo[1].hide()
        lineEdit.hide()
        buttonBox.hide()
        self.setNameFilter("bag (*.bag *.)")

        self.setDirectory(str(os.getcwd()))




    def accept(self):
        """Function override of the accept function of the QFileDialog to hide when the user double click on a element of the browser"""
        self.show()


class BagFilter(QtWidgets.QDialog, BagFilterDesign.Ui_dialog):
    """This class creates the main window of the application"""


    def __init__(self):
        """Initialization of the class, hide some unneeded ui, and connect the qt signal with the qt slot"""
        super(self.__class__, self).__init__()
        self.setupUi(self)

        #Setup of the ui elements
        self.labelProgress.hide()
        self.progressBar.hide()
        self.textClipboard.hide()
        self.treeSelectedTopics.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.buttonPlayBag.setText("Play bags")
        self.fileDialog = fileBrowser()
        self.horizontalLayout_2.insertWidget(0, self.fileDialog, 0)
        self.horizontalLayout_2.setStretch(0, 1)
        self.horizontalLayout_2.setStretch(1, 2)
        self.fileDialog.show()
        self.lineBagFile.setText("_filtered")
        self.lineBagFile.setEnabled(True)
        self.label.setText("Specify the suffix of your filtered bag name")
        self.buttonLoadBagPath.setText("Clear Bags")
        self.setTreeSize()
        # self.mplCanvas = MplCanvas(self)
        # self.plotLayout.addWidget(self.mplCanvas)
        #self.mplCanvas.setHidden(True)


        # Paring buttons to functions
        self.buttonLoadBagPath.clicked.connect(self.clearTree)
        self.buttonSaveBag.clicked.connect(self.saveNewBag)
        self.buttonQuit.clicked.connect(self.saveCsvFile)
        self.buttonPlayBag.clicked.connect(self.playBag)
        self.buttonClipboard.clicked.connect(self.showClipboard)
        # self.buttonPlots.clicked.connect(self.togglePlots)
        # self.mplCanvas.button_load.clicked.connect(self.loadDataInsidePlot)
        # self.mplCanvas.button_plot.clicked.connect(self.showPlots)
        self.treeSelectedTopics.itemDoubleClicked.connect(self.editBagTimeStamp)
        self.treeSelectedTopics.itemChanged.connect(self.changeBagTimeStamp)
        self.lineResearch.textChanged.connect(self.changeTextlineResearch)
        self.treeSelectedTopics.itemSelectionChanged.connect(self.onSelectionChanged)

        #Activates the drag and drop functionality of the window
        self.setAcceptDrops(True)
        self.dragDropEnable = True


        #Initializes data structures
        self.bagSize = 0
        self.tfDict = {}
        self.listOfTopics = {}
        self.dictBagsInfos = {}
        self.worker_dict = {}
        self.thread_dict = {}
        self.listOfFilteredToLoadBags = []
        self.multiThreadedProgression = [0.,0.] #indice 0 is sum of progression, indice one is number of message

    # def togglePlots(self):
    #     self.mplCanvas.setHidden(not self.mplCanvas.isHidden())
    #
    # def showPlots(self):
    #     self.mplCanvas.plot()
    #
    #
    # @QtCore.pyqtSlot(str,str,dict)
    # def killPlotThread(self, id,bag_name,dict_data):
    #     del self.worker_dict[id]
    #     self.thread_dict[id].quit()
    #     self.thread_dict[id].wait()
    #     del self.thread_dict[id]
    #
    #     for topic in dict_data.keys():
    #         panda_data_frame =  pd.DataFrame(dict_data[topic][1],
    #
    #                columns=dict_data[topic][0])
    #         self.mplCanvas.load_data_topic( bag_name, topic, dict_data[topic][0], panda_data_frame)
    #     #if last bag habe been filtered and no more thread, reload the bags and reenable the ui
    #     if len(self.thread_dict.keys()) == 0:
    #         # update ui elements
    #         self.labelProgress.hide()
    #         self.progressBar.hide()
    #         # Reenable the function button of the ui
    #         self.enableDisableButton(True)
    #
    #
    # def loadDataInsidePlot(self):
    #
    #     """Load the topic in csv format"""
    #
    #     # Disable the functions buttons
    #     self.enableDisableButton(False)
    #
    #     # retrieve all the selected informations from the topics tree widget
    #     bagSelection, dictTopicSelection, dictTfSelection, meaningfullItemSelected = self.bagSelected()
    #
    #     if meaningfullItemSelected:
    #             # Update ui elements
    #             self.labelProgress.show()
    #             self.progressBar.show()
    #             self.progressBar.setValue(0.)
    #             self.multiThreadedProgression = [0, 0]
    #             # iterate on all the bags index selected
    #             for bagIndex in bagSelection:
    #                 tempThread = QtCore.QThread()
    #                 tempThread.start()
    #                 bag_filename = str(self.treeSelectedTopics.topLevelItem(bagIndex).text(0))
    #                 bag_name = str(self.treeSelectedTopics.topLevelItem(bagIndex).text(0)).split('/')[-1][:-4]
    #                 loader = plot_loader(str(tempThread), bag_filename, bag_name, bagIndex,
    #                                         self.listOfTopics, self.treeSelectedTopics)
    #                 loader.progressSignal.connect(self.updateProgressBar)
    #                 loader.finishThread.connect(self.killPlotThread)
    #                 loader.moveToThread(tempThread)
    #                 loader.start.emit()
    #                 self.thread_dict[str(tempThread)] = tempThread
    #                 self.worker_dict[str(tempThread)] = loader
    #
    #                 # if there is no item in the topic tree widget and the user try to launch the csv exportation
    #     elif self.treeSelectedTopics.topLevelItemCount() == 0:
    #         QtWidgets.QMessageBox.warning(self, "Warning", "No bag selected")
    #     # if no tf topics or meaningful topic has been selected
    #     else:
    #         QtWidgets.QMessageBox.warning(self, "Warning", "No topics selected")
    #
    #     # Disable the functions buttons
    #     self.enableDisableButton(False)





    def onSelectionChanged(self):
        self.label_topics_selected.setText("Topics selected : "+str(len(self.treeSelectedTopics.selectedItems())))


    def changeTextlineResearch(self,text):
        self.treeSelectedTopics.clearSelection()
        text_list = text.split("|")
        items = []
        for topics in text_list:
            items = items +  self.treeSelectedTopics.findItems(topics, QtCore.Qt.MatchContains|QtCore.Qt.MatchRecursive,1)
            items = items + self.treeSelectedTopics.findItems(topics,
                                                              QtCore.Qt.MatchContains | QtCore.Qt.MatchRecursive, 2)
        # items = items + self.treeSelectedTopics.findItems(text, QtCore.Qt.MatchContains,2)
        for item in items:
            item.setSelected(True)

    def editBagTimeStamp(self, item, column):
        item.setFlags(item.flags() | QtCore.Qt.ItemIsEditable)
        if (column != 3 or item.parent()!=None):
            item.setFlags(item.flags()  & ~QtCore.Qt.ItemIsEditable)


    def changeBagTimeStamp(self, item, column):

        self.treeSelectedTopics.itemChanged.disconnect()

        if (column == 3):
            bagName = item.text(0)
            oldTimeStamped  = str(self.dictBagsInfos[bagName][2])
            try:
                newTimeStamped = float(item.text(3))
                self.enableDisableButton(False)

                self.labelProgress.setText("Editing bag "+bagName+" timestamp" )
                self.labelProgress.show()
                self.progressBar.show()
                self.progressBar.setValue(0.)

                bag = rosbag.Bag(bagName)

                splitBagName =  bagName.split("_timestamped_")
                if len(splitBagName) == 1 :
                    tempBagName = splitBagName[0][:-4]
                else:
                    tempBagName = splitBagName[0]

                newBagName = tempBagName+"_timestamped_"+str(newTimeStamped).replace(".","_")+".bag"

                if self.checkMeta.checkState() != 2:
                    newBagName = newBagName[:-4] + "_with_meta.bag"

                outbag = rosbag.Bag(newBagName, "w", options=bag.options )
                i = 0.

                if self.checkMeta.checkState() != 2:
                    metadata_msg = String(data='time when the bag was recorded ')
                    outbag.write('/metadata', metadata_msg, rospy.Time.from_sec(newTimeStamped))

                for topic, msg, t in bag.read_messages():
                    if self.checkMeta.checkState() == 2 and i == 0:
                        oldTimeStamped = t.to_sec()

                    i += 1
                    self.progressBar.setValue(int(float(i) / float(self.dictBagsInfos[bagName][0]) * 100))
                    newT = rospy.Time.from_sec((t.to_sec() - float(oldTimeStamped)) + newTimeStamped)
                    outbag.write(topic, msg, newT)

                if self.checkMeta.checkState() != 2:
                    metadata_msg = String(data='time when the bag was ended')
                    outbag.write('/metadata', metadata_msg, rospy.Time.from_sec((self.dictBagsInfos[bagName][3] - float(oldTimeStamped)) + newTimeStamped))

                outbag.close()
                bag.close()

                #os.system("mv -f "+newBagName+" "+bagName)

                if self.treeSelectedTopics.findItems(newBagName, QtCore.Qt.MatchExactly, 0) == []:
                    self.loadBag(newBagName)


                self.labelProgress.hide()
                self.progressBar.hide()
                self.enableDisableButton(True)
            except ValueError:
               QtWidgets.QMessageBox.warning(self, "Warning", "Enter only numbers")
            except TypeError:
               QtWidgets.QMessageBox.warning(self, "Warning", "Enter a positive number")

            item.setText(3, str(oldTimeStamped))
        self.treeSelectedTopics.itemChanged.connect(self.changeBagTimeStamp)


    def setTreeSize(self):
        """This function corrects the column header sizes of the tree widget which displays the topics and bags topics"""
        self.treeSelectedTopics.setColumnWidth(0, self.treeSelectedTopics.width() / 4.)
        self.treeSelectedTopics.setColumnWidth(1, self.treeSelectedTopics.width() / 4.)
        self.treeSelectedTopics.setColumnWidth(2, self.treeSelectedTopics.width() / 4.)
        self.treeSelectedTopics.setColumnWidth(3, self.treeSelectedTopics.width() / 4.)



    def showClipboard(self):
        """Show or hide the clipboard when the user click on the clipboard button"""
        if self.textClipboard.isVisible():
            self.textClipboard.hide()
        else:
            self.textClipboard.show()


    def clearTree(self):
        """Reset all the data structures and flush the tree widget"""
        self.treeSelectedTopics.clear()
        self.textClipboard.clear()
        self.listOfTopics = {}
        self.tfDict = {}
        # self.dictSameTypesItem = {}
        self.dictBagsInfos = {}


    def enableDisableButton(self, available):
        """Lock/unlock the ui by enabling or disabling button events when loading or processing bags"""
        self.buttonPlayBag.setEnabled(available)
        self.buttonSaveBag.setEnabled(available)
        self.buttonLoadBagPath.setEnabled(available)
        self.buttonQuit.setEnabled(available)
        self.dragDropEnable = available


    def loadBag(self, filename):
        """Read the topics contained in the bag as an argument and update the bag/topic tree widget and the data structures """
        #Disable the functions buttons
        self.enableDisableButton(False)

        #Check if the filename is not empty
        if filename == "":
            filePath = QtWidgets.QFileDialog.getOpenFileName(self, "Find your bag file", filter="bag (*.bag *.)")
            filePath = filePath[0]
        else:
            filePath = filename

        #Check if the specified file is a bag
        if filePath[-4:] == ".bag":
            self.labelProgress.setText("Loading Bag " + filePath)
            self.labelProgress.show()
            self.progressBar.show()
            self.progressBar.setValue(0.)
            # Create the bag item in the bag/topic tree widget
            bagItem = QtWidgets.QTreeWidgetItem()
            bagItem.setText(0, filePath)
            self.listOfTopics[filePath] = []
            self.tfDict[filePath] = [[], []]
            if self.checkEnableTF.checkState() == 2:
                # Load the bag
                self.labelProgress.setText("Loading bag into baggetfilter " + filePath + ", it can take time ...")
                self.progressBar.setValue(50)
                bag = rosbag.Bag(filePath)
                self.progressBar.setValue(95)
                self.labelProgress.setText("Loading TF of " + filePath)
                bagContents = bag.read_messages()
                bagName = bag.filename
                bagItem.setText(3, str(bag.get_start_time()))
                # Update the ui elements and the data structures
                self.bagSize = bag.get_message_count()
                #i variable is  used to update the progress bar
                i = 0.
                #Read the bag contents
                for topic, msg, t in bagContents:
                    self.progressBar.setValue(int(float(i) / self.bagSize * 100))
                    i += 1

                    #If the current topic hasn't been loaded in the tree widget then add it
                    if topic not in self.listOfTopics[filePath]:
                        item = QtWidgets.QTreeWidgetItem()
                        item.setText(1,topic)
                        bagItem.addChild(item)
                        self.listOfTopics[filePath].append(topic)
                        if topic == "/tf":
                            tf_item  = item

                    #if the current topic is tf, check if the frame and the child frame of this transformation have been added to the data structure and to the tree widget; if not then add it
                    if topic ==  "/tf":
                        try:
                            #in every tf msg, there can be multiples transformations messages.This iteration goes throught all theses transformations messages and save them in data structures
                            for index in range(len(msg.transforms)):
                                msg_string = "frame_id : "+str(msg.transforms[index].header.frame_id)+" | child_frame_id : "+str(msg.transforms[index].child_frame_id)
                                stringId = self.tfDict[filePath][0].index(msg_string) if msg_string in self.tfDict[filePath][0] else -1
                                if msg_string not in self.tfDict[filePath][0]:
                                    self.tfDict[filePath][0].append(msg_string)
                                    self.tfDict[filePath][1].append([index])
                                    sub_item = QtWidgets.QTreeWidgetItem()
                                    sub_item.setText(2,msg_string)
                                    tf_item.addChild(sub_item)
                                #register each index of transformation messages
                                elif stringId != -1 and index not in self.tfDict[filePath][1][stringId]:
                                    self.tfDict[filePath][1][stringId].append(index)
                        except:
                            pass
                bag.close()
                self.dictBagsInfos[filePath] = [self.bagSize, bag.get_end_time() - bag.get_start_time(),
                                                bag.get_start_time(), bag.get_end_time()]
            elif self.checkEnableTF.checkState() == 0:
                info_dict = yaml.load(
                    subprocess.Popen(['rosbag', 'info', '--yaml', filePath], stdout=subprocess.PIPE).communicate()[
                        0],yaml.Loader)

                # Load the bag
                bagItem.setText(3, str(info_dict["start"]))
                # Update the ui elements and the data structures

                self.bagSize = info_dict["messages"]

                for topic in info_dict["topics"]:
                    item = QtWidgets.QTreeWidgetItem()
                    item.setText(1, topic["topic"])
                    bagItem.addChild(item)
                    self.listOfTopics[filePath].append(topic["topic"])
                self.dictBagsInfos[filePath] = [self.bagSize, info_dict["end"]- info_dict["start"],
                                                info_dict["start"], info_dict["end"]]

            #update the bag/topic tree widget
            self.treeSelectedTopics.addTopLevelItem(bagItem)
            self.treeSelectedTopics.expandAll()
            self.treeSelectedTopics.setSortingEnabled(True)
            self.treeSelectedTopics.sortByColumn(1,QtCore.Qt.AscendingOrder)
            self.treeSelectedTopics.sortByColumn(2, QtCore.Qt.AscendingOrder)
            self.treeSelectedTopics.setSortingEnabled(False)




            #update ui elements at the end of the loading
            self.progressBar.hide()
            self.labelProgress.hide()
            self.textClipboard.appendPlainText('<node pkg="rosbag" type="play" name="player" output="screen" args="--clock '+filePath+'"/> \n')
            lastSlashIndex= filePath.rindex('/')

        #If the selected file is not a bag file
        elif filePath != "":
            QtWidgets.QMessageBox.warning(self, "Warning", "It's not a bag file")

        #Reenable the function button of the ui
        self.enableDisableButton(True)

        #changing directory
        filename_list = filename.split("/")
        filename_list = filename_list[:-1]
        directory = "/".join(filename_list)
        self.fileDialog.setDirectory(directory)


    def bagSelected(self):
        """This function retrieves and sorts the selected bags, selected topics, and selected tf topics from the topic tree widget.  It then returns them in temporary dictionaries and list"""

        bagSelection = []
        dictTopicSelection = {}
        dictTfSelection = {}
        #if some topic item or tf item are selected then set this variable to true
        meaningfullItemSelected = False

        #iterate over all the selected items
        for currentItem in self.treeSelectedTopics.selectedItems():

            #if the current selected item is a topic item
            if (currentItem.parent() != None) and (currentItem.text(1) != "/tf") and (currentItem.parent().text(1) != "/tf"):
                bagItem = currentItem.parent()

                #save the bagitem index selected if it is not already in the bagSelection list
                if self.treeSelectedTopics.indexOfTopLevelItem(bagItem) not in bagSelection:
                    bagSelection.append(self.treeSelectedTopics.indexOfTopLevelItem(bagItem))

                #store the topic name selected if it is not already in the topic selected dictionary
                if bagItem.text(0) not in dictTopicSelection.keys():
                    dictTopicSelection[bagItem.text(0)] = []
                    dictTopicSelection[bagItem.text(0)].append(currentItem.text(1))
                else:
                    dictTopicSelection[bagItem.text(0)].append(currentItem.text(1))
                meaningfullItemSelected = True

            # if the current selected item is a tf item
            if (currentItem.parent() != None) and (currentItem.parent().text(1) == "/tf"):
                bagItem = currentItem.parent().parent()

                #save the bagitem index selected if it is not already in the bagSelection list
                if self.treeSelectedTopics.indexOfTopLevelItem(bagItem) not in bagSelection:
                    bagSelection.append(self.treeSelectedTopics.indexOfTopLevelItem(bagItem))

                #retrieve the frame id and the child id from the tf item
                frameTfTopicString = str(currentItem.text(2)).split('|')
                frameId = frameTfTopicString[0].split(':')[1].strip()
                childFrameId = frameTfTopicString[1].split(':')[1].strip()
                childId = self.tfDict[bagItem.text(0)][0].index(str(currentItem.text(2)))
                indexList = self.tfDict[bagItem.text(0)][1][childId]

                # store the ttf topic name selected if it is not already in the tf topic selected dictionary
                if bagItem.text(0) not in dictTfSelection.keys():
                    dictTfSelection[bagItem.text(0)] = []
                    dictTfSelection[bagItem.text(0)].append([frameId, childFrameId, indexList])
                else:
                    dictTfSelection[bagItem.text(0)].append([frameId, childFrameId, indexList])

                meaningfullItemSelected = True

        return bagSelection, dictTopicSelection, dictTfSelection, meaningfullItemSelected



    @QtCore.pyqtSlot(str, int)
    def updateProgressBar(self, text, progression):
        if text != "":
            self.labelProgress.setText(text)
        if progression != -1:
            self.multiThreadedProgression[0] += progression
            self.multiThreadedProgression[1] += 1
            progression_mean = self.multiThreadedProgression[0] / self.multiThreadedProgression[1]
            self.progressBar.setValue(progression_mean)

    @QtCore.pyqtSlot(str)
    def killThread(self, id):
        del self.worker_dict[id]
        self.thread_dict[id].quit()
        self.thread_dict[id].wait()
        del self.thread_dict[id]
        # Update ui elements
        self.labelProgress.hide()
        self.progressBar.hide()
        # Reenable the function button of the ui
        self.enableDisableButton(True)


    def saveCsvFile(self):
            """Save the selected topics in csv files"""

            #Disable the functions buttons
            self.enableDisableButton(False)

            #retrieve all the selected informations from the topics tree widget
            bagSelection, dictTopicSelection, dictTfSelection, meaningfullItemSelected = self.bagSelected()

            if meaningfullItemSelected:
                #let the user choose the directory where he wants to store the csv file
                folder =  QtWidgets.QFileDialog.getExistingDirectory(self, "Choose a folder to store your csv topics", self.fileDialog.directoryUrl().path())
                #check if the user has chosen a valid folder
                if folder:
                    # Update ui elements
                    self.labelProgress.show()
                    self.progressBar.show()
                    self.progressBar.setValue(0.)
                    self.multiThreadedProgression = [0,0]
                    #iterate on all the bags index selected
                    for bagIndex in bagSelection:
                        tempThread = QtCore.QThread()
                        tempThread.start()
                        bag_filename = str(self.treeSelectedTopics.topLevelItem(bagIndex).text(0))
                        bag_name = str(self.treeSelectedTopics.topLevelItem(bagIndex).text(0)).split('/')[-1][:-4]
                        csv_worker = CSV_Worker(str(tempThread),bag_filename,bag_name,bagIndex,folder,self.listOfTopics,self.treeSelectedTopics)
                        csv_worker.progressSignal.connect(self.updateProgressBar)
                        csv_worker.finishThread.connect(self.killThread)
                        csv_worker.moveToThread(tempThread)
                        csv_worker.start.emit()
                        self.thread_dict[str(tempThread)] = tempThread
                        self.worker_dict[str(tempThread)] = csv_worker


                        #if there is no item in the topic tree widget and the user try to launch the csv exportation
            elif self.treeSelectedTopics.topLevelItemCount() == 0:
                QtWidgets.QMessageBox.warning(self, "Warning", "No bag selected")
            #if no tf topics or meaningful topic has been selected
            else:
                QtWidgets.QMessageBox.warning(self, "Warning", "No topics selected")


    @QtCore.pyqtSlot(str,str)
    def killFilterThread(self, id,filename):
        del self.worker_dict[id]
        self.thread_dict[id].quit()
        self.thread_dict[id].wait()
        del self.thread_dict[id]

        # load the newly created bags in the topic tree widget
        if self.checkLoad.checkState() == 2:
            self.listOfFilteredToLoadBags.append(filename)
        #if last bag habe been filtered and no more thread, reload the bags and reenable the ui
        if len(self.thread_dict.keys()) == 0:
            self.clearTree()
            for bag in self.listOfFilteredToLoadBags:
                self.loadBag(bag)
            self.listOfFilteredToLoadBags =[]
            QtWidgets.QMessageBox.information(self, "Information", "The new bag has been successfully created. \n")
            # update ui elements
            self.labelProgress.hide()
            self.progressBar.hide()
            # Reenable the function button of the ui
            self.enableDisableButton(True)

    def saveNewBag(self):
        """ This function filter the topic selected by the user in a new bag"""

        # Disable the functions buttons
        self.enableDisableButton(False)

        listFileName = []

        #if there is at least one bag loaded in the topic tree widget
        if len(self.treeSelectedTopics.selectedItems()) != 0:
            # let the user choose the directory where he wants to store the csv file
            folder = QtWidgets.QFileDialog.getExistingDirectory(self, "Choose a folder to store the new filtered bag",
                                                                self.fileDialog.directoryUrl().path())

            # check if the user has chosen a valid folder
            if folder:

                # retrieve all the selected informations from the topics tree widget
                bagSelection, dictTopicSelection, dictTfSelection, meaningfullItemSelected = self.bagSelected()

                if meaningfullItemSelected:

                    #iterate over the bag selection
                    for bagIndex in bagSelection:
                        #variable to update the progress bar
                        i = 0
                        # retrieve all the selected informations from current bag
                        if self.lineBagFile.text()[0] != "_":
                            suffix = "_" + self.lineBagFile.text()
                        else:
                            suffix = self.lineBagFile.text()

                        filename = folder+"/"+self.treeSelectedTopics.topLevelItem(bagIndex).text(0).split('/')[-1].replace(".","_") + suffix +".bag"
                        tfSelection = dictTfSelection[self.treeSelectedTopics.topLevelItem(bagIndex).text(0)] if self.treeSelectedTopics.topLevelItem(bagIndex).text(0) in  dictTfSelection.keys() else []
                        topicSelection = dictTopicSelection[self.treeSelectedTopics.topLevelItem(bagIndex).text(0)] if self.treeSelectedTopics.topLevelItem(bagIndex).text(0) in  dictTopicSelection.keys() else []

                        #update ui elements
                        self.labelProgress.show()
                        self.progressBar.show()
                        self.multiThreadedProgression = [0, 0]
                        listFileName.append(filename)
                        self.labelProgress.setText("Creating new bag : "+filename)
                        tempThread = QtCore.QThread()
                        tempThread.start()
                        filter_worker = BagFilter_Worker(str(tempThread),filename,bagIndex,self.treeSelectedTopics,topicSelection,tfSelection,self.dictBagsInfos,self.checkMeta.checkState(), self.bagSize)
                        filter_worker.progressSignal.connect(self.updateProgressBar)
                        filter_worker.finishThread.connect(self.killFilterThread)
                        filter_worker.moveToThread(tempThread)
                        filter_worker.start.emit()
                        self.thread_dict[str(tempThread)] = tempThread
                        self.worker_dict[str(tempThread)] = filter_worker



                # if no tf topics or meaningful topic has been selected
                else:
                    QtWidgets.QMessageBox.warning(self, "Warning", "No valid topic selected")

        # If there is no item in the topic tree widget and the user try to launch the bag filter
        else:
            QtWidgets.QMessageBox.warning(self, "Warning", "No bag selected")



    def playBag(self):
        """Display the play bag window"""
        if self.treeSelectedTopics.topLevelItemCount() != 0:
            self.playBagWindow = BagPlay(self.dictBagsInfos)
            self.playBagWindow.show()

        else:
            QtWidgets.QMessageBox.warning(self, "Warning", "No bag loaded")


    def dragEnterEvent(self, e):
        """Accept the drag and drop event"""
        if e.mimeData().hasText() and self.dragDropEnable:
            e.accept()
        else:
            e.ignore()


    def dropEvent(self, e):
        """Define the behavior when an item is dropped in the Main Window"""

        #Retrieve the paths of the items
        urls = e.mimeData().urls()
        errorString = ""
        #iterate over all the path
        for url in urls:
            filename = url.path()
            bagAlreadyInTree = False
            #Check if the bag has been already added to the topic tree widget
            for topItemIndex in range(self.treeSelectedTopics.topLevelItemCount()):
                if self.treeSelectedTopics.topLevelItem(topItemIndex).text(0) == filename:
                    bagAlreadyInTree = True

            #throw an error when the bag has aleady been loaded
            if not bagAlreadyInTree:
                self.loadBag(filename)
            else:
                errorString += "the bag : "+filename+" is already loaded \n"

        if errorString != "":
            QtWidgets.QMessageBox.warning(self, "Warning", errorString)


    def resizeEvent(self, event):
        """Adjust the header of the topic tree widget every time the size of the main window is updated"""
        self.setTreeSize()

    def closeEvent(self, event):
        for worker in self.worker_dict.items():
            del worker
        for thread in self.thread_dict.items():
            thread.quit()
            thread.wait()
            del thread



def main():
    """Create and run the Main Window"""
    app = QtWidgets.QApplication(sys.argv)
    form = BagFilter()
    form.showMaximized()
    app.exec_()


# Call main
if __name__ == '__main__':
    main()
