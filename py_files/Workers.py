#!/usr/bin/python3
# File: Workers.py

from PyQt5 import QtCore
import os


from PyQt5 import QtCore, QtGui, QtWidgets
import rospy
import rosbag,  csv
from std_msgs.msg import String
from tf2_ros import TFMessage
import numpy as np



class plot_loader(QtCore.QObject):

    #Variables for emitting starting, displaying progress, and closing signals
    start = QtCore.pyqtSignal()
    progressSignal = QtCore.pyqtSignal(str,int)
    finishThread = QtCore.pyqtSignal(str,str,dict)


    #Definition of a SSH_Transfer_File_Worker
    def __init__(self, id, bag_filename, bag_name, bag_index,list_of_topics ,treeSelectedTopic):
        super(plot_loader, self).__init__()
        self.bag_filename = bag_filename
        self.bag_name = bag_name
        self.bag_index = bag_index
        self.listOfTopics = list_of_topics
        self.treeSelectedTopics = treeSelectedTopic
        self.id = id
        self.start.connect(self.run)
        self.output_headers = []
        self.output_data = []
        self.all_bag_data = {}



    #This function connects to the remote robot and performs a git pull operation on the selected remote repository
    @QtCore.pyqtSlot()
    def run(self):
        # self.progressSignal.emit("blbbllbblbl " )
        # Load the bag
        self.progressSignal.emit("Loading bag " + self.bag_filename + " into baggetfilter , it can take time ...", 50)
        bag = rosbag.Bag(str(self.bag_filename))
        bagContents = bag.read_messages()
        bagName = str(self.bag_name)
        self.progressSignal.emit("Finished  " + self.bag_filename + " loading bag into baggetfilter !", 95)
        # variable to check if the csv files has been created
        fileCreated = False

        # iterate on all topics selected by the user
        for index, topicName in enumerate(
                self.listOfTopics[str(self.treeSelectedTopics.topLevelItem(self.bag_index).text(0))]):
            topicSize = bag.get_message_count(topicName)
            currentItem = self.treeSelectedTopics.topLevelItem(self.bag_index).child(index)

            # if the current topic is not a tf topic
            if currentItem.text(1) != "/tf":

                # if the current topic has been selected by the user
                if (currentItem in self.treeSelectedTopics.selectedItems()):

                    self.progressSignal.emit("Loading plot", -1)
                    firstIteration = True  # allows header row
                    indice = 0

                    # Iterate over all messages of the current bag
                    for subtopic, msg, t in bag.read_messages(
                            topicName):  # for each instant in time that has data for topicName
                        # parse data from this instant, which is of the form of multiple lines of "Name: value\n"
                        #	- put it in the form of a list of 2-element lists
                        indice += 1
                        # update the progress bar
                        self.progressSignal.emit("", int((float(
                            indice) / topicSize * 100)))


                        msgString = str(msg)
                        msgList = msgString.split('\n')
                        instantaneousListOfData = []
                        for nameValuePair in msgList:
                            splitPair = nameValuePair.split(':')
                            for i in range(len(splitPair)):  # should be 0 to 1
                                splitPair[i] = splitPair[i].strip()
                            if len(splitPair) == 2:
                                instantaneousListOfData.append(splitPair)
                        # write the first row from the first element of each pair
                        if firstIteration:  # header
                            self.output_headers = ["rosbagTimestamp"]  # first column header
                            for pair in instantaneousListOfData:
                                self.output_headers.append(pair[0])
                            firstIteration = False
                        # write the value from each pair to the file
                        values = [str(t)]  # first column will have rosbag timestamp
                        for pair in instantaneousListOfData:
                            values.append(pair[1])
                        self.output_data.append(values)
            else:
                # If the current topic is a tf topic
                # Iterate over the tf child of this topic
                for i in range(currentItem.childCount()):
                    tfItem = currentItem.child(i)

                    # if the current tf topic has been selected by the user
                    if (tfItem in self.treeSelectedTopics.selectedItems()):

                        # Retrieve the frame id and the child id from the tf item
                        frameTfTopicString = str(tfItem.text(2)).split('|')
                        frameId = frameTfTopicString[0].split(':')[1].strip()
                        childFrameId = frameTfTopicString[1].split(':')[1].strip()

                        self.progressSignal.emit("Loading plots", -1)
                        firstIteration = True  # allows header row
                        indiceTf = 0
                        for subtopic, msg, t in bag.read_messages(
                                topicName):  # for each instant in time that has data for topicName
                            # parse data from this instant, which is of the form of multiple lines of "Name: value\n"
                            #	- put it in the form of a list of 2-element lists
                            indiceTf += 1
                            self.progressSignal.emit("", int((float(
                                indice) / topicSize * 100)))

                            msgString = str(msg)

                            # Split the tf message thanks to the transformation separator and store every transformations messages in a list
                            transformList = msgString.split('  - \n')
                            transformList = transformList[1:]

                            # iterate on all the transformations messages
                            for indexTransform, transformString in enumerate(transformList):
                                msgList = transformString.split('\n')
                                instantaneousListOfData = []
                                for nameValuePair in msgList:
                                    splitPair = nameValuePair.split(':')
                                    for i in range(len(splitPair)):  # should be 0 to 1
                                        splitPair[i] = splitPair[i].strip()
                                    if len(splitPair) == 2:
                                        instantaneousListOfData.append(splitPair)

                                # write the first row from the first element of each pair
                                if firstIteration:  # header
                                    self.output_headers = ["rosbagTimestamp"]  # first column header
                                    for pair in instantaneousListOfData:
                                        self.output_headers.append(pair[0])
                                    firstIteration = False

                                if instantaneousListOfData[5][1][1:-1] == frameId and instantaneousListOfData[6][1][
                                                                                      1:-1] == childFrameId:
                                    values = [str(t)]  # first column will have rosbag timestamp
                                    for pair in instantaneousListOfData:
                                        values.append(pair[1])
                                    self.output_data.append(values)
            self.all_bag_data[topicName] = [self.output_headers,self.output_data]
        # Close the bag
        bag.close()

        self.finishThread.emit(self.id, self.bag_name,self.all_bag_data)




class CSV_Worker(QtCore.QObject):

    #Variables for emitting starting, displaying progress, and closing signals
    start = QtCore.pyqtSignal()
    progressSignal = QtCore.pyqtSignal(str,int)
    finishThread = QtCore.pyqtSignal(str)


    #Definition of a SSH_Transfer_File_Worker
    def __init__(self, id, bag_filename, bag_name, bag_index, folder,list_of_topics ,treeSelectedTopic):
        super(CSV_Worker, self).__init__()
        self.bag_filename = bag_filename
        self.bag_name = bag_name
        self.bag_index = bag_index
        self.folder = folder
        self.listOfTopics = list_of_topics
        self.treeSelectedTopics = treeSelectedTopic
        self.id = id
        self.start.connect(self.run)


    #This function connects to the remote robot and performs a git pull operation on the selected remote repository
    @QtCore.pyqtSlot()
    def run(self):
        # self.progressSignal.emit("blbbllbblbl " )
        # Load the bag
        self.progressSignal.emit("Loading bag " + self.bag_filename + " into baggetfilter , it can take time ...", 50)
        bag = rosbag.Bag(str(self.bag_filename))
        bagContents = bag.read_messages()
        bagName = str(self.bag_name)
        self.progressSignal.emit("Finished  " + self.bag_filename + " loading bag into baggetfilter !", 95)
        # variable to check if the csv files has been created
        fileCreated = False

        # iterate on all topics selected by the user
        for index, topicName in enumerate(
                self.listOfTopics[str(self.treeSelectedTopics.topLevelItem(self.bag_index).text(0))]):
            topicSize = bag.get_message_count(topicName)
            currentItem = self.treeSelectedTopics.topLevelItem(self.bag_index).child(index)

            # if the current topic is not a tf topic
            if currentItem.text(1) != "/tf":

                # if the current topic has been selected by the user
                if (currentItem in self.treeSelectedTopics.selectedItems()):

                    # Create a new CSV file for each topic
                    filename = self.folder + '/' + bagName + "_" + topicName.replace('/', '_slash_') + '.csv'
                    self.progressSignal.emit("Save csv file " + filename, -1)

                    with open(filename, 'w+') as csvfile:
                        fileCreated = True
                        filewriter = csv.writer(csvfile, delimiter=',')
                        firstIteration = True  # allows header row
                        indice = 0

                        # Iterate over all messages of the current bag
                        for subtopic, msg, t in bag.read_messages(
                                topicName):  # for each instant in time that has data for topicName
                            # parse data from this instant, which is of the form of multiple lines of "Name: value\n"
                            #	- put it in the form of a list of 2-element lists
                            indice += 1
                            # update the progress bar
                            self.progressSignal.emit("", int((float(
                                indice) / topicSize * 100)))


                            msgString = str(msg)
                            msgList = msgString.split('\n')
                            instantaneousListOfData = []
                            for nameValuePair in msgList:
                                splitPair = nameValuePair.split(':')
                                for i in range(len(splitPair)):  # should be 0 to 1
                                    splitPair[i] = splitPair[i].strip()
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
                # If the current topic is a tf topic
                # Iterate over the tf child of this topic
                for i in range(currentItem.childCount()):
                    tfItem = currentItem.child(i)

                    # if the current tf topic has been selected by the user
                    if (tfItem in self.treeSelectedTopics.selectedItems()):

                        # Retrieve the frame id and the child id from the tf item
                        frameTfTopicString = str(tfItem.text(2)).split('|')
                        frameId = frameTfTopicString[0].split(':')[1].strip()
                        childFrameId = frameTfTopicString[1].split(':')[1].strip()

                        # Create a new CSV file for each topic
                        filename = self.folder + '/' + bagName.replace(".", "_") + '_tf_frame_id' + frameId.replace('/',
                                                                                                               '_slash_') + "_child_frame_id_" + childFrameId.replace(
                            '/', '_slash_') + '.csv'
                        self.progressSignal.emit("Save csv file " + filename, -1)
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
                                self.progressSignal.emit("", int((float(
                                    indice) / topicSize * 100)))

                                msgString = str(msg)

                                # Split the tf message thanks to the transformation separator and store every transformations messages in a list
                                transformList = msgString.split('  - \n')
                                transformList = transformList[1:]

                                # iterate on all the transformations messages
                                for indexTransform, transformString in enumerate(transformList):
                                    msgList = transformString.split('\n')
                                    instantaneousListOfData = []
                                    for nameValuePair in msgList:
                                        splitPair = nameValuePair.split(':')
                                        for i in range(len(splitPair)):  # should be 0 to 1
                                            splitPair[i] = splitPair[i].strip()
                                        if len(splitPair) == 2:
                                            instantaneousListOfData.append(splitPair)

                                    # write the first row from the first element of each pair
                                    if firstIteration:  # header
                                        headers = ["rosbagTimestamp"]  # first column header
                                        for pair in instantaneousListOfData:
                                            headers.append(pair[0])
                                        filewriter.writerow(headers)
                                        firstIteration = False

                                    if instantaneousListOfData[5][1][1:-1] == frameId and instantaneousListOfData[6][1][
                                                                                          1:-1] == childFrameId:
                                        values = [str(t)]  # first column will have rosbag timestamp
                                        for pair in instantaneousListOfData:
                                            values.append(pair[1])
                                        filewriter.writerow(values)

        # Close the bag
        bag.close()



        # if fileCreated:
        #     QtWidgets.QMessageBox.information(self, "Information", "The csv files have been successfully created")


        #Closing the thread
        self.finishThread.emit(self.id)


class BagFilter_Worker(QtCore.QObject):

    #Variables for emitting starting, displaying progress, and closing signals
    start = QtCore.pyqtSignal()
    progressSignal = QtCore.pyqtSignal(str,int)
    finishThread = QtCore.pyqtSignal(str,str)


    #Definition of a SSH_Transfer_File_Worker
    def __init__(self, id, filename, index, treeSelectedTopics, topicSelection, tfSelection, dictBagsInfos, metaCheckbox,bagSize):
        super(BagFilter_Worker, self).__init__()
        self.id = id
        self.filename = filename
        self.treeSelectedTopics = treeSelectedTopics
        self.bagIndex = index
        self.tfSelection = tfSelection
        self.dictBagsInfos = dictBagsInfos
        self.metaCheckbox = metaCheckbox
        self.bagSize = bagSize
        self.topicSelection = topicSelection
        self.start.connect(self.run)


    #This function connects to the remote robot and performs a git pull operation on the selected remote repository
    @QtCore.pyqtSlot()
    def run(self):
        # Create a new Bag file for each topic
        self.progressSignal.emit("Loading bag " + self.filename + " into baggetfilter , it can take time ...", 50)
        with rosbag.Bag(self.filename, 'w') as outbag:
            if self.metaCheckbox:
                metadata_msg = String(data='time when the bag was recorded ')
                outbag.write('/metadata', metadata_msg, rospy.Time.from_sec(
                    self.dictBagsInfos[self.treeSelectedTopics.topLevelItem(self.bagIndex).text(0)][2]))
            i = 0

            for topic, msg, t in rosbag.Bag(self.treeSelectedTopics.topLevelItem(self.bagIndex).text(0)).read_messages():
                if i == 0:
                    self.progressSignal.emit("Writing filtered bag(s)", 0)
                # update ui elements
                i += 1

                self.progressSignal.emit("", int((float(
                    i) / self.bagSize * 100)))


                # if the message is a tf message and there is only one transformation in it
                if topic == "/tf" and len(msg.transforms) == 1:
                    for tfMsg in self.tfSelection:
                        # if the tf message match a selected tf topic
                        if tfMsg[0] == msg.transforms[0].header.frame_id and tfMsg[1] == msg.transforms[
                            0].child_frame_id:
                            outbag.write(topic, msg, t)

                # if the message is a tf message and there is more than one transformation in it
                elif topic == "/tf" and len(msg.transforms) > 1:
                    inSelection = False
                    temp_msg = TFMessage()

                    # iterate over the different transform messages
                    for tfMsg in self.tfSelection:
                        for index in tfMsg[2]:
                            # if the transformation message match a selected tf topic
                            if index < len(msg.transforms) and tfMsg[0] == msg.transforms[index].header.frame_id and \
                                    tfMsg[1] == msg.transforms[index].child_frame_id:
                                temp_msg.transforms.append(msg.transforms[index])
                                inSelection = True

                    if inSelection:
                        outbag.write(topic, temp_msg, t)
                else:
                    # if the message is a not a tf message and the topic of this message has been selected by the user
                    if topic in self.topicSelection:
                        outbag.write(topic, msg, t)
            if self.metaCheckbox:
                metadata_msg = String(data='time when the bag was ended ')
                outbag.write('/metadata', metadata_msg, rospy.Time.from_sec(
                    self.dictBagsInfos[self.treeSelectedTopics.topLevelItem(self.bagIndex).text(0)][3]))

        #Closing the thread
        self.finishThread.emit(self.id,self.filename)