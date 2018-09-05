#!/usr/bin/python2.7
#
# File: Main.py
# Author: Paul Buzaud
#
# Created:
#

from PyQt5 import QtCore, QtGui, QtWidgets
import PlayBagDesign
import RoscoreNode
import pexpect
import sys



class BagWorker(QtCore.QObject):
    """This class launch a bag play command and a roscore in some other thread"""

    start = QtCore.pyqtSignal()
    terminalInformation = QtCore.pyqtSignal(str, int)
    finishThread = QtCore.pyqtSignal()


    def __init__(self, bagPath, optionArguments):
        """Creates and runs the Bag_Worker class and its methods"""
        super(BagWorker, self).__init__()
        self.start.connect(self.run)
        self.stopSignal = False
        self.responseString = ""
        self.bagPath = bagPath
        self.optionArguments = optionArguments

    # This function pings the robot
    @QtCore.pyqtSlot()
    def run(self):
        """Send a bagplay command to the system and retrieve the std output"""

        #launch a roscore
        roscoreNode = RoscoreNode.Roscore()
        roscoreNode.run()

        #create the command string
        cmd = "rosbag play "+self.bagPath.replace(' ', '\ ')+self.optionArguments

        #add the possibles arguments to the command
        if self.optionArguments != []:
            cmd += self.optionArguments

        #launch the command
        self.bagPlayProcess = pexpect.spawn(cmd, timeout=None, logfile=sys.stdout)

        #initialisation of temporary variables
        line = ""
        tempLine = ""
        lineComplete = False
        progressionValue = 0


        #while the command process is not finished and some output data are in the pipe
        while True:

            #read the next character
            output = self.bagPlayProcess.read(1)

            #exit when the user sent a stop query or some data are still exchanged between the process and this thread
            if output == '' or output == None or self.stopSignal == True:
                break


            if output:
                #flush the templine et reset the linecomplete to false
                if lineComplete:
                    lineComplete = False
                    tempLine = ""

                #when the beginning of the new line is detected set the linecomplete boolean to true, retrieve the progression information and emit it to the display widget
                if output == "[":
                    lineComplete = True
                    self.terminalInformation.emit("["+tempLine, progressionValue)
                    tempLine = tempLine[:-1]
                    try:
                        durationList = tempLine.split("Duration: ")
                        if len(durationList) > 1:
                            durationLine = durationList[1]
                            number_list = durationLine.split(' ')
                            actualTime = float(number_list[0].strip())
                            bagDuration = float(number_list[2].strip())
                            progressionValue = actualTime / bagDuration * 100
                    except:
                        pass

                #add the new received character to the temporary line
                tempLine += str(output)

        # kill the thread
        self.finishThread.emit()
        #close the roscore
        roscoreNode.terminate()


    def pauseBagHandler(self):
        """Pause the bag progression when the user send a bag pause request"""
        self.bagPlayProcess.sendline("\x20")

    def stopBagHandler(self):
        """Finish the process when the user send a bag stop request"""
        self.bagPlayProcess.sendline("\x03")

    def stepBagHandler(self):
        """Read the bag step by step every time the user click on the step button"""
        self.bagPlayProcess.sendline("s")

# This class creates the main window of the application
class BagPlay(QtWidgets.QDialog, PlayBagDesign.Ui_dialog):
    """This class defines the window which play the selected bag"""

    def __init__(self, bagsInfo):
        """Initialization of the class, hide some of the ui part, connect the qt signal with the qt slot"""
        super(self.__class__, self).__init__()
        self.setupUi(self)
        self.setModal(True)
        self.bagsInfo = bagsInfo

        # Setup of the ui elements

        for key, value in bagsInfo.items():
            item = QtWidgets.QTreeWidgetItem()
            item.setText(0,key)
            item.setText(1,str(value[0]))
            item.setText(2, str(value[1]))
            self.treeBag.addTopLevelItem(item)

        self.treeBag.topLevelItem(0).setSelected(True)

        self.treeBag.setColumnWidth(0, self.width() / 3.)
        self.treeBag.setColumnWidth(1, self.width() / 3.)
        self.treeBag.setColumnWidth(2, self.width() / 3.)

        self.progressBar.setValue(0.)
        self.spinFactor.setEnabled(False)
        self.spinFrequency.setEnabled(False)
        self.spinPartTime.setEnabled(False)
        self.spinSkip.setEnabled(False)
        self.spinSleep_2.setEnabled(False)
        self.spinStart.setEnabled(False)
        self.spinFactor.setMaximum(1000000000)
        self.spinFrequency.setMaximum(1000000000)
        self.spinFrequency.setMaximum(1000000000)
        self.spinPartTime.setMaximum(1000000000)
        self.spinSkip.setMaximum(1000000000)
        self.spinSleep_2.setMaximum(1000000000)
        self.spinStart.setMaximum(10000000000)
        self.spinFactor.setMinimum(0.01)
        self.spinFrequency.setMinimum(0.01)
        self.spinFrequency.setMinimum(0.01)
        self.spinPartTime.setMinimum(0.01)
        self.spinSkip.setMinimum(0.01)
        self.spinSleep_2.setMinimum(0.01)
        self.spinStart.setMinimum(0.01)
        self.buttonStep.hide()
        self.buttonPause.hide()
        self.buttonStop.hide()
        self.terminalPlainText.hide()
        self.bagRunning = False

        # Paring buttons to functions
        self.checkFrequency.stateChanged.connect(self.checkFrequencyChanged)
        self.checkPartTime.stateChanged.connect(self.checkPartTimeChanged)
        self.checkSkip.stateChanged.connect(self.checkSkipChanged)
        self.checkSleep.stateChanged.connect(self.checkSleepChanged)
        self.checkStart.stateChanged.connect(self.checkStartChanged)
        self.checkFactor.stateChanged.connect(self.checkFactorChanged)
        self.pushButton.clicked.connect(self.playBag)
        self.buttonPause.clicked.connect(self.pauseBag)
        self.buttonStep.clicked.connect(self.stepBag)
        self.buttonStop.clicked.connect(self.stopBag)

    def playBag(self):
        """This function retrieve all the arguments informations and create the thread which will launch the bagplay command"""


        bagPath = str(self.treeBag.selectedItems()[0].text(0))
        bagDuration = self.bagsInfo[bagPath][1]

        errorString = ""


        #Retrieve the bagplay arguments from the window
        optionArguments = ""
        if self.checkFactor.checkState() == 2:
            optionArguments += (' -r ')
            optionArguments += (str(self.spinFactor.value()))


        if self.checkSleep.checkState() == 2:
            optionArguments += (' -d ')
            optionArguments += (str(self.spinSleep_2.value()))

        if self.checkStart.checkState() == 2:
            optionArguments += (' -s ')
            optionArguments += (str(self.spinStart.value()))
            if self.spinStart.value() > bagDuration:
                errorString += "The start parameter is too big. Please modify it \n"


        if self.checkPartTime.checkState() == 2:
            optionArguments += (' -u ')
            optionArguments += (str(self.spinPartTime.value()))
            if self.spinPartTime.value() > bagDuration:
                errorString += "The time part parameter is too big.  Please modify it \n"

            if self.checkStart.checkState() == 2 and (self.spinStart.value() + self.spinPartTime.value()) > bagDuration:
                errorString += "The time part parameter or the start parameter is too big. Please modify one of those\n"

        if self.checkSkip.checkState() == 2:
            optionArguments += (" --skip-empty="+str(self.spinSkip.value()))

        if self.checkFrequency.checkState() == 2:
            optionArguments += (' --clock ')
            optionArguments += ("--hz="+str(self.spinFrequency.value()))

        if self.checkLoop.checkState() == 2:
            optionArguments += (' -l')




        if errorString == "" :

            # Updates ui elements
            self.terminalPlainText.clear()
            self.terminalPlainText.show()
            self.pushButton.hide()
            self.buttonPause.show()
            self.buttonStop.show()
            self.bagRunning = True
            #Create the thread
            self.thread = QtCore.QThread()
            self.thread.start()
            #Create the worker
            self.worker = BagWorker(bagPath, optionArguments)
            #Connect the signals
            self.worker.terminalInformation.connect(self.updateBagPlayInformation)
            self.worker.finishThread.connect(self.killThread)
            self.worker.moveToThread(self.thread)
            #Launch the thread
            self.worker.start.emit()
        else:
            QtWidgets.QMessageBox.warning(self, "Warning", errorString)

    def pauseBag(self):
        """Send some Pause request to the command thread and update the ui of the window"""
        if self.bagRunning:
            self.buttonPause.setText("Resume")
            self.buttonStep.show()
        else:
            self.buttonPause.setText("Pause")
            self.buttonStep.hide()

        self.bagRunning = not(self.bagRunning)
        self.worker.pauseBagHandler()

    def stepBag(self):
        """Send some step request to the command thread"""
        self.worker.stepBagHandler()

    def stopBag(self):
        """Send some stop request to the command thread"""
        self.worker.stopBagHandler()

    @QtCore.pyqtSlot(str, int)
    def updateBagPlayInformation(self, output, progressionValue):
        """Handle the messages sent by the command thread and display the output of the command process in the plaintext widget """
        self.terminalPlainText.appendPlainText(output)
        self.progressBar.setValue(progressionValue)

    @QtCore.pyqtSlot()
    def killThread(self):
        """Kill the command thread and reset the ui elements"""
        self.progressBar.setValue(0.)
        self.buttonPause.hide()
        self.buttonPause.setText("Pause")
        self.buttonStep.hide()
        self.buttonStop.hide()
        self.pushButton.show()
        self.terminalPlainText.hide()
        self.terminalPlainText.clear()

        try:
            self.worker.stopSignal = True
            del self.worker
            self.thread.quit()
            self.thread.wait()
            del self.thread
        except:
            pass


    def checkFactorChanged(self):
        """enable/disable the related spinbox of the factor checkbox"""
        self.spinFactor.setEnabled(self.checkFactor.checkState())

    def checkSleepChanged(self):
        """enable/disable the related spinbox of the sleep checkbox"""
        self.spinSleep_2.setEnabled(self.checkSleep.checkState())

    def checkStartChanged(self):
        """enable/disable the related spinbox of the start checkbox"""
        self.spinStart.setEnabled(self.checkStart.checkState())

    def checkFrequencyChanged(self):
        """enable/disable the related spinbox of the frequency checkbox"""
        self.spinFrequency.setEnabled(self.checkFrequency.checkState())

    def checkPartTimeChanged(self):
        """enable/disable the related spinbox of the part time checkbox"""
        self.spinPartTime.setEnabled(self.checkPartTime.checkState())

    def checkSkipChanged(self):
        """enable/disable the related spinbox of the skip checkbox"""
        self.spinSkip.setEnabled(self.checkSkip.checkState())

    def closeEvent(self, event):
        """Kill the command thread when the bagplay window is closed by the user"""
        self.killThread()
        self.hide()
        event.accept()  # let the window close
        self.deleteLater()