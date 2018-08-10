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



# Creates and runs the Ping_Worker class and its methods
class BagWorker(QtCore.QObject):
    # Variables for emitting starting, displaying status, and closing signals
    start = QtCore.pyqtSignal()
    terminalInformation = QtCore.pyqtSignal(str, int)
    finishThread = QtCore.pyqtSignal()

    # Definition of a bagWorker
    def __init__(self, bagPath, optionArguments):
        super(BagWorker, self).__init__()
        self.start.connect(self.run)
        self.stopSignal = False
        self.responseString = ""
        self.bagPath = bagPath
        self.optionArguments = optionArguments

    # This function pings the robot
    @QtCore.pyqtSlot()
    def run(self):
        roscoreNode = RoscoreNode.Roscore()
        roscoreNode.run()

        cmd = "rosbag play "+self.bagPath.replace(' ', '\ ')+self.optionArguments

        if self.optionArguments != []:
            cmd += self.optionArguments

        self.bagPlayProcess = pexpect.spawn(cmd, timeout=None, logfile=sys.stdout)
        line = ""
        tempLine = ""
        lineComplete = False
        progressionValue = 0
        while True:
            output = self.bagPlayProcess.read(1)
            if output == '' or output == None or self.stopSignal == True:
                break
            if output:

                if lineComplete:
                    lineComplete = False
                    tempLine = ""
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

                tempLine += str(output)

        roscoreNode.terminate()
        self.finishThread.emit()

    def pauseBagHandler(self):
        self.bagPlayProcess.sendline("\x20")

    def stopBagHandler(self):
        self.bagPlayProcess.sendline("\x03")

    def stepBagHandler(self):
        self.bagPlayProcess.sendline("s")

# This class creates the main window of the application
class BagPlay(QtWidgets.QDialog, PlayBagDesign.Ui_Dialog):

    # Initializes and defines the Multilaunch window
    def __init__(self):
        super(self.__class__, self).__init__()
        self.setupUi(self)
        self.setModal(True)
        self.bagPath = ""

        self.progressBar.setValue(0.)
        self.spinFactor.setEnabled(False)
        self.spinFrequency.setEnabled(False)
        self.spinPartTime.setEnabled(False)
        self.spinSkip.setEnabled(False)
        self.spinSleep.setEnabled(False)
        self.spinStart.setEnabled(False)

        self.spinFactor.setMaximum(1000000000)
        self.spinFrequency.setMaximum(1000000000)
        self.spinPartTime.setMaximum(1000000000)
        self.spinSkip.setMaximum(1000000000)
        self.spinSleep.setMaximum(1000000000)
        self.spinStart.setMaximum(1000000000)

        self.buttonStep.hide()
        self.buttonPause.hide()
        self.buttonStop.hide()
        self.bagRunning = False


        self.checkFrequency.stateChanged.connect(self.checkFrequencyChanged)
        self.checkPartTime.stateChanged.connect(self.checkPartTimeChanged)
        self.checkSkip.stateChanged.connect(self.checkSkipChanged)
        self.checkSleep.stateChanged.connect(self.checkSleepChanged)
        self.checkStart.stateChanged.connect(self.checkStartChanged)

        self.pushButton.clicked.connect(self.playBag)
        self.buttonPause.clicked.connect(self.pauseBag)
        self.buttonStep.clicked.connect(self.stepBag)
        self.buttonStop.clicked.connect(self.stopBag)

    def playBag(self):
        self.terminalPlainText.clear()
        self.pushButton.hide()
        self.buttonPause.show()
        self.buttonStop.show()
        self.bagRunning = True

        optionArguments = ""

        print str(self.checkLoop.checkState())

        if self.checkFactor.checkState() == 2:
            optionArguments += (' -r ')
            optionArguments += (str(self.spinFactor.value()))

        if self.checkStart.checkState() == 2:
            optionArguments += (' -s ')
            optionArguments += (str(self.spinStart.value()))

        if self.checkSleep.checkState() == 2:
            optionArguments += (' -d ')
            optionArguments += (str(self.spinSleep.value()))

        if self.checkPartTime.checkState() == 2:
            optionArguments += (' -u ')
            optionArguments += (str(self.spinPartTime.value()))

        if self.checkSkip.checkState() == 2:
            optionArguments += (" --skip-empty="+str(self.spinSkip.value()))

        if self.checkFrequency.checkState() == 2:
            optionArguments += (' --clock ')
            optionArguments += ("--hz="+str(self.spinFrequency.value()))


        if self.checkLoop.checkState() == 2:
            optionArguments += (' -l')


        self.thread = QtCore.QThread()
        self.thread.start()
        # Create the worker
        self.worker = BagWorker(self.bagPath, optionArguments)
        self.worker.terminalInformation.connect(self.updateBagPlayInformation)
        self.worker.finishThread.connect(self.killThread)
        self.worker.moveToThread(self.thread)
        self.worker.start.emit()

    def pauseBag(self):
        if self.bagRunning:
            self.buttonPause.setText("Restart")
            self.buttonStep.show()
        else:
            self.buttonPause.setText("Pause")
            self.buttonStep.hide()

        self.bagRunning = not(self.bagRunning)

        self.worker.pauseBagHandler()

    def stepBag(self):
        self.worker.stepBagHandler()

    def stopBag(self):
        self.worker.stopBagHandler()

    @QtCore.pyqtSlot(str, int)
    def updateBagPlayInformation(self, output, progressionValue):
        self.terminalPlainText.appendPlainText(output)
        self.progressBar.setValue(progressionValue)

    @QtCore.pyqtSlot()
    def killThread(self):

        self.progressBar.setValue(0.)
        self.buttonPause.hide()
        self.buttonStep.hide()
        self.buttonStop.hide()
        self.pushButton.show()
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
        self.spinFactor.setEnabled(self.checkFactor.checkState())

    def checkSleepChanged(self):
        self.spinSleep.setEnabled(self.checkSleep.checkState())

    def checkStartChanged(self):
        self.spinStart.setEnabled(self.checkStart.checkState())

    def checkFrequencyChanged(self):
        self.spinFrequency.setEnabled(self.checkFrequency.checkState())

    def checkPartTimeChanged(self):
        self.spinPartTime.setEnabled(self.checkPartTime.checkState())

    def checkSkipChanged(self):
        self.spinSkip.setEnabled(self.checkSkip.checkState())

    def closeEvent(self, event):
        self.killThread()
        self.hide()
        event.accept()  # let the window close