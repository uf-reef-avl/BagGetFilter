import sys
from PyQt5.QtWidgets import QPushButton, QVBoxLayout, QWidget, QSplitter,QTreeWidget, QHBoxLayout, QTreeWidgetItem, QAbstractItemView
from PyQt5 import QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import random
class MplCanvas(QWidget):
    def __init__(self, parent=None):
        super(QWidget, self).__init__(parent)

        self.figure = plt.figure()
        self.tree_plot = QTreeWidget()
        # this is the Canvas Widget that displays the `figure`
        # it takes the `figure` instance as a parameter to __init__
        self.canvas = FigureCanvas(self.figure)

        # this is the Navigation widget
        # it takes the Canvas widget and a parent
        self.toolbar = NavigationToolbar(self.canvas, self)
        # Just some button connected to `plot` method
        self.button_load = QPushButton('Load selected topics')
        self.button_plot = QPushButton('Plot')
        #self.button.clicked.connect(self.plot)
        # set the layout
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.toolbar)
        #self.layout.addWidget(self.canvas)

        self.splitter = QSplitter(QtCore.Qt.Horizontal)
        self.splitter.addWidget(self.tree_plot)
        self.splitter.addWidget(self.canvas)
        self.layout.addWidget(self.splitter)
        self.buttonlayout = QHBoxLayout()
        self.buttonlayout.addWidget(self.button_load)
        self.buttonlayout.addWidget(self.button_plot)
        self.layout.addLayout(self.buttonlayout)
        self.setLayout(self.layout)


        #data
        self.data_model = {}  #bag_name_key then topic name_key then data type in a panda base
        self.tree_plot.headerItem().setText(0,  "Bags")
        self.tree_plot.headerItem().setText(1,  "Topics")
        self.tree_plot.headerItem().setText(2,  "Data")
        self.tree_plot.setSelectionMode(QAbstractItemView.ExtendedSelection)


    def setTreeSize(self):
        """This function corrects the column header sizes of the tree widget which displays the topics and bags topics"""
        self.tree_plot.setColumnWidth(0, self.tree_plot.width() / 4.)
        self.tree_plot.setColumnWidth(1, self.tree_plot.width() / 4.)
        self.tree_plot.setColumnWidth(2, self.tree_plot.width() / 4.)
        self.tree_plot.setColumnWidth(3, self.tree_plot.width() / 4.)


    def plot(self):
        ''' plot some random stuff '''

        self.figure.clear()
        ax = self.figure.add_subplot(111)
        for item in self.tree_plot.selectedItems():
            if (item.parent() != None) and (item.parent().parent() != None):
                bag_name = item.parent().parent().text(0)
                topic_name = item.parent().text(1)
                data_name = item.text(2)
                print("data_name "+str(self.data_model[bag_name][topic_name][data_name].to_numpy()))
                ax.plot( self.data_model[bag_name][topic_name]["rosbagTimestamp"].to_numpy(), self.data_model[bag_name][topic_name][data_name].to_numpy(), '*-', label=data_name)

        # refresh cnvas
        self.canvas.draw()

    def add_topic_to_tree(self,bag_name,topic_name,data_header):
        bagItem = None
        for id in range(self.tree_plot.topLevelItemCount()):
            item = self.tree_plot.takeTopLevelItem(id)
            if item.text(0) == bag_name:
                bagItem = item
        if bagItem == None:
            bagItem = QTreeWidgetItem()
        bagItem.setText(0, bag_name)
        topicItem = QTreeWidgetItem()
        topicItem.setText(1, topic_name)
        bagItem.addChild(topicItem)
        for header in data_header:
            headerItem = QTreeWidgetItem()
            headerItem.setText(2, header)
            topicItem.addChild(headerItem)
        self.tree_plot.addTopLevelItem(bagItem)




    def load_data_topic(self,bag_name,topic_name,data_header,panda_dataframe):
        self.add_topic_to_tree(bag_name,topic_name,data_header)
        if bag_name not in self.data_model.keys():
            self.data_model[bag_name] = {}
        self.data_model[bag_name][topic_name] = panda_dataframe
        print(panda_dataframe)




