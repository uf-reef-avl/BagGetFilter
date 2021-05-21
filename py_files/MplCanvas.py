import sys
from PyQt5.QtWidgets import QPushButton, QVBoxLayout, QWidget
from PyQt5 import QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.pyplot as plt


import random
class MplCanvas(QWidget):
    def __init__(self, parent=None):
        super(QWidget, self).__init__(parent)
        self.figure = plt.figure()
        # this is the Canvas Widget that displays the `figure`
        # it takes the `figure` instance as a parameter to __init__
        self.canvas = FigureCanvas(self.figure)

        # this is the Navigation widget
        # it takes the Canvas widget and a parent
        self.toolbar = NavigationToolbar(self.canvas, self)
        # Just some button connected to `plot` method
        self.button = QPushButton('Plot')
        #self.button.clicked.connect(self.plot)
        # set the layout
        layout = QVBoxLayout()
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)
        layout.addWidget(self.button)
        self.setLayout(layout)

        #data
        self.x_dict = {}
        self.y_dict = {}
        self.label = {}

    def plot(self):
        ''' plot some random stuff '''

        self.figure.clear()
        ax = self.figure.add_subplot(111)
        for key in self.x_dict.keys():
            ax.plot( self.x_dict[key], self.y_dict[key], '*-', label=self.label[key])

        # refresh cnvas
        self.canvas.draw()


