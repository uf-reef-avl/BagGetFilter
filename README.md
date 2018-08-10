# BagGetFilter
The BagGetFilter package is a simple and user friendly interface to manipulate rosbag. Thanks to it, the user can **play rosbags** or **filter and export topics in CSV format** from them.


## Prerequisites
If you want to update this package, you will need several libraries:

 - pyqt5
 - pexcept
 - ros kinetic
 - python 2.7
 - pyinstaller

If you want to use it as a tool, you won't need any library and you will just have to use the executable.

## Installation

To install it, clone the BagGetFilter's remote repository in your computer.

git clone http://192.168.1.101/AVL-Summer-18/BagGetFilter

## Usage

The executable of the BagGetFilter is located in the dist repository. To run it, navigate to the dist repository and type the command:

```
./BagGetFilter
```

Then a window will appear:

![Main Window display](./docs/img/Main_Window.png "Main Window")

2 ways are possible to load a bag file into the application:

 - Use the "select your file" button

![Select Button Bag file](./docs/img/Select_Bag_Button.png "Select Button Bag File")

 - Drag and drop the bag icon from the area on the left to the blank area on the right. Warning, the drag and drop system isn't really sensitive so if it's not working the first time, just try several times.

![Select Area Bag file](./docs/img/Select_Bag_Area.png "Select Area Bag File")

All the topics in this bag will be displayed in the blank area and the current bag path will be written on the first line. The clipboard area of the bag will be also populated with some roslaunch commands. The user will be able to see it by clicking on the "show Clipboard" button.

![Clipboard](./docs/img/Clipboard.png "Clipboard")

In order to select some topics just click on them.

![Selected Topics](./docs/img/Selected_Topics.png "Select Topics")

To manipulate the bag, 3 options are available:

![Bag Manipulation](./docs/img/Bag_Manipulation.png "Bag Manipulation")

 - Play the current bag by clicking on the "Play the bag button"

 - Creating csv files of the selected topics by clicking on the "Save selected topics to csv files". The files will be automatically named but the storage directory has to be specified.

 - Creating a new bag with only the selected topics by clicking on the "Save selected topics in a filtered bag"
