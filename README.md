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

```
git clone http://192.168.1.101/AVL-Summer-18/BagGetFilter
```

## Usage

### Main window utilisation

The executable of the BagGetFilter is located in the dist repository. To run it, navigate to the dist repository and type the command:

```
./BagGetFilter
```

Then a window will appear:

![Main Window display](./docs/imgs/Main_Window.png "Main Window")

To load a bag file into the application, drag and drop the bag icon from the area on the left to the blank area on the right. Warning, the drag and drop system isn't really sensitive so if it's not working the first time, just try several times. It is also possible to load multiples bags at the same time by keeping the "CTRL" or "SHIFT" key pressed and drag and drop mutliples items.

![Select Area Bag file](./docs/imgs/Select_Bag_Area.png "Select Area Bag File")

All the topics of the bags will be displayed in the blank area. The clipboard area of the bags will be also populated with some roslaunch commands. The user will be able to see it by clicking on the "Show Clipboard" button. By the way, the button "Clear Bags" is useful to flush the bags currently loaded in the application:

![Clipboard](./docs/imgs/Clipboard.png "Clipboard")

In order to select some topics just keep the "CTRL" or "SHIFT" key pressed and click on them.

![Selected Topics](./docs/imgs/Selected_Topics.png "Select Topics")

To manipulate the bags, 3 options are availables:

![Bag Manipulation](./docs/imgs/Bag_manipulation.png "Bag Manipulation")

 - Play the bags by clicking on the "Play the bag" button

 - Creating csv files of the selected topics by clicking on the "Save selected topics to csv files" button. The files will be automatically named but the storage directory has to be specified.

 - Creating news bag with only the selected topics by clicking on the "Save selected topics in a filtered bag" button. The files will be automatically named by keeping the same bag name and adding the suffix specified in the upper line. The storage directory must also be specified.
 
![Bag Suffix](./docs/imgs/Bag_suffix.png "Bag Suffix")

 - If the user wants to manipulate several bags with the same topics content, he can activate the "Matching Bag/Topic Highlighting" checkbox. Then when he will select a topic in one bag, this topic will be automatically selected in all the others bags with the same topics contents. 

 - To load the new filtered bags just after their creation ensure that the "Load the new filtered bag" checkbox is activated.

### Playing bags

After clicking on the "play bag" button, the "play bag" window will appear.

![Bag window](./docs/imgs/play_bags.png "Bag window")

Then, the user will be able to choose the bag that he wants to play between the previous loaded bag. 

![Bag Selection](./docs/imgs/play_bag_selection.png "Bag Selection")

Before playing it, he can also specify different playing arguments.

![Bag option](./docs/imgs/play_options.png "Bag Option")

While running the bag, the ouput informations will be shown in the plain text area. It is possible to pause the bag by clicking on the "Pause" button or to stop it definitely by clicking on the "Stop" button.

![Bag Running](./docs/imgs/play_bag_running.png "Bag Running")

When the bag is paused, the user can also run it step by step by clicking on the "Step" button or resume it or stop it.

![Bag Paused](./docs/imgs/play_bag_paused.png "Bag Paused")

