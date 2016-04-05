#Baxter3

## Synopsis

This is the README for the ME495 final project. The goal of this project was to pick and place six objects as quickly as possible. To accomplish this we used the camera in Baxter's hand to search for circles and then move to pickup the object. Right before the object is grasped the color of the object is determined with the hand camera and then placed into one or two boxes depending on which color the object is. 

### Robot Vision

The left hand is brought over the general object pick location. A stream of images are taken and converted to grey scale. The Hough Circle detector is used to detect all circles within the specified minimum and maximum radius. 

#### Case: No Objects Found

If no objects are detected the arm performs a scan of the entire reachable workspace until objects are detected.

#### Case: Objects Detected

The Hough circles are saved in a numpy array. If objects detected are already placed in the drop location or if the object is outside the reachable workspace, a red circle is drawn on the object and it will not be picked up.

If the object is in a valid pick location  the arm will search for the bottom right - most object and pick it first. Objects can be moved throughout the vision process; detection will contiue until an object is selected based on the aforementioned criteria. 

The next closest object is also detected and the gripper aligned such that it will avoid it when picking the bottom right - most object.

#### Colour Dectection

Colour detection is used to sort the objects. The arm moves directly above the selected object and saves a colour image of the object to be picked up. The Python Image Library is used to detect if the image contains mostly red or mostly blue pixels. If the colour cannot be determined, the image is cropped to include less of the table and more of the object to be picked. Should colour not be determined after two crop iterations the user can manually enter the colour through the GUI.

### Motion Planning
We initially planned to do motion planning with MoveIt, but decided that for our implementation simple paths generated using IK solutions were sufficient. This generally worked well, but occasionally there were problems with the arm coliding with the table or the boxes. There were also some issues with the IK service not finding a valid IK solution for desired locations. When no IK solution was found we attempted to iterate with different initial seeds. Although this should work in theory our implemtation still has issues finding solutions in some configurations. 

### GUI and Added Robustness

Our project includes several opportunities for the operator to intervene if Baxter makes a mistake. 
* When Baxter attempts to determine the color of the object being picked up, it asks the user if the color can't be determined. 
* Before Baxter picks up an object, it asks the operator if the position is okay. If the operator says no, then a menu is displayed where the operator can manually direct Baxter to the correct position/orientation. 
* After Baxter grasps an object, it asks if it actually grasped the object. If the operator says no, Baxter decrements the number of balls that it's collected and knows to search for this ball again. 

All of the operator interaction happens in the command line with menus and descriptive prompts. Relevant questions are also displayed on Baxter's head. 

## Contributors

Amanda Shorter,
Fan Bai,
Jane Miller,
Zack Woodruff