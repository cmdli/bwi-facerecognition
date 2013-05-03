BWI Face Recognition
===================

This is a repository for the code for Ethen Jennings', Mukund Rathi's, and Chris de la Iglesia's UT-Austin FRI project 
on the BWI stream. This is mainly for online collaboration.

Project Discription
===================
The FaceDetector node takes an rgb image as input and finds rectangles that contain faces using OpenCV's face detector. Each face is then extracted from its rectangle and fed into OpenCV's face detector to determine the identity of the person.

How to Use
===================

<b>Dependencies:</b> This project requires ROS and OpenCV to run.

To run the default code, start up the launch file 'face_detector.launch'.

This node, by default, takes in RGB images from the '/camera/rgb/image_color' topic, detects and labels faces inside the image, then publishes the new image to '/detector/faces'. The code will only label 'chris', 'ethan', and 'mukund' without updating the training data. In addition, this node publishes the '/cropped_faces' topic which contains the extraced faces from the image.


Configuring the Face Recognizer
====================

The face recognizer depends on prepared training data in the form of images fromthe topic '/cropped_faces' and labels, which are stored inside the 'faces' folder. Inside the folder, the various training images are stored in several directories named after the person whose images they contain (e.g. the 'chris' folder inside 'faces' contains images of Chris). The face recognizer also uses a file at the top of the package called 'faces.csv' to find each image in the 'faces' directory and their numerical label. For simplicity, a Python script is provided which takes in the 'faces' directory as an argument and prints out the CSV file for it (this script is stored in 'scripts/create_csv.py'). A third necessary step is giving the program the absolute path of the 'faces' folder, which is given to the program through the launch file argument 'face_directory_path'. In addition, the number-to-name conversion from the CSV file to actual names is hard-coded into the code, so that will need to be changed.

Launch File Options
===================

* input: the rgb camera input topic from which faces will be extracted and labeled (default: camera/rgb/image_color)
* output: this is the topic to which input images, with the detected faces and labels overlayed on it, are published (default: detector/faces)
* cropped_faces: this is the topic to which monochrome faces that have been detected and extracted from the input topic are published. Only the first face detected in each frame will be published here. (default: detector/croped_faces)
* face_cascade_file: a path to a cascade file which is required by OpenCV's face detection algorithm.
* csv_file: a CSV file that contains paths to all of the available training data
* training_data_path: the base path for all of the paths in the CSV file

Making Your Own Training Data
===================
We've included a bunch of images of ourselves as example training data. To make your own, you can listen to the  cropped_faces topic and save the images using some other node, such as image_view. Regardless of how you get the data, you will need at least three or four images per person and they need to be as close to square as possible. They can be different resolutions and stored in a common image format (we used .jpg). Make sure you run the Python script (scripts/create_csv.py) whenever you change the training data to update the CSV file.

CSV File Format
===================
Each line should have the path of an image file relative to training_data_path and it's numerical label separated by a simicolon. You can also comment out lines with a '#'

    #Example file:
    faces/ethan/face1.jpg;0
    faces/ethan/face2.jpg;0
    faces/chris/face1.jpg;1
    faces/chris/face2.jpg;1
    
Possible Improvements
=====================
These are ideas for anyone doing work on face detection and recognition who want to build off our project:

* The face detector false positive rate maybe be able to be improved by using depth from the kinect cameras. For example people's heads form ovals of depth who's edges are defined by a harsh dropoff. If a detected face doesn't have this depth shape, then it probably isn't a person. Additionally, depth info combined with the size of the face rectangle can be used to estimate the physical size of the person's head. Heads that have an unrealistic size can be filtered out.
* Right now, the training data and faces cropped in real time have some images that show the whole head including foreheads and hair and others that show just the eyes nose and mouth. The recognizer's accuracy could possibly be improved by normalziing faces by resizing them so that the same amount of face is visible in all the training data and in all the detected faces. This could possibly be achieved by detecting features like eyes and noses to calculate how much face is visible in each image.
* It would be useful to add the ability to track a face between frames to give the robot some sense of short term memory about who is in front of it.
* If someone wants to actually interface with our code, our node needs to be changed to publish custom messages about the location of detected people. Right now, the code just publishes an image with the faces outlined by rectangles.

