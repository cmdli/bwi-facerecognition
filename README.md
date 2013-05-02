BWI Face Recognition
===================

This is a repository for the code for Ethen Jennings', Mukund Rathi's, and Chris de la Iglesia's UT-Austin FRI project 
on the BWI stream. This is mainly for online collaboration.

How to Use
===================

To run the default code, start up the launch file 'face_detector.launch'.

This node, by default, takes in RGB images from the '/camera/rgb/image_color' topic, detects and labels faces inside the image, then publishes the new image to '/detector/faces'. The code will only label 'chris', 'ethan', and 'mukund' without updating the training data. In addition, this node publishes the '/cropped_faces' topic which contains the extraced faces from the image.


Configuring the Face Recognizer
====================

The face recognizer depends on prepared training data in the form of images fromthe topic '/cropped_faces' and labels, which are stored inside the 'faces' folder. Inside the folder, the various training images are stored in several directories named after the person whose images they contain (e.g. the 'chris' folder inside 'faces' contains images of Chris). The face recognizer also uses a file at the top of the package called 'faces.csv' to find each image in the 'faces' directory and their numerical label. For simplicity, a Python script is provided which takes in the 'faces' directory as an argument and prints out the CSV file for it (this script is stored in 'scripts/create_csv.py'). A third necessary step is giving the program the absolute path of the 'faces' folder, which is given to the program through the launch file argument 'face_directory_path'.

Launch File Options
===================

* input: the rgb camera input topic from which faces will be extracted and labeled (default: camera/rgb/image_color)
* output: this is the topic to which input images, with the detected faces and labels overlayed on it, are published (default: detector/faces)
* cropped_faces: this is the topic to which monochrome faces that have been detected and extracted from the input topic are published. Only the first face detected in each frame will be published here. (default: detector/croped_faces)
* face_cascade_file: a path to a cascade file which is required by OpenCV's face detection algorithm.
* csv_file: a CSV file that contains paths to all of the available training data
* training_data_path: the base path for all of the paths in the CSV file
* 

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
