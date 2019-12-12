  The main goal of this project is to create four robotic systems that can carry a flag in hurdle area. The basic working principle of these systems is to detect robots and obstacles at a certain height with a camera located out of the platform and to establish routes by color selection. With the routes calculated, we will establish a map for carrying the flag to another robot. The technical process of the Project is mainly based on the Image Processing, Embedded  stems, Robotics, artificial Intelligence and Control Systems.
  TECHNICAL SPECIFICATIONS   
	There are 4 important parts for this project. These are : 
		1. Brain Module
		2. Communication Module
		3. Movement Module
		4. Flag Changing Module
	Generally, Python was used for coding RaspberryPi 3B+ and OrangePi Zero
BRAIN MODULE
Robot localization(finding location and direction) with object detection  by using color filtering and contour detection
Smooth path planning with A* search algorithm by using custom Heuristic function
COMMUNICATION MODULE
Reliable data and message transfer using TCP packets
Responsible for communicating with the brain(Raspberry Pi) and the controller.
MOVEMENT MODULE
Path tracking with remotely operated PID controller
Modelling the robot and simulating the PID controller
Movement simulation with using mathematical model of the Differential Drive robot.
FLAG CHANCING MODULE
Used a metal plate as a flag.
Changing the flag by using electromagnetic module
CONCLUSION
In this project, we designed 4 robotics which can carry a metal plate to each other autonomously by using Python
	PYTHON – RASPBERRY PI – ORANGE PI
Used Python programming interface to program our controllers.
Used OpenCV for image processing.
Implemented the code for A* algorithm and PID controller for movement module.
Used RaspberryPi for our Brain module.
Used OrangePi Zero for our Movement module.


FOR MORE INFORMATİON, please READ WORD Documantation AND LOOK TO POSTER.
