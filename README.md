# Micromouse Final Project
The goal of this project is to design, build, and write code for a robot that solves a maze in the least amount of time possible. The robot navigates a maze that is 5 cells by 5 cells where each cell is 18cm by 18cm and is separated by walls that are 5cm high. Starting in a corner cell, the robot is supposed to navigate the maze so that it finds the center cell. 
This code runs on a robot that uses four mecanum wheels, four DC motors, a breadboard, an Arduino, an Adafruit motor shield, four ultrasonic sensors, and a 9-volt battery pack. 

## Running the Application
1. Clone this repository
```bash
git clone https://github.com/miscmei/robot.git
cd robot
```

2. Open the cloned project in Arduino IDE
- Go to ```File``` > ```Open...``` and navigate to the cloned project directory.
- Open the main ```.ino``` file of the project.

3. Select the board and port
- Go to ```Tools``` > ```Board``` and select your Arduino board model.
- Go to ```Tools``` > ```Port``` and select the port to which your Arduino board is connected.

4. Upload/compile the sketch
- Click the ```Upload``` button (right-pointing arrow) in the Arduino IDE toolbar.
- Wait for the code to compile and upload to the Arduino board.
- Once the upload is complete, the Arduino board will reset and start running the application.

## Technologies Used
- Arduino Uno
- Arduino IDE
- [Adafruit Motor Shield for Arduino](https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/overview)

## Acknowledgments
Authors: Nina van Hoorn, Anand Basu, and Maddy Fung.
This project was for a final project for Robotics which was taught by Evan Halstead at Skidmore College. 
