# Vision-Based Navigation in Webots: Intersection Detection using Digital Twins

This repository contains the Webots environments, controllers, trained models and traning scripts for **vision-based navigation for autonomous robots**. 

The focus of this work is on **detecting and identifying intersections** during robot navigation using **visual input from onboard cameras** within **Webots digital twin environments**.

## 🧭 Project Overview
This project focuses on **vision-based autonomous navigation** in **Webots** digital twin environments.  
The developed Webots controllers are **fully automated** — they perform complete navigation tasks without manual intervention.  

During each simulation run, the controller:
- Navigates the environment autonomously using camera-based perception.
- Detects and classifies intersections in real time.
- **Automatically saves captured data** (images, telemetry, detection outputs, etc.) while navigating.
- **Organizes outputs into folders** corresponding to possible movement directions at intersections.

Each folder is named using the format:


---

## 🚗💨 Simulation in Action
| Description | Webots Environment |
|--------------|--------|
| ENV 2m aisle | <img src="https://github.com/yeko31/ego-intersect/raw/main/webots/Videos/2.webm.gif" width="400"> |
| ENV 1m aisle | <img src="https://github.com/yeko31/ego-intersect/raw/main/webots/Videos/3_up.gif" width="400"> |
| ENV 1.3m aisle | <img src="https://github.com/yeko31/ego-intersect/raw/main/webots/Videos/4.gif" width="400"> |



This project explores how vision-based systems can be used for autonomous navigation in real world intersection environments.  
It uses **Webots** to simulate realistic environments and **Python-based controllers** for robot perception and decision-making.


---

## ⚙️ Steps to Run the Simulation

#Controllers
(A)Remember to first change the image saving prefix(line 62 and 367) by considering Aisle width, Environment Type and Altitude levels.

Naming Convention for the saved images

This happens according to the webots environemnt file and the altitude of the drone

Aisle 1m
env1_med_stocked -> a100_env1_ms_<height in cm>.jpg
env2_no_stocked  -> a100_env2_ns_<height in cm>.jpg

Aisle 1.3m
env1_med_stocked -> a130_env1_ms_<height in cm>.jpg
env2_no_stocked  -> a130_env2_ns_<height in cm>.jpg

Aisle 2m
env1_med_stocked -> a200_env1_ms_<height in cm>.jpg
env2_no_stocked  -> a200_env2_ns_<height in cm>.jpg


In each controller, change the OUTPUT_DIR and VALIDATION_DIR where you want to save the images.


How to add the controllers to Webots and run the simulation

1. Open Webots 2025a
2. File -> New -> New Robot Controller
3. Select Python and give an intuitive name
4. Finish
5. Copy and Paste the relavant code into the python file opened in text editor.
6. Do the process mentioned in (A)
7. Copy the pid_controller.py script to the relavant folder of the controller.
8. Open Webots and select the Robot"Crazyflie" node and from the drop down select controller
9. Choose the relavant controller
10. Then click play button
11. Wait until the crazyflie hovers and press keyboard "a" button to start autonomous data collection script.




