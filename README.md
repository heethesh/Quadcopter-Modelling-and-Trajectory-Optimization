# Quadcopter Modelling and Trajectory Optimization
This is the Air Mobility Project for 16-665 Robot Mobility in Air, Land, and Sea (Fall 2018) at Carnegie Mellon University.
Instructor for Air Mobility: Nathan Michael

This repository is a fork from my original repository [here](https://github.com/heethesh/16-665-Robot-Mobility).

## Objectives
- Develop the quadcopter model (and state-space version) and implement PID and LQR based feedback contollers.
- Develop a basic state machine to facilitate simulation that enables the robot to takeoff, hover, track a trajectory, and land.
- Introduce time-parameterized trajectories given fixed initial and final endpoint constraints and bounded velocity and acceleration.
- Extend the formulation to include piecewise continuous trajectories with fixed initial and final endpoint constraints.
- Evaluate tracking performance given different levels of flight performance and trajectory design (from slow to fast, straight and curved paths).
- Exploit the concepts and implementations developed through the previous exercises to achieve high-performance flight. 

## CMU Academic Integrity Policy
If you are currently enrolled in this course, please refer to Carnegie Mellon University Policy on Academic Integrity [here](https://www.cmu.edu/policies/student-and-student-life/academic-integrity.html) before referring to the any of the repository contents.

## Blender Simulation
### Quadrotor Pirouette
![blender](report/images/Pirouette-01.PNG?raw=true "Quadrotor Pirouette")

### Quadrotor Flip
![blender](report/images/Free-Skate-02.PNG?raw=true "Quadrotor Flip")

### Quadrotor Fly-Through-Ring
![blender](report/images/Free-Skate-01.PNG?raw=true "Quadrotor Fly-Through-Ring")

## Quadcopter Model
![model](report/images/Quadcopter-Model-01.png?raw=true "Quadrotor Controller Model")

## Flowchart
![flowchart](report/images/Software-Model-01.png?raw=true "Software Flowchart")

