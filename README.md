# Yoga-Simulation
This MATLAB code is designed to simulate human movements, specifically yoga poses, by using a set of 3D coordinates representing key body joints and limbs.

## Methodolgy -

# Initialization and Setup:
The code starts by defining the lengths of various body parts (e.g., shoulders, hips, elbows, arms, thighs, legs, and the body) using the Euclidean distance between relevant joints.

# Simulation Loop:
The main loop iterates over 5 frames (each representing a different pose).
In each iteration, the code calculates the actual positions of body joints for the current frame (p_actual) and estimates a set of angles (ang) using a nonlinear least squares optimization function (lsqnonlin).
The human2 function is used to generate these angles by minimizing the difference between the simulated joint positions and the actual positions (p_actual).

# Animation:
After the angles for all frames are calculated, the code animates the transition between frames.
This is done by interpolating between the angles of consecutive frames and rendering the human model at each step of the transition using the human1 function.

# Functions for Transformation and Visualization:
human2: Calculates the positions of the joints and limbs based on a given set of angles and compares them to the actual positions (p_actual), returning the squared difference.
human1: Visualizes the human pose by plotting the limbs based on the calculated joint positions.
Rotation Functions (rotX, rotY, rotZ): These functions apply rotation matrices around the X, Y, and Z axes, respectively, to simulate the rotation of limbs in 3D space.
plt_point: A helper function to plot the line segments representing limbs between two points (joints).
