# AnalisisCinematico

This program reads the data of a robot from a JSON file.
The input file has a hierarchy as follows:
- "dh_params": Array of numeric values separated by commas of the
Denavit-Hartenberg matrix. The theta and alfa values need to be in
degrees.
- "q_dot": Array of numeric values separated by commas of the q_dot
matrix.
- "degType": Array of numeric values (only 1's and 0's), where each
value corresponds to each type of degree of freedom of the robot.

Outputs the linear and angular velocity of the robot based on the
input file.

## Installation

Create a virtual environment by running
> python3 -m pyvenv <name_of_virtual_environment>

Then update the pip installer
> pip install --upgrade pip

Finally install the requirements file by running
> pip install -r requirements.txt

in the folder where the repo is installed.

## Running

To run the program, run
> python KinematicAnalysis.py

if you want to use the default filename. Else, run
> python KinematicAnalysis.py --filename <name_and_path_of_data_file>