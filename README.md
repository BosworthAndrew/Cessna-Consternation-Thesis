This repository serves as an extension of my Thesis appendix, housing the large aerodynamic datasets and the MATLAB run codes for my simulation.

To replicate the flight simulator, utilize the folder titled 'To-Run.' 
1. Open the MATLAB file, the Simulink architecture, and the two spreadsheets, ensuring all are in the same local folder.
2. Ensure FlightGear is installed on computer and set up to download new scenery.
3. Connect (plug in or via bluetooth) flight controller. Adjust the Simulink architecture as needed based on number of axes of controller (currently set up for four).
4. Run the XML code in terminal to establish connection between Simulink and FlightGear 'XML-Additions-FG.'
6. Run the FlightGear start-up code 'FG-runcode.'
7. Run the MATLAB file after setting desired simulation duration (MATLAB line 94) and microburst intensity (Microburst block in Simulink).
8. Fly (and hopefully land) the aircraft while encountering a microburst.

Access the full-scale autopilot landing simulation here: https://www.youtube.com/watch?v=ACZxgx7IUl8.

If the user does not possess a MATLAB license, a simple version of the microburst is also provided, to utilize with FlightGear's built-in dynamics. This weather scenario can thus be adapted to any aircraft type. To run this simulation, follow the below steps:
1. Save the Python file 'SimpleFGMicroburst.py' to local drive.
2. Run the FlightGear start-up code 'SimpleRunCode.'
3. Once FlightGear is running, open the file 'SimpleFGMicroburst.py' in a new terminal window.
4. Fly the aircraft with a mouse or any controller compatbilbe with FlightGear.
