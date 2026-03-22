This repository serves as an extension of my Thesis appendix, housing the large aerodynamic datasets and the MATLAB run codes for my simulation.

To replicate the flight simulator, utilize the folder titled 'To-Run.' 
1. Open the MATLAB file, the Simulink archiecture, and the two spreadsheets, ensuring all are in the same folder.
2. Ensure FlightGear is installed on computer and set up to download new scenery.
3. Connect (plug in or via bluetooth) flight controller. Adjust the Simulink archicture as needed based on number of axes of controller (currently set up for four).
4. Run the XML code in terminal to establish connection between Simulink and FlightGear (altering the FlightGear location).
5. Run the FlightGear start-up code (altering the FlightGear location).
6. Run the MATLAB file after setting desired simulation duration (MATLAB line 94) and microburst intensity (Microburst block in Simulink).
7. Fly (and hopefully land) the aircraft while encountering a microburst.
