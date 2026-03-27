# COEN320 p2.3 Display System
This is the display system for the air traffic control simulation. It receives data from the aircraft and displays it on the screen.
- You need to make the project as a separate process and run it in parallel with the aircraft process.
- You can use the shared memory to read aircraft data and IPC to send notifications to the communications system.
- Feel free to use any display layout to show the aircraft data, but make sure it is clear and easy to read.
- The notifications for collision warnings should be displayed prominently on the console.
- To be able to send commands to the aircraft, you can use IPC to send messages to the communications system, which will then forward the commands to the aircraft. (Check `Aircraft.cpp`: `Aircraft::updatePosition()`)
- **DONT USE RAW PRINTS TO DISPLAY AIRCRAFT DATA**