# Quadcopter Control

1. Matlab file to compute necessary data to use for the simulation.
2. Simulink model to simulate a drone's altitude and thrust. Thrust is sent to esp8266, real altitude is received from it.
3. Esp8266 code to estimate drone's altitude based on thrust. Could be upgraded to use that thrust on real actuators and read real altitude from a sensor.
