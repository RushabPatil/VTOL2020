from dronekit import connect

#Connect to the vehicle

vehicle = connect('/dev/ttyAM0', wait_ready = True, baud = 57600)
