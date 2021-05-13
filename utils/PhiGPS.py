from Phidget22.Phidget import *
from Phidget22.Devices.GPS import *
import time

def onError(self, code, description):
	print("Code: " + str(code))
	print("Description: " + str(description))

ch = GPS()

# Register for event before calling open
ch.setOnErrorHandler(onError)

ch.open()

while True:
	# Do work, wait for events, etc.
	time.sleep(1)
