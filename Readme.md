	# PyBath.py - Turn a $9 kettle into a $900 immersion bath!

	# To find the ROUGH emissivity of your material, take a reading with the
	# bath containing a liquid at a known temperature (ideally @ target temp), 
	# then use an IR temperature gun to measure the bath from the outside 
	# (below the meniscus) then use the following backyard formula;

	# A = IRTemp /(RealTemp/100)
	# emissivity = A/100

The PID controller is set to control the level of energy within the bath in Kj,
therefore the error in temperature is converted to a Kj deficit and fed to the 
PID controller, this prompts the PID controller to output energy required to 
reach it's goal (in Watt seconds). This is then sent to the arduino and the 
cycle repeats.

######Parts required;

* 1x Solid state relay 
* 2x Thermistors (and required resistors) or better
* 1x Kettle, or kettle element(s) mounted in a larger tub
* 1x Small hot-water capable pump for water movement over the element
