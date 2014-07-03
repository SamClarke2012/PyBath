	# Water_Bath.py - Turn a $9 kettle into a $900 immersion bath!
	# PID algorithm uses "ideal-case" thermodynamic transfer to guide movement.
	# To find the ROUGH emissivity of your material, take a reading with the
	# bath containing a liquid at a known temperature (ideally @ target temp), 
	# then use an IR temperature gun to measure the bath from the outside 
	# (below the meniscus) then use the following backyard formula;

	# A = IRTemp /(RealTemp/100)
	# emissivity = A/100

The PID controller is set to control the level of energy within the bath in Kj,
therefore the error in temperature is converted to Kj deficit and fed to the 
PID controller, this prompts the PID controller to output energy required to 
reach it's goal (in Kj). This is then converted into element time and sent to  
the arduino and the cycle repeats.

Parts required;

1x Solid state relay 
2x Thermistors (and required resistors) or better (1x outside, 1x waterproof)
1x Kettle, or kettle element(s) mounted in a larger tub (with lid!)
1x Small hot-water capable pump for water movement over the element

