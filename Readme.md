	# Water_Bath.py - Turn a $9 kettle into a $900 immersion bath!
	# PID algorithm uses "ideal-case" thermodynamic transfer to guide movement.
	# To find the ROUGH emissivity of your material, take a reading with the
	# bath containing a liquid at a known temperature (ideally @ target temp), 
	# then use an IR temperature gun to measure the bath from the outside 
	# (below the meniscus) then use the following backyard formula;

	# A = IRTemp /(RealTemp/100)
	# emissivity = A/100
