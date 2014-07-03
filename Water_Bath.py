# Wather_Bath.py - Turn a $9 kettle into a $900 immersion bath!
# PID algorithm uses "ideal-case" thermodynamic transfer to guide movement.
# To find the rough emissivity of your material, take a reading with the
# bath containing a liquid at a known temperature (ideally @ target temp), 
# then use an IR temperature gun to measure the bath from the outside 
# (below the meniscus) then use the following rough backyard formula;

# A = IRTemp /(RealTemp/100)
# emissivity = A/100
 

import time
import serial

class PID:
    """ PID control class.
    """
    def __init__(self):

        # Gain variables
        self.Kp = 0
        self.Kd = 0
        self.Ki = 0
        self.prev_err = 0
        # Result variables
        self.Cp = 0
        self.Ci = 0
        self.Cd = 0
        # initialize delta t variables
        self.currtm = time.time()
        self.prevtm = self.currtm

    def setKp(self, invar):
        """ Set proportional gain. """
        self.Kp = invar

    def setKi(self, invar):
        """ Set integral gain. """
        self.Ki = invar

    def setKd(self, invar):
        """ Set derivative gain. """
        self.Kd = invar

    def setPrevErr(self, preverr):
        """ Set previous error value. """
        self.prev_err = preverr

    def genOut(self, error):
        """ Performs a PID computation and returns a control value based on
            the elapsed time (dt) and the error signal from a summing junction
            (the error parameter).
        """
        self.currtm = time.time()               # get t
        dt = self.currtm - self.prevtm          # get delta t
        de = error - self.prev_err              # get delta error

        self.Cp = self.Kp * error               # proportional term
        self.Ci += error * dt                   # integral term

        self.Cd = 0
        if dt > 0:                              # no div by zero
            self.Cd = de/dt                     # derivative term

        self.prevtm = self.currtm               # save t for next pass
        self.prev_err = error                   # save t-1 error

        # sum the terms and return the result
        return self.Cp + (self.Ki * self.Ci) + (self.Kd * self.Cd)
        
    def inputHandler(self, envT, bthT, kg, cp):
        """ Returns current energy stored within the bath (Kj) """
        return cp*(bthT - envT)*kg
        
    def outputHandler(self, co, w):
        """ Returns time to be on from PID output (seconds) """
        # S = (Co*1000)/watts 
        return (co*1000)/w

    def boltzmanLoss (self, e, t, c, a):
        # P = EOA(T^4-Tc^4)
        """ Returns the Stephan-Boltzman radiated loss of the system (deg/sec).
            Takes the system emissivity (float/int e), the target temperature 
            (float/int t), the room temperature (float/int c) and the area 
            (m^2) of the tank.
        """
        return e*0.000000056703*a*((t**4)-(c**4))




def getTemp( c ):
    """ Returns requested data from hardware controller """
    ser.flush()
    if c == 'bath':
        ser.write('$')
        newTemp = ser.readline()
        try:
            newTemp = float(newTemp)
        except ValueError:
            print '\n\nCaught a dud packet from the control hardware :('
            print 'Malformed packet contains \''+ str(newTemp) + '\''
            print 'Going back for[ever]more.. :)\n\n'
            getTemp(c)
        return newTemp
    elif c == 'environment':
        ser.write('!')
        newTemp = ser.readline()
        try:
            newTemp = float(newTemp)
        except ValueError:
            print '\n\nCaught a dud packet from the control hardware :('
            print 'Malformed packet contains \''+ str(newTemp) + '\''
            print 'Going back for[ever]more.. :)\n\n'
            getTemp(c)
        return newTemp
    else:
        return None

p = PID()
p.setKp(0.02727)   
p.setKi(0.005454)  
p.setKd(0.00340875)
prev = 0
envTemp = 0.0
bathTemp = 0.0
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)

ser.write('0.001')                               #
ser.readline()                                   #
ser.flush()                                      #
  
setPoint = float(raw_input('Enter the desired temperature (\'c) '))
specHeat = float(raw_input('Enter the specific heat of the bath medium (Kj/Kg\'c) ')) 
weight   = float(raw_input('Enter the weight of the bath medium (Kg) '))
e        = float(raw_input('Enter the emissivity of the tank material (white plastic ~= 0.89) ')) 
a        = float(raw_input('Enter the area of the tank (m^2 - (note: calculate to meniscus)) '))
watts    = float(raw_input('Enter the wattage of the bath element (Watts)'))
# Save the temperature form of the set point
setTemp = setPoint
# Get fresh temperature data
bathTemp = getTemp('bath')
envTemp  = getTemp('environment')
# Boltzman degrees radiated per second * 10 seconds per cycle
setPoint += abs(p.boltzmanLoss (e, setPoint, envTemp, a)*10)
# Convert from degrees to K-joules
setPoint = p.inputHandler(envTemp, setPoint, weight, specHeat) #

while True:
    # Time this operation
    cycleStart = time.time()
    # Get fresh data
    bathTemp = getTemp('bath')
    envTemp  = getTemp('environment')
    # Get convert it to K-joules stored
    invars = p.inputHandler(envTemp, bathTemp, weight, specHeat)
    # get the controller output movement
    out = p.genOut(setPoint - invars)
    # Convert it from ideal total K-joules to Watt/seconds
    t = p.outputHandler(out, watts)
    # Convert that to element on-time (seconds)
    step = t - prev
    # Handle cooling and overshoot
    if bathTemp - setTemp > 0.1 or step < 0 : step = 0.0
    # Save the current energy state
    prev = t 
    # Send the planned step to the hardware controller
    ser.write(str(step) + '\n' )  
    # Update
    print '%.2f' %bathTemp

    #print 'Bath temperature %.2f \n' %bathTemp + \
    #'EnvironmentTemp %.1f\n' %envTemp +          \
    #'Heating element for %.4f ' %step +          \
    #'seconds\n' +                                \
    #'Target = %f\n' % setTemp

    # Stop the timer
    cycleStop = time.time() 
    # Wait 10 Seconds min (PID tuned to this value)
    if step < 10 : time.sleep(10 - (cycleStop - cycleStart))
    else : time.sleep(step - (cycleStop - cycleStart))
    
