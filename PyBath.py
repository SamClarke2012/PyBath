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
import pylab

class PID:
    """ PID control class.
    """
    def __init__(self, Kp, Ki, Kd):

        # Gain variables
        self.Kp = Kp
        self.Kd = Ki
        self.Ki = Kd
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
        # co = Controller output (KJ), w = Element wattage
        # co * 1000 = co in Joules (Joules = W/sec)
        # /w = Seconds to be on
        return co*1000/w

    def boltzmanLoss (self, e, t, c, a):
        # P = EOA(T^4-Tc^4)
        """ Returns the Stephan-Boltzman radiated loss of the system (deg/sec).
            Takes the system emissivity (float/int e), the bath temperature 
            (float/int t), the room temperature (float/int c) and the area 
            (m^2) of the tank.
        """
        t += 273.15 # Convert to Kelvin
        c += 275.15 # Convert to Kelvin
        # Return Watts lost per second (1W/sec = 1 Joule)
        return e*5.6703e-08*a*((t**4)-(c**4)) 

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

prev = 0
envTemp = 0.0
bathTemp = 0.0
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=10)

setPoint = float(raw_input('Enter the desired temperature (\'c) '))
specHeat = float(raw_input('Enter the specific heat of the bath medium (Kj/Kg\'c) ')) 
weight   = float(raw_input('Enter the weight of the bath medium (Kg) '))
e        = float(raw_input('Enter the emissivity of the tank material (white plastic ~= 0.89) ')) 
a        = float(raw_input('Enter the area of the tank (m^2 - (note: calculate to meniscus)) '))
watts    = float(raw_input('Enter the wattage of the bath element (Watts)'))
print '\n'

# Save the temperature form of the set point
setTemp = setPoint
# Get initial temperature data
bathTemp = getTemp('bath')
envTemp  = getTemp('environment')
# Setup the object
P = 0.004
I = 0.004
D = 0.0025
p = PID(P,I,D)
# Convert from degrees to K-Joules
setPoint = p.inputHandler(envTemp, setPoint, weight, specHeat) #
# Boltzman Watts/Joules radiated per second, to K-Joules
boltzmann = p.boltzmanLoss (e, setPoint, envTemp, a) / 1000.0
setPoint += boltzmann
# Save variables for graph
bathTemps = []
onTimes = []
steps = 0

diff = abs(setTemp - bathTemp)


while steps <= 120: # While true usually
    
    steps += 10
    # Time this operation
    cycleStart = time.time()
    # Get fresh data
    bathTemp = getTemp('bath')
    envTemp  = getTemp('environment')
    # Adaptive tuning
    d = abs(setTemp - bathTemp)
    # Below half way, agressive tuning
    if d >= diff/2:
        p.setKp(P*1.5)   
        p.setKi(I*1.5)  
        p.setKd(D*1.5)
    # Within 0.5 degree, conservative tuning
    elif d <= 0.5:
        p.setKp(P*0.7)   
        p.setKi(I*0.7)  
        p.setKd(D*0.7) 
    # Else normal tuning       
    else:
        p.setKp(P)   
        p.setKi(I)  
        p.setKd(D)
    # Convert it to K-joules stored
    invars = p.inputHandler(envTemp, bathTemp, weight, specHeat)
    # get the controller output movement
    out = p.genOut(setPoint - invars)
    # Convert it from ideal total K-joules to Watt/seconds
    t = p.outputHandler(out, watts)
    # Add calculated loss
    t += p.boltzmanLoss (e, bathTemp, envTemp, a) / 1000.0
    # Convert that to a step from our current energy state
    step = t - prev
    # Handle cooling and overshoot
    if bathTemp > setTemp or step < 0 : step = 0.0
    #if bathTemp - setTemp > 0.1 or step < 0 : step = 0.0
    # Save the current energy state
    prev = t 
    # graph lists
    bathTemps.append(bathTemp)
    onTimes.append(step+20)
    # Send the planned step to the hardware controller, limited to 10sec
    if step > 10:
        step = str(int(10*1000)) + '\n'
    else:
        step = str(int(step*1000)) + '\n'
    ser.write(step) 
    # update
    print 'Bath temperature %.2f \n' %bathTemp +     \
        'EnvironmentTemp %.1f\n' %envTemp +          \
        'Heating element for %s' %step[:-1] + 'ms\n' + \
        'Target = %f\n' % setTemp                  
    # Stop the timer
    cycleStop = time.time() 
    # Wait 10 Seconds
    if int(step)/1000 < 10 : time.sleep(10 - abs(cycleStop - cycleStart))
    else : time.sleep(10)


title = '{0} minutes, P = {1} I = {2} D = {3}'.format((steps/60.0),P,I,D)
pylab.title(title)
pylab.plot(bathTemps, label='Bath Temperature (C)')
pylab.plot(onTimes, label = 'Element Output (S) + 20')
sp = [setTemp for i in xrange(len(bathTemps))]
pylab.plot(sp, label = 'Target Temp (C)')
pylab.legend()
pylab.show()
