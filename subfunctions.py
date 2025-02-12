import numpy as np
import math
from scipy.special import erf


def get_mass(rover):
    if(not isinstance(rover, dict)):
        raise Exception("The input argument is not a valid dict.")
        
    total_mass = 0.0
    
    #Total mass of the six wheel assemblies
    total_mass += 6 * rover["wheel_assembly"]["wheel"]["mass"]
    total_mass += 6 * rover["wheel_assembly"]["speed_reducer"]["mass"]
    total_mass += 6 * rover["wheel_assembly"]["motor"]["mass"]
    
    #Total mass for chassis, science payload, and power subsystem
    total_mass += rover["chassis"]["mass"]
    total_mass += rover["science_payload"]["mass"]
    total_mass += rover["power_subsys"]["mass"]
    
    return total_mass
    
def F_gravity(terrain_angle, rover, planet):
    if(not np.isscalar(terrain_angle) and not isinstance(terrain_angle, np.ndarray)):
        raise Exception("The first input is neither a scalar nor a vector.")
    for x in terrain_angle:
        if(x < -75 or x > 75):
            raise Exception("There is an angle not within the valid range of -75 and 75 degrees.")
    if(not isinstance(rover, dict) or not isinstance(planet, dict)):
        raise Exception("The last two inputs are not both valid dictionaries.")
        
    g_planet = planet["g"] #in m/s
    
    gravitational_forces = []
    
    for x in terrain_angle:
        if(x <= 0):
            gravitational_forces.append(math.sin(x) * get_mass(rover) * g_planet * -1)
        else:
            gravitational_forces.append(math.sin(x) * get_mass(rover) * g_planet * -1)
    
    Fgt = np.array(gravitational_forces)
    return Fgt

def define_rover_1():
    #Initialize rover dict for testing
    wheel = {'radius':0.30,
             'mass':1}
    
    speed_reducer = {'type':'reverted',
                     'diam_pimion':0.04,
                     'diam_gear':0.07,
                     'mass':1.5}

    motor = {'torque_stall':170,
             'torque_noload':0,
             'speed_noload':3.80,
             'mass':5.00}
    
    chassis = {'mass':659}
    science_payload = {'mass':75}
    power_subsys = {'mass':90}
    
    wheel_assembly ={'wheel':wheel,
                     'speed_reducer':speed_reducer,
                     'motor':motor}
    
    rover = {'wheel_assembly':wheel_assembly,
             'chassis':chassis,
             'science_payload':science_payload,
             'power_subsys':power_subsys}
    
    planet = {'g':3.72}
    
    return rover, planet
    

def tau_dcmotor(omega,rover):
    omega = np.atleast_1d(omega)
    
    if not isinstance(omega,(int, float, np.ndarray)):    
        raise Exception('Omega must be a scalar or a vector (in array form).')
     
    torque_stall = rover['wheel_assembly']['motor']['torque_stall']
    torque_noload = rover['wheel_assembly']['motor']['torque_noload']
    speed_noload = rover['wheel_assembly']['motor']['speed_noload']    
    
    tau = []
    for i in range(len(omega)):
        
        
        if omega[i] < 0:
            tau.append(torque_stall)
            
        elif omega[i] > speed_noload:
            tau.append(0)
            
        else: 
            
            torque = (torque_stall) - (((torque_stall-torque_noload)/speed_noload)*omega[i]) 
            
            tau.append(torque)

    return tau

def get_gear_ratio(speed_reducer):
    
    if not isinstance(speed_reducer,dict):
        raise Exception('The input argument is not a dictionary.')
    
    if speed_reducer["type"] != "reverted":
        raise Exception('The type gear system in speed_reducer is not reverted')
        
    d1 = speed_reducer['diam_pimion']
    d2 = speed_reducer['diam_gear']
    
    # calculates gear ratio based on diameter
    Ng = ((d2/d1)**2)  
    
    return(Ng)    


    
def F_drive(omega,rover):
    
        omega = np.atleast_1d(omega)
    
        if not isinstance(omega,(int, float, np.ndarray)) :      
            raise Exception('Omega must be a scalar or a vector (in array form).')
            
        #torque into the speed_reducer
        tau = tau_dcmotor(omega, rover)  
        total_Fd = []
        
        for i in range(len(omega)):
            
            #calculate gear ratio
            Ng = get_gear_ratio(rover['wheel_assembly']['speed_reducer'])
            
            # calculate torque out of the spped reducer
            t_out = tau[i] * Ng 
            
            #divide by the radius of wheel
            Fd = (t_out / (rover['wheel_assembly']['wheel']['radius'])) 
            
            # 6 wheels
            total_Fd.append(6 * Fd) 

        return total_Fd




def F_rolling(omega, terrain_angle, rover, planet, Crr):
      
    omega_arr = np.atleast_1d(omega)
    terrain_arr = np.atleast_1d(terrain_angle)
    
    if omega_arr.shape != terrain_arr.shape:
        raise Exception("Omega and terrain_angle must be scalars or vectors of the same size.")
    
    # Validate terrain_angle values.
    if np.any(terrain_arr < -75) or np.any(terrain_arr > 75):
        raise Exception("All elements of terrain_angle must be between -75 and +75 degrees.")
    
    # Validate that rover and planet are dictionaries.
    if not isinstance(rover, dict):
        raise Exception("Rover must be a dictionary.")
    if not isinstance(planet, dict):
        raise Exception("Planet must be a dictionary.")
    
    # Validate that Crr is a positive scalar.
    if not (isinstance(Crr, (int, float)) and Crr > 0):
        raise Exception("Crr must be a positive scalar.")
    
    # Get the gear ratio and wheel radius.
    Ng = get_gear_ratio(rover['wheel_assembly']['speed_reducer'])
    wheel_radius = rover['wheel_assembly']['wheel']['radius']
    
    # Compute the rover’s velocity (m/s) from the motor shaft speed.
    # Motor shaft speed omega is reduced by the gear ratio to give the wheel’s angular speed.
    v = (omega_arr / Ng) * wheel_radius
    
    # Compute the total mass of the rover.
    mass = get_mass(rover)
    
    # Convert terrain_angle from degrees to radians.
    terrain_rad = np.deg2rad(terrain_arr)
    
    # Calculate the normal force on the rover.
    F_N = mass * planet['g'] * np.cos(terrain_rad)
    
    # The “constant” rolling resistance force (summed over all six wheels) is:
    F_rr_const = Crr * F_N
    
    # Compute the error function factor.
    # np.erf(40*v) is near zero when v is near zero, and saturates to ±1 for sufficiently high |v|.
    erf_factor = erf(40 * v)
    
    # To ensure that rolling resistance always opposes the rover’s motion, we include a minus sign.
    Frr = -erf_factor * F_rr_const
    
    return Frr   
        
    