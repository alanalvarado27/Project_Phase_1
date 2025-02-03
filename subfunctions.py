import numpy as np
import math

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
            raise Exception("There is an angle not within the valid range of -75 and 75 deg.")
    if(not isinstance(rover, dict) or not isinstance(planet, dict)):
        raise Exception("The last two inputs are not both valid dicts.")
        
    g_planet = planet["g"] #in m/s
    
    gravitational_forces = []
    
    for x in terrain_angle:
        if(x <= 0):
            gravitational_forces.append(math.sin(x) * get_mass(rover) * g_planet * -1)
        else:
            gravitational_forces.append(math.sin(x) * get_mass(rover) * g_planet * -1)
    
    Fgt = np.array(gravitational_forces)
    return Fgt
    
    
    
    
        
    