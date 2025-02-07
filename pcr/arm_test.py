import sys
import os
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from xarm.wrapper import XArmAPI

ARM: XArmAPI = XArmAPI('192.168.1.166')

ARM.motion_enable(enable=True)
ARM.set_mode(0)
ARM.set_state(0)
ARM.connect()

# Home Coordinates
x = 287
y = -194
z = 329

# Go to arms home
def goHome():
    ARM.set_position(x, y, z, speed=65, wait=True)

# Move y over after each letter
def addY():
    global y
    y = y + 100

# Write the letter E and go back home
def writeE():
    ARM.set_position(x, y, z + 150, speed=65, wait=True)
    ARM.set_position(x, y + 75, z + 150, speed=65, wait=True)
    ARM.set_position(x, y, z + 150, speed=65, wait=True)
    ARM.set_position(x, y, z + 75, speed=65, wait=True)
    ARM.set_position(x, y + 75, z + 75, speed=65, wait=True)
    ARM.set_position(x, y, z + 75, speed=65, wait=True)
    ARM.set_position(x, y, z, speed=65, wait=True)
    ARM.set_position(x, y + 75, z, speed=65, wait=True)
    
    # update y
    addY()
    
    # go back to home
    goHome()

# Write the letter T and go back home
def writeT():
    global y
    y = y + 32.5
    
    ARM.set_position(x, y, z + 150, speed=65, wait=True)
    ARM.set_position(x, y + 75, z + 150, speed=65, wait=True)
    ARM.set_position(x, y - 75, z + 150, speed=65, wait=True)
    ARM.set_position(x, y, z + 150, speed=65, wait=True)

    # update y
    addY()

    # go back to home
    goHome()

# Write the letter H and go back home
def writeH():
    ARM.set_position(x, y, z + 150, speed=65, wait=True)
    ARM.set_position(x, y, z + 75, speed=75, wait=True)
    ARM.set_position(x, y + 75, z + 75, speed=65, wait=True)
    ARM.set_position(x, y + 75, z + 150, speed=65, wait=True)
    ARM.set_position(x, y + 75, z, speed=65, wait=True)

    # update y
    addY()

    # go back to home
    goHome()

# Write the letter A and go back home
def writeA():
    ARM.set_position(x, y + 32.5, z + 150, speed=65, wait=True)
    ARM.set_position(x, y + 75, z, speed=65, wait=True)
    ARM.set_position(x, y + 53.75, z + 75, speed=65, wait=True)
    ARM.set_position(x, y + 16.25, z + 75, speed=65, wait=True)

    # update y
    addY()

    # go back to home
    goHome()

# Write the letter N and go back home
def writeN():
    ARM.set_position(x, y, z + 150, speed=65, wait=True)
    ARM.set_position(x, y + 75, z, speed=65, wait=True)
    ARM.set_position(x, y + 75, z + 150, speed=65, wait=True)

    # update y
    addY()

    # go back to home
    goHome()

# Function to generate waypoints for a circular arc
def generate_arc(center, radius, start_angle, end_angle, num_points):
    angles = np.linspace(start_angle, end_angle, num_points)
    waypoints = []
    for angle in angles:
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        waypoints.append((x, y, center[2]))  # Keep z constant for 2D circular arc
    return waypoints

def move_in_arc(center, radius, start_angle, end_angle, num_points):
    waypoints = generate_arc(center, radius, start_angle, end_angle, num_points)
    for x, y, z in waypoints:
        ARM.set_position(x, y, z, speed=65, wait=True)

# Example usage
center = (x, y)  # Center of the arc in the XY plane
radius = 50      # Radius of the arc
start_angle = 0  # Starting angle in radians
end_angle = np.pi / 2  # End angle in radians (90 degrees)
num_points = 20  # Number of waypoints

move_in_arc(center, radius, start_angle, end_angle, num_points)



def write_eight():
    ARM.set_position(x, y, z, speed=65, wait=True)


# Go to arms home
goHome()

# Write out 'ETHAN'
# writeE()
# writeT()
# writeH()
# writeA()
# writeN()


# Write out '8'
write_eight()


# Go back to original home
y = -194
goHome()

ARM.motion_enable(enable=False)
ARM.reset(wait=True)
ARM.disconnect()