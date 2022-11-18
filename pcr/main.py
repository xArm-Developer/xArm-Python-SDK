#!/usr/bin/env python3
import os
import sys
import argparse

from xarm.wrapper import XArmAPI
from gizzmos import CustomGripper, PressureSensor

class RunJob():

    def __init__(self, arm: XArmAPI) -> None:
        self.arm = arm

    def run(self) -> None:
        # Run a new PCR job
        self.grabTestTube()
        self.placeTestTubeInRack()
        self.grabPipette()
        self.attachPipetteTip()

    def grabTestTube(self) -> None:
        pass

    def placeTestTubeInRack(self) -> None:
        pass

    def grabPipette(self) -> None:
        pass

    def attachPipetteTip(self) -> None:
        pass


if __name__ == '__main__':
    # Add xArm python modules to the PATH
    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

    # Parse through all the arguments
    parser = argparse.ArgumentParser(description='Please add the configuration of the robotic arm needed at runtime')
    parser.add_argument('--ip', '-i', default='192.168.1.166', help='IP address of the xArm6 Robotic Arm')
    args = parser.parse_args()

    # Create an xArm instance
    arm = XArmAPI(args.ip)
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    arm.reset(wait=True)

    job = RunJob(arm)
    job.run()

    arm.reset(wait=True)
    arm.disconnect()