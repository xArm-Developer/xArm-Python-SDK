#!/usr/bin/env python3
import os
import sys
import time
import json
import argparse

from xarm.wrapper import XArmAPI
from gizzmos import CustomGripper, PressureSensor
from mypy_types import RequestDict

class RunJob():
    SPEED = 50
    ZERO_POS   = [0, 9.9, 31.8, 0, -70, 0]
    INTIAL_POS = [0, 9.9, 31.8, 0, -70, 0]

    def __init__(self, arm: XArmAPI, data: RequestDict) -> None:
        self.arm = arm
        self.verify_inputs(data)
        
        self.job_id        = data['job_id']
        self.cycles        = data['cycles']
        self.reactants     = data['reactants']
        self.thermal_times = data['times']

    def verify_inputs(self, data: RequestDict) -> None:
        # Check if the brand of thermal cycler and micropipette is currently supported
        if data['micropipette'] != 'eppendorf' or data['thermal_cycler'] != 'Thermo Fischer Scientific':
            raise Exception('JSON data is invalid')

    def run(self) -> None:
        # Run a new PCR job
        self.arm.set_servo_angle(angle=self.INTIAL_POS, speed=self.SPEED, wait=True)
        self.grabTestTube()
        self.placeTestTubeInRack()
        self.grabPipette()
        self.attachPipetteTip()
        self.arm.set_servo_angle(angle=self.ZERO_POS, speed=self.SPEED, wait=True)

    def grabTestTube(self) -> None:
        time.sleep(5)
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

    data = RequestDict(json.load(open('test_inputs.json')))

    # Create an xArm instance
    arm = XArmAPI(args.ip)
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    arm.reset(wait=True)

    job = RunJob(arm, data)
    job.run()

    arm.reset(wait=True)
    arm.disconnect()