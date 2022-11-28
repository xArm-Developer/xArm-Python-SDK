#!/usr/bin/env python3
import os
import sys
import time
import json
import argparse
from mypy_types import RequestDict
import logging
from pcr.gizzmos import AtalantaModule

from xarm.wrapper import XArmAPI
from gizzmos import CustomGripper, PressureSensor

class RunJob():
    SPEED = 50
    ZERO_POS   = [0, 0, 0, 0, 0, 0]
    GIZMO_POS  = [0, 9.9, 31.8, 0, -70, 0]
    INTIAL_POS = [0, 9.9, 31.8, 0, -70, 0]

    def __init__(self, arm: XArmAPI, data: RequestDict) -> None:
        self.arm = arm
        self.verify_inputs(data)
        
        self.job_id        = data['job_id']
        self.cycles        = data['cycles']
        self.reactants     = data['reactants']
        self.thermal_times = data['times']

        self.gripper  = CustomGripper(arm)
        self.atalanta = AtalantaModule()

    def verify_inputs(self, data: RequestDict) -> None:
        # Check if the brand of thermal cycler and micropipette is currently supported
        if data['micropipette'] != 'eppendorf' or data['thermal_cycler'] != 'Thermo Fischer Scientific':
            msg = 'JSON data is invalid'
            logging.error(msg)
            raise Exception(msg)

    def run(self) -> None:
        # Run a new PCR job
        self.arm.set_servo_angle(angle=self.INTIAL_POS, speed=self.SPEED, wait=True)
        self.grab_test_tube()
        self.place_tube_in_rack()
        self.grab_pipette()

        # Acquire the different reactants and their respective volumes
        for reactant in self.reactants:
            name   = reactant['name']
            volume = reactant['quantity']

            amt, units = volume.split(' ')

            if units == 'ul':
                amt = int(amt)
            elif units == 'ml':
                amt = int(amt)
                amt *= 100
            else:
                msg = f'Could not interpret volume of {volume}'
                logging.error(msg)
                raise Exception(msg)

            logging.info(f'Attempting to acquire {volume} of {name}')
            self.adjust_volume(amt)

            # Get the tip, obtain the right volume, go to test tube, deposit the
            # reactant, and drop the pipette tip
            self.attach_pipette_tip()
            self.move_to_reactant(reactant['location'])
            self.gripper.fill_pipette()
            self.move_to_test_tube()
            self.gripper.remove_pipette_tip()

        self.arm.set_servo_angle(angle=self.ZERO_POS, speed=self.SPEED, wait=True)

    def grab_test_tube(self) -> None:
        time.sleep(5)
        pass

    def move_to_test_tube(self) -> None:
        pass

    def place_tube_in_rack(self) -> None:
        self.move_to_test_tube()
        pass

    def grab_pipette(self) -> None:
        pass

    def adjust_volume(self, volume: int) -> None:
        err = self.atalanta.adjust_volume(volume)

        if err is not None:
            logging.error(err)
            raise Exception(err)

    def attach_pipette_tip(self) -> None:
        pass

    def move_to_reactant(reactant_idx: int) -> None:
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