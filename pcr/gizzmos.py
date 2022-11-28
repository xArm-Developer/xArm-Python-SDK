from xarm.wrapper import XArmAPI

'''
    Gizzmos are external hardware elements that assist the robotic arm
    complete the PCR challenge! This list includes ECLAIR's specially
    curated gripper, a pressure sensor, and the Atalanta Module which
    uses OCR to adjust the volume on the pipettes to the desired amount
    for the reactant
'''

class CustomGripper:

    def __init__(self, arm: XArmAPI) -> None:
        self.arm = arm
        pass

    def close_gripper(self) -> str:
        return None, None

    def open_gripper(self) -> str:
        return None, None

    def fill_pipette(self) -> str:
        return None, None

    def empty_pipette(self) -> str:
        return None, None

    def remove_pipette_tip(self) -> str:
        return None, None

class PressureSensor:

    def __init__(self):
        pass

class AtalantaModule:

    def __init__(self):
        pass

    def adjust_volume(self, volume: float) -> str:

        # Communicate with arduino/RPI externally somehow
        # TCP Network socket maybe?
        return None, None

    def check_connection(self) -> bool:
        return True