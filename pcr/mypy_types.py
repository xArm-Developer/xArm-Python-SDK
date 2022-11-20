from typing import Literal, TypedDict, List


class ReactantDict(TypedDict, total=False):
    name : str
    quantity : str
    type : str

class TimeDict(TypedDict):
    time : str
    temperature : str

class RequestDict(TypedDict):
    job_id : str
    version : str
    micropipette : Literal['eppendorf']
    thermal_cycler : Literal['Thermo Fischer Scientific']
    cycles : int
    reactants : List[ReactantDict]
    times : List[TimeDict]

