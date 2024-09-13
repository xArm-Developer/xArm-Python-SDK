__all__: list[str] = []

import typing as _typing


# Classes
class CoreML:
    # Functions
    def __init__(self) -> None: ...

    def cfgUseCPUOnly(self) -> CoreML: ...

    def cfgEnableOnSubgraph(self) -> CoreML: ...

    def cfgEnableOnlyNeuralEngine(self) -> CoreML: ...


class CUDA:
    # Functions
    @_typing.overload
    def __init__(self) -> None: ...
    @_typing.overload
    def __init__(self, dev_id: int) -> None: ...


class TensorRT:
    # Functions
    @_typing.overload
    def __init__(self) -> None: ...
    @_typing.overload
    def __init__(self, dev_id: int) -> None: ...


class OpenVINO:
    # Functions
    @_typing.overload
    def __init__(self) -> None: ...
    @_typing.overload
    def __init__(self, dev_type: str) -> None: ...

    def cfgCacheDir(self, dir: str) -> OpenVINO: ...

    def cfgNumThreads(self, nthreads: int) -> OpenVINO: ...

    def cfgEnableOpenCLThrottling(self) -> OpenVINO: ...

    def cfgEnableDynamicShapes(self) -> OpenVINO: ...


class DirectML:
    # Functions
    @_typing.overload
    def __init__(self) -> None: ...
    @_typing.overload
    def __init__(self, device_id: int) -> None: ...
    @_typing.overload
    def __init__(self, adapter_name: str) -> None: ...



