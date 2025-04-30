import os
import sys

if sys.version_info.major > 3 or (sys.version_info.major == 3 and sys.version_info.minor >= 8):
    # os.environ['_PYPROJECT_HOOKS_BUILD_BACKEND'] = 'hatchling.build'
    # requires = [
    #     "hatchling",
    # ]
    # build-backend = "hatchling.build"
    from hatchling.build import *
else:
    # os.environ['_PYPROJECT_HOOKS_BUILD_BACKEND'] = 'setuptools.build_meta'
    # requires = [
    #     "setuptools >= 40.8.0",
    # ]
    # build-backend = "setuptools.build_meta"
    from setuptools.build_meta import *