import os
from setuptools import setup, find_packages

here = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(here, "README.md"), encoding="utf-8") as f:
    long_description = f.read()

setup(
    name="vicpinky_api",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "pyserial",
        "rplidarc1",   # RPLIDAR C1 라이다
    ],
    author="mkh",
    author_email="kyung133851@pinklab.art",
    description="High-level Python API for Vic Pinky Robot",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/pinklab-art/vic_pinky_library",
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: POSIX :: Linux",
        "Topic :: Scientific/Engineering :: Robotics",
        "Intended Audience :: Developers",
    ],
    python_requires=">=3.8",
)
