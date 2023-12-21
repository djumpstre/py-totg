# Python Wrapper for Time Optimal Trajectory Generation

The used cpp source code for `TOTG` comes from the [tobiaskunz/trajectories](https://github.com/tobiaskunz/trajectories/tree/master) repository, authored by Tobias Kunz 


## Build

Prerequisites:
```bash
# 1. Pybind11
# make sure that the pybind11 is installed in the global side_pacakges
pip3 install pybind11[global]
# 2. Eigen
sudo apt install libeigen3-dev
``` 

Build:
```bash
mkdir build
cd build
cmake ..
make
```

Install:
```bash
pip3 install . 
```
