# DIME Inverse Kinematics Based Tele-Operation
This repository is part of the official implementation of [DIME](https://arxiv.org/abs/2203.13251). It contains the package for performing TeleOp using an Allegro Hand attached to a Kinova Arm.

## Setup instructions
- Setup the [ar-tracker-alvar](http://wiki.ros.org/ar_track_alvar) and [Realsense ROS](https://github.com/IntelRealSense/realsense-ros) packages before using this package.
- Also install the [`librealsense`](https://github.com/IntelRealSense/librealsense#installation-guide) SDK with the appropriate python wrapper.
- Clone the repository and use the following command to install the package:
```
pip3 install -e .
```

## Running the teleop script.
- To run the hardware teleop:
```
cd <path-to-this-repository>/ik_teleop
python3 teleop.py
```

- To run the simulation teleop:
```
cd <path-to-this-repository>/ik_teleop
python3 sim.py
```

## Citation

If you use this repo in your research, please consider citing the paper as follows:
```
@article{arunachalam2022dime,
  title={Dexterous Imitation Made Easy: A Learning-Based Framework for Efficient Dexterous Manipulation},
  author={Sridhar Pandian Arunachalam and Sneha Silwal and Ben Evans and Lerrel Pinto},
  journal={arXiv preprint arXiv:2203.13251},
  year={2022}
}
```
