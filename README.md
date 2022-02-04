# Setup instructions
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
cd <path-to-this-repository>
python3 ./ik_teleop/sim.py
```

- If recording demonstrations without ros/rospy and in simulation:
    python teleop_utils/calibrate.py

