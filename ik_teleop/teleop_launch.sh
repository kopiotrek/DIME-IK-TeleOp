#!/bin/bash
# Initialize Conda and source bashrc
source ~/.bashrc

# Open terminal 1 with the first set of commands
terminator -e "
cd ~/RPL/DIME-Controllers/;
source devel/setup.bash;
roslaunch allegro_hand allegro_hand.launch;
exec bash" &

sleep 2

# Run commands for terminal 2
terminator --new-tab -e "
cd ~/RPL/DIME-IK-TeleOp/ik_teleop/;
source ~/RPL/DIME-Controllers/devel/setup.bash;
source /home/piotr/anaconda3/etc/profile.d/conda.sh;
conda activate allegro;
python allegro_controller.py;
exec bash" &

# Run commands for terminal 3
terminator --new-tab -e "
cd ~/RPL/DIME-IK-TeleOp/ik_teleop/;
source ~/RPL/DIME-Controllers/devel/setup.bash;
source /home/piotr/anaconda3/etc/profile.d/conda.sh;
conda activate allegro;
python index_controller.py;
exec bash" &

# # Run commands for terminal 4
# gnome-terminal -- bash -c "
# cd ~/RPL/DIME-IK-TeleOp/ik_teleop/;
# source ~/RPL/DIME-Controllers/devel/setup.bash;
# source /home/piotr/anaconda3/etc/profile.d/conda.sh;
# conda activate allegro;
# python middle_controller.py;
# exec bash" &

# # Run commands for terminal 5
# gnome-terminal -- bash -c "
# cd ~/RPL/DIME-IK-TeleOp/ik_teleop/;
# source ~/RPL/DIME-Controllers/devel/setup.bash;
# source /home/piotr/anaconda3/etc/profile.d/conda.sh;
# conda activate allegro;
# python ring_controller.py;
# exec bash" &

# # Run commands for terminal 6
# gnome-terminal -- bash -c "
# cd ~/RPL/DIME-IK-TeleOp/ik_teleop/;
# source ~/RPL/DIME-Controllers/devel/setup.bash;
# source /home/piotr/anaconda3/etc/profile.d/conda.sh;
# conda activate allegro;
# python thumb_controller.py;
# exec bash" &

# # Run commands for terminal 7
# gnome-terminal -- bash -c "
# cd ~/RPL;
# source devel/setup.bash;
# source /home/piotr/anaconda3/etc/profile.d/conda.sh;
# conda activate allegro;
# roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=192.168.7.108 tcp_port:=10000;
# exec bash" &

# # Run commands for terminal 8
# gnome-terminal -- bash -c "
# cd ~/RPL/DIME-IK-TeleOp/ik_teleop/ik_core;
# source ~/RPL/DIME-Controllers/devel/setup.bash;
# source /home/piotr/anaconda3/etc/profile.d/conda.sh;
# conda activate allegro;
# python allegro_keypoints_publisher.py;
# exec bash" &

# # Run commands for terminal 9
# gnome-terminal -- bash -c "
# cd ~/RPL/DIME-IK-TeleOp/ik_teleop/ik_core;
# source ~/RPL/DIME-Controllers/devel/setup.bash;
# source /home/piotr/anaconda3/etc/profile.d/conda.sh;
# conda activate allegro;
# python hands_aligment.py;
# exec bash" &

sleep 5

# Make terminal 1 visible and run the first task
xdotool windowactivate $(xdotool search --name "Terminal" | head -n 1)
