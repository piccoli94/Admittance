STEP TO CONTROL KUKA MED WITH REGION CONTROLLER.

1) connect robot and sensor to the pc
2) check the connection from deviced
3) to acquire data from sensor, go to launch folder, open terminal, write " sh ati_launch.sh ". This will be launch an nft ros node inside catkin_folder.
4) launch the controller.

tips: in H-DR robot will not move itself, only with a human will make a movement. So if you start the test with error's norm equal to 0, robot won't move itself. 

tips: in TS-SR/SS-SR robot will not move. So if you start the test with error's norm greater then a_r, robot won't move. 




