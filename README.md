# Passivity-based interaction control for robot-aided limb rehabilitation
This is my master degree thesis and in this project i've designed an Adaptive Admittance control algorithm for human-machine colalboration. The aim is help patients, that have lost part of their limb motion control by Stroke and traumatic brain injury, during the rehabilitation process.

The key is Split out operative space into 3 regions:
-H-DR: Human dominant-region where human is free and there isn’t any control
-R-DR: Robot dominant-region where the robot apply a motion control to correct the patient
-S-SR: Safety stop-region where the controller blocks every dangerous movement

In addition to these controls there is as interaction controller which recognize the direction of the arm and choose to enchance this movemnt if it would be correct or block it if wouldn't. Therefore a fatigue factor has been added to the algorithm to adapt it to the patient condition. The system starts with fatigue equal to 1 (no effort of enhance). If the lost of motion control is significant, the fatigue is increased to 2 or more to enchance the movements. Otherwise, if the patient has regained part of his mobility, the fatigue is decreased to 0.5 or less to make him exert more effort and speed up his recovery.

I used it on KUKA MED 7 AND 14 with KUKA FRI 
