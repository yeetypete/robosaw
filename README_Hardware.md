### Mechanical
RoboSaw uses a roller-intake mechanism to feed lumber through the frame and to the Metabo miter saw. The intake clamps the wood from the top using a steel roller arm attached to two pairs of 10 lb gas springs. The gas springs damp the intake so that it conforms around the wood without snapback if the wood is removed.

The saw 
The turntable mechanism, which allows the RoboSaw to perform angled cuts, is still a work in progress. Currently, the mechanism uses a rack and pinion gear setup. The rack is mounted to the stationary back of the miter saw turntable and the pinon is mounted to the front 

# RoboSaw Blade Actuation Mechanism
This section will describe the RoboSaw Actuation Mechanism and its various components. This mechanism is responsible for performing the cut quickly and safely.

## Hardware
The RoboSaw Blade Actuation mechanism is comprised all aluminum components. These were either ordered or milled in the EPIC facilities using the provided CAD and CAM files.

Parts included in the CAD:
 
1. Plate.SLDPRT-The main structural component of the mechanism

2. 2313N39_Corner Machine Bracket.STEP - Right angled bracket connected the the components together.

3. Actuator_Mount.SLDPRT- The component the linear actuator attaches to.

All of the screws, bolts, and nuts can be found in the Bill of Materials for the LInear Actuator Mechanism.

### Notes for Assembly

- Both Plate.SLDPRT and the Actuator_Mount.SLDPRT were milled out of 3/8 inch aluminum. This was done to increase the rigidity of the entire mechanism during the cutting process.

- The mounting plate was mounted using existing holes  and threads on the stock saw. We mounted the plate using a 3/8-16 X 5 inch bolt through the center hinge of the saw and a 5/16-18 X 1 inch screw.

- All of the milled components and linear actuator mount were secured together using 1/2-20X1-1/2 inch screws and 1/2-20 nuts.

- The linear actuator was mounted using a mount purchased online to the Actuator_Mount.SLDPRT component. A M6X0.75mm, 40mm long screw was used to attach the other end of the linear actuator to an existing threaded hole on the saw.

- In order for the linear actuator to work properly, you must remove the stopper screw on the side that allows the saw to return to its original position. If this is not removed the linear actuator mechanism will experience high loads that can result in a bolt shearing, parts breaking, and potential injury.

# RoboSaw Turn Table Mechanism

This section describes the RoboSaw turn table mechanism. This mechanism allows the base of the saw to rotate so we can perform angled cuts.

## Hardware

There are many components that were purchased online but there are a few that were milled at the EPIC facilities. 

Parts included as CAD files:

1. rotating_table_gear_mount.SLDPRT- This attaches to the center of saw. It allows a gear to be attached in order for the motor to rotate the table.

2. rot_table_motor_mount.SLDPRT- The motor mounts to this plate next to the gear on the rotating_table_gear_mount.SLDPRT

### Notes For Assembly:

There were several version of the turn table mechanism. There ws Version 1 which was mounted underneath the table of the saw. There was also Version 2 that was mounted on the outside. Both used the rot_table_motor_mount.SLDPRT.

#### Version 1:

This version was mounted under neath the table of the saw. There was an issue with the motor slipping under load and disengaging the teeth. 

#### Version 2:

This version was mounted on the outside of the saw. We used a 3D printed rack and pinion to test the motion. This version worked however it limited out movement on one side of the saw. This limited the angel to 35 degrees on one side. The CAD models of the 3D printed rack and pinion are included.
