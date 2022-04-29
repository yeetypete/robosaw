<p align="center" width="100%">
    <img width="50%" src="./readme_media/logo.png">
</p>

The Engineering Addendum is quick-start documentation written to any future team that may continue to work on your project. This is where you outline the gotchas of your project, types of things to look out for, the current state of the project, etc.  The purpose of README.md is to save any future team weeks of detective work just to get to where you are today. Think back to what types of things you had wished you knew earlier, doing future teams a favor by passing that knowledge along.  

# RoboSaw Engineering Addendum

## Introduction
RobSaw is a collaborative robot miter saw that can cut lumber to size autonomously by detecting a line drawn on the wood with a pen or pencil using computer vision, and then making a cut. This README details the intricacies and nuances of the project and serves as a general guide for getting started with creating a new RoboSaw.

## Current State of the Project

### Hardware
Currently RoboSaw is able to perform cross cuts on 2x4, 2x6, and 4x4 lumber up to 8 ft in length. A control pendant is used to interface with the RoboSaw, containing an emergency stop button, a run button, and a cut button. The run button is pressed to move the wood through the RoboSaw until the computer vision system, which uses three separate cameras, detects a line. Once a line is detected, the cut button can be pressed to perform a cut, and the 

### Electronics


### Software


## Pitfalls to Avoid
### Mechanical
When designing mechanisms for RoboSaw it is important to make them only as complex as they need to be to accomplish their task. RoboSaw is a piece of industrial equipment where robustness and durability are requirements both from a user and safety perspective. If a mechanism breaks, work on the jobsite could slow down or unexpected behavior could occur, potentially resulting in human injury. Most complex mechanisms designed for RoboSaw, such a camming mechanism designed to hold the wood in place during a cut were later replaced with a simpler, more robust approach, in this case, simply angling the intake wheels towards the miter saw fence so that a portion of their force is directed at holding the wood in place.

Because RoboSaw must be capable of handling very heavy 20 lb or 30 lb pieces of lumber, each mechanism has to be capable of exerting large forces without bending or breaking. This challenge means that in almost all situations, 3D printed or plastic parts will simply not cut it. Parts that directly interact with wood stock should be machined out of 6061 Aluminum for flat plates and the harder 7075 Aluminum alloy for shafts and parts exposed to torsional loads. Most heavy-duty parts are 0.375 in or 0.25 in thick depending on their load requirements. Even with parts made out of metal, mechanisms should be designed in such a way as to minimize internal stress.

### Software
Camera 

## Future Work
While the RoboSaw has proved its capabilities in cutting wood stock to size as a prototype, much remains to be tested to see if the RoboSaw would hold up in a jobsite environment. Future work should build on the RoboSaw platform, adding to its capabilities and testing the current design approach to see if it is truly robust enough to perform consistently in a more uncontrolled environment such as an active construction site.

### Dust Ingress
A potential issue that may affect the RoboSaw in a jobsite environment is the ability of the computer vision system and the cameras to handle dust ingress. To date, RoboSaw has been exposed to a limited amount of sawdust produced by its own cuts and no dust from other external sources. Although steps were taken to minimize the impact of dust, such as putting the electronics in an enclosure and using fanless power supplies, dust still has the potential to affect the cameras which are currently only protected dust by their placement away from locations that are known to accumulate sawdust. Dust particles covering the camera, surfaces used for wood localization and the wood itself, will make seeing lines more difficult and may affect the RoboSaw's ability to align a drawn line with the blade or compromise the PID loop, which has been tuned in a dust free environment. Using cameras that are IP rated in a future revision will help make the system more durable and suited for a jobsite environment.

Particulate can also affect the mechanical components of RoboSaw. The linear actuator is IP65 rated, and the motors, while not IP rated, have some dust ingress protection because of their mounting position. Particulate can still affect the gas springs on the roller intake, as well as the friction between the wheels of the intake and the wood stock. These issues should be addressed in a future revision of RoboSaw.

### Turntable Mechanism
The turntable mechanism allows the RoboSaw to perform angled cuts by aligning the saw blade with the drawn line. The turntable mechanism was likely the most difficult to implement because it had mount to the unmodifiable Metabo Miter saw. A bevel gear implementation around the center bolt of the turntable was tested, but the mechanism had too much slop to fit within the angle cutting requirements of the project (the error is exacerbated the farther the radius of the turntable is from the center pivot).A rack and pinion gear setup seems to be the most promising. Such a design was successfully tested with the rack being mounted to the stationary part of the turntable and the pinion mounted to the motor being free to rotate with the turntable. This approach, however, limited the RoboSaw to being able to perform 45&deg; cuts only on one side of the straight cut position. A future revision could implement the same rack and pinion setup, except with the rack mounted underneath the RoboSaw moving together with the turntable and the pinion mounted to the motor being stationary. 

### Stand Adjustment
The miter saw stand for the RoboSaw required readjustment for each type and size of wood being cut by the RoboSaw because of the wood's different weights and different moments. A future revision should add support to the miter saw stand extensions so that they do not flex under load and misalign the wood the RoboSaw intake.

### Power Requirements
Currently RoboSaw requires two 120V AC power outlets to operate, one for powering the AC-DC 12V 20A converter that powers all the RoboSaw electronics, and one for the miter saw itself. Provided that the power draw does not exceed any load limitations, a future revision should combine the two cables and then reroute them internally once power reaches the RoboSaw.

### Software Improvements
The RoboSaw Python software currently runs in a single-threaded mode where only one camera feed may be open at a single time. A future revision of the software should enable support for retrieving camera frames from all three cameras on the RoboSaw simultaneously by using a multithreaded software approach. This will likely result in a significant speed increase because the computer vision image processing on RoboSaw will no longer by IO bound. To further improve the speed of the software, higher frame rate, global shutter cameras may also be used. Currently RoboSaw uses two 60 fps global shutter cameras for aligning the line with the blade and detecting the line drawn on the wood. 120 fps cameras may allow the PID loop to run more quickly, resulting in a smoother, faster, and more accurate cut.

Originally, the scope of the RoboSaw project also included software which enables the users to create cut lists without drawing a line on each piece of wood. Although this feature was not implemented, adding it to a future revision of the RoboSaw will increase its value proposition for repetitive wood stock cutting operations where the required lengths of wood stock are known in advance.

## Conclusion
The initial goals of this project was to create an extremely versatile and easy to use collaborative wood stock cutting robot for the jobsite construction industry.  Collaborative robot designs like RoboSaw take advantage of the strengths of humans and robots to increase the speed and efficiency at which tasks are performed. While not every one of the initial goals of this project were met, the the ability of RoboSaw to significantly improve the safety of cutting wood stock in addition to increasing the speed of cutting wood indicates that collaborative robots have a potential to transform a jobsite construction environment. 
