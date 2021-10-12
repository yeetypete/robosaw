# Ideas for RoboSaw design

## Miter saw to buy
1. https://www.amazon.com/gp/product/B07PX44JQM/ref=ox_sc_act_title_1?smid=ATVPDKIKX0DER&psc=1
1. https://www.homedepot.com/p/RYOBI-10-in-Compound-Miter-Saw-with-LED-TS1346/306939211
1. https://www.homedepot.com/p/DIABLO-10-in-x-40-Tooth-General-Purpose-Circular-Saw-Blade-D1040X/100055325

Metabo is probably better b/c it has mounting points for extenders and additional hardware.

## Issues to resolve
1. Linear actuator to bring down saw must tilt with saw (left side where bolts are)
1. Securing wood in place during cut (maybe can be two in one with the rolling mechanism, springs?)
1. Securing mitre saw in place during cut
1. Where to place camera
1. How to adjust bevel cut (more difficult b/c of heavy weight, can be secondary goal)
1. How to feed in wood consistently when more than one piece is stacked
1. How to handle excess wood
1. How to allow manual operation (lead screw pitch for backdrivability)
1. What needs position control feedback? Can we use the camera to compensate?
1. What electronics are needed?
1. Figure out motor torque ratings
1. How to wire electronics

## Issue solutions
1. off angle wheels to pull wood stock against side
1. Springs on rollers to hold wood stock down
1. Counterweights to allow motors to do less work
1. Side spring loaded wheels to push against wall

## Safety Things
1. Emergency shutoff button

# Todo
1. Electrically wire power button but still allow mechanical operation
1. Use existing mounting points and hardware as much as possible (only drill holes if necessary)
1. Can remove existing side plate if necessary
1. Math to determine mitre saw intersection position with wood when beveled