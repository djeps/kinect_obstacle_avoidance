# Table of Contents

- [System Setup](SETUP.md)
- [Installing `libfreenect`](LIBFREENECT.md)
- [Python prerequisites and environment](PYTHONENV.md)
- [About the Kinect v1](KINECT.md)
- [Basic operation](DEMO.md)

## Summary

What I tried accomplishing in this project was to use a depth sensor and the depth data it returns and
implement an *obstacle avoidance* application for vehicles performing parking activities. In doing so,
I consciously and deliberately made *few* assumptions and generalizations:

- For one, this type of functionality in any vehicle with automated or assisted driving capabilities is
more than likely implemented with more than one type of sensor
- In fact, it might not even involve a RGB-D camera in the first place
- I focused solely on moving forward and around obstacles and completely ignored going in reverse
- In the absence of any physical dimensions of a vehicle, the driving lane and the surrounding physics
were completely simulated
- The width, height and length of the vehicle were completely neglected
- in fact, *the vehicle* doesn't even exist!
- Another assumption I made for the sake of simplicity, is that the sensor is positioned in the front
of the imaginary vehicle and in the so-called ground plane
- The curvature of the lane going forward doesn't reflect the physics of Ackermann steering
- The horizon is also (more or less, take your pick) random

This project was a *Proof of Concept* or a POC, and even though with almost no practical implications,
it was nevertheless - a fun project to do. I happened to have an Xbox Kinect v1 lying around and was
looking for an example application. The Kinect was probably the first sensor of its kind back in the 
day and while it doesn't seem as if there are any open source projects dealing with the Kinect actively
maintained (this is especially true for the [OpenKinect project](http://www.openkinect.org/)), it still
remains a popular choice for hobbyist projects.
