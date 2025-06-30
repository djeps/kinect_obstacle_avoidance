[Home](README.md)

# System Setup

The HW in question (or rather the two main components) with which this POC was realized was:

-  x1 Nvidia Jetson Nano
-  x1 Xbox Kinect v1

The host OS on the Nano was Ubuntu 20.04 LTS - specially prepared for the Jetson Nano and not
an official one. For more info, please check out
[Q-engineering's GitHub repo](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image).
This I believe to be practically at the limit of what could be deployed and run on the Nano:

- Nvidia offers no support (official or otherwise) or any drivers beyond Ubuntu 18.04 LTS 
- Most of the (system) packages image are at very specific release numbers, and
- Have been built from source
- We won't be able to find or install newer releases because there are simply no packages
maintained

We would like to maintain this and thus work in an *isolated* or a virtual Python environment
for any additional packages we might need to *pip install*.

[Home](README.md)
