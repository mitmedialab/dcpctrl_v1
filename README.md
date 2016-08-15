# DCPControl_V1
-----------------

## Overview:
The DCPControl_V1 repository is a body of code used to control the
second iteration of the Digital Construction Platform, developed by the
Mediated Matter Group at the MIT Media Lab. This particular repository
captures the state of the project as of August 2016.

The DCPControl_V1 repository provides the following functionality: 
- Low-level communications with the Altec AT40GW electrohydraulic lift vehicle, via a LabJack T7 interface. 
- Low-level communications with the KUKA KR10 R1100 sixx WP robot arm, via a variety of interfaces. 
- File import, processing and trajectory generation for the DCP system (comprised of the AT40GW and KUKA arms). 
- Real-time control for the DCP system. - Run data processing & visualization for the DCP system.

The DCPControl_V1 repository enables the DCP system to be interacted
with like a conventional robotic arm, albeit in a limited fashion. The
code contained within the repository does not represent any substantial
improvement over the state of the art, except in that it enables the DCP
system to operate.

The DCPControl_V1 repository is not intended for plug-and-play use, or
even use at all (since effective use requires a DCP system, which is
slightly challenging to acquire). Rather, it is intended as a snapshot
of the DCP project's work as of August 2016, and as a reference for
others working in this area.

The DCPControl_V1 repository is licensed under the MIT License.

## Structure:
```
mdcp/ 
  apps/ -- very high-level behaviours to run truck/arm/both
  data/ -- stores data generated during runs/simulations
  lib/ -- External dependencies (e.g. rvctools) rvctools/
  robots/ -- robot-specific behaviours
  util/ -- utility functions
```
  
## Basic Usage:
In very rough terms, the procedure used to operate the DCP using this library is:
- Run init to set up environment
- Generate waypoints for run from some data. A good example of this is img2toolpath, which converts a simple image to a series of waypoints.
- Generate trajectory from waypoints. For example, use smoothcartsegtraj.m.
- Command operation. The most thoroughly developed controller currently available is ffpdcontroller.m. It can be run with simflag = 1 and vizflag = 1 to simulate operation of the DCP.