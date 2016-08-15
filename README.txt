DCPControl_V1
-----------------

OVERVIEW:
The DCPControl_V1 repository is a body of code used to control the second iteration of the Digital Construction Platform, developed by the Mediated Matter Group at the MIT Media Lab. This particular repository captures the state of the project as of August 2016.

The DCPControl_V1 repository provides the following functionality:
 - Low-level communications with the Altec AT40GW electrohydraulic lift vehicle, via a LabJack T7 interface.
 - Low-level communications with the KUKA KR10 R1100 sixx WP robot arm, via a variety of interfaces.
 - File import, processing and trajectory generation for the DCP system (comprised of the AT40GW and KUKA arms).
 - Real-time control for the DCP system.
 - Run data processing & visualization for the DCP system.

The DCPControl_V1 repository enables the DCP system to be interacted with like a conventional robotic arm, albeit in a limited fashion. The code contained within the repository does not represent any substantial improvement over the state of the art, except in that it enables the DCP system to operate.

STYLE GUIDE: 
https://sites.google.com/site/matlabstyleguidelines/
(Vaguely following this style guide)

STRUCTURE:
mdcp/
  *data/
    logs/
    cad/
    gcode/

  apps/ -- very high-level behaviours to run truck/arm/both
    kuka/
    at40gw/ -- examples:
      record.m
      physplayback.m
      virtplayback.m
      lightpainting.m
      foamprint.m
      home.m

  tools/ -- External tools (e.g. homeprint), perhaps move this to its own repo?
    homeprint/

  lib/ -- External dependencies (e.g. rvctools)
    rvctools/

  robots/ -- robot-specific behaviours and utility functions
    kuka/
      controllers/
      config/ -- settings and definitions
      lib/
    at40gw/
      controllers/
      config/ -- settings and definitions
      lib/
        getPos
	volt2linpos
	linpos2volt
	encoder2deg
	encoder2rad
	deg2encoder
	rad2encoder
