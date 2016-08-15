%{
mdl_AT40GW.m
Julian Leland
2016-05-05

This function defines a model of the Altec AT40GW for use with Peter
Corke's Robotics, Vision and Control (RVC) Toolbox. It uses the standard DH
parameter convention, as described in Corke's "Robotics, Vision & Control"
textbook. It assumes that the AT40GW's J2 is positioned at the "minimum
diameter" position that SK measured on 4/14; in this position, J2 is at
-2.886 V, or 40.781 deg.

INPUTS: None

OUTPUTS: None

DEPENDENCIES:
- rvctools

TO-DO:

CHANGES:

%}

J1 = Link([0,2624.4,-1303.6,pi/2,0,0]);
J2 = Link([-pi/2,329.4,0,-pi/2,0,-pi/2]); % Pi/2 offset to orient correctly
J3 = Link([0,0,0,0,1,4633.1]); % Prismatic joint. Offset of length of inner link

AT40GW = SerialLink([J1,J2,J3],'name','AT40GW');

% Need to set plot options to limit workspace, turn off tiles to speed up.
% Setting floorlevel = 0 to avoid bug associated with workspace (not sure
% why this occurs)
AT40GW.plotopt = {'workspace',[-5000 5000 -5000 5000 0 8000],'floorlevel',0,'notiles'};

%AT40GW.