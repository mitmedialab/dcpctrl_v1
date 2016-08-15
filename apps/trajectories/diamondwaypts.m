% Generates waypoints for a diamond
% This is a RELATIVE MOTION (starts at the current end effector position)

simflag = 0;

robot = config_at40gw;

%% Generate diamond
if simflag
    load('data/homerawpos');
    q0raw = homerawpos;
else
    q0raw = getrawpos_at40gw(handle, robot);
end

q0 = raw2joint_at40gw(robot,q0raw);
x0 = joint2cart_at40gw(q0);

% Define diamond
sidemm = 750;
waypts = [0 0 0;
          sidemm 0 0;
          sidemm sidemm 0;
          0 sidemm 0;
          0 0 0];

% Rotate diamond
thetaz = -45;
Rz = [cosd(thetaz) -sind(thetaz) 0;
      sind(thetaz) cosd(thetaz) 0;
      0 0 1];
thetay = -90;
Ry = [cosd(thetay) 0 sind(thetay);
      0 1 0;
      -sind(thetay) 0 cosd(thetay)];
thetaz2 = q0(1);
Rz2 = [cosd(thetaz2) -sind(thetaz2) 0;
       sind(thetaz2) cosd(thetaz2) 0;
       0 0 1;];
  
% Shift diamond to end effector
waypts = bsxfun(@plus,Rz2 * Ry * Rz * waypts', x0')';