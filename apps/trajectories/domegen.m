%DOMEGEN Generate dome of desired size, described as waypoints in joint
%space
%   Detailed explanation goes here
%clear;
clc
init;

%% Define dome and other run parameters
dia = 45; % Set desired dome diameter (in feet)
dia = dia * 304.8; % Convert to mm
rad = dia/2; % Calculate radius

spc = 1.5; % Set spacing between inner and outer trace (in feet)
spc = spc * 304.8; % Convert to mm

open = 1; % 1 = open dome, with aperture angle the; 0 = fully closed dome. NOTE: only works as open dome right now.
close_lay = 50; % Layer at which to close dome. Set to high number to keep fully open

ap_ang = 330; % Aperture angle in degrees. Centered around 180 deg rotation (right behind truck)
cl_ang = 330; % Aperture angle for closed dome.

layht = 45; % Layer height in mm. 45 mm is based off of results of previous print test with curved wall segment.
maxht = 20; % Total dome Z height in feet
maxht = maxht * 304.8; % Total dome Z height in mm. 6096 mm is based of estimate that we want a 20 ft dome.
n_lay = ceil(maxht/layht); % Number of layers to print

skiplayflag = 0; % Flag to skip layers
skiplay = 25; % Only print every skiplay'th layer
skiplayvect = []; % Vector to define which layers to skip & which ones to run
keeplay = [1:10]; % Vector to define which layers to keep (keeplay overrides skiplay currently)

K_off = [1130,0,-45]; % Offset between KUKA and AT40GW - do this until we have a better system

animate = 0; % Flag to decide whether to animate path writing (1) or not (0)

% Move settings (for use with old controller)
curJoints = [0,-3.7655,-3.48,-5];
moveSettings = [.9,.1,100,1];
inputParams = [5,200,10,.1];
txtName = 'out.txt';

% TODO: Get current robot position and use this to define the start
% position

%% Create a series of points that define starting positions of dome layers
%{
Starting Cartesian position for 50' dome (from Solidworks - rough estimate):
Delta X: 6238.16387006mm  [245.5970in ] 
Delta Y: 461.50512665mm  [18.1695in ] 
Delta Z: 305.65725000mm  [12.0338in ] 

spos_raw = [0,0.394,-9.338,-9.954]; % Minimum J2 position that will avoid hitting base (with computer removed).
%}

% Calculate the starting Cartesian position of the robot for a circle of
% diameter dia.
spos_cX = sqrt((rad-(K_off(1)))^2-329^2); % Starting position in X direction
spos_cY = -329; % Starting position in Y direction
spos_cZ = -305; % Starting position in Z direction. Adjust manually if dome needs to be moved up & down.
spos_c = [spos_cX, spos_cY, spos_cZ];
q0 = [0 0 0 0];
used = [1 0 1 1];
jpos_c = ikine_at40gw(spos_c,q0,used); % This gives the starting joint position for transcribing our circle

pos_c = [];
pos_c(1,:) = spos_c;
the = zeros(n_lay,1);
the(1) = 0;
pos_out = [];

for n = 2:n_lay;
    zpos = pos_c(1,3)+(n-1)*layht;
    theta = asin(zpos/rad);
    pos_outer = [spos_c(1)*cos(theta),spos_c(2),spos_c(1)*sin(theta)];
    pos_inner = [spos_c(1)*cos(theta)-spc,spos_c(2),spos_c(1)*sin(theta)];
    the(n) = theta;
    % pos_c(n,:) = pos;
    pos_new = [pos_outer;pos_inner];
    pos_c = [pos_c; pos_new];
    pos_nout = [pos_outer,pos_inner];
    pos_out = [pos_out;pos_nout]; % This is the variable we'll actually operate on.
end

scatter3(pos_c(:,1),pos_c(:,2),pos_c(:,3)); % Plot the points to see what they look like
axis([-10000 10000 -10000 10000 -1500 10000]); % Set bounds of plot to be larger than work volume of truck
daspect([1 1 1]); % Set aspect ratio of plot to be 1:1:1, so that we don't get any weird scaling effects

%% For each layer of dome, generate a horseshoe path
% Calculate joint positions at each waypoint
cpos_in = pos_out(:,4:6);
cpos_out = pos_out(:,1:3);
jpos_in = ikine_at40gw(cpos_in,q0,used);
jpos_out = ikine_at40gw(cpos_out,q0,used);

% Starting from the zero position of the axis, calculate J1 positions at +/-(180 - ap/2) degrees.
jpos_cwrot = jpos_c + [(180 - ap_ang/2) 0 0 0];
cwrot = jpos_cwrot(1);
jpos_ccwrot = jpos_c - [(180 - ap_ang/2) 0 0 0];
ccwrot = jpos_ccwrot(1);

% Starting from the zero position of the axis, calculate J1 positions at +/-(180 - cl_ang/2) degrees.
jpos_cwrot = jpos_c + [(180 - cl_ang/2) 0 0 0];
cwrot_cl = jpos_cwrot(1);
jpos_ccwrot = jpos_c - [(180 - cl_ang/2) 0 0 0];
ccwrot_cl = jpos_ccwrot(1);

jwaypts = []; % Empty vector for joint-space waypoints

%% Generate vector of layers we're going to cut out with skiplay
% Note: If you want to generate certain layers asides from those that
% haven't been skipped, just switch zeroes to ones here.

skiplayvect = [];

for n = 2:n_lay
    if ~mod(n,skiplay)
        skiplayvect = [skiplayvect,1];
    else
        skiplayvect = [skiplayvect,0];
    end
end

if ~skiplayflag
    skiplayvect(:,:) = 1;
end

if ~isempty(keeplay)
    skiplayvect(:) = 0;
    skiplayvect(keeplay) = 1;
end

%%
for n = 1:n_lay-1
    % Add change in J1 positions to inner and outer jpos values to get the
    % endpoints of the arc.
    if skiplayvect(n)
        if n < close_lay
            pos1 = jpos_out(n,:) + [cwrot 0 0 0]; % Outer ring, CW limit
            pos2 = jpos_in(n,:) + [cwrot 0 0 0]; % Inner ring, CW limit
            pos3 = jpos_in(n,:) + [ccwrot 0 0 0];% Inner ring, CCW limit
            pos4 = jpos_out(n,:) + [ccwrot 0 0 0];% Outer ring, CCW limit
        else
            pos1 = jpos_out(n,:) + [cwrot_cl 0 0 0]; % Outer ring, CW limit
            pos2 = jpos_in(n,:) + [cwrot_cl 0 0 0]; % Inner ring, CW limit
            pos3 = jpos_in(n,:) + [ccwrot_cl 0 0 0];% Inner ring, CCW limit
            pos4 = jpos_out(n,:) + [ccwrot_cl 0 0 0];% Outer ring, CCW limit
        end
        % Structure of jwaypts: pos is joint-space position to move to.
        % Following number indicates if it should be moved to as a cartesian
        % move (1) or a joint-space move (0)
        layer_waypts = [ pos1, 1
                    pos2, 1
                    pos3, 0
                    pos4, 1
                    pos1, 0 ];
        jwaypts = [jwaypts;layer_waypts];
    end
end
    % Add starting position back into beginning of trajectory, and adjust first
    % move type command
    initial = jwaypts(1,1:4);
    initial(1) = jpos_c(1);
    jwaypts = [initial,0;jwaypts];
    jwaypts(2,5) = 0; % Make the first move from the starting position to the end a joint move.

% for n = 1:n_lay
%     cpos_in = pos_new(n,4:6);
%     cpos_out = pos_new(n,1:3);
% end

%% Calculate Cartesian positions of trajectory and plot to verify.
cwaypts = joint2cart_at40gw(jwaypts(:,1:4));
hold on
scatter3(cwaypts(:,1),cwaypts(:,2),cwaypts(:,3));

%% Generate series of joint trajectories with mstraj + tseg
% Generate time segments
tseg = [];
for n = 1:length(jwaypts)
    if jwaypts(n,5) == 0
        t = 60;
    else
        t = 5;
    end
    tseg = [tseg,t];
end
tseg = tseg';

q = mstraj(jwaypts(:,1:4),[],tseg,jpos_c,1,1);
ctraj = joint2cart_at40gw(q);
if animate 
    for n = 1:size(ctraj)
        scatter3(ctraj(n,1),ctraj(n,2),ctraj(n,3),5,[1,0,0],'.');
        drawnow;
    end
else
    scatter3(ctraj(:,1),ctraj(:,2),ctraj(:,3),5,[1,0,0],'.');
end

%% Save for use with dometraj
save('data/jwaypts','jwaypts');

%% Generate joint waypoint list with velocities, other analog & digital signals for use with old controller
fileID = fopen(txtName,'w');
fprintf(fileID,'AT40GW Move File: %s\n',txtName);
[numPoints,~] = size(jwaypts);

maxVstr = strcat('[',num2str(moveSettings(1)),',',num2str(moveSettings(1)),',',num2str(moveSettings(1)),',',num2str(moveSettings(1)),']');
minVstr = strcat('[',num2str(moveSettings(2)),',',num2str(moveSettings(2)),',',num2str(moveSettings(2)),',',num2str(moveSettings(2)),']');
accstr = strcat('[',num2str(moveSettings(3)),',',num2str(moveSettings(3)),',',num2str(moveSettings(3)),',',num2str(moveSettings(3)),']');
stopstr = strcat('[',num2str(moveSettings(4)),',',num2str(moveSettings(4)),',',num2str(moveSettings(4)),',',num2str(moveSettings(4)),']');

for n = 1:numPoints
    fprintf(fileID,strcat('!m,[',num2str(jwaypts(n,1)),',',num2str(jwaypts(n,2)),',',num2str(jwaypts(n,3)),',',num2str(jwaypts(n,4)),'],',maxVstr,',',minVstr,',',accstr,',',stopstr,',[0,0,0,0,0,0,0,0,0,0],[0,0]','\n'));
end

fclose(fileID); % Close the file after writing
