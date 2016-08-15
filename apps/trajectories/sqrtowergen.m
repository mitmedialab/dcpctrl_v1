%function [ ] = sqrtowergen()

%SQRTOWERGEN Generate square tower, with optional rotation.

%{
    [ xwaypts ] = sqrtowergen()
    Julian Leland, MIT Media Lab, 2016-06-27
    This function generates a square tower of arbitrary edge length and
    height. It also allows the user to specify a rotation, as deg/height -
    the tower will rotate that many times over its height.

%}

robot = config_at40gw;

%% Define parameters of square and other run parameters

side = 3; % Side length, ft
side = side * 304.8; % Convert to mm

ht = 9;
ht = ht * 304.8; % Convert to mm

layht = 45; % Layer height, mm
numlay = floor(ht/layht); % Calculate number of layers. Naive - just rejects partial layers

rotflag = 1; % Flag to indicate whether to rotate                                                                                                                                                                                                                                                                                                                                                                                                                     
rot = 90; % Rotation of dome in degrees
layrot = rot/numlay; % Rotation of dome, per layer

skiplayflag = 1; % Flag to skip layers
skiplay = 10; % Only print every skiplay'th layer

simflag = 0; % Flag to simulate (use defined home position) or use actual robot position

dt = 0.3; % Timestep.
tacc = 3; % Acceleration time
spd = 100; % Linear speed, mm/s (cartesian)

%% Create series of points that define square
% Assume that 0,0,0 is at center of square
p0 = [0,0,0];
p1 = p0 + [side*sqrt(2)/4  side*sqrt(2)/4 0];
p2 = p0 + [side*sqrt(2)/4  -side*sqrt(2)/4 0];
p3 = p0 + [-side*sqrt(2)/4  -side*sqrt(2)/4 0];
p4 = p0 + [-side*sqrt(2)/4  side*sqrt(2)/4 0];
p5 = p0 + [side*sqrt(2)/4  side*sqrt(2)/4 0];

xwaypts = [p0;p1;p2;p3;p4;p5];

% plot3(xwaypts(:,1),xwaypts(:,2),xwaypts(:,3));

%% Generate further points in trajectory
laywaypts = xwaypts(2:6,:); % Grab just the last 4 waypoints along the trajectory
for n = 1:numlay
    laywaypts = bsxfun(@plus,laywaypts,[0, 0, layht]); % Add layer height
    
    if rotflag
        Rz = [  cosd(layrot) -sind(layrot) 0;
                sind(layrot) cosd(layrot) 0;
                0 0 1];
        laywaypts = [Rz*laywaypts']';
    end
    if ~skiplayflag
        xwaypts = [xwaypts;laywaypts];
    else
        if ~mod(n,skiplay)
            xwaypts = [xwaypts;laywaypts];
        end
    end
end
clf;
plot3(xwaypts(:,1),xwaypts(:,2),xwaypts(:,3));

%% Shift waypoints to end of robot
if simflag
    load('data/homerawpos');
    q0raw = homerawpos;
else
    q0raw = getrawpos_at40gw(handle, robot);
end

q0 = raw2joint_at40gw(robot,q0raw);
x0 = joint2cart_at40gw(q0);
waypts = bsxfun(@plus,xwaypts, x0);

qwaypts = ikine_at40gw(waypts, q0, [1 0 1 1]);
rawwaypts = joint2raw_at40gw(robot, qwaypts);

%% Generate trajectories
% Setup trajectories for each segment (blend later)
% Can pre-blend here
% Nx4xM, N 

nwaypts = size(qwaypts,1);
nsegs = nwaypts - 1;
qrawtrajs = cell(nsegs,1);
qdrawtrajs = cell(nsegs,1);
pwmtrajs = cell(nsegs,1);
segts = cell(nsegs,1); % relative timestamps
absts = cell(nsegs,1); % absolute timetamps
mtypes = cell(nsegs,1);
ttf = 0;
usedjoints = logical([1 0 1 1]);

dispstat('','init');
for i = 1:nsegs
    % Get endpoint information
    dispstat(sprintf('Generating traj for segment: %d/%d',i,nsegs));
    q0 = qwaypts(i,:);
    qf = qwaypts(i+1,:);
    mtype = 1;
    x0 = joint2cart_at40gw(q0);
    xf = joint2cart_at40gw(qf);
    xdir = (xf-x0)./norm(xf-x0);

    % Get the parameterized trajectory
    [xt, xdt, xddt, ts] = lspb3(0,norm(xf-x0),spd,tacc,dt);

    % Transform into useable trajectories
    xtraj = bsxfun(@times, xt, xdir);
    xtraj = bsxfun(@plus, xtraj, x0);
    xdtraj = bsxfun(@times, xdt, xdir);

    qtraj = ikine_at40gw(xtraj, q0, usedjoints);
    qrawtraj = joint2raw_at40gw(robot, qtraj);
    qdtraj = cartvel2jointvel_at40gw(qtraj, xdtraj, usedjoints);
    qdrawtraj = jointvel2rawvel_at40gw(robot, qtraj, qdtraj);
    pwmtraj = rawvel2pwm_at40gw(robot, qdrawtraj);
    
    % Save all the trajectories
    mtypes{i} = mtype;
    qrawtrajs{i} = qrawtraj;
    qdrawtrajs{i} = qdrawtraj;
    pwmtrajs{i} = pwmtraj;
    segts{i} = ts;
    absts{i} = ts + ttf;
    ttf = ttf + ts(end);
end


%end


