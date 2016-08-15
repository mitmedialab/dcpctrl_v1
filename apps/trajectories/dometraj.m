%% Settings
robot = config_at40gw;

dt = 0.05;
curved_speed = 125; % mm/s, cartesian
curved_tacc = 1;
linear_speed = 125; % mm/s, cartesian
linear_tacc = 1;
vertical_speed = 30; % mm/s, cartesian
vertical_tacc = 0.5;
blend_t = 0;

%% Waypoints
% Any waypoint setup necessary
%load('data/dome_45_jwaypts.mat'); % loads jwaypts
load('data/jwaypts','jwaypts');
%load('data/domesample.mat');
qwaypts = jwaypts; % TODO: rename
waypts = joint2cart_at40gw(jwaypts(:,1:4));
nwaypts = size(waypts,1);

%% Trajectories
% Setup trajectories for each segment (blend later)
% Can pre-blend here
% Nx4xM, N 
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
    mtype = qf(5);
    q0 = q0(1:4);
    qf = qf(1:4);
    x0 = joint2cart_at40gw(q0);
    xf = joint2cart_at40gw(qf);
    xdir = (xf-x0)./norm(xf-x0);
    
    % 1 - cart, 0 - joint
    if mtype == 1
        % Vertical segments are much shorter
        if norm(xf-x0) < 200
            spd = vertical_speed;
            tacc = vertical_tacc;
        else
            spd = linear_speed;
            tacc = linear_tacc;
        end
        
        % Get the parameterized trajectory
        [xt, xdt, xddt, ts] = lspb3(0,norm(xf-x0),spd,tacc,dt);
        
        % Transform into useable trajectories
        xtraj = bsxfun(@times, xt, xdir);
        xtraj = bsxfun(@plus, xtraj, x0);
        xdtraj = bsxfun(@times, xdt, xdir);
        
        qtraj = ikine_at40gw(xtraj, q0, usedjoints);
        qtraj(:,1) = q0(1); % Make sure j1 does not move
        qrawtraj = joint2raw_at40gw(robot, qtraj);
        qdtraj = cartvel2jointvel_at40gw(qtraj, xdtraj, usedjoints);
        qdtraj(:,1) = 0; % Make sure j1 does not move
        qdrawtraj = jointvel2rawvel_at40gw(robot, qtraj, qdtraj);
        pwmtraj = rawvel2pwm_at40gw(robot, qdrawtraj);
    else
        % Transform end-effector velocity to joint 1 velocity
        qdir = sign(qf(1)-q0(1));
        xddir = cross([x0(1:2) 0], [0 0 qdir]);
        xddir = xddir ./ norm(xddir) .* curved_speed;
        qddes = cartvel2jointvel_at40gw(q0, xddir, usedjoints);
        
        [q1traj, qd1traj, qdd1traj, ts] = lspb3(q0(1), qf(1), qddes(1), curved_tacc, dt);
        
        % Keep all other joints fixed
        qtraj = bsxfun(@plus, zeros(size(q1traj,1),4), q0);
        qtraj(:,1) = q1traj;
        qrawtraj = joint2raw_at40gw(robot, qtraj);
        qdtraj = zeros(size(q1traj,1),4);
        qdtraj(:,1) = qd1traj;
        qdrawtraj = jointvel2rawvel_at40gw(robot, qtraj, qdtraj);
        pwmtraj = rawvel2pwm_at40gw(robot, qdrawtraj);
    end
    
    % Save all the trajectories
    mtypes{i} = mtype;
    qrawtrajs{i} = qrawtraj;
    qdrawtrajs{i} = qdrawtraj;
    pwmtrajs{i} = pwmtraj;
    segts{i} = ts;
    absts{i} = ts + ttf;
    ttf = ttf + ts(end);
end
