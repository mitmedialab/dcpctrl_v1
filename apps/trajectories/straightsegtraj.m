%% Settings
dt = 0.05;
tacc = 0.01;
spd = 100; % mm/s (cartesian)

simflag = 0;
rel = 1; % relative vs. absolute move

robot = config_at40gw;

if simflag
    load('data/homerawpos');
    q0raw = homerawpos;
else
    q0raw = getrawpos_at40gw(handle, robot);
end
q0 = raw2joint_at40gw(robot, q0raw);

%% Make it a relative move
if rel
    x0 = joint2cart_at40gw(q0);
    offset = x0 - waypts(1,:);
    waypts = bsxfun(@plus, waypts, offset);
end

%% Trajectories
% Setup trajectories for each segment (blend later)
% Can pre-blend here
% Nx4xM, N

qwaypts = ikine_at40gw(waypts, q0, [1 0 1 1]);
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
    lightmode{i} = 1
end

% Check for exceeded joint positions
exceeded = checkjointlimits_at40gw(robot, cell2mat(qrawtrajs));
if any(exceeded(:))
    disp(['WARNING Joints exceeded limits in trajectory, joints are: ' num2str(find(any(exceeded,1)))]);
end
