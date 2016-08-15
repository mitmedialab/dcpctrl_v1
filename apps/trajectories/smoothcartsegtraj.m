%% Smooth cartesian trajectories

%% Settings
dt = 0.05;
tacc = 0.01;
spd = 150; % mm/s (cartesian)
simflag = 1;
rel = 1; % relative vs. absolute move

robot = config_at40gw;

if simflag
    load('data/homerawpos');
    q0raw = homerawpos;
else
    q0raw = getrawpos_at40gw(handle, robot);
end
q0 = raw2joint_at40gw(robot, q0raw);

%% Set up waypts if cell vs. matrix
if iscell(waypts)
    nsegs = length(waypts);
else
    nsegs = 1;
    temp_waypts = waypts;
    waypts = {};
    waypts{1} = temp_waypts;
end

%% Make it a relative move
if rel
    x0 = joint2cart_at40gw(q0);
    offset = x0 - waypts{1}(1,1:3);
    for n = 1:nsegs
        waypts{n} = bsxfun(@plus, waypts{n}, [offset,0,0]);
    end
end

%% Trajectories
% Setup trajectories for each segment (blend later)
% Can pre-blend here
% Nx4xM, N

qrawtrajs = {};
qdrawtrajs = {};
segts = {};
lightmode = {};
usedjoints = logical([1 0 1 1]);
i = 1;

for n = 1:nsegs
    lightmode{i} = 1;
    disp(['Pathing segment ', num2str(n),'...']);
    
    % Compute time estimates along segments
    [xtraj, xdtraj] = mstraj2(waypts{n}(2:end,1:3),spd,[],waypts{n}(1,1:3),dt,tacc);
    ts = (0:(size(xtraj,1)-1)) .* dt;
    
    qtraj = ikine_at40gw(xtraj, q0, usedjoints);
    qdtraj = cartvel2jointvel_at40gw(qtraj, xdtraj, usedjoints);
    qrawtraj = joint2raw_at40gw(robot, qtraj);
    qdrawtraj = jointvel2rawvel_at40gw(robot, qtraj, qdtraj);

    qrawtrajs = [qrawtrajs;qrawtraj];
    qdrawtrajs = [qdrawtrajs;qdrawtraj];
    segts = [segts;ts];
    q0 = qtraj(end,:);
    i = i + 1;
    if n < nsegs && any(waypts{n}(end,1:3) ~= waypts{n+1}(1,1:3))
        disp(['Pathing transition segment ', num2str(n),'...']);
        lightmode{i} = 0;
        
        % Get the parameterized trajectory
        xf = waypts{n+1}(1,1:3);
        x0 = waypts{n}(end,1:3);
        xdir = (xf-x0)./norm(xf-x0);
        [xt, xdt, xddt, ts] = lspb3(0,norm(xf-x0),spd,tacc,dt);

        % Transform into useable trajectories
        xtraj = bsxfun(@times, xt, xdir);
        xtraj = bsxfun(@plus, xtraj, x0);
        xdtraj = bsxfun(@times, xdt, xdir);
        
        %[xtraj, xdtraj] = mstraj2(waypts{n+1}(1,1:3),spd,[],waypts{n}(end,1:3),dt,tacc);
        %ts = (0:(size(xtraj,1)-1)) .* dt;
        
        qtraj = ikine_at40gw(xtraj, q0, usedjoints);
        q0 = qtraj(end,:);
        qdtraj = cartvel2jointvel_at40gw(qtraj, xdtraj, usedjoints);
        qrawtraj = joint2raw_at40gw(robot, qtraj);
        qdrawtraj = jointvel2rawvel_at40gw(robot, qtraj, qdtraj);

        qrawtrajs = [qrawtrajs;qrawtraj];
        qdrawtrajs = [qdrawtrajs;qdrawtraj];
        segts = [segts;ts];
        i = i + 1;
    end
end

% Check for exceeded joint positions
exceeded = checkjointlimits_at40gw(robot, cell2mat(qrawtrajs));
if any(exceeded(:))
    disp(['WARNING Joints exceeded limits in trajectory, joints are: ' num2str(find(any(exceeded,1)))]);
end

% Reset nsegs to be correct length
nsegs = length(qrawtrajs);

%% 
figure;
hold on;
joint2check = 4;
je = joint2check;
qm = cell2mat(qrawtrajs);
xm = joint2cart_at40gw(raw2joint_at40gw(robot,qm));
plot3(xm(~exceeded(:,je),1),xm(~exceeded(:,je),2),xm(~exceeded(:,je),3),'b');
plot3(xm(exceeded(:,je),1),xm(exceeded(:,je),2),xm(exceeded(:,je),3),'r');
xlabel('x');
