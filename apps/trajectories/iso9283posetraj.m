%% Define test parameters
dt = 0.1;
tacc = 1;
spd = 150; % mm/s (cartesian)

len = 2000; % Length of cube size, mm
numreps = 30; % Number of times to move through trajectory
cornerdelay = 7; % Delay (in sec) at each corner of a trajectory
enddelay = 15; % Delay (in sec) at end of each run.

simflag = 0; % Flag to simulate at default home position (1), or to run at current position (0)
robot = config_at40gw;

%% Generate cube
% Robot endpoint is assumed to start at center of cube - we will need to
% shift coordinates to here.
C0 = [0,0,0]; % Center of cube
C1 = [len/2,len/2,len/2];
C2 = [len/2,-len/2,len/2];
C3 = [-len/2,-len/2,len/2]; 
C4 = [-len/2,len/2,len/2]; 
C5 = [len/2,len/2,-len/2]; 
C6 = [len/2,-len/2,-len/2];
C7 = [-len/2,-len/2,-len/2];
C8 = [-len/2,len/2,-len/2];
c = [C0;C1;C2;C3;C4;C5;C6;C7;C8];
diaglen = norm([len len len]);

%% Select which corners of cube should be used & generate plane
coff = 0.1*diaglen; % Calculate offset from corners to create P points
cl = sqrt((coff^2)/3);
plntype = 'a'; % Available choices: a,b,c,d,xz,yz,xy. We will implement other plane types later.
switch plntype
    case 'a'
        xtraj_0 = [C0; ...
            C1+[-cl,-cl,-cl];...
            C2+[-cl,cl,-cl];...
            C7+[cl,cl,cl];...
            C8+[cl,-cl,cl];...
            C0];  
end

%% Get home position of robot & shift points to there
if simflag
    load('data/homerawpos');
    q0raw = homerawpos;
else
    q0raw = getrawpos_at40gw(handle, robot);
end

q0 = raw2joint_at40gw(robot, q0raw);
x0 = joint2cart_at40gw(q0); 

xtraj_t = bsxfun(@plus, xtraj_0, x0);
xtraj_t = [xtraj_t,ones(size(xtraj_t,1),1)];

%% Replicate as many times as specified
xtraj = xtraj_t'; % Transpose for speed
for n = 2:numreps
    temp = [bsxfun(@times,xtraj_t,[1,1,1,n])]';
    xtraj = [xtraj,temp];
end
waypts = xtraj';

% %% Add extra delay segments.
% for n = 1:length(waypts)-1
%     if waypts(n,4) ~= waypts(n+1,4)
        

%% Move from Cartesian to joint space.
qwaypts = ikine_at40gw(waypts(:,1:3), q0, [1 0 1 1]);
qwaypts = [qwaypts,waypts(:,4)];
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
    %dispstat(sprintf('Generating traj for segment: %d/%d',i,nsegs));
    fprintf('Generating traj for segment: %d/%d\n',i,nsegs)
    if qwaypts(i,5) == qwaypts(i+1,5)
        % We are moving
        q0 = qwaypts(i,1:4);
        qf = qwaypts(i+1,1:4);
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
        
        % Add additional points for dwell at end of move
        tscount = cornerdelay/dt; % Number of timesteps of zero motion that we need to inject
        qrawtraj = [qrawtraj;bsxfun(@times,ones(tscount,4),qrawtraj(end,:))];
        qdrawtraj = [qdrawtraj;zeros(tscount,4)];
        pwmtraj = [pwmtraj;ones(tscount,4).*80000];
        ts = [ts;[ts(end):dt:(ts(end)+dt*tscount)]'];
        
        % Check that velocities don't exceed max joint velocities at any
        % point in trajectory - break if so.
        exceeded = checkjointvellimits_at40gw(robot, qdrawtraj);
        if any(exceeded(:))
            fprintf(['WARNING Joints exceeded velocity limits in trajectory, joints are: ' num2str(find(any(exceeded,1))),'\n']);
            fprintf(['Trajectory segment: ', num2str(i),'\n']);
        end

        % Save all the trajectories
        mtypes{i} = mtype;
        qrawtrajs{i} = qrawtraj;
        qdrawtrajs{i} = qdrawtraj;
        pwmtrajs{i} = pwmtraj;
        segts{i} = ts;
        absts{i} = ts + ttf;
        ttf = ttf + ts(end);
        
    else
        % We are at the end of a diamond. Generate static dwell points to
        % stop.
        mtype = 1; % Setting mtype to 1 for now - don't think it matters for now
        tscount = enddelay/dt; % Number of timesteps of zero motion that we need to inject
        qrawtraj = bsxfun(@times,ones(tscount,4),qrawtrajs{i-1,1}(end,:));
        qdrawtraj = zeros(tscount,4);
        pwmtraj = ones(tscount,4).*80000;
        ts = [0:dt:enddelay]';
        mtypes{i} = mtype;
        qrawtrajs{i} = qrawtraj;
        qdrawtrajs{i} = qdrawtraj;
        pwmtrajs{i} = pwmtraj;
        segts{i} = ts;
        absts{i} = ts + ttf;
        ttf = ttf + ts(end);
    end
end