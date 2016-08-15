% Relative move spiral

spd = 100; % mm/s, this isn't entirely correct, will be slightly faster
dt = 0.01;
tacc = 2;
h = 1000; % mm
r = 750;
nspirals = 2;
dire = 1; % around z-axis relative to truck

simflag = 0;
usedjoints = logical([1 0 1 1]);

if simflag
    load('data/homerawpos');
    q0raw = homerawpos;
else
    q0raw = getrawpos_at40gw(handle, robot);
end

q0 = raw2joint_at40gw(robot, q0raw);
x0 = joint2cart_at40gw(q0);

% Get magnitude for circular velocity
%thetaspd = -(dire*spd)/(r*sind(90));
thetaspd = spd/r;

% Compute the trajectories and cartesian velocities
[theta, thetad, ~, ts] = lspb3(0,360*nspirals,rad2deg(thetaspd),tacc,dt);

npoints = size(theta,1);
tt = npoints*dt;
%ts = (0:(npoints-1)).*dt;

xdtraj = -r.*sind(theta).*deg2rad(thetad).*dt;
ydtraj = r.*cosd(theta).*deg2rad(thetad).*dt;
[ztraj, zdtraj] = lspb(0,h,ts);

%zd = h / tt;
%zdtraj = ones(size(xdtraj)).*zd;

xtraj = r.*cosd(theta);
ytraj = r.*sind(theta);
%ztraj = (1:npoints)'.*zd;

xtraj = [xtraj ytraj ztraj];
xtraj = bsxfun(@plus, xtraj, x0-xtraj(1,:));
xdtraj = [xdtraj ydtraj zdtraj];

qtraj = ikine_at40gw(xtraj, q0, usedjoints);
qdtraj = cartvel2jointvel_at40gw(qtraj,xdtraj,usedjoints);

qrawtraj = joint2raw_at40gw(robot, qtraj);
qdrawtraj = jointvel2rawvel_at40gw(robot,qtraj,qdtraj);

nsegs = 1;
qrawtrajs{1} = qrawtraj;
qdrawtrajs{1} = qdrawtraj;
segts{1} = ts;

% Check for exceeded joint positions
exceeded = checkjointlimits_at40gw(robot, cell2mat(qrawtrajs));
if any(exceeded(:))
    disp(['WARNING Joints exceeded limits in trajectory, joints are: ' num2str(find(any(exceeded,1)))]);
end

