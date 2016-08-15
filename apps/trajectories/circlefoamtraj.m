% Trajectory just for figuring out foam parameters

tacc = 1;
dt = 0.3;
spds = [50 100 150 200 250]; % mm/s

nsegs = length(spds);

dire = -1;
tdeg = 90;
sdeg = tdeg / (nsegs+1);

q0raw = getrawpos_at40gw(handle, robot);
% load('data/homerawpos');
% q0raw = homerawpos;
q0 = raw2joint_at40gw(robot, q0raw);

a0 = q0(1);
as = a0 + [0:dire*sdeg:a0+dire*tdeg]';

qrawtrajs = cell(nsegs,1);
qdrawtrajs = cell(nsegs,1);
pwmtrajs = cell(nsegs,1);
segts = cell(nsegs,1);

for i = 1:nsegs
    spd = spds(i);
    a0 = as(i);
    af = as(i+1);
    q0 = [a0 q0(2) q0(3) q0(4)];
    qf = [af q0(2) q0(3) q0(4)];
    x0 = joint2cart_at40gw(q0);
    
    qdir = sign(af - a0);
    xvel = cross([x0(1:2) 0], [0 0 qdir]);
    xvel = xvel ./ norm(xvel) .* spd;
    qddes = cartvel2jointvel_at40gw(q0, xvel, logical([1 0 1 1]));
    
    [ades, addes, ~, ts] = lspb3(a0,af,qddes(1),tacc,dt);
    nsteps = size(ades,1);
    
    qtraj = repmat(q0,nsteps,1);
    qtraj(:,1) = ades;
    qdtraj = zeros(nsteps,4);
    qdtraj(:,1) = addes;
    qrawtraj = joint2raw_at40gw(robot, qtraj);
    qdrawtraj = jointvel2rawvel_at40gw(robot, qtraj, qdtraj);
    
    qrawtrajs{i} = qrawtraj;
    qdrawtrajs{i} = qdrawtraj;
    pwmtrajs{i} = rawvel2pwm_at40gw(robot, qdrawtraj);
    segts{i} = ts;
end