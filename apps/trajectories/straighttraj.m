%% Generate straight line trajectory
% Must specify q0raw and qfraw in the workspace
% Outputs qrawtraj into the workspace

dt = 0.3;
speed = 120;

usedjoints = logical([1 0 1 1]);

q0 = raw2joint_at40gw(robot, q0raw);
qf = raw2joint_at40gw(robot, qfraw);
x0 = joint2cart_at40gw(q0);
xf = joint2cart_at40gw(qf);

d = norm(xf-x0);
tt = d / speed;
nsteps = round(tt / dt);

% Hijack jtraj for generating cartesian paths instead
[xtraj, xdtraj] = jtraj(x0, xf, nsteps);
qs = ikine_at40gw(xtraj, q0, usedjoints);
qds = cartvel2jointvel_at40gw(qs, xdtraj, usedjoints);
qrawtraj = joint2raw_at40gw(robot, qs);
