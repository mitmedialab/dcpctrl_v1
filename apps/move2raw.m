function [ logs ] = move2raw( handle, robot, qfraw, tol, spd )
%MOVE2RAW Moves the boom arm to the desired raw joint positions
% WARNING: This does NOT do any collision checking at all!!!
% This function should ONLY be used if the arm is already near the desired
% position. The speed is only approximate!

if nargin < 4
    tol = 30; % mm
end

if nargin < 5
    spd = 150; % mm/s
end

q0raw = getrawpos_at40gw(handle, robot);

%load('data/homerawpos');
%q0raw = homerawpos;

q0 = raw2joint_at40gw(robot, q0raw);
x0 = joint2cart_at40gw(q0);
xf = joint2cart_at40gw(raw2joint_at40gw(robot, qfraw));

% Compute the path
d = norm(xf-x0);
tf = d/spd + 10;
dt = 0.05; % sec
nsteps = round(tf/dt);
ts = (1:nsteps).*dt;

[qrawtraj, qdrawtraj] = jtraj(q0raw, qfraw, ts);

xtraj = joint2cart_at40gw(raw2joint_at40gw(robot, qrawtraj));

% Display and get verification from user
xarm0 = joint2arm_at40gw(q0);

% figure;
% hold on;
% plot3(xtraj(:,1),xtraj(:,2),xtraj(:,3));
% plot3(xarm0(:,1),xarm0(:,2),xarm0(:,3),'g','LineWidth',5);
% view([1 1 0.5]);
% drawnow;
% hold off;

str = input('Do you want to continue (default is NO)? [y/N]: ', 's');

if ~strcmp(str, 'y')
    disp('Cancelling');
    return;
end

% Make the move

% Setup gains
usedjoints = logical([1 0 1 1]);
Kps = [robot.Joint.Kp];
Kds = [robot.Joint.Kd];
Kvs = [robot.Joint.Kv];
GainInvs = [robot.Joint.GainInv];
movethresh = [robot.Joint.moveThresh];

rawlimstructs = [robot.Joint.PosLim];
rawmaxlim = [rawlimstructs.Max];
rawminlim = [rawlimstructs.Min];
% WARNING: handle this corner case more elegantly somehow
[rawmaxlim(2), rawminlim(2)] = deal(rawminlim(2),rawmaxlim(2));

pwmlimstructs = [robot.Joint.PWMLim];
pwmmax = [pwmlimstructs.Max];
pwmdbmax = [pwmlimstructs.DBUpper];
pwmdbmin = [pwmlimstructs.DBLower];
pwmmin = [pwmlimstructs.Min];

% Follow trajectory
qrawdesprev = qrawtraj(1,:);
qdrawdesprev = qdrawtraj(1,:);
errprev = 0;
tprev = 0;

qrawprev = q0raw;

logs = {};
logi = 1;

% Follow the trajectory
tic;
while true
    t = toc;

    %qraw = qrawprev + qdrawcomm.*(t-tprev);
    qraw = getrawpos_at40gw(handle,robot);
    %qraw = lpfilter(qrawprev, qraw, [0.97 0.97 0.97 0.97]);
    
    % End condition
    if all(abs(qraw(usedjoints)-qfraw(usedjoints)) < movethresh(usedjoints))
        setpwms_at40gw(handle, robot, robot.PWMZero .* ones(1,4));
        disp('Finished.');
        break;
    end
    
    % End condition
    %{
    xc = joint2cart_at40gw((raw2joint_at40gw(robot, qraw)));
    if norm(xf-xc) < tol && t > tf
        setpwms_at40gw(handle, robot, robot.PWMZero .* ones(1,4));
        disp('Finished.');
        break;
    end
    %}
    
    % Time-consistent controls
    if (t-tprev) < dt
        continue;
    end

    % Setup for iteration
    ti = min(ceil(t / dt), size(qrawtraj,1));
    talpha = mod(t, dt) / dt;
    if ti ~= 1
        qrawdesprev = qrawtraj(ti-1, :);
        qdrawdesprev = qdrawtraj(ti-1, :);
    end
    qrawdes = (1-talpha) .* qrawdesprev + talpha .* qrawtraj(ti, :);
    qdrawdes = (1-talpha) .* qdrawdesprev + talpha .* qdrawtraj(ti, :);

    if t > tf
        qrawdes = qrawtraj(end, :);
        qdrawdes = qdrawtraj(end, :);
    end

    % Compute errors
    err = qrawdes - qraw;
    derr = err - errprev;
    dmeaserr = qraw - qrawprev;

    % Compute command
    Kffv = Kvs .* qdrawdes;
    Kpe = Kps .* err;
    Kde = Kds .* dmeaserr;
    %qrawcomm = clamp(qrawcomm, rawminlim, rawmaxlim);

    qdrawcomm = Kffv + Kpe + Kde;

    % Convert to PWM
    pwmcomm = rawvel2pwm_at40gw(robot, qdrawcomm);
    pwmcomm = round(pwmcomm);
    
    % Clamp to minimum values if we need a move
    need2mv = abs(qraw-qrawtraj(end,:)) > movethresh & usedjoints;
    dir2mv = sign(qdrawcomm).*[robot.Joint.GainInv];
    pwmlims = [robot.Joint.PWMLim];
    u = (dir2mv==1) & need2mv & pwmcomm < [pwmlims.DBUpper];
    l = (dir2mv==-1) & need2mv & pwmcomm > [pwmlims.DBLower]; 
    for j=1:4
        if u(j)
            pwmcomm(j) = robot.Joint(j).PWMLim.DBUpper;
        elseif l(j)
            pwmcomm(j) = robot.Joint(j).PWMLim.DBLower;
        end
    end

    % Send PWM command
    setpwms_at40gw(handle, robot, pwmcomm);

    % Setup for next iteration
    tprev = t;
    errprev = err;
    qrawprev = qraw;
    
    if nargout > 0
        logs(logi).t = t;
        logs(logi).i = logi;
        logs(logi).pwm = pwmcomm';
        logs(logi).qraw = qraw';
        logs(logi).qrawdes = qrawdes';
        logs(logi).qdrawdes = qdrawdes';
        logs(logi).Kffv = Kffv';
        logs(logi).Kpe = Kpe';
        logs(logi).Kde = Kde';
        logs(logi).qdrawcomm = qdrawcomm';
        logs(logi).err = err';
        logs(logi).derr = derr';
        logs(logi).dmeaserr = dmeaserr';
        logi = logi + 1;
    end
end

end

