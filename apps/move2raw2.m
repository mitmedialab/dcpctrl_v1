function [ logs ] = move2raw2( handle, robot, qfraw, usedjoints, max_pwm_factor )
%MOVE2RAW Moves the boom arm to the desired raw joint positions
% and uses a naive PD controller only, there is no feed forward component,
% works mostly like Julian's controller
%
% INPUTS:
%   max_spd_factor - [0,1], 0 means no movement, 1 means full possible,
%                     default is 0.1
%   speed
%
% WARNING: This does NOT do any collision checking at all!!!
% This function should ONLY be used if the arm is already near the desired
% position. The speed is only approximate!

if nargin < 5
    max_pwm_factor = 0.4;
end

if nargin < 4
    usedjoints = logical([1 0 1 1]);
end

dt = 0.05;

q0raw = getrawpos_at40gw(handle, robot);

%load('data/homerawpos');
%q0raw = homerawpos;

q0 = raw2joint_at40gw(robot, q0raw);
x0 = joint2cart_at40gw(q0);
xf = joint2cart_at40gw(raw2joint_at40gw(robot, qfraw));

% str = input('Do you want to continue (default is NO)? [y/N]: ', 's');
% 
% if ~strcmp(str, 'y')
%     disp('Cancelling');
%     return;
% end

% Make the move

% Setup gains
% usedjoints = logical([1 0 1 1]);
% Kps = [robot.Joint.Kp];
% Kds = [robot.Joint.Kd];
% Kvs = [robot.Joint.Kv];

%usedjoints = logical([1 1 1 1]);
Kps = [1 1 1 1];
Kds = [0 0 0 0];
Kvs = [0 0 0 0];

GainInvs = [robot.Joint.GainInv];
movethresh = [robot.Joint.moveThresh];
movethresh = [300 0.04 0.04 0.04];

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
%qrawdesprev = qrawtraj(1,:);
%qdrawdesprev = qdrawtraj(1,:);
errprev = 0;
tprev = 0;

qrawprev = q0raw;

logs = {};
logi = 1;

% Incorporate max pwm factor
%pwmmax = (pwmmax - robot.PWMZero).*max_pwm_factor + robot.PWMZero;

% Follow the trajectory
tic;
while true
    t = toc;

    %qraw = qrawprev + qdrawcomm.*(t-tprev);
    qraw = getrawpos_at40gw(handle,robot);
    %qraw = lpfilter(qrawprev, qraw, [0.97 0.97 0.97 0.97]);
    
    if all(abs(qfraw(usedjoints)-qraw(usedjoints)) < movethresh(usedjoints))
        setpwms_at40gw(handle, robot, robot.PWMZero .* ones(1,4));
        disp('Finished.');
        break;
    end

    % Time-consistent controls
    if (t-tprev) < dt
        continue;
    end

    % Compute errors
    err = qfraw - qraw;
    derr = err - errprev;
    dmeaserr = qraw - qrawprev;

    % Compute command
    Kpe = Kps .* err;
    Kde = Kds .* dmeaserr;

    qdrawcomm = Kpe + Kde;
    
    % Scale by max speed factor
    qdrawcomm = qdrawcomm;

    % Convert to PWM
    pwmcomm = rawvel2pwm_at40gw(robot, qdrawcomm);
    pwmcomm = round(pwmcomm);
    pwmcomm = clamp(pwmcomm, pwmmin, pwmmax);
    
    pwmcomm = (pwmcomm - robot.PWMZero).*max_pwm_factor + robot.PWMZero;
    
    % Clamp to minimum values if we need a move
    need2mv = abs(qraw-qfraw) > movethresh & usedjoints;
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
    
    pwmcomm(~need2mv) = robot.PWMZero;

    % Send PWM command
    pwmcomm(~usedjoints) = robot.PWMZero;
    setpwms_at40gw(handle, robot, pwmcomm);

    % Setup for next iteration
    tprev = t;
    errprev = err;
    qrawprev = qraw;
end

end

