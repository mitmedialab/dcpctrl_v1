function [ log, errors ] = followrawtraj_at40gw( handle, robot, rawtraj, dt, logging )
%FOLLOWTRAJ_AT40GW Uses the boom arm to follow a trajectory in joint space
% TODO: Update with changes from testmove

errors = [];

if logging
    log.Qrawcomm = [];
    log.Qrawdes = [];
    log.Qrawmeas = [];
    log.Qrawerr = [];
    log.PWMcomm = [];
    log.T = [];
    i = 1;
end

% Vectorize settings
Kps = [robot.Joint.Kp];
GainInvs = [robot.Joint.GainInv];
movethresh = [robot.Joint.moveThresh];

rawlimstructs = [robot.Joint.PosLim];
rawmaxlim = [rawlimstructs.Max];
rawminlim = [rawlimstructs.Min];
pwmlimstructs = [robot.Joint.PWMLim];
pwmmax = [pwmlimstructs.Max];
pwmdbmax = [pwmlimstructs.DBUpper];
pwmdbmin = [pwmlimstructs.DBLower];
pwmmin = [pwmlimstructs.Min];

% Total time to traverse trajectory
tt = size(rawtraj,1) * dt;

%TODO: try linearly interpolating the times for q_des
tic;
qrawprev = getrawpos_at40gw(handle, robot);

while true
    t = toc;
    
    % If settled, exit
    if all(getrawpos_at40gw(handle, robot) == robot.PWMZero) && t > tt
        break;
    end
    
    % Get desired joint position at this time
    if t > tt
        qrawdes = qfraw;
    else
        ti = ceil(t / dt);
        qrawdes = rawtraj(ti, :);
    end
    %qrawdes = qfraw;
    
    % Get current raw joint positions
    qrawcurr = getrawpos_at40gw(handle, robot);
    qrawcurr = lpfilter(qrawprev, qrawcurr);
    qrawprev = qrawcurr;
    
    % Compute the error
    qrawerr = qrawdes - qrawcurr;
    
    % Compute commanded raw joint values
    qrawcomm = Kps .* GainInvs .* qrawerr;
    qrawcomm = clamp(qrawcomm, rawminlim, rawmaxlim);
    
    % Convert to PWMs
    posi = qrawcomm > 0;
    nega = qrawcomm < 0;
    zer = abs(qrawerr) < movethresh;
    
    pwmcomm = zeros(size(qrawcomm));
    % TODO: is this transformation correct?
    % TODO: adjust the speed factor?
    pwmcomm(posi) = mapRange(qrawcomm(posi), 0, rawmaxlim(posi), pwmdbmax(posi), pwmmax(posi));
    pwmcomm(nega) = mapRange(qrawcomm(nega), 0, rawminlim(nega), pwmdbmin(nega), pwmmin(nega));
    pwmcomm(zer) = robot.PWMZero;
    pwmcomm = round(pwmcomm);
    
    % Send the command to the labjack
    setpwms_at40gw(handle, robot, pwmcomm);
    
    if logging
        log.Qrawdes = [log.Qrawdes qrawdes'];
        log.Qrawcomm = [log.Qrawcomm qrawcomm'];
        log.Qrawerr = [log.Qrawerr qrawerr'];
        log.Qrawmeas = [log.Qrawmeas qrawcurr'];
        log.PWMcomm = [log.PWMcomm pwmcomm'];
        log.T = [log.T t];

        i = i + 1;
    end
end

end

