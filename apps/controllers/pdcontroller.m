% PD Controller Script (Joint-space PD controller)
% This script is primarily for testing purposes, will be turned into a
% function once thoroughly tested.
% Reference: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
%
% INPUTS:
%   qrawtraj - joint space trajectory in raw format (counts/voltages)
% OUTPUTS:
%   logi - struct containing log data

%% Settings
useinterp = 1; % Interpolate desired positions 
usestep = 0; % Step function (tuning only)
usepd = 1; % 1 if PD, 0 if only P control
usedmeas = 1; % Use derivative of measurement
waittosettle = 0; % Only stop at the end if the arm settles
tol = 5; % in mm, tolerance of goal, only used if waittosettle is off

%% Initialize live monitor
q0 = raw2joint_at40gw(robot, getrawpos_at40gw(handle,robot));
xtraj = joint2cart_at40gw(raw2joint_at40gw(robot, qrawtraj));
xarm0 = joint2arm_at40gw(q0);
figure;
hold on;
h = plot3(x0(:,1),x0(:,2),x0(:,3),'r');
plot3(xtraj(:,1),xtraj(:,2),xtraj(:,3),'b');
ha = plot3(xarm0(:,1),xarm0(:,2),xarm0(:,3),'g');
hx = plot3(x0(:,1),x0(:,2),x0(:,3),'gx');
view([1 1 0.5]);
axlim = max(max(abs(xarm0(:,1:2)))) + 2500;
axis([-axlim axlim -axlim axlim 0 5000]);
grid on;
hold off;

%% Execute (code similar to followrawtraj, but allows for easy access to variables)
% TODO: turn this into a function once it's been thoroughly tested

xtraversed = [];
xarm = [];
logi.Qrawcomm = [];
logi.Qrawdes = [];
logi.Qrawmeas = [];
logi.Qrawmeasu = []; % Unfiltered measurements
logi.Qrawerr = [];
logi.err = [];
logi.derr = [];
logi.PWMcomm = [];
logi.T = [];
i = 1;

% Vectorize settings
Kps = [robot.Joint.Kp];
Kds = [robot.Joint.Kd];
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

% Total time to traverse trajectory
tt = size(qrawtraj,1) * dt;

% Follow trajectory
disp('Now following');

tic;
qrawprev = getrawpos_at40gw(handle, robot);
qrawdesprev = qrawprev;
errprev = 0;
tprev = 0;

while true
    t = toc;
    
    % If settled, exit
    if waittosettle && all(getpwms_at40gw(handle, robot) == robot.PWMZero) && t > tt
        break;
    elseif norm(getrawpos_at40gw(handle,robot)-qrawtraj(end,:)) < tol && t > tt
        setpwms_at40gw(handle, robot, robot.PWMZero .* ones(1,4));
        break;
    end
    
    % Get desired joint position at this time
    if t > tt
        qrawdes = qrawtraj(end,:);
    elseif useinterp
        % Linearly interpolate desired position and velocity (NOT C0
        % Continuous!)
        ti = ceil(t / dt);
        talpha = mod(t, dt) / dt;
        if ti ~= 1
            qrawdesprev = qrawtraj(ti-1, :);
        end
        qrawdes = (1-talpha) .* qrawdesprev + talpha .* qrawtraj(ti, :);
    elseif usestep
        % For tuning controllers
        qrawdes = qrawtraj(end,:);
    else
        ti = ceil(t / dt);
        qrawdes = qrawtraj(ti, :);
    end
    
    % Get current raw joint positions and velocities
    qrawcurru = getrawpos_at40gw(handle, robot);
    qrawcurr = lpfilter(qrawprev, qrawcurru, [0.96 0.97 0.97 0.97]);
    %qrawcurr = lpfilter(qrawprev, qrawcurru, [0 0 0 0]); % Unfiltered
    
    % Compute the error
    err = qrawdes - qrawcurr;
    dtcurr = t - tprev;
    derr = (err - errprev) / dtcurr;
    
    % Compute commanded raw joint values
    qrawcomm = Kps .* GainInvs .* err;
    if usepd && usedmeas
        dmeaserr = (qrawcurr - qrawprev) / dtcurr;
        qrawcomm = qrawcomm + Kds .* GainInvs .* dmeaserr;
    elseif usepd
        qrawcomm = qrawcomm + Kds .* GainInvs .* derr;
    end
    qrawcomm = clamp(qrawcomm, rawminlim, rawmaxlim);
    
    % Convert to PWMs
    posi = qrawcomm > 0;
    nega = qrawcomm < 0;
    zer = abs(err) < movethresh;
    
    pwmcomm = zeros(size(qrawcomm));
    % TODO: is this transformation correct?
    % TODO: adjust the speed factor?
    pwmcomm(posi) = mapRange(qrawcomm(posi), 0, rawmaxlim(posi), pwmdbmax(posi), pwmmax(posi));
    pwmcomm(nega) = mapRange(qrawcomm(nega), 0, rawminlim(nega), pwmdbmin(nega), pwmmin(nega));
    pwmcomm(zer) = robot.PWMZero;
    pwmcomm = round(pwmcomm);
    
    % Send the command to the labjack
    setpwms_at40gw(handle, robot, pwmcomm);
    
    % Prep next cycle
    qrawprev = qrawcurr;
    errprev = err;
    tprev = t;
    
    % Update monitor and logs
    xtraversed = [xtraversed; joint2cart_at40gw(raw2joint_at40gw(robot,qrawcurr))];
    xarm = joint2arm_at40gw(raw2joint_at40gw(robot,qrawcurr));
    xcurr = joint2cart_at40gw(raw2joint_at40gw(robot, qrawdes));
    
    set(h,'xdata',xtraversed(:,1),'ydata',xtraversed(:,2),'zdata',xtraversed(:,3));
    set(ha,'xdata',xarm(:,1),'ydata',xarm(:,2),'zdata',xarm(:,3));
    set(hx,'xdata',xcurr(:,1),'ydata',xcurr(:,2),'zdata',xcurr(:,3));
    
    drawnow; 
    
    logi.Qrawdes = [logi.Qrawdes qrawdes'];
    logi.Qrawcomm = [logi.Qrawcomm qrawcomm'];
    logi.Qrawerr = [logi.Qrawerr err'];
    logi.Qrawmeasu = [logi.Qrawmeasu qrawcurru'];
    logi.Qrawmeas = [logi.Qrawmeas qrawcurr'];
    logi.PWMcomm = [logi.PWMcomm pwmcomm'];
    logi.err = [logi.err err'];
    logi.derr = [logi.derr derr'];
    logi.T = [logi.T t];
    i = i + 1;
end

disp('Done!');
beep;