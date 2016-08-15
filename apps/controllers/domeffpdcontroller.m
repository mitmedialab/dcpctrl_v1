%% Feed Forward Velocity and Position PD Controller
% INPUTS (expects in the workspace):
%   qrawtrajs - cells containing segments to follow
%   qdrawtrajs - cells containing segment raw joint velocities
%   dt - period for controller loop
%
% SETTINGS:
%   simflag - 1 enable simflaglation, 0 enable real robot interface
%   logflag - 1 enable logging, 0 disable logging
%   vizflagflag - 1 show visualization during print, 0 hide it
%   tol - (mm) tolerance for end of a segment

% Setup
logflag = 1;
vizflag = 1;
simflag = 1;
wait4settle = 1; % Waits at end of each segment to stop moving
tol = 50; % mm
foldername = 'partialdome';
usedjoints = logical([1 0 1 1]);

if simflag
    foldername = ['data/sim/' foldername '/'];
else
    foldername = ['data/' foldername '/'];
end

if logflag && ~exist(foldername,'dir')
    mkdir(foldername);
end

global stopflag;
stopflag = 0;

%% Setup visualization
if vizflag
    if ~simflag
        q0 = raw2joint_at40gw(robot, getrawpos_at40gw(handle,robot));
    else
        load('data/homerawpos.mat');
        q0raw = homerawpos;
        q0 = raw2joint_at40gw(robot,q0raw);
    end

    xttraj = joint2cart_at40gw(raw2joint_at40gw(robot, cell2mat(qrawtrajs)));
    arm0 = joint2arm_at40gw(q0);
    seg0 = joint2cart_at40gw(raw2joint_at40gw(robot, qrawtrajs{1}));
    xtrav = xttraj(1,:);

    figure;
    hold on;
    plot3(xttraj(:,1),xttraj(:,2),xttraj(:,3),'c');
    harm = plot3(arm0(:,1),arm0(:,2),arm0(:,3),'g','LineWidth',5);
    hdes = plot3(xttraj(1,1),xttraj(1,2),xttraj(1,3),'mx','MarkerSize',13,'LineWidth',2);
    hseg = plot3(seg0(:,1),seg0(:,2),seg0(:,3),'b','LineWidth',3);
    htrav = plot3(xtrav(1,1),xtrav(1,2),xtrav(1,3),'r.');
    hpause = uicontrol('Style', 'PushButton', 'String', 'Pause','Callback','pauseprint(1)');
    axis equal;
    hold off;
end

%% Setup FFPD Controller gains
robot = config_at40gw;

Kps = [robot.Joint.Kp];
Kds = [robot.Joint.Kd];
Kis = [robot.Joint.Ki];
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

%% Reset the segments
segstart = 1;

% Can use this with move2raw to auto position
qfraw = qrawtrajs{segstart}(1,:);

%% Follow trajectory
dispstat('','init');
for segi = segstart:nsegs
    
    if stopflag == 1
        disp(['Just finished segment ' num2str(segi-1) ' next segment is ' num2str(segi)]);
        segstart = segi;
        qfraw = qrawtrajs{segi}(1,:);
        stopflag = 0;
        set(hpause,'Enable','on');
        break;
    end
    
    % Setup for segment
    qrawtraj = qrawtrajs{segi};
    qdrawtraj = qdrawtrajs{segi};
    ts = segts{segi};
    tf = ts(end)+dt;
    npoints = size(qrawtraj,1);
    
    mtype = mtypes{segi};
    
    ti = 1;
    qdrawcomm = zeros(1,4);
    tprev = 0;
    qrawdesprev = qrawtraj(1,:);
    qdrawdesprev = qdrawtraj(1,:);
    errprev = 0;
    
    % Reset Kie for each segment (should we do this, or allow it to
    % accumulate)
    Kie = zeros(1,4);
    
    xf = joint2cart_at40gw((raw2joint_at40gw(robot, qrawtraj(end,:))));
    
    if simflag
        qrawprev = qrawtraj(1,:);
    else
        qrawprev = getrawpos_at40gw(handle, robot);
    end
    
    logs = {};
    logi = 1;
    
    % Update visualization (segment only)
    if vizflag
        segt = joint2cart_at40gw(raw2joint_at40gw(robot, qrawtraj));
        set(hseg,'xdata',segt(:,1),'ydata',segt(:,2),'zdata',segt(:,3));
        drawnow;
    end
    
    % Follow the trajectory
    tic;
    t = 0;
    while true
        
        if stopflag
            set(hpause,'Enable','off');
        end
        
        t = toc;
        if simflag
            qraw = qrawprev + qdrawcomm.*(t-tprev);
        else
            qraw = getrawpos_at40gw(handle,robot);
        end
        
        % End condition
        %xc = joint2cart_at40gw((raw2joint_at40gw(robot, qraw)));
        %if norm(xf-xc) < tol && t > tf
        % TODO: tf should be absolute tf, and not segment tf
        
        % 1 - cart, 0 - joint
        if mtype == 1
            usedjoints = logical([0 0 1 1]);
        else
            usedjoints = logical([1 0 0 0]);
        end

        if ~wait4settle && segi == nsegs && t > tf && all(abs(qraw(usedjoints)-qrawtrajs{end}(end,usedjoints)) < movethresh(usedjoints))
            if ~simflag
                setpwms_at40gw(handle, robot, robot.PWMZero .* ones(1,4));
            end
            dispstat(['Just finished trajectory.'],'keepthis');
            break;
        elseif wait4settle && t > tf && all(abs(qraw(usedjoints)-qrawtraj(end,usedjoints)) < movethresh(usedjoints)) 
            if ~simflag
                setpwms_at40gw(handle, robot, robot.PWMZero .* ones(1,4));
            end
            dispstat(['Just finished segment ' num2str(segi)],'keepthis');
            break;
        end
        
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
        
        if t > tf && segi < nsegs && ~wait4settle
            break;
        elseif t > tf
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
        Kde = - Kds .* dmeaserr; % This must be negative
        Kie = Kis .* err + Kie;
        Kie = clamp(Kie, rawminlim, rawmaxlim); % clamp on raw or draw?
        %qrawcomm = clamp(qrawcomm, rawminlim, rawmaxlim);
        
        qdrawcomm = Kffv + Kpe + Kde + Kie;
        
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
        
        % 1 - cart, 0 - joint
        if mtype == 1
            pwmcomm(1) = robot.PWMZero;
        else
            pwmcomm(3:4) = robot.PWMZero;
        end
        
        % TODO: Verifications (none of these should ever trigger);
        
        % Send PWM command
        if ~simflag
            setpwms_at40gw(handle, robot, pwmcomm);
        end
        

        
        % Update visualization
        if vizflag
            qv = raw2joint_at40gw(robot, qraw);
            armv = joint2arm_at40gw(qv);
            xdesv = joint2cart_at40gw(raw2joint_at40gw(robot, qrawdes));
            xtrav = [xtrav; joint2cart_at40gw(raw2joint_at40gw(robot, qraw))];

            set(htrav, 'xdata', xtrav(:,1), 'ydata', xtrav(:,2), 'zdata', xtrav(:,3));
            set(harm, 'xdata', armv(:,1), 'ydata', armv(:,2), 'zdata', armv(:,3));
            set(hdes, 'xdata', xdesv(1), 'ydata', xdesv(2), 'zdata', xdesv(3));
            drawnow;
            
            dispstat(sprintf(['Joint 1: | Current position: %.3f | Commanded position: %.3f | Error: %.4f | PWM Command: %d\n',...
                 'Joint 2: | Current position: %.3f | Commanded position: %.3f | Error: %.4f | PWM Command: %d\n',...
                 'Joint 3: | Current position: %.3f | Commanded position: %.3f | Error: %.4f | PWM Command: %d\n',...
                 'Joint 4: | Current position: %.3f | Commanded position: %.3f | Error: %.4f | PWM Command: %d\n',...
                 'Time: %.2fmin / %.2fmin, dt: %.4fsec'],...
                 qraw(1),qrawdes(1),err(1),pwmcomm(1),...
                 qraw(2),qrawdes(2),err(2),pwmcomm(2),...
                 qraw(3),qrawdes(3),err(3),pwmcomm(3),...
                 qraw(4),qrawdes(4),err(4),pwmcomm(4),...
                 absts{segi}(ti)/60, absts{nsegs}(end)/60, t-tprev));
        end
        
        % Log data
        if logflag
            logs(logi).t = t;
            logs(logi).tdes = ts(ti);
            logs(logi).i = logi;
            logs(logi).segi = segi;
            logs(logi).pwm = pwmcomm';
            logs(logi).qraw = qraw';
            logs(logi).qrawdes = qrawdes';
            logs(logi).qdrawdes = qdrawdes';
            logs(logi).Kffv = Kffv';
            logs(logi).Kpe = Kpe';
            logs(logi).Kde = Kde';
            logs(logi).Kie = Kie';
            logs(logi).qdrawcomm = qdrawcomm';
            logs(logi).err = err';
            logs(logi).derr = derr';
            logs(logi).dmeaserr = dmeaserr';
            logi = logi + 1;
        end
        
        % Setup for next iteration
        tprev = t;
        errprev = err;
        qrawprev = qraw;
    end
    
    % Save the log
    if logflag
        filename = [foldername 'segment_' num2str(segi)];
        if simflag
            filename = [filename '_sim'];
        end
        disp(['Saving ' filename]);
        save(filename, 'logs');
        prevlogs = logs; % Maintain last one in workspace for fast reference
        clear logs;
    end
end
