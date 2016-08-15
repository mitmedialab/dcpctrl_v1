% Very simple repeatability test
% Moves away, and then slowly attempts to return to the same position at
% the slowest possible movement.

%% Get goal position
%qfraw = getrawpos_at40gw(handle,robot);

%% Move to goal position as slow as possible
robot = config_at40gw;
setpwms_at40gw(handle,robot,[80000 80000 80000 80000]);

tic;
t = 0;

thresh = [50 0.004 0.004 0.004];
dispstat('','init');
while true
    qraw = getrawpos_at40gw(handle,robot);
    draw = qfrawprec - qraw;
    
    if abs(draw) < thresh
        draw
        pwmcomm = ones(1,4).*robot.PWMZero;
        setpwms_at40gw(handle,robot,pwmcomm);
        disp('Arrived');
        break;
    end
    dire = sign(draw);
    
    pwmcomm = ones(1,4).*robot.PWMZero;
    if abs(draw(1)) < thresh(1)
        pwmcomm(1) = robot.PWMZero;
    elseif dire(1) == 1
        pwmcomm(1) = robot.Joint(1).PWMLim.DBUpper;
    else
        pwmcomm(1) = robot.Joint(1).PWMLim.DBLower;
    end
    
    if abs(draw(2)) < thresh(2)
        pwmcomm(2) = robot.PWMZero;
    elseif dire(2) == 1
        pwmcomm(2) = robot.Joint(2).PWMLim.DBUpper;
    else
        pwmcomm(2) = robot.Joint(2).PWMLim.DBLower;
    end
    
    if abs(draw(3)) < thresh(3)
        pwmcomm(3) = robot.PWMZero;
    elseif dire(3) == 1
        pwmcomm(3) = robot.Joint(3).PWMLim.DBLower;
    else
        pwmcomm(3) = robot.Joint(3).PWMLim.DBUpper;
    end
    
    if abs(draw(4)) < thresh(4)
        pwmcomm(4) = robot.PWMZero;
    elseif dire(4) == 1
        pwmcomm(4) = robot.Joint(4).PWMLim.DBLower;
    else
        pwmcomm(4) = robot.Joint(4).PWMLim.DBUpper;
    end
    pwmcomm(2) = robot.PWMZero; % Safety
    setpwms_at40gw(handle,robot,pwmcomm);
    
    dispstat(sprintf('raw distance:\t%.4f %.4f %.4f %.4f\nPWMs:\t%d %d %d %d',draw,pwmcomm));

end