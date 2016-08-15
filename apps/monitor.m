%% Monitor joints, cartesian positions, PWMs
% Live monitor of data, quick way to check values/intuition

% Initial Setup
qraw = getrawpos_at40gw(handle, robot);
q = raw2joint_at40gw(robot, qraw);
xarm = joint2arm_at40gw(q);

figure;
h = plot3(xarm(:,1),xarm(:,2),xarm(:,3),'g','LineWidth',5);
axis([-5000 5000 -5000 5000 -100 3000]);

dispstat('','init');

% Monitor all data and plot
tic;
tprev = toc;
while 1
    
    t = toc;
    prd = t - tprev;
    fq = 1/prd;
    tprev = t;
    
    qraw = getrawpos_at40gw(handle, robot);
    pwms = getpwms_at40gw(handle, robot);
    q = raw2joint_at40gw(robot, qraw);
    x = joint2cart_at40gw(q);
    xarm = joint2arm_at40gw(q);
    
    dispstat(sprintf('Speed:\t%.3fHz -- %.3fsec\nRaw joints:\t\t%.4f %.4f %.4f %.4f\nJoints:\t\t\t%.4f %.4f %.4f %.4f\nPWM:\t\t\t%i %i %i %i\nCart Position:\t%.4f %.4f %.4f %.4f', fq, prd, qraw, q, pwms, x));
    
    set(h,'XData',xarm(:,1),'YData',xarm(:,2),'ZData',xarm(:,3));
    drawnow;
end