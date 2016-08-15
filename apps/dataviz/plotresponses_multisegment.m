% PLOTRESPONSES_MULTISEGMENT Version of plotresponses built to work with
% multi-segment trajectories and new data names

%{
    Julian Leland, MIT Media Lab, 2016-07-13
    INPUTS (in workspace):
        Complete set of data from loadlogsegments (run this function
        first).
%}


%% Get data from workspace
% This section is mostly redundant - just for JL's bookkeeping while he's
% writing this code.
%jdes = raw2joint_at40gw(robot, qrawdes); % Desired joint angle
%jmeas = raw2joint_at40gw(robot, qraw); % Actual joint angle
seg; % Vector of segment start points
qrawdes; % Desired joint angle, raw values
qdes = raw2joint_at40gw(robot,qrawdes); % Desired joint angle, degrees
qraw; % Actual joint angle, raw values
q = raw2joint_at40gw(robot,qraw); % Actual joint angle, degrees
pwm; % Commanded PWM velocity
perr = Kpe; % Proportional error command
derr = Kde; % Derivative error command
ierr = Kie; % Integral error command
ff = Kffv; % Feedforward command
ts = ts;
PWMcomm = pwm;
pts = [1:length(q)]';

%% Create consistent ts and tdes vectors to plot against
tsfull = [ts(1)];
tsadd = 0;
tdesfull = [tdes(1)];
tdesadd = 0;
for n = 2:length(ts)
    if ts(n) < ts(n-1)
        tsadd = tsadd + ts(n-1);
    end
    if tdes(n) < tdes(n-1)
        tdesadd = tdesadd + tdes(n-1);
    end
    tsfull = [tsfull; ts(n)+tsadd];
    tdesfull = [tdesfull; tdes(n)+tdesadd];
end
        

%% Plot 3D arm trajectory
% Convert joint angles to Cartesian space
xdes = joint2cart_at40gw(raw2joint_at40gw(robot,qrawdes));
xmeas = joint2cart_at40gw(raw2joint_at40gw(robot,qraw));
arm0 = joint2arm_at40gw(raw2joint_at40gw(robot,qraw(1,:)));

figure;
hold on;
plot3(xdes(:,1),xdes(:,2),xdes(:,3),'b');
plot3(xmeas(:,1),xmeas(:,2),xmeas(:,3),'r');
plot3(arm0(:,1),arm0(:,2),arm0(:,3),'g');

% Label all waypoints
segnum = 1;
for n = 1:length(seg)
    if seg(n)
       txt = strcat('Seg. ',num2str(segnum));
       text(xmeas(n,1),xmeas(n,2),xmeas(n,3),txt);
       segnum = segnum + 1;
    end
end
legend('desired traj', 'measured traj', 'initial arm config');
axlim = max(max(abs(xmeas)));
%axis([-axlim axlim -axlim axlim 0 axlim]);
xlabel('X, mm');
ylabel('Y, mm');
zlabel('Z, mm');
axis equal
axis image
grid on;
view(45,45);
hold off;

%% Plot individual joint trajectories
figure;

segshow = 0; % Label move segments

subplot(5,2,1)
hold on;
plot(pts, qdes(:,1));
plot(pts, q(:,1));
if segshow
    segnum = 1;
    for n = 1:length(seg)
        if seg(n)
           txt = strcat('Seg. ',num2str(segnum));
           text(pts(n),qdes(n,1),txt);
           segnum = segnum + 1;
        end
    end
end
title('Joint 1 (encoder counts)');
axis tight
xLimits = xlim;
xlim(xLimits);
hold off;

subplot(5,2,3)
[ax1,h11,h21] = plotyy(pts, pwm(:,1),pts, Kpe(:,1));
if segshow
    segnum = 1;
    for n = 1:length(seg)
        if seg(n)
           txt = strcat('Seg. ',num2str(segnum));
           text(pts(n),pwm(n,1),txt);
           segnum = segnum + 1;
        end
    end
end
hold(ax1(2),'on');
plot(ax1(2),pts, Kde(:,1),':','Color',[0.8500    0.3250    0.0980]);
legend('PWM\_comm','P\_err','D\_err');
hold off;
hold(ax1(1),'on');
dbup = ones(length(pts),1)*robot.Joint(1).PWMLim.DBUpper;
dblow = ones(length(pts),1)*robot.Joint(1).PWMLim.DBLower;
plot(ax1(1),pts, dbup,'--','Color',[     0    0.4470    0.7410]);
plot(ax1(1),pts, dblow,'--','Color',[     0    0.4470    0.7410]);
hold (ax1(1),'off');
xlim(ax1(1),xLimits)
xlim(ax1(2),xLimits)

subplot(5,2,2)
hold on;
plot(pts, qdes(:,2));
plot(pts, q(:,2));
if segshow
    segnum = 1;
    for n = 1:length(seg)
        if seg(n)
           txt = strcat('Seg. ',num2str(segnum));
           text(pts(n),qdes(n,2),txt);
           segnum = segnum + 1;
        end
    end
end
title('Joint 2 (degrees)');
xlim(xLimits);
hold off;

subplot(5,2,4)
[ax2,h11,h21] = plotyy(pts, pwm(:,2),pts, Kpe(:,2));
if segshow
    segnum = 1;
    for n = 1:length(seg)
        if seg(n)
           txt = strcat('Seg. ',num2str(segnum));
           text(pts(n),pwm(n,2),txt);
           segnum = segnum + 1;
        end
    end
end
hold(ax2(2),'on');
plot(ax2(2),pts, Kde(:,2),':','Color',[0.8500    0.3250    0.0980]);
legend('PWM_comm','P_err','D_err');
hold off;
hold(ax2(1),'on');
dbup = ones(length(pts),1)*robot.Joint(2).PWMLim.DBUpper;
dblow = ones(length(pts),1)*robot.Joint(2).PWMLim.DBLower;
plot(ax2(1),pts, dbup,'--','Color',[     0    0.4470    0.7410]);
plot(ax2(1),pts, dblow,'--','Color',[     0    0.4470    0.7410]);
hold (ax2(1),'off');
xlim(ax2(1),xLimits)
xlim(ax2(2),xLimits)
ylim auto

subplot(5,2,5)
hold on;
plot(pts, qdes(:,3));
plot(pts, q(:,3));
if segshow
    segnum = 1;
    for n = 1:length(seg)
        if seg(n)
           txt = strcat('Seg. ',num2str(segnum));
           text(pts(n),qdes(n,3),txt);
           segnum = segnum + 1;
        end
    end
end
title('Joint 3 (degrees)');
xlim(xLimits);
hold off;

subplot(5,2,7)
[ax1,h11,h21] = plotyy(pts, pwm(:,3),pts, Kpe(:,3));
if segshow
    segnum = 1;
    for n = 1:length(seg)
        if seg(n)
           txt = strcat('Seg. ',num2str(segnum));
           text(pts(n),pwm(n,3),txt);
           segnum = segnum + 1;
        end
    end
end
hold(ax1(2),'on');
plot(ax1(2),pts, Kde(:,3),':','Color',[0.8500    0.3250    0.0980]);
legend('PWM\_comm','P\_err','D\_err');
hold off;
hold(ax1(1),'on');
dbup = ones(length(pts),1)*robot.Joint(3).PWMLim.DBUpper;
dblow = ones(length(pts),1)*robot.Joint(3).PWMLim.DBLower;
plot(ax1(1),pts, dbup,'--','Color',[     0    0.4470    0.7410]);
plot(ax1(1),pts, dblow,'--','Color',[     0    0.4470    0.7410]);
hold off;
xlim(ax1(1),xLimits)
xlim(ax1(2),xLimits)
ylim auto

subplot(5,2,6)
hold on;
plot(pts, qdes(:,4));
plot(pts, q(:,4));
if segshow
    segnum = 1;
    for n = 1:length(seg)
        if seg(n)
           txt = strcat('Seg. ',num2str(segnum));
           text(pts(n),qdes(n,4),txt);
           segnum = segnum + 1;
        end
    end
end
title('Joint 4 (mm)');
xlim(xLimits);
hold off;

subplot(5,2,8)
[ax1,h11,h21] = plotyy(pts, PWMcomm(:,4),pts, Kpe(:,4));
if segshow
    segnum = 1;
    for n = 1:length(seg)
        if seg(n)
           txt = strcat('Seg. ',num2str(segnum));
           text(pts(n),pwm(n,4),txt);
           segnum = segnum + 1;
        end
    end
end
hold(ax1(2),'on');
plot(ax1(2),pts, Kde(:,4),':','Color',[0.8500    0.3250    0.0980]);
legend('PWM\_comm','P\_err','D\_err');
hold off;
hold(ax1(1),'on');
dbup = ones(length(pts),1)*robot.Joint(4).PWMLim.DBUpper;
dblow = ones(length(pts),1)*robot.Joint(4).PWMLim.DBLower;
plot(ax1(1),pts, dbup,'--','Color',[     0    0.4470    0.7410]);
plot(ax1(1),pts, dblow,'--','Color',[     0    0.4470    0.7410]);
hold off;
xlim(ax1(1),xLimits)
xlim(ax1(2),xLimits)
ylim auto

d_ts = diff(ts);
d_ts = [d_ts;0];
subplot(5,2,9)
hold on;
plot(pts,d_ts);
if segshow
    segnum = 1;
    for n = 1:length(seg)
        if seg(n)
           txt = strcat('Seg. ',num2str(segnum));
           text(pts(n),d_ts(n),txt);
           segnum = segnum + 1;
        end
    end
end
title('Control Loop dt (sec)');
xlim(xLimits);

%% Plot just one plot by itself against time
jnt = 3; % Set which joint we want to look at
segshow = 1; % Label move segments

figure;
subplot(2,1,1);
[ax1,h11,h21] = plotyy(pts, pwm(:,jnt),pts, Kpe(:,jnt));
if segshow
    segnum = 1;
    for n = 1:length(seg)
        if seg(n)
           txt = strcat('Seg. ',num2str(segnum));
           text(pts(n),pwm(n,jnt),txt);
           segnum = segnum + 1;
        end
    end
end
hold(ax1(2),'on');
plot(ax1(2),pts, Kde(:,jnt),'--','Color',[0.8500    0.3250    0.0980]);
plot(ax1(2),pts, Kie(:,jnt),'--','Color',[0.4940    0.1840    0.5560]);
plot(ax1(2),pts, Kffv(:,jnt),'--','Color',[0.4660    0.6740    0.1880]);
legend('PWM\_comm','P\_err','D\_err','I\_err','FFv');
txt = sprintf('PWM & Error: Joint %d',jnt);
title(txt);
hold off;
hold(ax1(1),'on');
dbup = ones(length(pts),1)*robot.Joint(jnt).PWMLim.DBUpper;
dblow = ones(length(pts),1)*robot.Joint(jnt).PWMLim.DBLower;
plot(ax1(1),pts, dbup,'--','Color',[     0    0.4470    0.7410]);
plot(ax1(1),pts, dblow,'--','Color',[     0    0.4470    0.7410]);
hold (ax1(1),'off');
xlim(ax1(1),xLimits)
xlim(ax1(2),xLimits)
ylim auto


d_ts = diff(ts);
d_ts = [d_ts;0];
subplot(2,1,2);
hold on;
plot(pts,d_ts);
title('Control Loop dt (sec)');
xlim(xLimits);

%% Plot desired position vs. desired time against actual position vs. actual time
jnt = 4; % Set which joint we want to look at
segshow = 1; % Label move segments

figure;
[ax1,h11,h21] = plotyy(tsfull, q(:,jnt),tdesfull, qdes(:,jnt));
if segshow
    segnum = 1;
    for n = 1:length(seg)
        if seg(n)
           txt = strcat('Seg. ',num2str(segnum));
           text(tsfull(n),q(n,jnt),txt);
           segnum = segnum + 1;
        end
    end
end