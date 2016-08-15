%% If robot hasn't been loaded, load it.
if isempty(robot)
    init;
end

%% Plot 3D arm trajectory
jdes = raw2joint_at40gw(robot, logi.Qrawdes');
jmeas = raw2joint_at40gw(robot, logi.Qrawmeas');
jmeasu = raw2joint_at40gw(robot, logi.Qrawmeasu');
jerr = logi.Qrawerr';
perr = logi.err';
derr = logi.derr';
ts = logi.T';
PWMcomm = logi.PWMcomm';
cdes = joint2cart_at40gw(jdes);
cmeas = joint2cart_at40gw(jmeas);
arm0 = joint2arm_at40gw(jmeas(1,:));
figure;
hold on;
plot3(cdes(:,1),cdes(:,2),cdes(:,3),'b');
plot3(cmeas(:,1),cmeas(:,2),cmeas(:,3),'r');
plot3(arm0(:,1),arm0(:,2),arm0(:,3),'g');
legend('desired traj', 'measured traj', 'initial arm config');
view([1 1 1]);
axlim = max(max(abs(cmeas)));
axis([-axlim axlim -axlim axlim 0 axlim]);
hold off;

%% Plot position response curves
figure;
ts = logi.T;
subplot(5,2,1)
hold on;
plot(ts, jdes(:,1));
plot(ts, jmeas(:,1));
title('Joint 1 (encoder counts)');
axis tight
xLimits = xlim;
xlim(xLimits);
hold off;

subplot(5,2,3)
[ax1,h11,h21] = plotyy(ts, PWMcomm(:,1),ts, perr(:,1));
hold(ax1(2),'on');
plot(ax1(2),ts, derr(:,1),':','Color',[0.8500    0.3250    0.0980]);
legend('PWM\_comm','P\_err','D\_err');
hold off;
hold(ax1(1),'on');
dbup = ones(length(ts),1)*robot.Joint(1).PWMLim.DBUpper;
dblow = ones(length(ts),1)*robot.Joint(1).PWMLim.DBLower;
plot(ax1(1),ts, dbup,'--','Color',[     0    0.4470    0.7410]);
plot(ax1(1),ts, dblow,'--','Color',[     0    0.4470    0.7410]);
hold (ax1(1),'off');
xlim(ax1(1),xLimits)
xlim(ax1(2),xLimits)

subplot(5,2,2)
hold on;
plot(ts, jdes(:,2));
plot(ts, jmeas(:,2));
title('Joint 2 (degrees)');
xlim(xLimits);
hold off;

subplot(5,2,4)
[ax2,h11,h21] = plotyy(ts, PWMcomm(:,2),ts, perr(:,2));
hold(ax2(2),'on');
plot(ax2(2),ts, derr(:,2),':','Color',[0.8500    0.3250    0.0980]);
legend('PWM_comm','P_err','D_err');
hold off;
hold(ax2(1),'on');
dbup = ones(length(ts),1)*robot.Joint(2).PWMLim.DBUpper;
dblow = ones(length(ts),1)*robot.Joint(2).PWMLim.DBLower;
plot(ax2(1),ts, dbup,'--','Color',[     0    0.4470    0.7410]);
plot(ax2(1),ts, dblow,'--','Color',[     0    0.4470    0.7410]);
hold (ax2(1),'off');
xlim(ax2(1),xLimits)
xlim(ax2(2),xLimits)
ylim auto

subplot(5,2,5)
hold on;
plot(ts, jdes(:,3));
plot(ts, jmeas(:,3));
title('Joint 3 (degrees)');
xlim(xLimits);
hold off;

subplot(5,2,7)
[ax1,h11,h21] = plotyy(ts, PWMcomm(:,3),ts, perr(:,3));
hold(ax1(2),'on');
plot(ax1(2),ts, derr(:,3),':','Color',[0.8500    0.3250    0.0980]);
legend('PWM\_comm','P\_err','D\_err');
hold off;
hold(ax1(1),'on');
dbup = ones(length(ts),1)*robot.Joint(3).PWMLim.DBUpper;
dblow = ones(length(ts),1)*robot.Joint(3).PWMLim.DBLower;
plot(ax1(1),ts, dbup,'--','Color',[     0    0.4470    0.7410]);
plot(ax1(1),ts, dblow,'--','Color',[     0    0.4470    0.7410]);
hold off;
xlim(ax1(1),xLimits)
xlim(ax1(2),xLimits)
ylim auto

subplot(5,2,6)
hold on;
plot(ts, jdes(:,4));
plot(ts, jmeas(:,4));
title('Joint 4 (mm)');
xlim(xLimits);
hold off;

subplot(5,2,8)
[ax1,h11,h21] = plotyy(ts, PWMcomm(:,4),ts, perr(:,4));
hold(ax1(2),'on');
plot(ax1(2),ts, derr(:,4),':','Color',[0.8500    0.3250    0.0980]);
legend('PWM\_comm','P\_err','D\_err');
hold off;
hold(ax1(1),'on');
dbup = ones(length(ts),1)*robot.Joint(4).PWMLim.DBUpper;
dblow = ones(length(ts),1)*robot.Joint(4).PWMLim.DBLower;
plot(ax1(1),ts, dbup,'--','Color',[     0    0.4470    0.7410]);
plot(ax1(1),ts, dblow,'--','Color',[     0    0.4470    0.7410]);
hold off;
xlim(ax1(1),xLimits)
xlim(ax1(2),xLimits)
ylim auto

d_ts = diff(ts);
d_ts = [d_ts,0];
subplot(5,2,9)
hold on;
plot(ts,d_ts);
title('Control Loop dt (sec)');
xlim(xLimits);

%% Plot just one plot by itself against time
jnt = 1;

figure;
subplot(2,1,1);
[ax1,h11,h21] = plotyy(ts, PWMcomm(:,jnt),ts, perr(:,jnt));
hold(ax1(2),'on');
plot(ax1(2),ts, derr(:,1),':','Color',[0.8500    0.3250    0.0980]);
legend('PWM\_comm','P\_err','D\_err');
txt = sprintf('PWM & Error: Joint %d',jnt);
title(txt);
hold off;
hold(ax1(1),'on');
dbup = ones(length(ts),1)*robot.Joint(jnt).PWMLim.DBUpper;
dblow = ones(length(ts),1)*robot.Joint(jnt).PWMLim.DBLower;
plot(ax1(1),ts, dbup,'--','Color',[     0    0.4470    0.7410]);
plot(ax1(1),ts, dblow,'--','Color',[     0    0.4470    0.7410]);
hold (ax1(1),'off');
xlim(ax1(1),xLimits)
xlim(ax1(2),xLimits)
ylim auto


d_ts = diff(ts);
d_ts = [d_ts,0];
subplot(2,1,2);
hold on;
plot(ts,d_ts);
title('Control Loop dt (sec)');
xlim(xLimits);

%% Plot qrawmeas & qrawmeasu against errors
jnt = 1;

figure;
ts = logi.T;
subplot(4,1,1)
hold on;
plot(ts, jdes(:,jnt));
plot(ts, jmeas(:,jnt));
title('Joint 1 (encoder counts)');
axis tight
xLimits = xlim;
xlim(xLimits);
hold off;

subplot(4,1,2);
[ax1,h11,h21] = plotyy(ts, PWMcomm(:,jnt),ts, perr(:,jnt));
hold(ax1(2),'on');
plot(ax1(2),ts, derr(:,1),':','Color',[0.8500    0.3250    0.0980]);
legend('PWM\_comm','P\_err','D\_err');
txt = sprintf('PWM & Error: Joint %d',jnt);
title(txt);
hold off;
hold(ax1(1),'on');
dbup = ones(length(ts),1)*robot.Joint(jnt).PWMLim.DBUpper;
dblow = ones(length(ts),1)*robot.Joint(jnt).PWMLim.DBLower;
plot(ax1(1),ts, dbup,'--','Color',[     0    0.4470    0.7410]);
plot(ax1(1),ts, dblow,'--','Color',[     0    0.4470    0.7410]);
hold (ax1(1),'off');
xlim(ax1(1),xLimits)
xlim(ax1(2),xLimits)
ylim auto

subplot(4,1,3);
[ax1,h11,h21] = plotyy(ts, jmeas(:,jnt),ts, jmeasu(:,jnt));
legend('Jmeas','Jmeas_u');
txt = sprintf('Measurement & filtered measurement: Joint %d',jnt);
title(txt);

d_ts = diff(ts);
d_ts = [d_ts,0];
subplot(4,1,4);
hold on;
plot(ts,d_ts);
title('Control Loop dt (sec)');
xlim(xLimits);