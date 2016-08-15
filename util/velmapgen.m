%{
VELMAPGEN Generate mapping surface between joint angular velocity and joint
sensor velocity, as a function of joint angle

Julian Leland, MIT Media Lab, 2016-06-22

This function creates a surface that maps between joint angular velocity
and joint sensor velocity, as a function of joint angle. This is only
useful for joints where the relationship between sensor velocity
(volts/sec, counts/sec, etc.) and joint angular (or linear, although this
will be less common) velocity is nonlinear. On the AT40GW, this is true for
Joints 2 and 3. The below code is set up to operate on J3, although it
could be repurposed for J2 as well.

%}
%% Initialize
init;

%% Using known data from velocity tests, calculate range of dV_dt values that we will see

% Range of PWM DCs we want to calculate dV/dt at
PWM_vect = 30000:500:80000;
% NOTE: This function is only actually valid up to around 64500 or so.
% We are calculating to 80000 to generate entire half-range.
% We stop calculating at 30000 because above that PWM DC, velocity is
% constant.

% Input coefficients of fit line. Below example uses order 2 poly fit
p1 =    4.747e-10;
p2 =    -7.765e-05;
p3 =    3.283;

% Calculate dV_dt range
for n = 1:length(PWM_vect)
    dV_dt(n) = p1*PWM_vect(n)^2 + p2*PWM_vect(n) + p3;
end

%% Then calculate range of dthe_dt values that we will see as robot moves through joint range, for these dV_dt values
% Extract min & max angles from robot
min_V = robot.Joint(3).PosLim.Min;
min_pos = raw2joint_at40gw(robot,[0,0,min_V,0]);
min_ang = min_pos(3);
max_V = robot.Joint(3).PosLim.Max;
max_pos = raw2joint_at40gw(robot,[0,0,max_V,0]);
max_ang = max_pos(3);

% Create linear vector of thetas
the_vect = min_ang:-10:max_ang;

dthe_struct = {}; % Empty struct to hold results
line = []; % Data element within struct

for n = 1:length(the_vect)
    the = the_vect(n); % Theta for this test
    dthe_struct{1,n} = the; % Set first element of struct to be theta
    jointPos = joint2raw_at40gw(robot,[0,0,the,0]);
    V_the = jointPos(3);
    line = [];
    for m = 1:length(dV_dt)
        dV = dV_dt(m);
        PWM = PWM_vect(m);
        d_rawPos = raw2joint_at40gw(robot,[0,0,(V_the + dV),0]);
        dthe_dt = d_rawPos(3)-the_vect(n);
        data = [dthe_dt,dV,PWM];
        line = [line;data];
    end
    dthe_struct{2,n} = line;
end

% Get max and min of dthe_dt calculated above, so we know what range to calculate on
dthe_max = mean(dthe_struct{2,1}(:,1)); % Set max & min to averages of a random curve - probably safe, but not robust.
dthe_min = mean(dthe_struct{2,1}(:,1));
for n = 1:length(dthe_struct)
    maxdV = max(dthe_struct{2,n}(:,1));
    mindV = min(dthe_struct{2,n}(:,1)); 
    if maxdV >= dthe_max
        dthe_max = maxdV;
    end
    if mindV <= dthe_min
        dthe_min = mindV;
    end
end

%% Generate matrices of the and dthe to create surface from
% Define grid precision as number of points acroos dthe and the ranges to
% calculate at
surfprec = 200;

% Generate linspace from max to -max of dthe. We do this instead of max to
% min so that we can simulate velocities in both directions. However, this
% is imprecise!!!
% TODO: Use actual measured data to generate a full dV_dt range, then use
% that to calculate the full dthe range.
dthe_range = max(abs(dthe_min),abs(dthe_max));
dthe_lin = linspace(-dthe_range,dthe_range,surfprec);

% Generate linspace from max to min of the joint range
the_lin = linspace(min_ang, max_ang,surfprec);

%% Iterate over dthe_lin and the_lin to generate dV vector
mesh_temp = [];
mesh_lin = [];
for n = 1:length(the_lin)
    the_temp = the_lin(n);
    rawpos = joint2raw_at40gw(robot,[0,0,the_temp,0]);
    for m = 1:length(dthe_lin)
        dthe_temp = dthe_lin(m);
        d_rawpos = joint2raw_at40gw(robot,[0,0,(the_temp + dthe_temp),0]);
        dV_temp = d_rawpos - rawpos;
        dV_temp = dV_temp(3);
        mesh_temp = [the_temp;dthe_temp;dV_temp];
        mesh_lin = [mesh_lin,mesh_temp];
    end
end

%% Optional: Flatten dV to max limits. Don't do this for fit generation - just for visualization
d_rawmax = 1.39; % Maximum raw velocity that system can achieve
d_rawmin = -1.39; % Minimum raw velocity that system can achieve
flatten = 0; % Switch to flatten mesh
if flatten
    for n = 1:length(mesh_lin)
        if mesh_lin(3,n) > d_rawmax
            mesh_lin(3,n) = d_rawmax;
        end
        if mesh_lin(3,n) < d_rawmin
            mesh_lin(3,n) = d_rawmin;
        end
    end
end
%% Reshape out_lin matrix to get matrices we can plot as a surface
dV_mat = reshape(mesh_lin(3,:),length(the_lin),length(dthe_lin));
dthe_mat = reshape(mesh_lin(2,:),length(the_lin),length(dthe_lin));
the_mat = reshape(mesh_lin(1,:),length(the_lin),length(dthe_lin));

%% Plot surface scatter plot with these three matrices
figure;
mesh(the_mat,dthe_mat,dV_mat)
xlabel('Theta')
ylabel('d\_Theta')
zlabel('d\_V')

%% Generate curve fit
% At this point, you need to use cftool to generate a curve fit to the
% surface. JL has found that a polynomial surface that is degree 3 in both
% X (theta) and Y (d_theta) works well.

%{
Model for J3, found as of 2016-06-22:

Linear model Poly33:
     f(x,y) = p00 + p10*x + p01*y + p20*x^2 + p11*x*y + p02*y^2 + p30*x^3 + p21*x^2*y 
                    + p12*x*y^2 + p03*y^3
Coefficients (with 95% confidence bounds):
       p00 =   -0.008501  (-0.009817, -0.007185)
       p10 =    0.000809  (0.0007638, 0.0008543)
       p01 =     -0.2209  (-0.221, -0.2207)
       p20 =   2.403e-05  (2.189e-05, 2.617e-05)
       p11 =   0.0002211  (0.0002166, 0.0002256)
       p02 =    0.000196  (0.0001856, 0.0002064)
       p30 =   8.846e-08  (5.885e-08, 1.181e-07)
       p21 =   2.497e-05  (2.489e-05, 2.506e-05)
       p12 =   2.542e-05  (2.514e-05, 2.57e-05)
       p03 =   8.814e-06  (7.753e-06, 9.876e-06)

%}

