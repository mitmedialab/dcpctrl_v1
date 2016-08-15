function [p, pd, pdd, t] = lspb3(q0, q1, vdes, tacc, dt)
% Custom lspb function, attempts to accomodate for a constant velocity
% controller. Currently only supports scalars
%
% WARNING: Though dt is an input, if that last timestamp of the trajectory
% is not divisible by dt, then we will still attach the last position and
% time anyway, but will clearly not be the same time step
%
% INPUTS:
%   q0, q1: Scalars, initial and goal positions
%   vdes: desired speed
%   tacc: time of acceleration phase
%   dt: resolution in sec for discretization

d = abs(q1 - q0);
dire = sign(q1-q0);
vdes = dire * abs(vdes);

% acceleration phase
dacc = abs(vdes * tacc);
a = vdes / tacc;

if dacc > d
    disp(['WARNING: Never reaches vdes of ' num2str(vdes) ' from ' num2str(q0) ' to ' num2str(q1)]);
end

% steady phase
dv = d - dacc;
tv = dv / abs(vdes);

% total
tf = tv + 2*tacc;

% trajectory
t = [0:dt:tf]';
p = zeros(size(t));
pd = zeros(size(t));
pdd = zeros(size(t));

for i = 1:length(t)
    tt = t(i);
    
    if tt < tacc
        p(i) = q0 + a/2*tt^2; % integrate a twice
        pd(i) = a*tt;
        pdd(i) = a;
    elseif tt < tf - tacc
        p(i) = (q1+q0-vdes*tf)/2 + vdes*tt;
        pd(i) = vdes;
        pdd(i) = 0;
    else
        p(i) = q1 - a/2*tf^2 + a*tf*tt - a/2*tt^2;
        pd(i) = a*tf - a*tt;
        pdd(i) = -a;
    end
end

if t(end) ~= tf
    p(end+1) = q1;
    pd(end+1) = 0;
    pdd(end+1) = -a;
    t(end+1) = tf;
end

end