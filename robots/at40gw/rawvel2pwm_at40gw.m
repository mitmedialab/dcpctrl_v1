function pwm = rawvel2pwm_at40gw(robot, qdraw)
% J1 Notes (DIESEL):
%   Vel = PWM * 3.787875e-01 + -3.517091e+04
%   MaxVel: 19260 counts/sec -- not right, using 22432/sec instead
% J3 Notes (DIESEL):
%   Rotating up: + voltage, -angular, <50 pwm
%   Vel = PWM * -3.208623e-05 + 2.248113e+00
%   MaxVel: 1.39 volts/sec -- not right, using 1.75 V/s instead
% J4 Notes (DIESEL):
%   VelExtend = PWM * -2.882588e-05 + 2.069631e+00
%   VelRetract = PWM * -1.741648e-05 + 1.535560e+00
% J1 Notes (ELECTRIC):
%   Vel = PWM * 0.3499 - 31977
% TODO: should we clamp inside this function or leave it to user higher up?

pwmlims = [robot.Joint.PWMLim];
pwmmax = [pwmlims.Max];
pwmmin = [pwmlims.Min];
pwmdbhi = [pwmlims.DBUpper];
pwmdblo = [pwmlims.DBLower];
pwmzero = [robot.Joint.PWM_zero];
rawthresh = [robot.Joint.moveThresh];

pwm = ones(size(qdraw)) .* robot.PWMZero;

% Joint 1
pwm(:,1) = draw2pwm_j1_linear(robot, qdraw(:,1));

% Joint 3
pwm(:,3) = draw2pwm_j3(robot, qdraw(:,3));

% Joint 4
pwm(:,4) = draw2pwm_j4(robot, qdraw(:,4));

% Naive clamping
%l = (pwmzero + pwmdblo)./2;
%u = (pwmzero + pwmdbhi)./2;
%pwmlz = bsxfun(@gt, pwm, l) & bsxfun(@lt, pwm, pwmzero);
%pwmll = bsxfun(@gt, pwm, pwmdblo) & bsxfun(@lt, pwm, l);
%pwmuz = bsxfun(@gt, pwm, pwmzero) & bsxfun(@lt, pwm, u);
%pwmuu = bsxfun(@gt, pwm, u) & bsxfun(@lt, pwm, pwmdbhi);
%pwm(pwmlz) = robot.PWMZero;
%pwm(pwmuz) = robot.PWMZero;

%for i = 1:4
%   pwm(pwmuu(:,i),i) = pwmdbhi(i);
%   pwm(pwmll(:,i),i) = pwmdblo(i);
%end

% Old code for J4. Works well enough, but new code should *probably* do a
% better job. 
% posi4 = qdraw(:,4) > 0;
% zer = qdraw(:,4) == 0;
% pwm(posi4,4) = mapRange(qdraw(posi4,4), 0, 0.89, robot.Joint(4).PWMLim.DBLower, robot.Joint(4).PWMLim.Min);
% pwm(~posi4,4) = mapRange(qdraw(~posi4,4), 0, -0.965, robot.Joint(4).PWMLim.DBUpper, robot.Joint(4).PWMLim.Max);
% pwm(zer,4) = robot.PWMZero;
% 
% pwm(:,4) = max(min(pwm(:,4), robot.Joint(4).PWMLim.Max), robot.Joint(4).PWMLim.Min);

    % TODO: possibly move these mappings into the configuration file
%     j1pwm = [];
%     j1raw = [];
%     j3pwm = [];
%     j3raw = [];
%     j4pwm = [5000 43000 63000 69000 90500 102500 132500 152500];
%     j4raw = [0.91 0.8872 0.2368 0.08389 -0.05196 -0.2605 -0.7854 -0.7703];
%     
%     pwm(:,1) = interp1(qdraw(:,1), j1raw, j1pwm, 'linear','extrapolate');
%     pwm(:,3) = interp1(qdraw(:,3), j3raw, j3pwm, 'linear','extrapolate');
%     pwm(:,4) = interp1(qdraw(:,4), j4raw, j4pwm, 'linear','extrapolate');
%     
%     pwmlims = [robot.Joint.PWMLim];
%     pwmmax = [pwmlims.Max];
%     pwmmin = [pwmlims.Min];
%     pwm = clamp(pwm, pwmmin, pwmmax);
end
