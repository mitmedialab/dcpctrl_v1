function [ pwm, err ] = draw2pwm_j1( robot, d_raw  )
%DRAW2PWM_J1 Convert J1 sensor velocities to PWM output (for LabJack T7)
% INPUTS
%   d_raw - 1xN or Nx1 vector of raw joint velocities (V/s or counts/s)
% OUTPUT:
%   pwm - PWM signals
%   err - 0 : no error
%            1 : Clamping J4 at knee, could affect simultaneous joint movements


%{
    [ pwm ] = draw2pwm_j1( robot, d_raw )
    Julian Leland, MIT Media Lab, 2016-06-22
%}

% Get PWM max and min limits
pwm_max = robot.Joint(1).PWMLim.Max;
pwm_min = robot.Joint(1).PWMLim.Min;
pwm_dbup = robot.Joint(1).PWMLim.DBUpper;
pwm_dblo = robot.Joint(1).PWMLim.DBLower;
pwm_zero = robot.PWMZero;

% Get upper and lower knee limits
knee_upper = robot.Joint(1).VelMap.PosKnee; % Highest possible sensor velocity value
knee_lower = robot.Joint(1).VelMap.NegKnee; % Lowest possible sensor velocity value

% Clamping at knee
err = 0;
if any(d_raw < knee_lower | d_raw > knee_upper)
    %disp('WARNING: Clamping J1 PWMs, might throw off coordinations');
    err = 1;
end

d_raw = min(max(d_raw, knee_lower), knee_upper);

pwm = zeros(size(d_raw));

% TODO: Reintroduce these limits?
posvellim = robot.Joint(1).VelMap.PosVelLim;
negvellim = robot.Joint(1).VelMap.NegVelLim;

posvellim = 0;
negvellim = 0;

% Positive mapping
posi = d_raw > posvellim;
p1 =  robot.Joint(1).VelMap.A_pos;
p2 =  robot.Joint(1).VelMap.B_pos;
p3 =  robot.Joint(1).VelMap.C_pos;
p4 =  robot.Joint(1).VelMap.D_pos;
pwm(posi) = p1.*d_raw(posi).^3 + p2.*d_raw(posi).^2 + p3.*d_raw(posi) + p4;

% Negative mapping
nega = d_raw < negvellim;
p1 =   robot.Joint(1).VelMap.A_neg;
p2 =   robot.Joint(1).VelMap.B_neg;
p3 =   robot.Joint(1).VelMap.C_neg;
p4 =   robot.Joint(1).VelMap.D_neg;
pwm(nega) = p1.*d_raw(nega).^3 + p2.*d_raw(nega).^2 + p3.*d_raw(nega) + p4;

% Zero mapping
zer = (d_raw >= negvellim & d_raw <= posvellim);
%zer = abs(d_raw) < eps;
pwm(zer) = pwm_zero;

% Further PWM-side clamping (necessary?)
pwm = min(max(pwm, pwm_min), pwm_max);

end

