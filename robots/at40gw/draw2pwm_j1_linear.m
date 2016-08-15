function [ pwm, err ] = draw2pwm_j1_linear( robot, d_raw  )
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
movethresh = robot.Joint(1).moveThresh;

% Get upper and lower knee limits
knee_upper = robot.Joint(1).VelMap.PosKnee; % Highest possible sensor velocity value
knee_lower = robot.Joint(1).VelMap.NegKnee; % Lowest possible sensor velocity value

pwm = zeros(size(d_raw));

posi = d_raw > 10;
nega = d_raw < -10;
zer = d_raw >= -10 & d_raw <= 10;

pwm(posi) = mapRange(d_raw(posi),10,knee_upper,pwm_dbup,pwm_max);
pwm(nega) = mapRange(d_raw(nega),-10,knee_lower,pwm_dblo,pwm_min);
pwm(zer) = robot.PWMZero;
pwm = min(max(pwm, pwm_min), pwm_max);

end