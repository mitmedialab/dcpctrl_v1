function [ pwm, err ] = draw2pwm_j3( robot, d_raw )
%DRAW2PWM_J3 Convert J3 sensor velocities to PWM output (for LabJack T7)
% OUTPUT:
%   pwm - PWM signals
%   err - 0 : no error
%         1 : Clamping J3 at knee, could affect simultaneous joint movements

%{
    [ pwm ] = draw2pwm_j3( robot, d_raw )
    Julian Leland, MIT Media Lab, 2016-06-22
%}

%% Get PWM max and min limits
pwm_max = robot.Joint(3).PWMLim.Max;
pwm_min = robot.Joint(3).PWMLim.Min;
pwm_dbup = robot.Joint(3).PWMLim.DBUpper;
pwm_dblo = robot.Joint(3).PWMLim.DBLower;
pwm_zero = robot.PWMZero;

% Clamp at knee
%1.39 > 
err = 0;
if any(d_raw > robot.Joint(3).VelMap.PosKnee) || any(d_raw < robot.Joint(3).VelMap.NegKnee)
    %disp('Warning: Clamping J3 at knee, could affect simultaneous joint movements');
    err = 1;
end
d_raw = max(min(d_raw, robot.Joint(3).VelMap.PosKnee), robot.Joint(3).VelMap.NegKnee);

pwm = zeros(size(d_raw));
%% Define mappings

posvellim = robot.Joint(3).VelMap.PosVelLim;
negvellim = robot.Joint(3).VelMap.NegVelLim;

posvellim = 0.095;
negvellim = -0.055;

%posvellim = 0;
%negvellim = 0;

posthrottle = d_raw > posvellim;
a =   robot.Joint(3).VelMap.A_pos;
b =   robot.Joint(3).VelMap.B_pos;
c =   robot.Joint(3).VelMap.C_pos;
pwm(posthrottle) = a.*d_raw(posthrottle).^b+c;

negthrottle = d_raw < negvellim;
p1 =  robot.Joint(3).VelMap.A_neg;
p2 =  robot.Joint(3).VelMap.B_neg;
p3 =  robot.Joint(3).VelMap.C_neg;
pwm(negthrottle) = p1.*d_raw(negthrottle).^2 + p2.*d_raw(negthrottle) + p3;

zer = ~(posthrottle | negthrottle);
pwm(zer) = pwm_zero;

% Further PWM-side clamping (necessary?)
pwm = min(max(pwm, pwm_min), pwm_max);
   
end

