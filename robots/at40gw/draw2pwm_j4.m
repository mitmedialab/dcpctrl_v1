function [ pwm, err ] = draw2pwm_j4( robot, d_raw )
%DRAW2PWM_J4 Convert J4 sensor velocities to PWM output (for LabJack T7)
% OUTPUT:
%   pwm - PWM signals
%   err - 0 : no error
%         1 : Clamping J4 at knee, could affect simultaneous joint movements

%{
    [ pwm ] = draw2pwm_j4( robot, d_raw )
    Julian Leland, MIT Media Lab, 2016-06-27
%}

%% Get PWM max and min limits
pwm_max = robot.Joint(4).PWMLim.Max;
pwm_min = robot.Joint(4).PWMLim.Min;
pwm_dbup = robot.Joint(4).PWMLim.DBUpper;
pwm_dblo = robot.Joint(4).PWMLim.DBLower;
pwm_zero = robot.PWMZero;

% Clamp at knee
%1.39 > 
err = 0;
if any(d_raw > robot.Joint(4).VelMap.PosKnee) || any(d_raw < robot.Joint(4).VelMap.NegKnee)
    err = 1;
    %disp('Warning: Clamping J4 at knee, could affect simultaneous joint movements');
end
d_raw = max(min(d_raw, robot.Joint(4).VelMap.PosKnee), robot.Joint(4).VelMap.NegKnee);

pwm = zeros(size(d_raw));

%% Define mappings

posvellim = robot.Joint(4).VelMap.PosVelLim;
negvellim = robot.Joint(4).VelMap.NegVelLim;

posthrottle = d_raw > posvellim;
p1 =   robot.Joint(4).VelMap.A_pos;
p2 =   robot.Joint(4).VelMap.B_pos;
p3 =   robot.Joint(4).VelMap.C_pos;
pwm(posthrottle) = p1.*d_raw(posthrottle).^2 + p2.*d_raw(posthrottle) + p3;

negthrottle = d_raw < negvellim;
p1 =  robot.Joint(4).VelMap.A_neg;
p2 =  robot.Joint(4).VelMap.B_neg;
pwm(negthrottle) = p1.*d_raw(negthrottle) + p2;

zer = ~(posthrottle | negthrottle);
pwm(zer) = pwm_zero;

% Further PWM-side clamping (necessary?)
pwm = min(max(pwm, pwm_min), pwm_max);


end

