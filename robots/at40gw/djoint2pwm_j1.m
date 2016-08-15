function [ pwm ] = djoint2pwm_j1( robot, the, d_the)
%DJOINT2PWM_J1 Convert joint angular velocity to PWM for J1.

%{
    [ pwm ] = djoint2pwm_j3( robot, the, d_the )
    Julian Leland, MIT Media Lab, 2016-06-22
    Note that joint angle is requested (to make usage similar to other
    joints), but is not used
%}

% Convert joint velocity to sensor (raw) velocity
d_raw = djoint2draw_j1(robot,the,d_the)

% Convert raw velocity to PWM
pwm = draw2pwm_j1(robot,d_raw);
end

