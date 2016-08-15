function [ pwm ] = djoint2pwm_j3( robot, the, d_the)
%DJOINT2PWM_J3 Convert joint angular velocity to PWM for J3 at a given
%joint angle

%{
    [ pwm ] = djoint2pwm_j3( robot, the, d_the )
    Julian Leland, MIT Media Lab, 2016-06-22
%}

% Convert joint velocity to sensor (raw) velocity
d_raw = djoint2draw_j3(robot,the,d_the);

% Convert raw velocity to PWM
pwm = draw2pwm_j3(robot,d_raw);


end

