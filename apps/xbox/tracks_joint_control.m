function [] = tracks_joint_control(ljh, robot, State)

treads = [2.5,2.5];
LeftThumbDead = double(State.Gamepad.LeftThumbDeadZone);
RightThumbDead = double(State.Gamepad.RightThumbDeadZone);
JoystickMax = 32768;


if double(State.Gamepad.LeftThumbY) > LeftThumbDead;
    treads(1) = mapRange(double(State.Gamepad.LeftThumbY), LeftThumbDead, JoystickMax, 2.5 , 0);
end

if double(State.Gamepad.LeftThumbY) < -1*LeftThumbDead;
    treads(1) = mapRange(double(State.Gamepad.LeftThumbY), -1*LeftThumbDead, -1*JoystickMax, 2.5 , 5);
end

if double(State.Gamepad.RightThumbY) > RightThumbDead;
    treads(2) = mapRange(double(State.Gamepad.RightThumbY), RightThumbDead, JoystickMax, 2.5 , 0);
end

if double(State.Gamepad.RightThumbY) < -1*RightThumbDead;
    treads(2) = mapRange(double(State.Gamepad.RightThumbY), -1*RightThumbDead, -1*JoystickMax, 2.5 , 5);
end
display(treads)
setauxout_at40gw(ljh,robot,[],treads)
end