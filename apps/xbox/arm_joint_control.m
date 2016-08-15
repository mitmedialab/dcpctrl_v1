function [] = arm_joint_control(ljh, robot, State)

    pwms = [robot.PWMZero,robot.PWMZero,robot.PWMZero,robot.PWMZero];
    RightThumbDead = double(State.Gamepad.RightThumbDeadZone);
    LeftThumbDead = double(State.Gamepad.LeftThumbDeadZone);
    JoystickMax = 32768;
    
    %Control Joint one with Right Stick X axis
    if double(State.Gamepad.RightThumbX) > RightThumbDead;
        pwms(1) = mapRange(double(State.Gamepad.RightThumbX), RightThumbDead, JoystickMax, robot.Joint(1).PWMLim.DBLower , robot.Joint(1).PWMLim.Min);
    end
    if double(State.Gamepad.RightThumbX) < -1*RightThumbDead;
        pwms(1) = mapRange(double(State.Gamepad.RightThumbX), -1*RightThumbDead, -1*JoystickMax, robot.Joint(1).PWMLim.DBUpper , robot.Joint(1).PWMLim.Max);
    end
    
    %Control J2 the the Triggers (Right is up)
    if double(State.Gamepad.RightTrigger) > double(State.Gamepad.TriggerThreshold)
        pwms(2) = mapRange(double(State.Gamepad.RightTrigger), double(State.Gamepad.TriggerThreshold), 255, robot.Joint(2).PWMLim.DBLower , robot.Joint(2).PWMLim.Min);
    end
    if double(State.Gamepad.LeftTrigger) > double(State.Gamepad.TriggerThreshold)
        pwms(2) = mapRange(double(State.Gamepad.LeftTrigger), double(State.Gamepad.TriggerThreshold), 255, robot.Joint(2).PWMLim.DBUpper , robot.Joint(2).PWMLim.Max);
    end
    %Control Joint 3 with Right stick Y Axis
    if double(State.Gamepad.RightThumbY) > RightThumbDead;
        pwms(3) = mapRange(double(State.Gamepad.RightThumbY), RightThumbDead, JoystickMax, robot.Joint(3).PWMLim.DBLower , robot.Joint(3).PWMLim.Min);
    end
    if State.Gamepad.RightThumbY < -1*RightThumbDead;
        pwms(3) = mapRange(double(State.Gamepad.RightThumbY), -1*RightThumbDead, -1*JoystickMax, robot.Joint(3).PWMLim.DBUpper , robot.Joint(3).PWMLim.Max);
    end
    
    %Control Joint 4 with Left Stick Y axis
    if double(State.Gamepad.LeftThumbY) > LeftThumbDead;
        pwms(4) = mapRange(double(State.Gamepad.LeftThumbY), LeftThumbDead, JoystickMax, robot.Joint(4).PWMLim.DBLower , robot.Joint(4).PWMLim.Min);
    end
    if State.Gamepad.LeftThumbY < -1*LeftThumbDead;
        pwms(4) = mapRange(double(State.Gamepad.LeftThumbY), -1*LeftThumbDead, -1*JoystickMax, robot.Joint(4).PWMLim.DBUpper , robot.Joint(4).PWMLim.Max);
    end
       
    %Send PWM Signals to AT40
    errors = setpwms_at40gw( ljh, robot, pwms );
    display(pwms);
end