%%Control AT40 Arm with Cartesian Xbox commands
function [] = arm_cart_control(ljh, robot, State)


RightThumbDead = double(State.Gamepad.RightThumbDeadZone);
LeftThumbDead = double(State.Gamepad.LeftThumbDeadZone);
JoystickMax = 32768;

CartVelMax = 100;
handle = ljh;

    %Read Controller State and set Vels to zero
    pwms = [robot.PWMZero,robot.PWMZero,robot.PWMZero,robot.PWMZero];
    CartVel = [0,0,0];
    
    %Map Right Stick X axis to Cartesian Velocity X
    if double(State.Gamepad.RightThumbX) > RightThumbDead;
        CartVel(2) = mapRange(double(State.Gamepad.RightThumbX), RightThumbDead, JoystickMax, 0 , CartVelMax);
    end
    if double(State.Gamepad.RightThumbX) < -1*RightThumbDead;
        CartVel(2) = mapRange(double(State.Gamepad.RightThumbX), -1*RightThumbDead, -1*JoystickMax, 0 , -1*CartVelMax);
    end
    
    %Map Right Stick Y axis to Cartesian Velocity Y
    if double(State.Gamepad.RightThumbY) > RightThumbDead;
        CartVel(1) = mapRange(double(State.Gamepad.RightThumbY), RightThumbDead, JoystickMax, 0 , CartVelMax);
    end
    if State.Gamepad.RightThumbY < -1*RightThumbDead;
        CartVel(1) = mapRange(double(State.Gamepad.RightThumbY), -1*RightThumbDead, -1*JoystickMax, 0 , -1*CartVelMax);
    end
    
    %Map Left Stick Y Axis to Cartesian Velocity Z
    if double(State.Gamepad.LeftThumbY) > LeftThumbDead;
        CartVel(3) = mapRange(double(State.Gamepad.LeftThumbY), LeftThumbDead, JoystickMax, 0 , CartVelMax);
    end
    if State.Gamepad.LeftThumbY < -1*LeftThumbDead;
        CartVel(3) = mapRange(double(State.Gamepad.LeftThumbY), -1*LeftThumbDead, -1*JoystickMax, 0 , -1*CartVelMax);
    end
    
    
    %Get Joint Raw Positions
    [ jRawPos, errors ] = getrawpos_at40gw( handle, robot );
    jPos = raw2joint_at40gw(robot, jRawPos);
    
    %Get Desired Joint Velocities then Convert to Raw Joint Velocities
    [ JointVel ] = cartvel2jointvel_at40gw( jPos, CartVel);
    [RawJointVel] = jointvel2rawvel_at40gw( robot, jPos, JointVel );
    
    
    
    %Convert Raw Vel to PWM and add J2 command
    pwms = rawvel2pwm_at40gw(robot, RawJointVel);
    
    %Control J2 the the Triggers (Right is up)
    if double(State.Gamepad.RightTrigger) > double(State.Gamepad.TriggerThreshold)
        display(double(State.Gamepad.RightTrigger))
        pwms(2) = mapRange(double(State.Gamepad.RightTrigger), double(State.Gamepad.TriggerThreshold), 255, robot.Joint(2).PWMLim.DBLower , robot.Joint(2).PWMLim.Min);
    end
    if double(State.Gamepad.LeftTrigger) > double(State.Gamepad.TriggerThreshold)
        pwms(2) = mapRange(double(State.Gamepad.LeftTrigger), double(State.Gamepad.TriggerThreshold), 255, robot.Joint(2).PWMLim.DBUpper , robot.Joint(2).PWMLim.Max);
    end
    
    %Send PWM Signals to AT40
    display(pwms)
    errors = setpwms_at40gw( ljh, robot, pwms );
    
end

        