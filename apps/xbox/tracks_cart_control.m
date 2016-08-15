function [] = tracks_cart_control(ljh, robot, State)
treads = [2.5,1];
Forwards = 1;
Right = 1;
LeftThumbDead = double(State.Gamepad.LeftThumbDeadZone);
RightThumbDead = double(State.Gamepad.RightThumbDeadZone);
JoystickMax = 32768;

    if double(State.Gamepad.LeftThumbY) > LeftThumbDead;
        treads(1) = mapRange(double(State.Gamepad.LeftThumbY), LeftThumbDead, JoystickMax, 2.5 , 0);
        Forwards = 1;
    end
    if State.Gamepad.LeftThumbY < -1*LeftThumbDead;
        treads(1) = mapRange(double(State.Gamepad.LeftThumbY), -1*LeftThumbDead, -1*JoystickMax, 2.5 , 5);
        Forwards = 0;
    end
    
    if double(State.Gamepad.RightThumbX) > RightThumbDead;
        treads(2) = mapRange(double(State.Gamepad.RightThumbX), RightThumbDead, JoystickMax, 1 , 0);
        Right = 1;
    end
    if State.Gamepad.RightThumbX < -1*RightThumbDead;
        treads(2) = mapRange(double(State.Gamepad.RightThumbX), -1*RightThumbDead, -1*JoystickMax, 1 , 0);
        Right = 0;
    end
    
    KillRight = (Right == Forwards);
    
    if KillRight;
        setauxout_at40gw(ljh,robot,[],[treads(1),2.5+(treads(1)-2.5)*treads(2)]);
        display([treads(1),2.5+(treads(1)-2.5)*treads(2)])
    else
        setauxout_at40gw(ljh,robot,[],[(treads(1)-2.5)*treads(2)+2.5,treads(1)]);
        display([(treads(1)-2.5)*treads(2)+2.5,treads(1)])
    end
    end