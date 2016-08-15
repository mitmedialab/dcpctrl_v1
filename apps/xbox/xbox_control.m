%% Sample Code for Xbox 360 Controller for Windows

% Visit http://sharpdx.org/documentation/api for more API details
% Joysticks range from 32768 to -32768


controllerLibrary = NET.addAssembly([pwd '\apps\xbox\SharpDX.XInput.dll']);
myController = SharpDX.XInput.Controller(SharpDX.XInput.UserIndex.One);
ljh = handle;
spray = 0;


if spray
    %Open serial connection to Arduino
    [sprayhandle,errs] = initspray();
    errs = stopspray(sprayhandle);
end

Mode = 1

while true;
    try
    State = myController.GetState();
    ButtonStates = ButtonStateParser(State.Gamepad.Buttons); % Put this into a structure

    if ButtonStates.DPadUp;
        Mode = 1
    end
    if ButtonStates.DPadRight;
        Mode = 2
    end
    if ButtonStates.DPadDown;
        Mode = 3
    end
    if ButtonStates.DPadLeft;
        Mode = 4
    end
        
    if Mode == 1
        arm_joint_control(ljh, robot, State);
    end
     
    if Mode == 2;
        tracks_joint_control(ljh, robot, State);
    end
    
    if Mode == 3;
        tracks_cart_control(ljh, robot, State);
    end
    
    if Mode == 4;
        arm_cart_control(ljh, robot, State);
    end
    
    if spray
        spray_control(sprayhandle,State);
    end
    
    %Emergency Stop
    if ButtonStates.Start
        setauxout_at40gw(handle,robot,[],[2.5,2.5]);
        setpwms_at40gw( ljh, robot, [robot.PWMZero,robot.PWMZero,robot.PWMZero,robot.PWMZero] );
        if spray
            closespray(sprayhandle);
        end
        k = waitforbuttonpress;
    end
    
    
    pause(.001);

    
    catch error
    setauxout_at40gw(handle,robot,[],[2.5,2.5])
    setpwms_at40gw( ljh, robot, [robot.PWMZero,robot.PWMZero,robot.PWMZero,robot.PWMZero] )
    if spray
        closespray(sprayhandle)
    end
    display(error)
    pause(2);
    myController = SharpDX.XInput.Controller(SharpDX.XInput.UserIndex.One);   
    display('reconnecting')
end    
 
end
%Stop all joint motion and close spray
setauxout_at40gw(handle,robot,[],[2.5,2.5]);
errors = setpwms_at40gw( ljh, robot, [robot.PWMZero,robot.PWMZero,robot.PWMZero,robot.PWMZero] );
closespray(sprayhandle);