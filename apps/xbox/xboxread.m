%% Sample Code for Xbox 360 Controller for Windows

% Visit http://sharpdx.org/documentation/api for more API details

controllerLibrary = NET.addAssembly([pwd '\apps\xbox\SharpDX.XInput.dll']);
myController = SharpDX.XInput.Controller(SharpDX.XInput.UserIndex.One);

VibrationLevel = SharpDX.XInput.Vibration;



while true;
    State = myController.GetState();
    ButtonStates = ButtonStateParser(State.Gamepad.Buttons); % Put this into a structure
    %disp(State.Gamepad);
    disp(ButtonStates);
    disp(ButtonStates.RightBumper);
    VibrationLevel.LeftMotorSpeed = double(State.Gamepad.LeftTrigger) * 255;
    VibrationLevel.RightMotorSpeed = double(State.Gamepad.RightTrigger) * 255;
    Left = double(State.Gamepad.LeftTrigger);
    Right = double(State.Gamepad.RightTrigger);
    
    myController.SetVibration(VibrationLevel); % If your controller supports vibration
    
    
    
    if ButtonStates.Start;
        break
    end
    pause(.001);
end    

VibrationLevel.LeftMotorSpeed = 0;
VibrationLevel.RightMotorSpeed = 0;
myController.SetVibration(VibrationLevel); % If your controller supports vibration

