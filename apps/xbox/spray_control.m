function [] = spray_control(sprayhandle, State)

%RB Spray on, LB Spray off
    if ButtonStates.RightBumper;
        fullspray(sprayhandle);
    end
    if ButtonStates.LeftBumper;
        stopspray(sprayhandle);
    end   
end