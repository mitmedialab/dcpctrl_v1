function [ handle ] = initlabjack(robot)
%INITLABJACK Initialize LabJack for use with AT40GW. Returns LabJack handle
%for use in future functions

% [ handle ] = initlabjack()

%   Julian Leland, MIT Media Lab, 2016-06-06
%   Function to initialize Labjack for use with AT40GW and return handle
%   for use in other functions. Function first disables all functions, then
%   sets up clock, PWM, analog & rotary encoder inputs, and external inputs
%   & outputs.

%% Initialize LabJack
ljasm = NET.addAssembly('LabJack.LJM'); % Make UD .NET assembly visible in MATLAB
handle = 0; % Number to select which one of the found LabJacks we should connect to (if more than one)

temp_MD = robot;

try
    [ljmerror, handle] = LabJack.LJM.OpenS('ANY','ANY','ANY',handle); % Open LabJack object. Syntax is (device type, comms type, device ID, handle). Can use numbers instead if you use .Open instead of .OpenS
    %showDeviceInfo(handle);
    
    % Disable all extended features
    for i = 1:4
        LabJack.LJM.eWriteName(handle,temp_MD.Joint(i).RegNames.PWM_En,0); % Set all channel EF to zero (disabled)
        if temp_MD.Joint(i).EncType == 'Rotary' % If a joint is rotary, disable those EFs too.
            LabJack.LJM.eWriteName(handle,temp_MD.Joint(i).RegNames.REnc_EnA,0);
            LabJack.LJM.eWriteName(handle,temp_MD.Joint(i).RegNames.REnc_EnB,0);
        end
    end
    
    % Set up clock
    LabJack.LJM.eWriteName(handle,'DIO_EF_CLOCK0_ENABLE',0); % Disable clock source
    LabJack.LJM.eWriteName(handle,'DIO_EF_CLOCK0_DIVISOR',1); % Set clock divisor to 1
    LabJack.LJM.eWriteName(handle,'DIO_EF_CLOCK0_ROLL_VALUE',160000); % Set clock roll value to 160000 -> 500 MHz clock
    LabJack.LJM.eWriteName(handle,'DIO_EF_CLOCK0_ENABLE',1); % Re-enable clock
    
    % Set PWM registers
    for i = 1:4
        LabJack.LJM.eWriteName(handle,temp_MD.Joint(i).RegNames.PWM_Indx,0); % Select index 0 = PWM output
        LabJack.LJM.eWriteName(handle,temp_MD.Joint(i).RegNames.PWM_Opts,0); % Set DIO to use clock source 0
        LabJack.LJM.eWriteName(handle,temp_MD.Joint(i).RegNames.PWM_CfgA, temp_MD.Joint(i).PWM_zero); % Set DC to 50%
    
    % Set AIN registers
        if temp_MD.Joint(i).EncType == 'Linear' % If a joint is linear, set up AIN
            LabJack.LJM.eWriteName(handle,temp_MD.Joint(i).RegNames.AIN_NegCh,199); % Set AIN negative channel to 199 (single-sided)
            LabJack.LJM.eWriteName(handle,temp_MD.Joint(i).RegNames.AIN_Range,10); % Set AIN range to +/- 10 V
            LabJack.LJM.eWriteName(handle,temp_MD.Joint(i).RegNames.AIN_ResIndx, 9); % Set AIN resolution index to 3 (default is 8)
    
    % Set REnc registers
        elseif temp_MD.Joint(i).EncType == 'Rotary' % Otherwise, set up as rotary joint
            LabJack.LJM.eWriteName(handle,temp_MD.Joint(i).RegNames.REnc_IndxA,10); % Set up appropriate pins as quad input
            LabJack.LJM.eWriteName(handle,temp_MD.Joint(i).RegNames.REnc_IndxB,10); 
        end
    end
    
    % Re-enable all extended features
    for i = 1:4
        LabJack.LJM.eWriteName(handle,temp_MD.Joint(i).RegNames.PWM_En,1); % Set all PWM channel EFs to 1 (enabled)
        if temp_MD.Joint(i).EncType == 'Rotary' % If a joint is rotary, enable those EFs too.
            LabJack.LJM.eWriteName(handle,temp_MD.Joint(i).RegNames.REnc_EnA,1);
            LabJack.LJM.eWriteName(handle,temp_MD.Joint(i).RegNames.REnc_EnB,1);
        end
    end
    
    % Set all digital & analog pins to zero
    for i = 1:10
        LabJack.LJM.eWriteName(handle,temp_MD.DigOut(i).RegNames.DIO_Name,0); % Write digital pin
    end
    for i = 1:2
        LabJack.LJM.eWriteName(handle,temp_MD.AnOut(i).RegNames.DAC_Name,0); % Write analog pin
    end
    
    % Set up auxiliary analog inputs
    for n = 1:2
        LabJack.LJM.eWriteName(handle,temp_MD.AnIn(i).RegNames.AIN_NegCh,199); % Set AIN negative channel to 199 (single-sided)
        LabJack.LJM.eWriteName(handle,temp_MD.AnIn(i).RegNames.AIN_Range,10); % Set AIN range to +/- 10 V
        LabJack.LJM.eWriteName(handle,temp_MD.AnIn(i).RegNames.AIN_ResIndx,5); % Set AIN resolution for auxilliary analog inputs ONLY. Range is 0 (fastest) to 15 (VERY slow)
    end

catch e
    %showErrorMessage(e)    
end