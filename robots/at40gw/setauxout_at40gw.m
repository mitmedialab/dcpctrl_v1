function [] = setauxout_at40gw(handle,robot,Dx_vect,Ax_vect)
%SETAUXOUT_AT40GW Set auxilliary (digital and analog) outputs on AT40GW
%
%{  
    [] = setauxout(handle,robot,Dx_vect,Ax_vect)
    Julian Leland, MIT Media Lab, 2016-06-20
    setauxout_at40gw sets the analog and digital auxiliary outputs on the
    AT40GW.
    INPUTS:
    - handle: LabJack handle
    - robot: Robot description struct
    - Dx_vect: 1x10 vector of digital output values. Values may be 1 or 0.
    Entire vector may be left empty optionally to skip setting entirely.
    Individual digital outputs may be set to -1 to preserve state.
    - Ax_vect: 1x2 vector of analog output values. Values may be between 0
    and 5. Entire vector may be left empty optionally to skip setting entirely.
    Individual digital outputs may be set to -1 to preserve state.

    OUTPUTS: None (currently)
%}

if isempty(Dx_vect) ~= 1
    for n = 1:10
        if Dx_vect(n) ~= -1
            LabJack.LJM.eWriteName(handle,robot.DigOut(n).RegNames.DIO_Name,Dx_vect(n)); % Write digital pin
        end
    end
end
if isempty(Ax_vect) ~= 1
    for n = 1:2
        if Ax_vect(n) ~= -1
            LabJack.LJM.eWriteName(handle,robot.AnOut(n).RegNames.DAC_Name,Ax_vect(n)); % Write analog pin
        end
    end
end

end

