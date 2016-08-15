function [ errs ] = setio_kukakrl( handle, outchannel, value )
%SETIO_KUKAKRL Set the digital output value of the labjack corresponding to a
%KUKA KRL command

errs = LabJack.LJM.eWriteName(handle, temp_MD.DigOut(outchannel).RegNames.DIO_Name, value);

end

