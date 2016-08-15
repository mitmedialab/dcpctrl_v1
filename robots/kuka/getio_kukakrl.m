function [ value, errs ] = getio_kukakrl( handle, inchannel )
%SETIO_KUKAKRL Get the pin value of the labjack digital in channel corresponding to a
%KUKA KRL command

[errs, value] = LabJack.LJM.eWriteName(handle, temp_MD.DigIn(inchannel).RegNames.DIO_Name, 0);

end
