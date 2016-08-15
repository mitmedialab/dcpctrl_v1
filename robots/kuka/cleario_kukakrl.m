function [ errs ] = cleario_kukakrl( handle, robot )
%SETIO_KUKAKRL Set the digital output value of the labjack corresponding to a
%KUKA KRL command

err5 = LabJack.LJM.eWriteName(handle, robot.DigOut(5).RegNames.DIO_Name, 0);
err8 = LabJack.LJM.eWriteName(handle, robot.DigOut(8).RegNames.DIO_Name, 0);
err9 = LabJack.LJM.eWriteName(handle, robot.DigOut(9).RegNames.DIO_Name, 0);
err10 = LabJack.LJM.eWriteName(handle, robot.DigOut(10).RegNames.DIO_Name, 0);

if nargout > 0
    errs = {err5; err8; err9; err10};
end
end