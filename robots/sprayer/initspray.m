function [ sprayhandle, errs ] = initspray()
%INITSPRAY Opens serial communication with sprayer

try
    fclose(instrfindall);
    delete(instrfindall);
catch
end

errs = 0;
sprayhandle = [];
availports = instrhwinfo('serial');

if isempty(availports.SerialPorts)
    errs = 1;
    return;
end

serPort = availports.SerialPorts{1};
sprayhandle = serial(serPort,'BaudRate',9600);
fopen(sprayhandle);

end

