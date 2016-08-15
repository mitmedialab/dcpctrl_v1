function [ errs ] = closespray( sprayhandle )
%CLOSESPRAY Closes file handler for the sprayer serial port

errs = 0;
try
    fclose(sprayhandle);
    fclose(instrfindall);
    delete(instrfindall);
catch
    errs = 1;
end

end

