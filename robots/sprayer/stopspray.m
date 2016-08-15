function [ errs ] = stopspray( sprayhandle )
%STOPSPRAY Completely stops spray

    errs = setrawspray(sprayhandle, 0);

end

