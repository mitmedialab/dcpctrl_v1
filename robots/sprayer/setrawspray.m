function [ errs ] = setrawspray( sprayhandle, value )
%SET_SPRAYER Summary of this function goes here
%   Detailed explanation goes here

% Attempt to write value for setting spray
errs = 0;
try
    fprintf(sprayhandle,'%d\n',value);
catch
    errs = 1;
end

end