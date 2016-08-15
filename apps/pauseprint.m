function [ stopflag ] = pauseprint(value)
%PAUSEPRINT Summary of this function goes here
%   Detailed explanation goes here
    global stopflag;
    
    stopflag = value;
    
    if stopflag == 1
        disp('Pausing...');
    end
end

