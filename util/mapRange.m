function [ out_value ] = mapRange(x, in_min, in_max, out_min, out_max )
%MAPRANGE Map value from one range to a second range.

% [ out_value ] = mapRange(x, in_min, in_max, out_min, out_max )
%   Julian Leland, MIT Media Lab, 2016-06-07
%   Exact knockoff of Arduino map function - see here for details: 
%   https://www.arduino.cc/en/Reference/Map
%   INPUTS:
%   x - input value to be mapped. May be a vector.
%   in_min - minumum value in range that is being mapped from
%   in_max - maximum value in range that is being mapped from
%   out_min - minimum value in range that is being mapped to
%   out_max - maximum value in range that is being mapped to
%
%   Example usage:
%   foo = [100;5;3]
%   bar = mapRange(foo,0,100,500,1000)
%   bar = [1000;525;515]

out_value = ((x-in_min).*(out_max-out_min))./(in_max-in_min)+out_min;

end

