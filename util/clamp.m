function [ x_clamped ] = clamp( x, xmin, xmax )
%CLAMP Clamps input vector element-wise by values in vectors xmin and xmax
%   NOTE: This is vectorized, xmin and xmax can be scalars OR matrices that
%   match the size of x, x can be scalar, vector, or matrix
x_clamped = min(max(x, xmin), xmax);

end

