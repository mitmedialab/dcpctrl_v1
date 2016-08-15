function x = lpfilter(x_prev, x_curr, alpha)
% LPFILTER implements a first-order low-pass digital filter
%   (also known as an exponential smoothing), default alpha is 0.97, but
%   can be easily changed. x_prev, x_curr, and alpha can be vectors where each
%   value represents an independent separate channel to be filted
% See slide 15 of http://medesign.seas.upenn.edu/uploads/Courses/510-11C-L13.2.pdf
% TODO, rather than take in alpha, compute it given a frequency cutoff
% f (rad/sec) = 1/tau = (1 - alpha) / (alpha * dt)

if nargin < 3
    if isscalar(x_prev)
        alpha = 0.97;
    else
        alpha = zeros(size(x_prev)) .* 0.97;
    end

end

x = alpha .* x_prev + (1 - alpha) .* x_curr;

end