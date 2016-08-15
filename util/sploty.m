function [ Y ] = sploty( X, Y, Y1 )
%SPLOT Plots a bunch of subplots of the input
% INPUT:
%   X - Nx1 vector containing X values
%   Y - NxM matrix, N data points, M channels

nchannels = size(Y,2);
npoints = size(Y,1);

figure;
for i = 1:nchannels
    subplot(nchannels,1,i);
    hold on;
    plot(X, Y(:,i));
    if nargin > 2
        plot(X, Y1(:,i));
    end
    hold off;
end

end

