function [ Y ] = splot( X, X1, X2, X3 )
%SPLOT Plots a bunch of subplots of the input
% INPUT:
%   X - NxM matrix, N data points, M channels
% TODO: Actually have this handle Xn...

nchannels = size(X,2);
npoints = size(X,1);

figure;
for i = 1:nchannels
    subplot(nchannels,1,i);
    hold on;
    plot(1:npoints, X(:,i),'b');

    if nargin > 1
        plot(1:npoints, X1(:,i),'r');
    end
    
    if nargin > 2
        plot(1:npoints, X2(:,i),'m');
    end
    
    if nargin > 3
        plot(1:npoints, X3(:,i),'g');
    end
    
    hold off;
end
disp('Legend (color in order of input): b, r, m, g')

end

