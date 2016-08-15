function [ Y ] = segplot( X, S )
% SEGPLOT - plots the vector X in segments S

nsegs = max(S);
colors = hsv(nsegs);
colors = hsv(2);

figure;
hold on;
for i=1:max(S)
    s = find(S == i);
    plot(s,X(S==i,:),'Color',colors(mod(i,2)+1,:));
end
hold off;