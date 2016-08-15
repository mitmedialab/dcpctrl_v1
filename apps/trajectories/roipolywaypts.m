simflag = 1;

im = imread('/Users/Damien Martin/Desktop/mediated_matter.jpg');
im = imresize(im,[1000 NaN]);
[~,~,~,xi,yi] = roipoly(im);
[~,~,~,xj,yj] = roipoly(im);

xi = [xi;xj]
yi = [yi;yj]

imwaypts = [zeros(size(xi)) xi -yi];

% Scale down, translate
if simflag
    load('data/homerawpos');
    q0raw = homerawpos;
else
    q0raw = getrawpos_at40gw(handle, robot);
end

x0 = joint2cart_at40gw(raw2joint_at40gw(robot, q0raw));
waypts = bsxfun(@plus, imwaypts, x0 - imwaypts(1,:));