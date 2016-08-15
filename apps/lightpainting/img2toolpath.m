% IMG2TOOLPATH Converts an image to a toolpath for the AT40GW

%{
    Julian Leland, MIT Media Lab, 2016-08-02
%}

%% Setup
addpath('apps/lightpainting/');

%% Import image and convert to grayscale
in_img = imread('mm_only.png','png');
%in_img = rgb2gray(in_img);
bw_img = edge(in_img,'canny');
imshow(bw_img);

%% Define desired output size of image
asr = size(bw_img,1)/size(bw_img,2); % Aspect ratio of image
imh = 1000; % Height of image, mm.
imw = imh/asr;

xpos = 0; % Define x position of output image

%% Extract row & column indices for white points in image
out_img = [];
[row,col] = find(bw_img == 1);
out_img = [zeros(length(row),1),col,row];

%% Map image onto desired size range
out_img(:,2) = mapRange(out_img(:,2),min(out_img(:,2)),max(out_img(:,2)),0,imw);
out_img(:,3) = mapRange(out_img(:,3),max(out_img(:,3)),min(out_img(:,3)),0,imh);

%% OPTIONAL: Plot each point in sequence to see what the system is going to print
scatter3(out_img(:,1),out_img(:,2),out_img(:,3),[],hsv(length(out_img)));
hold on;
plot3(out_img(1,1),out_img(1,2),out_img(1,3),'mx');
hold off;
%{
figure;
hold on;
for n = 1:length(out_img)
    scatter3(out_img(n,1),out_img(n,2),out_img(n,3),'b');
    drawnow;
end
hold off;
%}

%% Set start point and re-order points to draw in sequence
% Method from http://stackoverflow.com/questions/11631934/sort-coordinates-points-in-matlab
dist = pdist2(out_img,out_img);

N = size(out_img,1);
result = NaN(1,N);
result(1) = 1;

for n = 2:N
    dist(:,result(n-1)) = Inf;
    [~, closest_idx] = min(dist(result(n-1),:));
    result(n) = closest_idx;
end

result = result';

%% OPTIONAL: Plot each point in sequence to make sure that system is filtering correctly
%{
figure;
hold on;
for n = 1:length(out_img)
    scatter3(out_img(result(n),1),out_img(result(n),2),out_img(result(n),3),'b');
    drawnow;
end
hold off;
%}

%% Sort out_img by result
out_img_sorted = [];
for n = 1:length(out_img)
    out_img_sorted = [out_img_sorted;out_img(result(n),:)];
end

%% OPTIONAL: Plot each point in sequence to make sure that system is filtering correctly
%{
figure;
view([60,20]);
hold on;
for n = 1:length(out_img_sorted)
    scatter3(out_img_sorted(n,1),out_img_sorted(n,2),out_img_sorted(n,3),'b');
    drawnow;
end
hold off;
%}

%% Identify mean distance between points, and use this to set flags indicating when new section of image has been reached
segflag = zeros(length(out_img_sorted),1);
%meandist = mean(diff(out_img_sorted));
ptdist = [0];
for n = 1:length(out_img_sorted)-1
    ptdist = [ptdist,pdist2(out_img_sorted(n,:),out_img_sorted(n+1,:))];
end
ptdist = ptdist';
meandist = mean(ptdist);
dist_filt = 5; % Distances between points must be greater than dist_filt*meandist to trigger a new segment
for n = 1:length(out_img_sorted)
    if ptdist(n) >= meandist*dist_filt % If the current distance between points is greater than the mean distance between points
        segflag(n) = 1;
    end
end

out_img_sorted = [out_img_sorted,ptdist,segflag];

%% Pick every qth point to actually draw (reduce density of points)
q = 20; % Pick every qth waypoint
out_img_sm = [];
for n = 1:length(out_img_sorted)
    if mod(n,q) == 0
        out_img_sm = [out_img_sm;out_img_sorted(n,:)];
    end
    if out_img_sorted(n,5) == 1
        out_img_sm = [out_img_sm;out_img_sorted(n,:)];
    end
end

%% OPTIONAL: Plot each point in sequence to make sure that system is filtering correctly
%%{
figure;
view([60,20]);
hold on;
for n = 1:length(out_img_sm)
    scatter3(out_img_sm(n,1),out_img_sm(n,2),out_img_sm(n,3),'b');
    drawnow;
end
hold off;
%}

%% Split up into subsegments with index
subsegts = {[]};
segt_ct = 1;
for n = 1:length(out_img_sm)
    if out_img_sm(n,5) == 1
        disp('New segment!');
        segt_ct = segt_ct + 1;
        subsegts{segt_ct} = [];
    end
    subsegts{segt_ct} = [subsegts{segt_ct};out_img_sm(n,:)];
end

%% For each subsegment, add the first point back in to the end to close each segment
for n = 1:length(subsegts)
    subsegts{n} = [subsegts{n};subsegts{n}(1,:)];
    subsegts{n}(1,5) = 1; % Set flag in first row to 1
    subsegts{n}(end,5) = 1; % Set flag in last row to 1
end

%% Turn into waypts
waypts = {}; % Keep waypoints for each segment separated
waypts_comb = []; % Collect all waypoints into single trajectory

for n = 1:length(subsegts)
    waypts{n} = subsegts{n};
    waypts_comb = [waypts_comb;subsegts{n}(:,:)];
end
