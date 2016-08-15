%% Load waypts from Homeprint output
hpwaypts = dlmread('data/tumor1.txt');

%% Remove duplicates
prev = hpwaypts(1,:);
dup = false(size(hpwaypts,1),1);

for i = 2:size(hpwaypts,1)
    curr = hpwaypts(i,:);
    if all(curr == prev)
        dup(i) = 1;
    end
    prev = curr;
end

hpwaypts(dup,:) = [];

%% Determine segmentation splits before resizing
% Separate inter and intra layer segmentation
dist_btwn_layers = 50; % mm
dist_btwn_segs = 500; % mm

df = diff(hpwaypts);
dxy = sqrt(sum(df(:,1:2).^2,2));
dz = sqrt(sum(df(:,3).^2,2));
d = sqrt(sum(df.^2,2));

splits = dxy > dist_btwn_segs | dz > dist_btwn_layers;
nsplits = sum(splits);
isplits = find(splits);

%% Resize (naive, axis-aligned)
des_max_sz = 9000;

hpwayptsc = bsxfun(@minus, hpwaypts, mean(hpwaypts));
max_sz = max(max(hpwayptsc)-min(hpwayptsc));

scale = des_max_sz/max_sz;

scalem = eye(3) * scale;

hpwayptsc = (scalem * hpwayptsc')';
hpwayptsc = ([1 0 0; 0 1 0; 0 0 0.7] * hpwayptsc')';

hpwaypts = hpwayptsc;

%% Rotate around z and shift up

hpwayptsc = hpwaypts;
zh = 700;

xy0 = hpwaypts(1,1:2);
%deg2rot = -180 - acosd(xy0(1) / norm(xy0));
%deg2rot = deg2rot + 45;
deg2rot = -180-50;
deg2rot = 0;

hpwayptsc = ([cosd(deg2rot) -sind(deg2rot) 0; ...
              sind(deg2rot) cosd(deg2rot) 0; ...
              0 0 1] * hpwayptsc')';

z0 = hpwayptsc(1,3);
zd = zh - z0;
hpwayptsc = bsxfun(@plus, hpwayptsc, [0 0 zd]);

hpwaypts = hpwayptsc;

%% Create segments
waypts = {};
prevsplit = 1;
figure;
hold on;
for i = 1:nsplits
    split = isplits(i);
    waypts{i} = [hpwaypts(prevsplit:split,:) zeros(split-prevsplit+1,2)];
    prevsplit = split+1;
    
    plot3(waypts{i}(:,1),waypts{i}(:,2),waypts{i}(:,3))
end
i = i + 1;
waypts{i} = [hpwaypts(prevsplit:end,:) zeros(size(hpwaypts,1)-prevsplit+1,2)];
plot3(waypts{i}(:,1),waypts{i}(:,2),waypts{i}(:,3))
axis equal;
hold off;

%% Select segments

%waypts = waypts([6 8 11 14 16 17 18]);
%dires = [1 0 0 1 0 0 0]; % Manually figured out, 1 = CCW, 0 = CW

waypts = waypts([8 11 14 16 17 18]);
dires = [0 0 1 0 0 0]; % Manually figured out, 1 = CCW, 0 = CW

figure;
hold on;
for i = 1:length(waypts)
    plot3(waypts{i}(:,1),waypts{i}(:,2),waypts{i}(:,3));
end
xlabel('x');
ylabel('y');
hold off;

%% Alternate segment directions
dire = 1;

for i = 1:length(waypts)
    if dire ~= dires(i)
        waypts{i} = flipud(waypts{i});
    end
    dire = ~dire;
end

%% Quick run through of waypoints
% figure;
% hold on;
% cs = hsv(length(waypts));
% for i = 1:length(waypts)
%     for p = 1:size(waypts{i},1)
%         plot3(waypts{i}(p,1),waypts{i}(p,2),waypts{i}(p,3),'bo','Color',cs(i,:));
%         drawnow;
%     end
% end
% hold off;


%% Rotate everything
close all
js = ikine_at40gw(waypts{1}(1,1:3),  raw2joint_at40gw(robot,[-167.0000   -1.5229   -8.4200   -1.8866]),[1 0 1 1]);
deg2rot = -180-js(1);
deg2rot = 120;
figure;
hold on;
for i = 1:length(waypts)
    
    waypts{i}(:,1:3) = ([cosd(deg2rot) -sind(deg2rot) 0; ...
                         sind(deg2rot) cosd(deg2rot) 0;
                         0 0 1;] * waypts{i}(:,1:3)')';
    plot3(waypts{i}(:,1),waypts{i}(:,2),waypts{i}(:,3));
    plot3(waypts{i}(1,1),waypts{i}(1,2),waypts{i}(1,3),'mx')
end
xlabel('x');
ylabel('y');

