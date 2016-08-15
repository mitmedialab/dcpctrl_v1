% Loads a log directory
% NOTE:
%   Style of directory must be, where [optional] and <variable>:
%       data/[sim]/<logname>/
%           segment_1.mat,
%           segment_2.mat, etc...
% WARNING: No longer works with legacy data logs

%% Select directory to analyze
startdir = pwd;
directory = 'data/TEST_NAME/';
filelist = dir(directory);

% Remove '.' directories
filelist = filelist(arrayfun(@(x) x.name(1), filelist) ~= '.');
nfiles = size(filelist,1);

multiplefiles = 1; % 0 if you want to run on just one file,
singfilepath = 'data/diamond01dt/diamond_segment_1.mat';

%% Concatenate all tests into a single file
if multiplefiles
    testdata = {};
    for n = 1:nfiles
        filename = [directory 'segment_' num2str(n) '.mat'];
        curfile = fopen(filename);
        if curfile ~= -1
            testdata = [testdata, load(filename)];
            fclose(curfile);
        end
    end

%% Collect out new logs vectors from each test
    seg = [];
    nseg = [];
    Kffv = [];
    Kpe = [];
    Kde = [];
    Kie = [];
    qraw = [];
    qrawdes = [];
    qdrawdes = [];
    pwm = [];
    ts = [];
    tdes = [];
    err = [];
    dmeaserr = [];
    for n = 1:length(testdata)
        logs = testdata{1,n}.logs;
        % Identify the first point in every segment - use this later to
        % identify segment start and endpoints
        len = length(logs);
        seg_temp = zeros(len,1);
        seg_temp(1) = 1;
        seg = [seg; seg_temp];
        nseg = [nseg; ones(len,1).*n];
        % Collect the rest of the data
        Kffv = [Kffv; [logs.Kffv]'];
        Kpe = [Kpe; [logs.Kpe]'];
        Kde = [Kde; [logs.Kde]'];
        Kie = [Kie; [logs.Kie]'];
        qraw = [qraw; [logs.qraw]'];
        qrawdes = [qrawdes; [logs.qrawdes]'];
        qdrawdes = [qdrawdes; [logs.qdrawdes]'];
        pwm = [pwm;[logs.pwm]'];
        ts = [ts; [logs.t]'];
        tdes = [tdes;[logs.tdes]'];
        err = [err; [logs.err]'];
        dmeaserr = [dmeaserr; [logs.dmeaserr]'];
    end
%% Alternative - plot data from a single trajectory segment    
else
    load(singfilepath);
    seg = zeros(length([logs.t]));
    Kffv = [logs.Kffv]';
    Kpe = [logs.Kpe]';
    Kde = [logs.Kde]';
    Kie = [logs.Kie]';
    qraw = [logs.qraw]';
    qrawdes = [logs.qrawdes]';
    qdrawdes = [logs.qdrawdes]';
    pwm = [logs.pwm]';
    ts = [logs.t]';
    tdes = [logs.tdes]';
    err = [logs.err]';
    dmeaserr = [logs.dmeaserr]';
end

qdrawcomm = Kffv + Kpe + Kde + Kie;
cd(startdir);

%% Plot path in 3D
qdes = raw2joint_at40gw(robot,qrawdes);
xdes = joint2cart_at40gw(qdes);
qact = raw2joint_at40gw(robot,qraw);
xact = joint2cart_at40gw(qact);

gcf
hold on
hdes = plot3(xdes(:,1),xdes(:,2),xdes(:,3),'g.');
hact = plot3(xact(:,1),xact(:,2),xact(:,3),'m');

segnum = 1;
for n = 1:length(seg)
    if seg(n)
       txt = strcat('Seg. ',num2str(segnum));
       text(xact(n,1),xact(n,2),xact(n,3),txt);
       segnum = segnum + 1;
    end
end
axis equal
axis image
xlabel('X, mm');
ylabel('Y, mm');
zlabel('Z, mm');
grid on;
view(45,45);
title(['Desired vs. raw position: Experiment ',directory]);
