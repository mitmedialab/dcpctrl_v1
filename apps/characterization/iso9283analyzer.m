% ISO9283ANALYZER.M
% Performs calculations specified in ISO 9283 standard. Assumes that data
% has been broken up into segments segts using testsegmenter.m

%{
    Julian Leland, MIT Media Lab, 2016-07-25
%}

%% TWO POINT SEGMENTS (not actually ISO 9283)
%{
%% Plot a single segment
segnum = 4; % Segment to look at

figure;
hold on;
plot(segts{1,segnum}(:,6),segts{1,segnum}(:,2));
plot(segts{1,segnum}(:,6),segts{1,segnum}(:,3));
plot(segts{1,segnum}(:,6),segts{1,segnum}(:,4));
hold off

%% Find start and end points of each run (2-point tests only)
for n = 1:length(segts)
    % Define starting position, position vector and d_position vector
    startpos = segts{1,n}(1,2:4); % Select first data point as starting position. This may not be robust (in case filtering algorithm has placed start position in wrong place) - keep an eye on this.
    samplenum = segts{1,n}(:,1); % Sample number
    pos = segts{1,n}(:,2:4); % Position
    d_pos = diff(pos); % Derivative of position
    d_pos = [[0,0,0];d_pos]; % Pad with extra zero at beginning to make pos & d_pos agree.
    
    % Identify points that are part of start & end position
    startposvect = [];
    endposvect = [];
    indexvect = [];
    minpoints = 5; % If there are fewer than this number of points in startposvect or endposvect, then there is something wrong with our data - we will flag for user to review.
    posthresh = 50; % Threshold of distance from start position
    d_posthresh = 5; % Threshold for zero derivative/when system is stopped.
    for m = 1:length(pos)
       if all(abs(d_pos(m,:)) <= d_posthresh)
           if all(abs(pos(m,:) - startpos(1,:)) <= posthresh) % We are at the start position
               startposvect = [startposvect;[samplenum(m,:),pos(m,:)]];
               %endposvect = [endposvect;zeros(1,3)];
           else % We are at the end position
               %startposvect = [startposvect;zeros(1,3)];
               endposvect = [endposvect;[samplenum(m,:),pos(m,:)]];
           end
           indexvect = [indexvect;m];
       end
    end
    if length(startposvect) < minpoints
        disp(['Not enough points in start position at segment ',num2str(n),'. Please retry.']);
        break;
    elseif length(endposvect) < minpoints
        disp(['Not enough points in end position at segment ',num2str(n),'. Please retry.']);
        break;
    end
    segts{2,n} = mean(startposvect,1);
    segts{3,n} = mean(endposvect,1);
end

%% Plot each mean value at its measured time over the original plot of data to make sure we're getting the right results
figure
hold on
for n = 1:length(segts)
    plot(segts{1,n}(:,1),segts{1,n}(:,2:4),'Color',colors(j,:));
    plot(segts{2,n}(:,1),segts{2,n}(:,2:4),'x','Color','red');
    plot(segts{3,n}(:,1),segts{3,n}(:,2:4),'x','Color','blue');
    j = j+1;
    if j == 7
        j = 1;
    end
end

%% Scratch code for plotting a single segment of above loop to see what's going wrong
figure
hold on
plotyy(segts{1,n}(:,6),segts{1,n}(:,2:4),segts{1,n}(:,1),d_pos(:,1:3));
legend('X','Y','Z','dX','dY','dZ');
hold off

%% Repeatability
% Calculate mean X, Y, Z values for start and end positions - x_bar, y_bar, z_bar in
% ISO 9283
mean_startvect = zeros(length(segts),length(segts{3,1}(1,2:4)));
mean_endvect = zeros(length(segts),length(segts{2,1}(1,2:4)));
for n = 1:length(segts)
    mean_startvect(n,:) = segts{3,n}(:,2:4);
    mean_endvect(n,:) = segts{2,n}(:,2:4);
end
mean_start = mean(mean_startvect);
mean_end = mean(mean_endvect);

% Calculate mean Cartesian error
l_js = [];
l_je = [];
for n = 1:length(segts)
    x_js = mean_startvect(n,1);
    y_js = mean_startvect(n,2);
    z_js = mean_startvect(n,3);
    x_je = mean_endvect(n,1);
    y_je = mean_endvect(n,2);
    z_je = mean_endvect(n,3);
    l_js = [l_js, sqrt((mean_start(1,1)-x_js)^2 + (mean_start(1,2)-y_js)^2 + (mean_start(1,3)-z_js)^2)];
    l_je = [l_je, sqrt((mean_end(1,1)-x_je)^2 + (mean_end(1,2)-y_je)^2 + (mean_end(1,3)-z_je)^2)];
end

l_jsbar = sum(l_js)/length(l_js);
l_jebar = sum(l_je)/length(l_je);

% Calculate std deviation (corrected sample std dev, per ISO 9283)
l_js_ssq = (l_js - l_jsbar).^2;
l_je_ssq = (l_je - l_jebar).^2;

si_s = sqrt(sum(l_js_ssq)/(length(l_js_ssq)-1));
si_e = sqrt(sum(l_je_ssq)/(length(l_je_ssq)-1));

posrep_s = l_jsbar + 3*si_s
posrep_e = l_jsbar + 3*si_e

%% Plot full run data, with spheres representing repeatability ranges around endpoints

j = 1; % Index to start colormap at
figure;

colors = get(gca,'ColorOrder');
posVect1 = [ 0.05 0.3 0.45 0.65];
fig1 = subplot('Position',posVect1)
hold on;

% Plot the 3D trajectory
for n = 1:length(segts)
    plot3(segts{1,n}(:,2),segts{1,n}(:,3),segts{1,n}(:,4),'Color',[colors(j,:)]);
    j = j+1;
    if j == 7
        j = 1;
    end
end

% Plot the first sphere
txtoffst = 50; % Lateral offset for text labels

phi=linspace(0,pi,30);
theta=linspace(0,2*pi,40);
[phi,theta]=meshgrid(phi,theta);

xs=posrep_s*sin(phi).*cos(theta) + mean_start(1,1);
ys=posrep_s*sin(phi).*sin(theta) + mean_start(1,2);
zs=posrep_s*cos(phi) + mean_start(1,3);

plot3(xs,ys,zs,':','Color','red');
text(mean_start(1)+txtoffst,mean_start(2)+txtoffst,mean_start(3),'Start Position');

% Plot the second sphere
xe=posrep_e*sin(phi).*cos(theta) + mean_end(1,1);
ye=posrep_e*sin(phi).*sin(theta) + mean_end(1,2);
ze=posrep_e*cos(phi) + mean_end(1,3);

plot3(xe,ye,ze,':','Color','red');
text(mean_end(1)+txtoffst,mean_end(2)+txtoffst,mean_end(3),'End Position');

grid on
axis image
axis equal
xlabel('X, mm');
ylabel('Y, mm');
zlabel('Z, mm');
title({'3D Position';'Red spheres = repeatability boundaries';''},'FontWeight','Normal');
view(55,35)

% Plot the 2D plot
j = 1;
posVect2 = [ 0.55 0.3 0.4 0.65];
fig2 = subplot('Position',posVect2);
hold on;
for n = 1:length(segts)
    plot(segts{1,n}(:,6),segts{1,n}(:,2),'Color',colors(1,:));
    plot(segts{1,n}(:,6),segts{1,n}(:,3),'Color',colors(2,:));
    plot(segts{1,n}(:,6),segts{1,n}(:,4),'Color',colors(3,:));
    % Commenting below out so we can plot X, Y, Z data together
%     j = j+1;
%     if j == 7
%         j = 1;
%     end
end
legend('X Position','Y Position','Z Position');
xlabel('Time, s');
ylabel('Measured Position in Direction, mm');
title({'X/Y/Z Position-Per-Axis vs. Time';''},'FontWeight','Normal');

suptitle('Pose Repeatability: 3D & Position-Per-Axis');

% Plot the run data
posVect3 = [ 0.1 0.05 0.8 0.1];
fig3 = subplot('Position',posVect3,'Visible','off');

results = { ['\bf Start Position\rm - Mean error: ',num2str(l_jsbar),' mm | Std. Dev.: ', num2str(si_s),' mm | Repeatability: ', num2str(posrep_s),' mm'];
            ' ';
            ['\bf End Position\rm - Mean error: ',num2str(l_jebar),' mm | Std. Dev.: ', num2str(si_e),' mm | Repeatability: ', num2str(posrep_e),' mm']};        

axes(fig3) % sets ax1 to current axes
text(.05,0.6,results)

testname = 'fullrepeatabilitytest\_35cycles'; % Put the name of the test here
testspecs = {   ['\bf Test Name:\rm ', testname];
                ' ';
                ['\bf Measurement Unit:\rm Leica AT901 Laser Tracker | \bf Measurement MPE:\rm 0.0141 mm maximum | \bf Measurement Frequency:\rm ',num2str(1/dt),' Hz']};
text(.55,0.6,testspecs)

%% Plot just 3D plot with no additional text
j = 1; % Index to start colormap at
figure;
colors = get(gca,'ColorOrder');
hold on;

% Plot the 3D trajectory
for n = 1:length(segts)
    plot3(segts{1,n}(:,2),segts{1,n}(:,3),segts{1,n}(:,4),'Color',[colors(j,:)]);
    j = j+1;
    if j == 7
        j = 1;
    end
end

grid on
axis image
axis equal
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
view(55,35)

%% Output all test data into convenient format for import into Excel

startpos_datavect = [];
endpos_datavect = [];
for n = 1:length(segts)
    startpos_datavect = [startpos_datavect;[segts{1,n}(1,1),segts{3,n}(:,2:4),mean_start,segts{3,n}(:,2:4)-mean_start]]; % Each row is measurement number at start of cycle, X/Y/Z average measurement for entire cycle, average X/Y/Z measurement for all cycles, error for cycle vs. average of all cycles.
    endpos_datavect = [endpos_datavect;[segts{1,n}(1,1),segts{2,n}(:,2:4),mean_end,segts{2,n}(:,2:4)-mean_end]];
end

% Multiply cycle number by dt to get measurement time
startpos_datavect(:,1) = startpos_datavect(:,1)*dt;
endpos_datavect(:,3) = endpos_datavect(:,1)*dt;

% Convert into cells to contain data names
startpos_datacell = {};
startpos_datacell = num2cell(startpos_datavect);
startpos_datacell(2:end+1,:) = startpos_datacell(1:end,:)
startpos_datacell(1,:) = {'Timestamp','X Avg Run Pos','Y Avg Run Pos','Z Avg Run Pos','X Mean Pos','Y Mean Pos','Z Mean Pos','X Err','Y Err','Z Err'};

endpos_datacell = {};
endpos_datacell = num2cell(endpos_datavect);
endpos_datacell(2:end+1,:) = endpos_datacell(1:end,:)
endpos_datacell(1,:) = {'Timestamp','X Avg Run Pos','Y Avg Run Pos','Z Avg Run Pos','X Mean Pos','Y Mean Pos','Z Mean Pos','X Err','Y Err','Z Err'};


% Export as Excel file
filename = strcat('fullreptest_35cycles.xlsx');
start_sheet = 'StartPosition';
end_sheet = 'EndPosition';
xlrange = 'A1'
xlswrite(filename,startpos_datacell,start_sheet,xlrange);
xlswrite(filename,endpos_datacell,end_sheet,xlrange);

%}




%% MULTI-POINT SEGMENTS (actual ISO 9283)
%%{
%% Find how many waypoints are present in each run
timei = segts{1,1}(:,6);
posi = segts{1,1}(:,2:4); % Position from initial run
d_posi = diff(posi); % Derivative of position
d_posi = [[0,0,0];d_posi]; % Pad with extra zero at beginning to make pos & d_pos agree.
d_posi_filt = lpfiltervec(d_posi,.75); % Filter with Veevee's awesome LP filter to get rid of noise on Y signal
minpoints = 5; % If there are fewer than this number of points in any of our "stopped" states, there may be something wrong with our data - we will flag for user to review.
d_posthresh = 10; % Threshold for zero derivative/when system is stopped.

% Set flags in posi indicating which waypoint coordinates belong to
for n = 1:length(posi)
    if all(abs(d_posi_filt(n,1:3)) <= d_posthresh) % If all of our derivatives are within +/- d_posthresh of zero
        posi(n,4) = 1; % Set flag indicating that this is part of a new waypoint
    else
        posi(n,4) = 0;
    end
end

% Code to plot position vs. d_position_filtered - check on how much noise is present
% in derivative. OPTIONAL
%{
figure;
hold on;
plotyy(timei,posi,timei,d_posi_filt);
plot(timei,posi(:,4)*1000)
hold off
%}

% Collect all points at a waypoint and generate an average position from
% them
waypt_postemp = [];
waypt_pos = [];
waypt_ct = 1;
for n = 2:length(posi)
    if posi(n,4) == 1 % This is part of the current waypoint
        waypt_postemp = [waypt_postemp;posi(n,1:3)]; % Add current set of posi to temp waypoint
    end
    
    if posi(n,4) == 0 && posi(n-1,4) == 1; % We are leaving a waypoint
        waypt_avg = mean(waypt_postemp);
        waypt_pos = [waypt_pos;[waypt_ct,waypt_avg]]; % Add current waypoint count & waypoint average to vector
        waypt_postemp = []; % Clear waypt_postemp
        waypt_ct = waypt_ct + 1; % Increment waypt_ct
    elseif n == length(posi)% && posi(n,4) == 1 % We are at the end of the run & have an average value
        waypt_avg = mean(waypt_postemp);
        waypt_pos = [waypt_pos;[waypt_ct,waypt_avg]]; % Add current waypoint count & waypoint average to vector
        waypt_postemp = []; % Clear waypt_postemp
        waypt_ct = waypt_ct + 1; % Increment waypt_ct
    else
        continue;
    end
end

% Remove first waypoint from list, since it is a duplicate, and adjust
% first column to match
waypt_pos = waypt_pos(2:end,:);
waypt_pos(:,1) = waypt_pos(:,1) - 1;

% Plot 3D run data from this run, plus XYZ coordinates of average endpoints
figure
hold on
plot3(segts{1,1}(:,2),segts{1,1}(:,3),segts{1,1}(:,4));
plot3(waypt_pos(:,2),waypt_pos(:,3),waypt_pos(:,4),'x','Color','red')
grid on
axis image
axis equal
title('First Segment: Trajectory vs. Calculated Waypoint Position');
legend('Trajectory','Calc. Waypt. Locations');
xlabel('X, mm');
ylabel('Y, mm');
zlabel('Z, mm');
view(-45,30);
disp('Please verify that waypoints are correctly located before continuing');

%% Find waypoints of all runs
% Iterate over all runs
for n = 1:length(segts) % We may only want to do 2:length(segts), since we've already done 1 above?
    % Define starting position, position vector and d_position vector
    samplenum = segts{1,n}(:,1); % Sample number
    pos = segts{1,n}(:,2:4); % Position
    d_pos = diff(pos); % Derivative of position
    d_pos = [[0,0,0];d_pos]; % Pad with extra zero at beginning to make pos & d_pos agree.
    d_pos_filt = lpfiltervec(d_pos,.75); % Filter with Veevee's awesome LP filter to get rid of noise on Y signal
    
    % Set up storage vectors and threshold variables
    run_waypts = {[],[],[],[],[]}; % Set up empty struct to store points that correspond to a waypoint.
    minpoints = 5; % If there are fewer than this number of points in startposvect or endposvect, then there is something wrong with our data - we will flag for user to review.
    posthresh = 100; % Threshold of distance from start position
    d_posthresh = d_posthresh; % Placeholder to indicate that we're using the same criteria for zero derivative here as we are for initial waypoint identification
    
    % Iterate over all points within run - if points are close enough to a waypoint, sort them into that waypoint
    for m = 1:length(pos)
       if all(abs(d_pos(m,:)) <= d_posthresh) % If our derivative is near enough to zero, 
           if any(all(abs(bsxfun(@minus,waypt_pos(:,2:4),pos(m,1:3))) <= posthresh,2));
               %startposvect = [startposvect;[samplenum(m,:),pos(m,:)]];
               runindx = find(all(abs(bsxfun(@minus,waypt_pos(:,2:4),pos(m,1:3))) <= posthresh,2));
               if any(size(runindx) > 1) % Check to make sure that we are only matching one run index value
                   disp(['Major error at datapoint ',num2str(m),' in Segment ',num2str(n),': Datapoint matches multiple waypoints!']); 
                   break;
               end
               run_waypts{1,runindx} = [run_waypts{1,runindx};[samplenum(m,:),pos(m,:)]]; % Append samplenum and position to run_waypts
           else % We are at the end position
               disp('Invalid waypoint - zero derivative, but incorrect position!');
           end
       end
    end
  
    % TODO: Re-implement check for too-short segments
    %     if length(startposvect) < minpoints
    %         disp(['Not enough points in start position at segment ',num2str(n),'. Please retry.']);
    %         break;
    %     elseif length(endposvect) < minpoints
    %         disp(['Not enough points in end position at segment ',num2str(n),'. Please retry.']);
    %         break;
    %     end
    
    % Find average positions for each waypoint for this run, and store them
    % in segts
    for o = 1:length(run_waypts)
        segts{o+1,n} = mean(run_waypts{1,o},1);
    end
end

%% Quick test plot to make sure that we're locating our average positions in the right places
% Note: Because we use data from the start and end of a segment to generate
% the mean position at this waypoint, the marker for this waypoint will
% appear to be right in the middle of the segment. This is okay!
%{
figure
hold on
for n = 1:length(segts)
    plot(segts{1,n}(:,1),segts{1,n}(:,2:4),'Color',colors(j,:));
    j = j+1;
    if j == 7
        j = 1;
    end
    for m = 2:size(segts,1)
        plot(segts{m,n}(:,1),segts{m,n}(:,2:4),'x','Color','red');
    end
end
%}

%% Repeatability
% Calculate mean X, Y, Z values for start and end positions - x_bar, y_bar, z_bar in
% ISO 9283

% Collect all mean positions at waypoints into vectors
mean_posvect = {[];[];[];[];[]};
for n = 2:size(segts,1) % For each waypoint (rows in segts)
    for m = 1:length(segts)
        mean_posvect{n-1,1} = [mean_posvect{n-1,1};segts{n,m}(1,2:4)];
    end
end

% Find mean of mean positions
mean_pos = zeros(size(segts,1)-1,3);
for n = 1:length(mean_posvect)
    mean_pos(n,:) = mean(mean_posvect{n,1});
end

% Calculate mean Cartesian error
l_j = zeros(length(mean_pos),1); % Vector to store mean Cartesian errors
temp_err = [];
for n = 1:length(segts)
    for m = 1:length(mean_pos)
        temp_err = sqrt((mean_pos(m,1)-segts{m+1,n}(1,2))^2 + (mean_pos(m,2)-segts{m+1,n}(1,3))^2 + (mean_pos(m,3)-segts{m+1,n}(1,4))^2);
        l_j(m,n) = temp_err;
    end
end
l_jbar = sum(l_j,2)/length(l_j); % This is the mean Cartesian error

% Calculate std deviation (corrected sample std dev, per ISO 9283)
l_j_ssq = bsxfun(@minus,l_j,l_jbar).^2;
si = sqrt(sum(l_j_ssq,2)/(length(l_j_ssq)-1));
posrep = l_jbar + 3*si;

%% Plot full run data, with spheres representing repeatability ranges around endpoints

j = 1; % Index to start colormap at
figure;

colors = get(gca,'ColorOrder');
posVect1 = [ 0.05 0.3 0.45 0.65];
fig1 = subplot('Position',posVect1)
hold on;

% Plot the 3D trajectory
for n = 1:length(segts)
    plot3(segts{1,n}(:,2),segts{1,n}(:,3),segts{1,n}(:,4),'Color',[colors(j,:)]);
    j = j+1;
    if j == 7
        j = 1;
    end
end

% Plot a repeatability sphere for each waypoint
txtoffst = 50; % Lateral offset for text labels

for n = 1:length(posrep)
    phi=linspace(0,pi,30);
    theta=linspace(0,2*pi,40);
    [phi,theta]=meshgrid(phi,theta);

    xs=posrep(n,1)*sin(phi).*cos(theta) + mean_pos(n,1);
    ys=posrep(n,1)*sin(phi).*sin(theta) + mean_pos(n,2);
    zs=posrep(n,1)*cos(phi) + mean_pos(n,3);

    plot3(xs,ys,zs,':','Color','red');
    text(mean_pos(n,1)+txtoffst,mean_pos(n,2)+txtoffst,mean_pos(n,3),['Waypoint ',num2str(n)]);
end

grid on
axis image
axis equal
xlabel('X, mm');
ylabel('Y, mm');
zlabel('Z, mm');
title({'3D Position';'Red spheres = repeatability boundaries';''},'FontWeight','Normal');
view(-45,30)

% Plot the 2D plot
j = 1;
posVect2 = [ 0.55 0.3 0.4 0.65];
fig2 = subplot('Position',posVect2);
hold on;
for n = 1:length(segts)
    plot(segts{1,n}(:,6),segts{1,n}(:,2),'Color',colors(1,:));
    plot(segts{1,n}(:,6),segts{1,n}(:,3),'Color',colors(2,:));
    plot(segts{1,n}(:,6),segts{1,n}(:,4),'Color',colors(3,:));
    % Commenting below out so we can plot X, Y, Z data together
%     j = j+1;
%     if j == 7
%         j = 1;
%     end
end
legend('X Position','Y Position','Z Position');
xlabel('Time, s');
ylabel('Measured Position in Direction, mm');
title({'X/Y/Z Position-Per-Axis vs. Time';''},'FontWeight','Normal');

suptitle({'Pose Repeatability: 3D & Position-Per-Axis';'ISO 9283-1998'});

% Plot the run data
posVect3 = [ 0.1 0.05 0.8 0.1];
fig3 = subplot('Position',posVect3,'Visible','off');

% Generate run metrics for each waypoint
results = {};
for n = 1:length(posrep)
    line_n = {['\bf Waypoint ', num2str(n), '\rm - Mean error: ',num2str(l_jbar(n)),' mm | Std. Dev.: ', num2str(si(n)),' mm | Repeatability: ', num2str(posrep(n)),' mm']};
    results = [results;line_n];
end

%results = { ['\bf Start Position\rm - Mean error: ',num2str(l_jsbar),' mm | Std. Dev.: ', num2str(si_s),' mm | Repeatability: ', num2str(posrep_s),' mm'];
 %           ' ';
  %          ['\bf End Position\rm - Mean error: ',num2str(l_jebar),' mm | Std. Dev.: ', num2str(si_e),' mm | Repeatability: ', num2str(posrep_e),' mm']};        

axes(fig3) % sets ax1 to current axes
text(.05,0.6,results)

testname = 'isorepeatability\_full'; % Put the name of the test here
testspecs = {   ['\bf Test Name:\rm ', testname];
                ' ';
                ['\bf Measurement Unit:\rm Leica AT901 Laser Tracker | \bf Measurement MPE:\rm 0.0141 mm maximum | \bf Measurement Frequency:\rm ',num2str(1/dt),' Hz']};
text(.55,0.6,testspecs)

%% Create image of just 3D plot, with all extraneous marks removed
j = 1; % Index to start colormap at
figure;

colors = get(gca,'ColorOrder');
hold on;

% Plot the 3D trajectory
for n = 1:length(segts)
    plot3(segts{1,n}(:,2),segts{1,n}(:,3),segts{1,n}(:,4),'Color',[colors(j,:)]);
    j = j+1;
    if j == 7
        j = 1;
    end
end

% Plot a repeatability sphere for each waypoint
txtoffst = 50; % Lateral offset for text labels

for n = 1:length(posrep)
    phi=linspace(0,pi,30);
    theta=linspace(0,2*pi,40);
    [phi,theta]=meshgrid(phi,theta);

    xs=posrep(n,1)*sin(phi).*cos(theta) + mean_pos(n,1);
    ys=posrep(n,1)*sin(phi).*sin(theta) + mean_pos(n,2);
    zs=posrep(n,1)*cos(phi) + mean_pos(n,3);

    plot3(xs,ys,zs,':','Color','red');
    %text(mean_pos(n,1)+txtoffst,mean_pos(n,2)+txtoffst,mean_pos(n,3),['Waypoint ',num2str(n)]);
end

grid on
axis image
axis equal
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
view(-45,30)

%}