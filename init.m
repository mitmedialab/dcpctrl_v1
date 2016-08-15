%% Flags
usinglabjack = 1;

%% Initialize path
addpath('apps');
addpath('robots/at40gw');
addpath('robots/kuka');
addpath('robots/sprayer');
addpath('lib/');
addpath('lib/rvctools/robot');
addpath('lib/rvctools/common');
addpath('data');
addpath('util')

%% Initialize apps (might not be necessary...)
addpath('apps/trajectories');
addpath('apps/controllers');
addpath('apps/dataviz');
addpath('apps/experimental/mmcontrol');
addpath('apps/kuka');
addpath('apps/xbox');

%% Initialize AT40GW functions
%if exist('robot')~=1
disp('Loading AT40GW configuration');
robot = config_at40gw;
%end

%% Initialize LabJack functions (if working with hardware
if usinglabjack
    % NOTE: This also resets the sensor reading for Joint 1 (so wherever it
    % is currently is now the new 0 position)
    disp('Initializing LabJack (resets Joint 1 to 0!)');
    handle = initlabjack(robot);
    cleanupObj = onCleanup(@() resetsystem_at40gw(handle, robot));
end