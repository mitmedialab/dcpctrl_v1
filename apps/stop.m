%% Reset
past = getpwms_at40gw(handle, robot);
resetsystem_at40gw(handle, robot);
present = getpwms_at40gw(handle, robot);

disp(['PWM set from ' num2str(past) '  --> ' num2str(present)]);