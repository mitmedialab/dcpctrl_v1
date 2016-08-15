vmax = 5
vmin = 0
huemax = 62978;
huemin = 43636;
x = 0;
brightness = 65535;
temp = 3500;
saturation = 65535;

while 1
    %[~, v] = LabJack.LJM.eReadName(handle,'AIN2',0)
    v = sin(x)*5
    x = 0.001+x;
    hue = mapRange(v, vmin, vmax, huemin, huemax);
    system(['python apps/LifxScripts/light_set_color.py ' hue brightness saturation temp])
    
end