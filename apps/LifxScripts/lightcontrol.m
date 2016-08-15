function [] = lightcontrol(x)
if x == 0
    system('python apps/LifxScripts/light_off.py');

elseif x == 1
    system('python apps/LifxScripts/light_on.py');
elseif isstr(x)
    system('python apps/LifxScripts/light_on.py');
    system(['python apps/LifxScripts/light_set_color.py ' x])
else 
    system('python apps/LifxScripts/light_on.py');
    system(['python apps/LifxScripts/light_set_color.py ' mat2str(x(1)) ' ' mat2str(x(2)) ' ' mat2str(x(3)) ' ' mat2str(x(4))])
end
end