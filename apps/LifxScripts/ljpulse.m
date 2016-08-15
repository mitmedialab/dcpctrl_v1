while 1
    [~, flag] = LabJack.LJM.eReadName(handle, 'DIO9',0);
    if flag
        lightcontrol(1)
        lightcontrol(0)
    end
end