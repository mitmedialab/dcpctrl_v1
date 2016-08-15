function alignment = align(handle, robot, qrawdes, tol)
% ALIGN - tool to help manually align the boom arm to a specific
% joint-space location

    if nargin < 4
        tol = 1e-2; % mm or deg depending on joint
    end

    qraw = getrawpos_at40gw(handle, robot);
    q = raw2joint_at40gw(robot, qraw);
    xarm = joint2arm_at40gw(q);
    
    qdes = raw2joint_at40gw(robot, qrawdes);
    xdes = joint2cart_at40gw(qdes);
    xarmdes = joint2arm_at40gw(qdes);

    figure;
    hold on;
    harm = plot3(xarm(:,1),xarm(:,2),xarm(:,3),'r','LineWidth',5);
    harmdes = plot3(xarmdes(:,1),xarmdes(:,2),xarmdes(:,3),'b','LineWidth',5);
    axis([-5000 5000 -5000 5000 -100 3000]);
    hold off;
    
    hjs = [];
    hdes = [];
    ax = [];
    ylims = zeros(4,2);
    figure;
    for i = 1:4
        subplot(4,1,i);
        hold on;
        hjs(i) = plot(0, qraw(i), 'xr', 'MarkerSize', 5);
        hdes(i) = plot(0, qrawdes(i), 'ob', 'MarkerSize', 5);
        ax(i) = gca;
        ylims(i,:) = ylim;
        hold off;
    end
   
    dispstat('','init');

    % Monitor all data and plot
    while 1
        qraw = getrawpos_at40gw(handle, robot);
        pwms = getpwms_at40gw(handle, robot);
        q = raw2joint_at40gw(robot, qraw);
        x = joint2cart_at40gw(q);
        xarm = joint2arm_at40gw(q);
        
        err = abs(qrawdes - qraw);
        atgoal = err < tol;
        
        dispstat(sprintf(['Raw joints:\t\t%.4f %.4f %.4f %.4f\n'...
                          'Raw des:\t\t%.4f %.4f %.4f %.4f\n'...
                          'Raw diff:\t\t%.4f %.4f %.4f %.4f\n'...
                          'Cart des:\t\t%.4f %.4f %.4f\n'...
                          'Cart:\t\t%.4f %.4f %.4f\n'...
                          'Joints:\t\t\t%.4f %.4f %.4f %.4f\n'...
                          'PWM:\t\t\t%i %i %i %i\n'...
                          'Cart Position:\t%.4f %.4f %.4f %.4f'], qraw, qrawdes, err, xdes, x, q, pwms, x));
        
        if all(atgoal)
            set(harm,'XData',xarm(:,1),'YData',xarm(:,2),'ZData',xarm(:,3),'Color','g');
        else
            set(harm,'XData',xarm(:,1),'YData',xarm(:,2),'ZData',xarm(:,3),'Color','r');
        end

        for j=1:4
            if atgoal(j)
                set(hjs(j),'ydata',qraw(j),'Color','g');
                set(ax(j),'ylim',ylims(j,:));
            else
                set(hjs(j),'ydata',qraw(j),'Color','r');
            end
        end
        
        drawnow;
    end
end