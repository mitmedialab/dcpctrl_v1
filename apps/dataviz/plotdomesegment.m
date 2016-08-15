function logs = plotdomesegment(filename)

logs = load(filename);
logs = logs.logs;

t = [logs.t];
pwm = [logs.pwm]';
qraw = [logs.qraw]';
qrawdes = [logs.qrawdes]';
qdrawdes = [logs.qdrawdes]';
%%
% Plot the PWMs
figure;
for j = 1:4
    subplot(4,1,j);
    hold on;
    plot(t,pwm(:,j));
    title(['J' num2str(j) ' PWMs']);
    hold off;
end

% Plot the desired velocities
figure;
for j = 1:4
    subplot(4,1,j);
    hold on;
    plot(t,qdrawdes(:,j));
    title(['J' num2str(j) ' Raw Desired Velocity']);
    hold off;
end

% Plot the positions
figure;
for j = 1:4
    subplot(4,1,j);
    hold on;
    plot(t,qraw(:,j),t,qrawdes(:,j));
    title(['J' num2str(j) ' Raw Position']);
    legend('Measured','Desired');
    xlabel('t (s)');
    hold off;
end
end