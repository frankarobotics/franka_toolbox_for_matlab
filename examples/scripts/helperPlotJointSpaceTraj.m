function helperPlotJointSpaceTraj(titleText,t,q,qd,wpts,tpts)

% Plot results
figure;

% Plot joint positions 
subplot(2,1,1);
plot(t,q);
if nargin > 5
    hold all
    plot(tpts, wpts, 'x', 'MarkerSize', 7,'LineWidth',2);
end
%xlabel('Time')
ylim('padded');
title(titleText);

% Plot joint velocities
subplot(2,1,2);
plot(t,qd)
xlabel('Time')
ylim('padded');
title('Joint-Space Velocities');

end