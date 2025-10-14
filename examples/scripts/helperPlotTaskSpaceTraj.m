function helperPlotTaskSpaceTraj(titleText,t,q,qd,wpts,tpts)

% Plot 2D position 
figure
sgtitle(titleText);

subplot(2,2,[1 3]);
plot(q(1,:),q(2,:));
hold all
plot(wpts(1,:),wpts(2,:),'x','MarkerSize',7,'LineWidth',2);
xlim('padded');
ylim('padded');
xlabel('X');
ylabel('Y');
title('2-D Trajectory')

% Plot X and Y position with time
subplot(2,2,2);
plot(t,q);
if nargin > 5
    hold all
    plot(tpts,wpts,'x','MarkerSize',7,'LineWidth',2);
end
ylim('padded')
xlabel('Time');
ylabel('Position');
title('Position vs Time');
legend({'X','Y'})

% Plot X and Y velocity with time
subplot(2,2,4);
plot(t,qd)
ylim('padded')
xlabel('Time');
ylabel('Velocity');
title('Velocity vs Time');
legend({'X','Y'})

end