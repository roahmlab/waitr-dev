%% Case 2024 Format Trajectory Visualization Figure
% Zachary Brei

% % Trajectory Visualization Figure
% ylim([0 55])
xlabel('Time (s)')
ylabel('Joint 1 Angle (deg)')
set(gcf,'Color','w');
% fontsize(fig, 14, "points")
ax = gca; % Get the current axes
ax.FontSize = 14; % Set the font size


% Force Visualization Figure
% ylim([-2 2])
% xlabel('Time (s)')
% ylabel('x-axis Force (N)')
% set(gcf,'Color','w');
% % fontsize(fig, 14, "points")
% ax = gca; % Get the current axes
% ax.FontSize = 14; % Set the font size