
% Extract data from the Simulink output structure
X_act = out.Xact; % Actual X data
Y_act = out.Yact; % Actual Y data
X_des = out.Xdes; % Desired X data
Y_des = out.Ydes; % Desired Y data
V = out.v;        %Control input V
Phi = out.phi;    %control input phi
xe = out.Xe;      %x error 
ye = out.Ye;      %y error 
thetae = out.thetae;      %thetae error 
% Create a new figure for the plot
figure;

% Plot actual trajectory
plot(X_act, Y_act, 'b', 'LineWidth', 2, 'DisplayName', 'Actual Trajectory');
hold on; % Hold the plot for overlaying

% Plot desired trajectory
plot(X_des, Y_des, 'r--', 'LineWidth', 2, 'DisplayName', 'Desired Trajectory');

% Customize the plot
grid on;                % Add a grid
xlabel('X-axis');       % Label for X-axis
ylabel('Y-axis');       % Label for Y-axis
title('Actual vs. Desired Trajectories'); % Title of the plot
legend('Location', 'best'); % Add a legend in the best location

% Adjust axis scaling for better visual comparison
axis equal;

% Release the hold
hold off;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

time = 0:0.01:50;
% Create a new figure for the plot
figure;

% Plot velocity vs. time
plot(time, V, 'b--', 'LineWidth', 2, 'DisplayName', 'V vs. Time');
% Customize the plot
grid on;
xlabel('Time (s)');
ylabel('V');
title('V vs. Time');
legend('Location', 'best');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
% Plot phi vs. time
plot(time, Phi, 'r-', 'LineWidth', 2, 'DisplayName', 'phi vs. Time');
% Customize the plot
grid on;
xlabel('Time (s)');
ylabel('phi');
title('phi vs. Time');
legend('Location', 'best');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
% Plot xe,ye,thetae vs. time
plot(time, xe, 'r-', 'LineWidth', 2, 'DisplayName', 'xe vs. Time');
hold on; % Hold the plot for overlaying

plot(time, ye, 'b-', 'LineWidth', 2, 'DisplayName', 'ye vs. Time');
hold on; % Hold the plot for overlaying

plot(time, thetae, 'g-', 'LineWidth', 2, 'DisplayName', 'thetae vs. Time');

% Customize the plot
grid on;
xlabel('Time (s)');
ylabel('xe,ye,thetae');
title('xe vs. Time');
legend('Location', 'best');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%------------------LIVE PLOTTING---------------------%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%{
% Extract data from Simulink output structure
X_act = out.Xact.Data; % Actual X data
Y_act = out.Yact.Data; % Actual Y data
X_des = out.Xdes.Data; % Desired X data
Y_des = out.Ydes.Data; % Desired Y data
time = out.Xact.Time;  % Time vector

% Create a new figure for live plotting
figure;
hold on;
grid on;

% Initialize animated lines for actual and desired trajectories
actual_line = animatedline('Color', 'b', 'LineWidth', 2, 'DisplayName', 'Actual Trajectory');
desired_line = animatedline('Color', 'r', 'LineStyle', '--', 'LineWidth', 2, 'DisplayName', 'Desired Trajectory');

% Customize plot appearance
xlabel('X-axis');
ylabel('Y-axis');
title('Live Actual vs Desired Trajectories');
legend('Location', 'best');
axis equal;

% Live plot the trajectories
for i = 1:length(time)
    % Add points to animated lines
    addpoints(actual_line, X_act(i), Y_act(i));
    addpoints(desired_line, X_des(i), Y_des(i));
    
    % Update the plot
    drawnow;
    
    % Optional: Slow down the update speed for better visualization
    pause(0.00001); % Adjust as necessary
end

hold off;
%}
