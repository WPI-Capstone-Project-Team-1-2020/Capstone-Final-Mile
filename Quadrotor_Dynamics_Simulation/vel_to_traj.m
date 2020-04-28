% Offset time to start at zero
time = cmds(:,1);
time = time(:) - time(1);

% Get the linear velocity command from the ROS output file
vel_x = [time, cmds(:,2)];
vel_y = [time, cmds(:,3)];
vel_z = [time, cmds(:,4)];

% Get the angular velocity
w_vel_x = [time, cmds(:,5)];
w_vel_y = [time, cmds(:,6)];
w_vel_z = [time, cmds(:,7)];

% Get the linear acceleration
acc_x = [time, cmds(:,8)];
acc_y = [time, cmds(:,9)];
acc_z = [time, cmds(:,10)];

% Get the angular acceleration
euler_x = [time, cmds(:,11)];
euler_y = [time, cmds(:,12)];
euler_z = [time, cmds(:,13)];

% Integrate velocity commands to get position trajectories
traj_x = [vel_x(:,1), cumsum(vel_x(:,2))];
traj_y = [vel_y(:,1), cumsum(vel_y(:,2))];
traj_z = [vel_z(:,1), cumsum(vel_z(:,2))];

% Start with row r
r = 2;
traj_x = traj_x(r:end, :);
traj_y = traj_y(r:end, :);
traj_z = traj_z(r:end, :);

% visualize trajectory to determine if valid
figure;
hold on
% plot(traj_x(:,1), traj_x(:,2));
% plot(traj_x(:,1), traj_y(:,2));
plot(traj_x(:,1), traj_z(:,2));
% plot(sim_time, sim_x(:,1));
% plot(sim_time, sim_y(:,1));
plot(sim_time, sim_z(:,1));
%legend('X', 'Y', 'Z', 'sim X', 'sin Y', 'sim Z');
legend('Gazebo Z', 'Sim Z');
hold off

% % visualize dynamics
% sim_time = 0:length(x_lin_v) - 1;
% sim_time = sim_time .* 0.25;
% figure;
% hold on
% plot(vel_x(:,1), vel_x(:,2));
% plot(vel_y(:,1), vel_y(:,2));
% plot(vel_z(:,1), vel_z(:,2));
% plot(sim_time, x_lin_v(:,1));
% plot(sim_time, y_lin_v(:,1));
% plot(sim_time, z_lin_v(:,1));
% legend('X', 'Y', 'Z', 'sim X', 'sin Y', 'sim Z');
% % plot(euler_x(:,1), euler_x(:,2));
% % % plot(euler_y(:,1), euler_y(:,2));
% % % plot(euler_z(:,1), euler_z(:,2));
% % plot(sim_time, sim_euler_x(:,1));
% % % plot(sim_time, sim_euler_y(:,1));
% % % plot(sim_time, sim_euler_z(:,1));
% % legend('Gazebo X', 'Sim X');
% hold off
% 
% 
