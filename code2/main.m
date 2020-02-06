clear; clc; close all;

global map zone robot task direction
%% initialization of some parameters
robot_num = 3;
task_num = 10;
carrier_num = 4; % must be integer times of 2
carrier_absence_rate = 0.4; % 0 ~ 1
run_mode = 1;   % 1 for run, else for debug
last_data = 0;   % to use the last data, 0 to generate new data
fig = figure;

%% load the last data
robot_pos = [];
task_pos = [];
if last_data == 1
    load 'data.mat'
    map = [map(:, 1:end-2), [-1,-1;zeros(size(map,1)-2, 2);-1,-1], map(:, end-1:end)];
    map_full = [map_full(:, 1:end-2), [-1,-1;zeros(size(map_full,1)-2, 2);-1,-1], map_full(:, end-1:end)];
end

%% map initialization
if last_data == 0
	[map, map_full] = create_map(carrier_num, carrier_absence_rate, task_num);
end
[zone, direction] = ZoneDirection(carrier_num, map_full, 2);


% robot initialization
robot = robot_init(robot_num, last_data, robot_pos);

% task initialization
task = task_init(task_num, last_data, task_pos);

if last_data == 0
    % save the data for reuse
    robot_pos = reshape([robot.path], 2, length(robot))';
    task_pos = reshape([task.position], 2, length(task))';
    save 'data.mat' map map_full robot_pos task_pos
end

clear task_num carrier_absence_rate robot_num map_full carrier_num task_pos robot_pos

%% testing the performance
global times_plan total_time_plan total_path_len
times_plan = 0;
total_time_plan = 0;
total_path_len = 0;

%% timer: dynamic plot
t_dyna_plot = timer('ExecutionMode','fixedSpacing','Period',0.4,'BusyMode','queue');
if run_mode == 1
    t_dyna_plot.TasksToExecute = inf;
else
    t_dyna_plot.TasksToExecute = 1;
end
t_dyna_plot.TimerFcn = @plot_map;
start(t_dyna_plot);

%% timer: path_planning
t_path_plan = timer('ExecutionMode','fixedSpacing','Period',0.5,'TasksToExecute',inf,'StartDelay',0.5,'BusyMode','queue');
t_path_plan.TimerFcn = @path_planning;
if run_mode == 1
    start(t_path_plan);
end

%% timer: task allocation
t_task_alloc = timer('ExecutionMode','fixedSpacing','Period',2,'BusyMode','drop');
if run_mode == 1
    t_task_alloc.TasksToExecute = inf;
else
    t_task_alloc.TasksToExecute = 1;
end
t_task_alloc.TimerFcn = @task_distribution;
start(t_task_alloc);

%% keyboard GUI
set(fig,'windowkeyreleasefcn',{@keyreleasefcn, t_dyna_plot, t_path_plan, t_task_alloc});