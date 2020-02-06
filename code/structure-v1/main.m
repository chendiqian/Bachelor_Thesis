clear; clc; close all;

%% map and direction initialization
% carrier_num = unidrnd(5);
carrier_num = 9;

[map, crossroad] = create_map(carrier_num);
% plot_map(map);
global direction_mat;
direction_mat = direction_init(map, crossroad);

%% robot initialization
robot_num = 1;
global robot;
robot = robot_init(robot_num, map);
clear robot_num

%% task initialization
task_num = 3;
global task;
task = task_init(task_num, map);
clear task_num
plot_map(map);

%% timer: task distribution
t_task_alloc = timer('ExecutionMode','fixedSpacing','Period',0.2,'TasksToExecute',inf);
t_task_alloc.TimerFcn = @task_distribution;
% t.UserData = struct('task',task,'robot',robot);
start(t_task_alloc);

%% execution
t_dynam_move = timer('ExecutionMode','fixedSpacing','Period',0.2,'TasksToExecute',inf,...
    'StartDelay', 2);
t_dynam_move.TimerFcn = {@dynamic_move, map};
start(t_dynam_move);

%% stop timer(temporary)
% stop(t_dynam_move)