t1 = timer('ExecutionMode','fixedSpacing','Period',0.5,'TasksToExecute',inf);
t1.TimerFcn = @trial_timer3_fcn1;
global count
count = 0;
start(t1);

t2 = timer('ExecutionMode','fixedSpacing','Period',0.1,'TasksToExecute',inf, 'BusyMode','drop');
t2.TimerFcn = @trial_timer3_fcn2;
global count2;
count2 = -1;
start(t2);