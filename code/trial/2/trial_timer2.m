% trial, to call another function in TimerFcn
t = timer('ExecutionMode','fixedSpacing','Period',0.5,'TasksToExecute',inf);
t.TimerFcn = @trial_timer2_fcn;
global count
count = 0;
start(t);