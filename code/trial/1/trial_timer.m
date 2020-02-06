% trial of user data
t = timer('ExecutionMode','fixedSpacing','Period',0.5,'TasksToExecute',inf);
t.UserData = struct('string','time: ','count',0);
t.TimerFcn = @trial_timer_fcn;
start(t);