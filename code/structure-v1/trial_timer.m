t = timer('ExecutionMode','fixedRate','Period',0.5,'TasksToExecute',inf);
t.UserData = struct('string','time: ','count',0);
t.TimerFcn = @draw;
start(t);