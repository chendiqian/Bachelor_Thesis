function trial_timer_fcn(obj, event)
% a trial of userdata
ud = obj.UserData;
fprintf("%s, %d\n", ud.string, ud.count);
ud.count = ud.count + 1;
set(obj,'UserData',ud);

if ud.count == 10
    stop(obj)
end