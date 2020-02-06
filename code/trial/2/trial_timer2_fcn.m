function trial_timer2_fcn(obj, event)
global count
disp(count)
timer2_count_plus();
if count == 10
    stop(obj)
end