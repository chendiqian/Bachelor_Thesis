function trial_timer3_fcn2(obj, event)

global count2

disp(count2)
count2 = count2 - 1;

if count2 == -20
    stop(obj)
end