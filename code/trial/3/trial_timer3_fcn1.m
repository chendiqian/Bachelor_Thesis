function trial_timer3_fcn1(obj, event)
global count
disp(count)
pause(3)
count = count + 1;
if count == 5
    stop(obj)
end