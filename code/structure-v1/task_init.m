function task = task_init(task_num, map)
% tstate: new, assigned, to_exit, from_exit, done; 
exit_num = ceil( (size(map, 1)-2) / 3);
[m, n] = size(map);

task_space = zeros(size(map, 1), size(map, 2));
for i = 1:task_num
    exit_num = unidrnd(exit_num);
    exit_position = [(exit_num-1)*3+2, n - 1];
    task(i) = struct('tstate', {"new"}, 'position', {[]}, 'prior', {i}, ...
        'exit', {exit_position});
    
    position = [unidrnd(m-4)+2, unidrnd(n-4)+2];
    while map(position(1), position(2)) ~= 1 || task_space(position(1), position(2)) == 1
        position = [unidrnd(m-4)+2, unidrnd(n-4)+2];
    end
    task(i).position = position;
    task_space(position(1), position(2)) = 1;
end

fprintf("%d tasks generated!\n", task_num);