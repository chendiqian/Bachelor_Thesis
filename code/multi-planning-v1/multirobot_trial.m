%% plan the path for multiple robots
% problems: still not considered the collision
% if the robot has already reached the goal, it still moves around, only 
% satisfying that it will be at the goal by the end time.

clear; clc;

t_min = 2;
t_max = 20;

height = 5;
length = 5;
map = zeros(height, length);
% map(2, 1:3) = 1;
% map(4, 2:5) = 1;
% imagesc(map);

spoint_1 = [1, 1];
epoint_1 = [5, 5];
spoint_2 = [2, 4];
epoint_2 = [5, 3];
no_robot = 2;

for T = t_min: t_max
    %% initialize a 4-dimensional matrix
    x = zeros(height, length, no_robot, T);
    
    %% give the names of variables according to the position of it
    names = cell(size(x));
    for i=1: size(x, 1)
        for j=1: size(x, 2)
            for k=1: size(x, 3)
                for l=1: size(x, 4)
                    names{i, j, k, l} = strcat('x', int2str(i), int2str(j), int2str(k), int2str(l));
                end
            end
        end
    end
    clear i j k l;
    
    %% definition of some labels in model
    model.A = [];
    model.sense = '';
    model.rhs = [];
    model.vtype = 'B';
    model.modelsense = 'min';
    model.varnames = names(:);
    
    obj_mat = ones(size(x));
    model.obj = obj_mat(:)';
    clear obj_mat;
    
    %% define the spoint = 1
    for i = 1: no_robot
        mat = zeros(size(names));
        cur_spoint = eval(strcat("spoint_", int2str(i)));
        mat(cur_spoint(1), cur_spoint(2), i, 1) = 1;
        model.A = [model.A; mat(:)'];
        model.sense = strcat(model.sense, '=');
        model.rhs = [model.rhs; 1];
    end
    clear mat i;
    
    %% define the epoint = 1
    for i = 1: no_robot
        mat = zeros(size(names));
        cur_epoint = eval(strcat("epoint_", int2str(i)));
        mat(cur_epoint(1), cur_epoint(2), i, T) = 1;
        model.A = [model.A; mat(:)'];
        model.sense = strcat(model.sense, '=');
        model.rhs = [model.rhs; 1];
    end
    clear mat i;
    
    %% for each time step, there is only one position for the robots
    for r = 1: no_robot
        for t = 1: T
            mat = zeros(size(names));
            mat(:, :, r, t) = 1;
            model.A = [model.A; mat(:)'];
            model.sense = strcat(model.sense, '=');
            model.rhs = [model.rhs; 1];
        end
    end
    clear mat t r;
    
    %% the obstacles are not allowed to enter
    mat = zeros(size(names));
    for i = 1: size(names, 1)
        for j = 1: size(names, 2)
            if map(i, j) == 1
                mat(i, j, :, :) = 1;
            end
        end
    end
    model.A = [model.A; mat(:)'];
    model.sense = strcat(model.sense, '=');
    model.rhs = [model.rhs; 0];
    clear mat;
    
    %% the constraint matrix must be sparse
    model.A = sparse(model.A);
    
    %% nonlinear constraints
    %     to make the path continuous:
    %     for t = 1: T-1
    %         x(i, j, no_robot, t)*x(i-1, j, no_robot, t+1)+...
    %         x(i, j, no_robot, t)*x(i, j-1, no_robot, t+1)+...
    %         x(i, j, no_robot, t)*x(i+1, j, no_robot, t+1)+...
    %         x(i, j, no_robot, t)*x(i, j+1, no_robot, t+1)+...
    %         x(i, j, no_robot, t)*x(i, j,   no_robot, t+1) <= 1
    %     end
    %     if the left side == 1, it means the robot was at this position at
    %     time = t; if the left side == 0, it means the robot wasn't there.
    count = 1;
    for r = 1: no_robot
        for t = 1: T - 1
            qc_mat = zeros(numel(names), numel(names));
            for i = 1: size(names, 1)
                for j = 1: size(names, 2)
                    qc_mat(names==strcat("x",int2str(i),int2str(j),int2str(r),int2str(t)),...
                        names==strcat("x",int2str(i),int2str(j),int2str(r),int2str(t+1))) = 1;
                    
                    if i-1>=1
                        qc_mat(names==strcat("x",int2str(i),int2str(j),int2str(r),int2str(t)),...
                            names==strcat("x",int2str(i-1),int2str(j),int2str(r),int2str(t+1))) = 1;
                    end
                    
                    if i+1<=size(names, 1)
                        qc_mat(names==strcat("x",int2str(i),int2str(j),int2str(r),int2str(t)),...
                            names==strcat("x",int2str(i+1),int2str(j),int2str(r),int2str(t+1))) = 1;
                    end
                    
                    if j-1>=1
                        qc_mat(names==strcat("x",int2str(i),int2str(j),int2str(r),int2str(t)),...
                            names==strcat("x",int2str(i),int2str(j-1),int2str(r),int2str(t+1))) = 1;
                    end
                    
                    if j+1<=size(names, 2)
                        qc_mat(names==strcat("x",int2str(i),int2str(j),int2str(r),int2str(t)),...
                            names==strcat("x",int2str(i),int2str(j+1),int2str(r),int2str(t+1))) = 1;
                    end
                end
            end
            model.quadcon(count).Qc = sparse(qc_mat);
            model.quadcon(count).q = zeros(numel(names), 1);
            model.quadcon(count).rhs = 1;
            model.quadcon(count).name = 'rot_cone';
            model.quadcon(count).sense = '>';
            count = count + 1;
        end
    end
    clear r t i j count;
    
    %% solve the model
    params.outputflag = 0;
    result = gurobi(model, params);
    
    if result.status == "OPTIMAL"
        break;
    end
end

solution = reshape(result.x, height, length, no_robot, T);
for i = 1: no_robot
    fprintf("The path for the %dth robot", i);
    disp(solution(:, :, i, :));
end