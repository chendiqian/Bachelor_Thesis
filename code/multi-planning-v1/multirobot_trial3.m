%% plan the path for multiple robots
% solve the problem of head-on and meet collision

clear; clc;

t_min = inf;
t_max = 25;

height = 5;
length = 5;
map = zeros(height, length);
map(2, 1:3) = 1;
map(4, 2:5) = 1;
imagesc(map);
colormap(flipud(gray));
axis square;
set(gca,'XTick',0.5:size(map,2),'YTick',0.5:size(map,1),...
    'XTickLabel','','YTicklabel','','dataaspect',[1 1 1],...
    'XGrid','on','YGrid','on','GridColor','k','GridAlpha',1)

spoint_1 = [1, 1];
epoint_1 = [1, 4];
spoint_2 = [1, 4];
epoint_2 = [1, 1];
% spoint_3 = [1, 5];
% epoint_3 = [5, 5];
% spoint_4 = [5, 5];
% epoint_4 = [1, 5];
no_robot = 1;

for i = 1: no_robot
    dis = sum(abs(eval(strcat("spoint_", int2str(i))) - eval(strcat("epoint_", int2str(i)))));
    if t_min > dis
        t_min = dis + 1;
    end
end
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
    
    model.obj = zeros(1, numel(x));
    
    Q_mat = cell2mat(struct2cell(load(strcat("Qmat_", int2str(height), "_", ...
        int2str(length), "_", int2str(no_robot), "_", int2str(T)))));
    model.Q = sparse(Q_mat);
    
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
    clear mat i j;
    
    %% no pair of robots should appear at the same point at a time
    % to avoid meet collision
    for t = 1: T
        for i = 1: size(names, 1)
            for j = 1: size(names, 2)
                mat = zeros(size(names));
                mat(i, j, :, t) = 1;
                model.A = [model.A; mat(:)'];
                model.sense = strcat(model.sense, '<');
                model.rhs = [model.rhs; 1];
            end
        end
    end
    clear t i j mat;
    
    %% to avoid head-on collision
    for t = 1: T - 1
        for i = 1: size(x, 1)
            for j = 1: size(x, 2)
                for r1 = 1: no_robot
                    for r2 = 1: no_robot
                        if r1 ~= r2
                            if i+1 <= size(x, 1)
                                mat = zeros(size(names));
                                mat(i,   j, r1, t  ) = 1;
                                mat(i+1, j, r2, t  ) = 1;
                                mat(i,   j, r2, t+1) = 1;
                                mat(i+1, j, r1, t+1) = 1;
                                model.A = [model.A; mat(:)'];
                                model.sense = strcat(model.sense, '<');
                                model.rhs = [model.rhs; 3];
                            end
                            
                            if j+1 <= size(x, 2)
                                mat = zeros(size(names));
                                mat(i, j,   r1, t  ) = 1;
                                mat(i, j+1, r2, t  ) = 1;
                                mat(i, j,   r2, t+1) = 1;
                                mat(i, j+1, r1, t+1) = 1;
                                model.A = [model.A; mat(:)'];
                                model.sense = strcat(model.sense, '<');
                                model.rhs = [model.rhs; 3];
                            end
                        end
                    end
                end
            end
        end
    end
    
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
                    qc_mat(sub2ind(size(names),i,j,r,t), sub2ind(size(names),i,j,r,t+1)) = 1;
                    
                    if i-1>=1
                        qc_mat(sub2ind(size(names),i,j,r,t),sub2ind(size(names),i-1,j,r,t+1)) = 1;
                    end
                    
                    if i+1<=size(names, 1)
                        qc_mat(sub2ind(size(names),i,j,r,t),sub2ind(size(names),i+1,j,r,t+1)) = 1;
                    end
                    
                    if j-1>=1
                        qc_mat(sub2ind(size(names),i,j,r,t),sub2ind(size(names),i,j-1,r,t+1)) = 1;
                    end
                    
                    if j+1<=size(names, 2)
                        qc_mat(sub2ind(size(names),i,j,r,t),sub2ind(size(names),i,j+1,r,t+1)) = 1;
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

for t = 1: T
    fprintf("The map at time step %d\n", t);
    disp(sum(solution(:, :, :, t), 3));
end