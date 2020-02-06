%% plan the path for multiple robots
% linearization

clear; clc;

t_min = 0;
t_max = 25;

height = 5;
length = 9;
map = cell2mat(struct2cell(load("map.mat")));
imagesc(map);
colormap(flipud(gray));
axis square;
set(gca,'XTick',0.5:size(map,2),'YTick',0.5:size(map,1),...
    'XTickLabel','','YTicklabel','','dataaspect',[1 1 1],...
    'XGrid','on','YGrid','on','GridColor','k','GridAlpha',1)

spoint_1 = [3, 1];
epoint_1 = [5, 6];
spoint_2 = [3, 2];
epoint_2 = [3, 8];
spoint_3 = [3, 9];
epoint_3 = [3, 3];
no_robot = 1;

for i = 1: no_robot
    dis = sum(abs(eval(strcat("spoint_", int2str(i))) - eval(strcat("epoint_", int2str(i)))));
    if t_min < dis
        t_min = dis + 1;
    end
end

for T = t_min: t_max
    %% initialize a 4-dimensional matrix
    fprintf("time step = %d\n", T);
    
    %% give the names of variables according to the position of it
    names = cell(height, length, no_robot, T);
    for i=1: height
        for j=1: length
            for k=1: no_robot
                for l=1: T
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
    
    var_num = height * length * no_robot * T + no_robot * (T-1) * (4*3 + ...
        4 * (length-2) * 2 + 4 * (height-2) * 2 + 5 * (height-2) * (length-2));
    
    model.varnames = cell(var_num, 1); % column wise
    model.varnames(1: numel(names)) = names(:);
    
    model.obj = zeros(1, var_num);
    
    %% definition of quadratic obj matrix
    % Q_mat = zeros(numel(names), numel(names));
    % the number of auxiliary variables should be (no_robot * (T-1) * (4*3
    % + 4*(length-2)*2 + 4*(height-2)*2 + 5*(height-2)*(length-2)))
    count = numel(names);
    for r = 1: no_robot
        for t = 1: T - 1
            % a constraint to make the path continuous
            mat_continuous = zeros(1, var_num);
            
            for i = 1: size(names, 1)
                for j = 1: size(names, 2)
                    % Q_mat(sub2ind(size(names), i,j,r,t),sub2ind(size(names), i,j,r,t+1)) = (T - t) / 10;
                    % add a new auxiliary variable
                    count = count + 1;
                    model.varnames{count} = strcat('aux_',int2str(r),...
                        '_',int2str(t),'_',int2str(i),'_',int2str(j),'_',...
                        int2str(i),'_',int2str(j));
                    model.obj(count) = (T - t) / 10;
                    
                    % y >= x1 + x2 - 1
                    mat = zeros(1, var_num);
                    mat(count) = -1;
                    mat(sub2ind(size(names),i,j,r,t)) = 1;
                    mat(sub2ind(size(names),i,j,r,t+1)) = 1;
                    model.A = [model.A; mat];
                    model.sense = strcat(model.sense, '<');
                    model.rhs = [model.rhs; 1];
                    % y <= x1;
                    mat = zeros(1, var_num);
                    mat(count) = -1;
                    mat(sub2ind(size(names),i,j,r,t)) = 1;
                    model.A = [model.A; mat];
                    model.sense = strcat(model.sense, '>');
                    model.rhs = [model.rhs; 0];
                    % y <= x2
                    mat = zeros(1, var_num);
                    mat(count) = -1;
                    mat(sub2ind(size(names),i,j,r,t+1)) = 1;
                    model.A = [model.A; mat];
                    model.sense = strcat(model.sense, '>');
                    model.rhs = [model.rhs; 0];
                    % y=[0,1]
                    
                    mat_continuous(count) = 1;
                    
                    
                    if i - 1 >= 1
                        % Q_mat(sub2ind(size(names), i,j,r,t),sub2ind(size(names), i-1,j,r,t+1)) = t;
                        count = count + 1;
                        model.varnames{count} = strcat('aux_',int2str(r),...
                            '_',int2str(t),'_',int2str(i),'_',int2str(j),'_',...
                            int2str(i-1),'_',int2str(j));
                        model.obj(count) = t;
                        
                        % y >= x1 + x2 - 1
                        mat = zeros(1, var_num);
                        mat(count) = -1;
                        mat(sub2ind(size(names),i,j,r,t)) = 1;
                        mat(sub2ind(size(names),i-1,j,r,t+1)) = 1;
                        model.A = [model.A; mat];
                        model.sense = strcat(model.sense, '<');
                        model.rhs = [model.rhs; 1];
                        % y <= x1;
                        mat = zeros(1, var_num);
                        mat(count) = -1;
                        mat(sub2ind(size(names),i,j,r,t)) = 1;
                        model.A = [model.A; mat];
                        model.sense = strcat(model.sense, '>');
                        model.rhs = [model.rhs; 0];
                        % y <= x2
                        mat = zeros(1, var_num);
                        mat(count) = -1;
                        mat(sub2ind(size(names),i-1,j,r,t+1)) = 1;
                        model.A = [model.A; mat];
                        model.sense = strcat(model.sense, '>');
                        model.rhs = [model.rhs; 0];
                        % y=[0,1]
                        
                        mat_continuous(count) = 1;
                    end
                    
                    if i + 1 <= size(names, 1)
                        % Q_mat(sub2ind(size(names), i,j,r,t),sub2ind(size(names), i+1,j,r,t+1)) = t;
                        count = count + 1;
                        model.varnames{count} = strcat('aux_',int2str(r),...
                        '_',int2str(t),'_',int2str(i),'_',int2str(j),'_',...
                        int2str(i+1),'_',int2str(j));
                        model.obj(count) = t;
                        
                        % y >= x1 + x2 - 1
                        mat = zeros(1, var_num);
                        mat(count) = -1;
                        mat(sub2ind(size(names),i,j,r,t)) = 1;
                        mat(sub2ind(size(names),i+1,j,r,t+1)) = 1;
                        model.A = [model.A; mat];
                        model.sense = strcat(model.sense, '<');
                        model.rhs = [model.rhs; 1];
                        % y <= x1;
                        mat = zeros(1, var_num);
                        mat(count) = -1;
                        mat(sub2ind(size(names),i,j,r,t)) = 1;
                        model.A = [model.A; mat];
                        model.sense = strcat(model.sense, '>');
                        model.rhs = [model.rhs; 0];
                        % y <= x2
                        mat = zeros(1, var_num);
                        mat(count) = -1;
                        mat(sub2ind(size(names),i+1,j,r,t+1)) = 1;
                        model.A = [model.A; mat];
                        model.sense = strcat(model.sense, '>');
                        model.rhs = [model.rhs; 0];
                        % y=[0,1]
                        
                        mat_continuous(count) = 1;
                    end
                    
                    if j - 1 >= 1
                        % Q_mat(sub2ind(size(names), i,j,r,t),sub2ind(size(names), i,j-1,r,t+1)) = t;
                        count = count + 1;
                        model.varnames{count} = strcat('aux_',int2str(r),...
                        '_',int2str(t),'_',int2str(i),'_',int2str(j),'_',...
                        int2str(i),'_',int2str(j-1));
                        model.obj(count) = t;
                        
                        % y >= x1 + x2 - 1
                        mat = zeros(1, var_num);
                        mat(count) = -1;
                        mat(sub2ind(size(names),i,j,r,t)) = 1;
                        mat(sub2ind(size(names),i,j-1,r,t+1)) = 1;
                        model.A = [model.A; mat];
                        model.sense = strcat(model.sense, '<');
                        model.rhs = [model.rhs; 1];
                        % y <= x1;
                        mat = zeros(1, var_num);
                        mat(count) = -1;
                        mat(sub2ind(size(names),i,j,r,t)) = 1;
                        model.A = [model.A; mat];
                        model.sense = strcat(model.sense, '>');
                        model.rhs = [model.rhs; 0];
                        % y <= x2
                        mat = zeros(1, var_num);
                        mat(count) = -1;
                        mat(sub2ind(size(names),i,j-1,r,t+1)) = 1;
                        model.A = [model.A; mat];
                        model.sense = strcat(model.sense, '>');
                        model.rhs = [model.rhs; 0];
                        % y=[0,1]
                        
                        mat_continuous(count) = 1;
                    end
                    
                    if j + 1 <= size(names, 2)
                        % Q_mat(sub2ind(size(names), i,j,r,t),sub2ind(size(names), i,j+1,r,t+1)) = t;
                        count = count + 1;
                        model.varnames{count} = strcat('aux_',int2str(r),...
                        '_',int2str(t),'_',int2str(i),'_',int2str(j),'_',...
                        int2str(i),'_',int2str(j+1));
                        model.obj(count) = t;
                        
                        % y >= x1 + x2 - 1
                        mat = zeros(1, var_num);
                        mat(count) = -1;
                        mat(sub2ind(size(names),i,j,r,t)) = 1;
                        mat(sub2ind(size(names),i,j+1,r,t+1)) = 1;
                        model.A = [model.A; mat];
                        model.sense = strcat(model.sense, '<');
                        model.rhs = [model.rhs; 1];
                        % y <= x1;
                        mat = zeros(1, var_num);
                        mat(count) = -1;
                        mat(sub2ind(size(names),i,j,r,t)) = 1;
                        model.A = [model.A; mat];
                        model.sense = strcat(model.sense, '>');
                        model.rhs = [model.rhs; 0];
                        % y <= x2
                        mat = zeros(1, var_num);
                        mat(count) = -1;
                        mat(sub2ind(size(names),i,j+1,r,t+1)) = 1;
                        model.A = [model.A; mat];
                        model.sense = strcat(model.sense, '>');
                        model.rhs = [model.rhs; 0];
                        % y=[0,1]
                        
                        mat_continuous(count) = 1;
                    end
                end
            end
            model.A = [model.A; mat_continuous];
            model.sense = strcat(model.sense, '>');
            model.rhs = [model.rhs; 1];
        end
    end
    % model.Q = sparse(Q_mat);
    
    %% define the spoint = 1
    for i = 1: no_robot
        % mat = zeros(size(names));
        mat = zeros(1, var_num);
        cur_spoint = eval(strcat("spoint_", int2str(i)));
        mat(sub2ind(size(names), cur_spoint(1), cur_spoint(2), i, 1)) = 1;
        model.A = [model.A; mat];
        model.sense = strcat(model.sense, '=');
        model.rhs = [model.rhs; 1];
    end
    clear mat i;
    
    %% define the epoint = 1
    for i = 1: no_robot
        % mat = zeros(size(names));
        mat = zeros(1, var_num);
        cur_epoint = eval(strcat("epoint_", int2str(i)));
        mat(sub2ind(size(names), cur_epoint(1), cur_epoint(2), i, T)) = 1;
        model.A = [model.A; mat];
        model.sense = strcat(model.sense, '=');
        model.rhs = [model.rhs; 1];
    end
    clear mat i;
    
    %% for each time step, there is only one position for the robots
    for r = 1: no_robot
        for t = 1: T
            mat = zeros(size(names));
            mat(:, :, r, t) = 1;
            model.A = [model.A; [mat(:)', zeros(1, var_num-numel(names))]];
            model.sense = strcat(model.sense, '=');
            model.rhs = [model.rhs; 1];
        end
    end
    clear mat t r;
    
    %% the obstacles are not allowed to enter
    % maybe except the s/epoint
    mat = zeros(size(names));
    for i = 1: size(names, 1)
        for j = 1: size(names, 2)
            if map(i, j) == 1
                mat(i, j, :, :) = 1;
            end
        end
    end
    model.A = [model.A; [mat(:)', zeros(1, var_num-numel(names))]];
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
                model.A = [model.A; [mat(:)', zeros(1, var_num-numel(names))]];
                model.sense = strcat(model.sense, '<');
                model.rhs = [model.rhs; 1];
            end
        end
    end
    clear t i j mat;
    
    %% to avoid head-on collision
    % unfinished
%     for t = 1: T - 1
%         for i = 1: size(names, 1)
%             for j = 1: size(names, 2)
%                 mat = zeros(1, var_num);
%                 for r = 1: no_robot
%                     if i+1 <= size(names, 1)
%                         mat(find(model.varnames == strcat("aux_",int2str(r),"_",int2str(t),"_",int2str(i),"_",int2str(j),"_",...
%                         int2str(i+1),"_",int2str(j)))) = 1;
%                     end
%                     
%                     
%                 end
%             end
%         end
%     end
    
    %% the constraint matrix must be sparse
    model.A = sparse(model.A);
    
    %% solve the model
    params.outputflag = 0;
    result = gurobi(model, params);
    
    if result.status == "OPTIMAL"
        break;
    end
end

solution = reshape(result.x(1: numel(names)), height, length, no_robot, T);
for i = 1: no_robot
    fprintf("The path for the %dth robot", i);
    disp(solution(:, :, i, :));
end

for t = 1: T
    fprintf("The map at time step %d\n", t);
    disp(sum(solution(:, :, :, t), 3));
end