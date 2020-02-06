function MIP(zone_i, zone_j)
global zone robot times_plan total_time_plan

submap = zone(zone_i, zone_j).submap;
robot_local = robot(zone(zone_i, zone_j).robot);
no_robot = length(robot_local);
if no_robot == 0
    fprintf("No robot in zone!\n");
    return
end

states = [robot_local.rstate]';
spoint = reshape([robot_local.spoint], 2, length(zone(zone_i, zone_j).robot))' - zone(zone_i, zone_j).corner(1:2) + [1, 1];
epoint = reshape([robot_local.epoint], 2, length(zone(zone_i, zone_j).robot))' - zone(zone_i, zone_j).corner(1:2) + [1, 1];

obs_ind = find(submap == 1);

[row, column] = size(submap);

t_min = max(sum(abs(spoint - epoint), 2))+3;
t_max = 30;

%% begin
for T = t_min: 2: t_max
    %% varnames of the matrix
    names = cell(2, no_robot, T);
    for i = 1:2
        for j = 1: no_robot
            for k = 1: T
                names{i,j,k} = int2str(sub2ind(size(names), i, j, k));
            end
        end
    end
    
    %% basic parameters
    num_constraints = 2 * no_robot + 4 * no_robot * (T-1) + ...
        5 * T * no_robot * length(obs_ind) + 5 * T * (no_robot * (no_robot-1) / 2) ...
        + 9 * (T-1) * (no_robot * (no_robot-1) / 2) + 4 * T * no_robot;
    num_varnames = numel(names) + length(obs_ind) * T * no_robot * 4 ...
        + T * (no_robot * (no_robot-1) / 2) * 4 ...
        + (T-1) * (no_robot * (no_robot-1) / 2) * 8 + 2 * T * no_robot;
    
    model.A = zeros(num_constraints, num_varnames);
    index_cons = 0;
    model.varnames = cell(num_varnames, 1);
    model.varnames(1:numel(names)) = names(:);
    model.rhs = zeros(num_constraints, 1);
    model.modelsense = 'min';
    model.sense = blanks(num_constraints);
    model.vtype = 'I';
    lb = zeros(num_varnames, 1);
    lb(1:numel(names)) = ones(numel(names), 1);
    ub = ones(num_varnames, 1);
    for i = 1: numel(names)
        if rem(i, 2) == 0
            ub(i) = column;
        else
            ub(i) = row;
        end
    end
    
    %% static obstacles
    %  xi <=  xmin + M*ti1
    % -xi <= -xmax + M*ti2
    %  yi <=  ymin + M*ti3
    % -yi <= -ymax + M*ti4
    % sigma ti <= 3
    % obs_num * T * no_robot * 4 new variables
    M = 100000; % a large enough number
    count = 0; % every index after numel(names)
    obj = zeros(1, num_varnames);
    for r = 1: no_robot
        % define the spoint and epoint
        model.A(index_cons+1, sub2ind(size(names), 1, r, 1)) = 1;
        model.A(index_cons+2, sub2ind(size(names), 2, r, 1)) = 1;
        
        model.rhs(index_cons+1 : index_cons+2) = [spoint(r,1);spoint(r,2)];
        
        model.sense(index_cons+1 : index_cons+2) = '==';
        
        index_cons = index_cons + 2;
        
        for t = 1: T
            if t < T
                model.A([index_cons+1,index_cons+3], sub2ind(size(names), 1, r, t)) = 1;
                model.A([index_cons+1,index_cons+3], sub2ind(size(names), 1, r, t+1)) = -1;
                model.A([index_cons+1,index_cons+3], sub2ind(size(names), 2, r, t)) = 1;
                model.A([index_cons+1,index_cons+3], sub2ind(size(names), 2, r, t+1)) = -1;
                model.A([index_cons+2,index_cons+4], sub2ind(size(names), 1, r, t)) = 1;
                model.A([index_cons+2,index_cons+4], sub2ind(size(names), 1, r, t+1)) = -1;
                model.A([index_cons+2,index_cons+4], sub2ind(size(names), 2, r, t)) = -1;
                model.A([index_cons+2,index_cons+4], sub2ind(size(names), 2, r, t+1)) = 1;
                
                model.rhs(index_cons+1 : index_cons+4) = [1; 1; -1; -1];
                model.sense(index_cons+1 : index_cons+4) = '<<>>';
                
                index_cons = index_cons + 4;
            end
            % object formula
            % xi - 2 <= wi_x, -xi + 2 <= wi_x
            % yi - 1 <= wi_y, -yi + 1 <= wi_y
            model.A(index_cons+1, sub2ind(size(names), 1, r, t)) = 1;
            model.A(index_cons+1, numel(names)+count + 1) = -1;
            model.A(index_cons+2, sub2ind(size(names), 1, r, t)) = -1;
            model.A(index_cons+2, numel(names)+count + 1) = -1;
            model.A(index_cons+3, sub2ind(size(names), 2, r, t)) = 1;
            model.A(index_cons+3, numel(names)+count + 2) = -1;
            model.A(index_cons+4, sub2ind(size(names), 2, r, t)) = -1;
            model.A(index_cons+4, numel(names)+count + 2) = -1;
            
            obj(numel(names) + count + 1) = t^2;
            obj(numel(names) + count + 2) = t^2;
            
            model.rhs(index_cons+1 : index_cons+4) = [epoint(r,1);
                -epoint(r,1);
                epoint(r,2);
                -epoint(r,2)];
            
            model.sense(index_cons+1 : index_cons+4) = '<<<<';
            
            lb(numel(names) + count + 1) = 0;
            ub(numel(names) + count + 1) = row-1;
            lb(numel(names) + count + 2) = 0;
            ub(numel(names) + count + 2) = column-1;
            
            model.varnames{numel(names)+count+1} = int2str(count);
            count = count + 1;
            model.varnames{numel(names)+count+1} = int2str(count);
            
            count = count + 1;
            index_cons = index_cons + 4;
            
            for i = 1: size(obs_ind, 1)
                [X, Y] = ind2sub(size(submap), obs_ind(i));
                model.A(index_cons+1, sub2ind(size(names), 1, r, t)) = 1;
                model.A(index_cons+2, sub2ind(size(names), 1, r, t)) = -1;
                model.A(index_cons+3, sub2ind(size(names), 2, r, t)) = 1;
                model.A(index_cons+4, sub2ind(size(names), 2, r, t)) = -1;
                model.A(index_cons+1:index_cons+4, numel(names)+count+1 : numel(names)+count+4) = -M * eye(4);
                model.A(index_cons+5, numel(names)+count+1 : numel(names)+count+4) = 1;
                
                % not the s or epoint, not allowed to enter
                if ~isequal([X, Y], epoint(r, :)) && ~isequal([X, Y], spoint(r, :)) && states(r) ~= "idle" && states(r) ~= "to_task"
                    model.rhs(index_cons+1 : index_cons+5) = [X-1; -X-1; Y-1; -Y-1; 3];
                else
                    % else allowed
                    model.rhs(index_cons+1 : index_cons+5) = [X-1; -X-1; Y-1; -Y-1; 5];
                end
                
                for h = 1:4
                    model.varnames{numel(names)+count+1} = int2str(count);
                    count = count + 1;
                end
                
                model.sense(index_cons+1 : index_cons+5) = '<<<<<';
                
                index_cons = index_cons + 5;
            end
        end
    end
    clear h i
    
    
    for t = 1: T
        for p = 1: no_robot - 1
            for q = p + 1: no_robot
                % meet collision
                % to keep the distance between every two robots
                % for every t, every two robots
                % -xp + xq <= -1 + M * b1
                %  xp - xq <= -1 + M * b2
                % -yp + yq <= -1 + M * b3
                %  yp - yq <= -1 + M * b4
                % sigma b <= 3
                model.A(index_cons+1, sub2ind(size(names), 1, p, t)) = -1;
                model.A(index_cons+1, sub2ind(size(names), 1, q, t)) = 1;
                model.A(index_cons+2, sub2ind(size(names), 1, p, t)) = 1;
                model.A(index_cons+2, sub2ind(size(names), 1, q, t)) = -1;
                model.A(index_cons+3, sub2ind(size(names), 2, p, t)) = -1;
                model.A(index_cons+3, sub2ind(size(names), 2, q, t)) = 1;
                model.A(index_cons+4, sub2ind(size(names), 2, p, t)) = 1;
                model.A(index_cons+4, sub2ind(size(names), 2, q, t)) = -1;
                model.A(index_cons+1:index_cons+4, numel(names)+count+1 : numel(names)+count+4) = -M * eye(4);
                model.A(index_cons+5, numel(names)+count+1 : numel(names)+count+4) = 1;
                
                model.rhs(index_cons+1 : index_cons+5) = [-1; -1; -1; -1; 3];
                
                for i = 1:4
                    model.varnames{numel(names)+count+1} = int2str(count);
                    count = count + 1;
                end
                
                model.sense(index_cons+1 : index_cons+5) = '<<<<<';
                
                index_cons = index_cons + 5;
                
                
                if t < T
                    % head-on collision
                    % x1t = x2(t+1), y1t = y2(t+1)
                    % x2t = x1(t+1), y2t = y1(t+1)
                    %
                    % x1t <= x2(t+1) - 1 + Mt, x1t >= x2(t+1) + 1 - Mt
                    % y1t <= y2(t+1) - 1 + Mt, y1t >= y2(t+1) + 1 - Mt
                    % x2t <= x1(t+1) - 1 + Mt, x2t >= x1(t+1) + 1 - Mt
                    % y2t <= y1(t+1) - 1 + Mt, y2t >= y1(t+1) + 1 - Mt
                    % at least one t is 0: sigma t <= 7
                    model.A(index_cons+1, sub2ind(size(names), 1, p, t)) = 1;
                    model.A(index_cons+1, sub2ind(size(names), 1, q, t+1)) = -1;
                    model.A(index_cons+2, sub2ind(size(names), 1, p, t)) = -1;
                    model.A(index_cons+2, sub2ind(size(names), 1, q, t+1)) = 1;
                    model.A(index_cons+3, sub2ind(size(names), 2, p, t)) = 1;
                    model.A(index_cons+3, sub2ind(size(names), 2, q, t+1)) = -1;
                    model.A(index_cons+4, sub2ind(size(names), 2, p, t)) = -1;
                    model.A(index_cons+4, sub2ind(size(names), 2, q, t+1)) = 1;
                    model.A(index_cons+5, sub2ind(size(names), 1, q, t)) = 1;
                    model.A(index_cons+5, sub2ind(size(names), 1, p, t+1)) = -1;
                    model.A(index_cons+6, sub2ind(size(names), 1, q, t)) = -1;
                    model.A(index_cons+6, sub2ind(size(names), 1, p, t+1)) = 1;
                    model.A(index_cons+7, sub2ind(size(names), 2, q, t)) = 1;
                    model.A(index_cons+7, sub2ind(size(names), 2, p, t+1)) = -1;
                    model.A(index_cons+8, sub2ind(size(names), 2, q, t)) = -1;
                    model.A(index_cons+8, sub2ind(size(names), 2, p, t+1)) = 1;
                    model.A(index_cons+1:index_cons+8, numel(names)+count+1 : numel(names)+count+8) = -M * eye(8);
                    model.A(index_cons+9, numel(names)+count+1:numel(names)+count+8) = 1;
                    
                    model.rhs(index_cons+1 : index_cons+9) = [-1; -1; -1; -1; -1; -1; -1; -1; 7];
                    
                    for i = 1:8
                        model.varnames{numel(names)+count+1} = int2str(count);
                        count = count + 1;
                    end
                    
                    model.sense(index_cons+1 : index_cons+9) = '<<<<<<<<<';
                    
                    index_cons = index_cons + 9;
                end
            end
        end
    end
    
    %% solve
    model.A = sparse(model.A);
    model.lb = lb;
    model.ub = ub;
    model.obj = obj;
    
    params.outputflag = 0;
    results = gurobi(model, params);
    if results.status == "OPTIMAL"
        break;
    end
end

x = reshape(results.x(1:numel(names)), 2, no_robot, T);

path = cell(1, no_robot);
for i = 1: no_robot
    if robot(zone(zone_i, zone_j).robot(i)).path_get == 1
        path{1, i} = reshape(x(:, i, :), size(x, 1), size(x, 3))';
        
        [Lia, Locb] = ismember(epoint(i, :), path{1, i}, 'rows');
        
        if Lia == 1 && Locb < size(path{1, i}, 1) && robot(zone(zone_i, zone_j).robot(i)).rstate ~= "idle"
            path{1, i}(Locb+1:end, :) = [];
        end
        
        robot(zone(zone_i, zone_j).robot(i)).path = path{1, i} + zone(zone_i, zone_j).corner(1:2) - [1, 1];
    end
end

times_plan = times_plan + 1;
total_time_plan = total_time_plan + results.runtime;
fprintf("Solve time: %f\n\n", results.runtime);