clear;clc;

height = 10;
length = 10;
% map = cell2mat(struct2cell(load("map.mat")));
map = zeros(height, length);
map(2, 1: 8) = 1;
map(5, 3:10) = 1;
%{
imagesc(map);
colormap(flipud(gray));
axis square;
set(gca,'XTick',0.5:size(map,2),'YTick',0.5:size(map,1),...
    'XTickLabel','','YTicklabel','','dataaspect',[1 1 1],...
    'XGrid','on','YGrid','on','GridColor','k','GridAlpha',1)
%}

spoint = [1, 1;
          3, 2;
          3, 9];
epoint = [3, 1;
          10, 8;
          1, 6];
no_robot = 1;

t_min = max(sum(abs(spoint - epoint), 2)) + 1;
t_max = 25;

%% begin
for T = t_min: t_max
    fprintf("time step = %d\n", T);
    
    %% varnames
    names = cell(2, no_robot, T);
    for i = 1:2
        for j = 1: no_robot
            for k = 1: T
                names{i,j,k} = strcat('x_',int2str(i),'_',int2str(j),'_',int2str(k));
            end
        end
    end
    
    %% basic parameters
    model.A = [];
    model.varnames = names(:);
    model.rhs = [];
    model.modelsense = 'min';
    model.sense = '';
    model.vtype = 'I';
    model.lb = ones(numel(names), 1);
    ub = height * ones(numel(names), 1);
    for i = 1: numel(names)
        if rem(i, 2) == 0
            ub(i) = length;
        end
    end
    model.ub = ub;
    clear ub;
    
    % min sigma t * sum(([x,y] - epoint)^2)
    obj = zeros(1, numel(names));
    for r = 1: no_robot
        cur_epoint = epoint(r,:);
        for t = 1: T
            obj(sub2ind(size(names), 1, r, t)) = -2 * t * cur_epoint(1);
            obj(sub2ind(size(names), 2, r, t)) = -2 * t * cur_epoint(2);
        end
    end
    model.obj = obj;
    clear obj cur_epoint
    
    Q = zeros(numel(names), numel(names));
    for r = 1: no_robot
        for t = 1: T
            Q(sub2ind(size(names), 1, r, t), sub2ind(size(names), 1, r, t)) = t;
            Q(sub2ind(size(names), 2, r, t), sub2ind(size(names), 2, r, t)) = t;
        end
    end
    
    
    %% define the spoint and epoint
    for i = 1: no_robot
        % spoint
        cur_spoint = spoint(i,:);
        mat = zeros(1, numel(names));
        mat(sub2ind(size(names), 1, i, 1)) = 1;
        model.A = [model.A; mat];
        model.rhs = [model.rhs; cur_spoint(1)];
        model.sense = strcat(model.sense, '=');
        
        mat = zeros(1, numel(names));
        mat(sub2ind(size(names), 2, i, 1)) = 1;
        model.A = [model.A; mat];
        model.rhs = [model.rhs; cur_spoint(2)];
        model.sense = strcat(model.sense, '=');
        
        % epoint
        cur_epoint = epoint(i,:);
        mat = zeros(1, numel(names));
        mat(sub2ind(size(names), 1, i, T)) = 1;
        model.A = [model.A; mat];
        model.rhs = [model.rhs; cur_epoint(1)];
        model.sense = strcat(model.sense, '=');
        
        mat = zeros(1, numel(names));
        mat(sub2ind(size(names), 2, i, T)) = 1;
        model.A = [model.A; mat];
        model.rhs = [model.rhs; cur_epoint(2)];
        model.sense = strcat(model.sense, '=');
    end
    
    %% the path must be continuous
    % dx, dy = -1, 0, 1
    % dx * dy = 0
    % -1 <= dx + dy <= 1
    % -1 <= dx - dy <= 1
    for i = 1: no_robot
        for t = 1: T-1
            mat = zeros(1, numel(names));
            mat(sub2ind(size(names), 1, i, t)) = 1;
            mat(sub2ind(size(names), 1, i, t+1)) = -1;
            mat(sub2ind(size(names), 2, i, t)) = 1;
            mat(sub2ind(size(names), 2, i, t+1)) = -1;
            
            model.A = [model.A; mat; mat];
            model.rhs = [model.rhs; 1; -1];
            model.sense = strcat(model.sense, '<>');
            
            mat = zeros(1, numel(names));
            mat(sub2ind(size(names), 1, i, t)) = 1;
            mat(sub2ind(size(names), 1, i, t+1)) = -1;
            mat(sub2ind(size(names), 2, i, t)) = -1;
            mat(sub2ind(size(names), 2, i, t+1)) = 1;
            
            model.A = [model.A; mat; mat];
            model.rhs = [model.rhs; 1; -1];
            model.sense = strcat(model.sense, '<>');
        end
    end
    
    %% static obstacles
    %  xi <=  xmin + M*ti1
    % -xi <= -xmax + M*ti2
    %  yi <=  ymin + M*ti3
    % -yi <= -ymax + M*ti4
    % sigma ti <= 3
    obs_ind = find(map == 1);
    M = 100000;
    count = 0;
    for i = 1: size(obs_ind, 1)
        [X, Y] = ind2sub(size(map), obs_ind(i));
        for t = 1: T
            for r = 1: no_robot
                count = count + 1;
                % add a new variable t_i1
                mat = zeros(1, size(model.A, 2)+1);
                mat(sub2ind(size(names), 1, r, t)) = 1;
                mat(size(model.A, 2)+1) = -M;
                model.A = [model.A, zeros(size(model.A,1), 1); mat];
                model.rhs = [model.rhs; X-1];
                
                % add a new variable t_i2
                mat = zeros(1, size(model.A, 2)+1);
                mat(sub2ind(size(names), 1, r, t)) = -1;
                mat(size(model.A, 2)+1) = -M;
                model.A = [model.A, zeros(size(model.A,1), 1); mat];
                model.rhs = [model.rhs; -X-1];
                
                % t_i3
                mat = zeros(1, size(model.A, 2)+1);
                mat(sub2ind(size(names), 2, r, t)) = 1;
                mat(size(model.A, 2)+1) = -M;
                model.A = [model.A, zeros(size(model.A,1), 1); mat];
                model.rhs = [model.rhs; Y-1];
                
                % t_i4
                mat = zeros(1, size(model.A, 2)+1);
                mat(sub2ind(size(names), 2, r, t)) = -1;
                mat(size(model.A, 2)+1) = -M;
                model.A = [model.A, zeros(size(model.A,1), 1); mat];
                model.rhs = [model.rhs; -Y-1];
                
                % add new variables
                model.varnames = [model.varnames; 
                    strcat('t_',int2str(count),'1');
                    strcat('t_',int2str(count),'2');
                    strcat('t_',int2str(count),'3');
                    strcat('t_',int2str(count),'4')];
                model.lb = [model.lb; 0;0;0;0];
                model.ub = [model.ub; 1;1;1;1];
                model.sense = strcat(model.sense, '<<<<');
                model.obj = [model.obj,0,0,0,0];
                
                % sigma t <= 3
                mat = zeros(1, size(model.A, 2));
                mat((size(model.A, 2)-3) : size(model.A, 2)) = 1;
                model.A = [model.A; mat];
                model.rhs = [model.rhs; 3];
                model.sense = strcat(model.sense, '<');
            end
        end
    end
            
    
    %% meet collision
    
    %% head-on collision
    
    %% solve
    model.A = sparse(model.A);
    Q_final = zeros(size(model.A, 2), size(model.A, 2));
    Q_final(1:numel(names), 1:numel(names)) = Q;
    model.Q = sparse(Q_final);
    
    params.outputflag = 0;
    results = gurobi(model, params);
    if results.status == "OPTIMAL"
        break;
    end
end

x = reshape(results.x(1:numel(names)), 2, no_robot, T);
disp(results.runtime);