% simplfy the head-on collision condition
clc;clear;
t_min = 21;
t_max = 25;
height = 5;
length = 9;
max_no_robot = 6;

tic
for no_robot = 2: 2
    for T = 11: 11
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
        model.varnames = names(:);
        
        model.obj = zeros(1, numel(names));
        
        Q_mat = cell2mat(struct2cell(load(strcat("Qmat/Qmat_", int2str(height), "_", ...
            int2str(length), "_", int2str(no_robot), "_", int2str(T)))));
        model.Q = sparse(Q_mat);
        clear Q_mat;
        
        %% define the spoint = 1
        %         for i = 1: no_robot
        %             mat = zeros(size(names));
        %             cur_spoint = eval(strcat("spoint_", int2str(i)));
        %             mat(cur_spoint(1), cur_spoint(2), i, 1) = 1;
        %             model.A = [model.A; mat(:)'];
        %             model.sense = strcat(model.sense, '=');
        %             model.rhs = [model.rhs; 1];
        %         end
        %         clear mat i;
        
        %% define the epoint = 1
        %         for i = 1: no_robot
        %             mat = zeros(size(names));
        %             cur_epoint = eval(strcat("epoint_", int2str(i)));
        %             mat(cur_epoint(1), cur_epoint(2), i, T) = 1;
        %             model.A = [model.A; mat(:)'];
        %             model.sense = strcat(model.sense, '=');
        %             model.rhs = [model.rhs; 1];
        %         end
        %         clear mat i;
        
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
        % maybe except the s/epoint
        %         mat = zeros(size(names));
        %         for i = 1: size(names, 1)
        %             for j = 1: size(names, 2)
        %                 if map(i, j) == 1
        %                     mat(i, j, :, :) = 1;
        %                 end
        %             end
        %         end
        %         model.A = [model.A; mat(:)'];
        %         model.sense = strcat(model.sense, '=');
        %         model.rhs = [model.rhs; 0];
        %         clear mat i j;
        
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
            for i = 1: size(names, 1)
                for j = 1: size(names, 2)
                    if i+1 <= size(names, 1)
                        mat = zeros(size(names));
                        mat(i,   j, :, t  ) = 1;
                        mat(i+1, j, :, t  ) = 1;
                        mat(i,   j, :, t+1) = 1;
                        mat(i+1, j, :, t+1) = 1;
                        model.A = [model.A; mat(:)'];
                        model.sense = strcat(model.sense, '<');
                        model.rhs = [model.rhs; 3];
                    end
                    
                    if j+1 <= size(names, 2)
                        mat = zeros(size(names));
                        mat(i, j,   :, t  ) = 1;
                        mat(i, j+1, :, t  ) = 1;
                        mat(i, j,   :, t+1) = 1;
                        mat(i, j+1, :, t+1) = 1;
                        model.A = [model.A; mat(:)'];
                        model.sense = strcat(model.sense, '<');
                        model.rhs = [model.rhs; 3];
                    end
                end
            end
        end
        
        %% the constraint matrix must be sparse
        % model.A = sparse(model.A);
        
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
        
        toc
        save(strcat("model_",int2str(height),"_",int2str(length),"_",int2str(no_robot),...
            "_",int2str(T)), "model");
        clear model;
    end
end