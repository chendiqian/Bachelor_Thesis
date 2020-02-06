clc; clear;
max_no_robot = 10;
height = 5;
length = 9;
t_min = 2;
t_max = 25;

tic
for no_robot = 1: max_no_robot
    for T = t_min: t_max
        x = zeros(height, length, no_robot, T);
        size_mat = [height, length, no_robot, T];
        
        Q_mat = zeros(numel(x), numel(x));
        for r = 1: no_robot
            for t = 1: T - 1
                for i = 1: size(x, 1)
                    for j = 1: size(x, 2)
                        Q_mat(sub2ind(size_mat, i,j,r,t),sub2ind(size_mat, i,j,r,t+1)) = (T - t) / 10;
                        
                        if i - 1 >= 1
                            Q_mat(sub2ind(size_mat, i,j,r,t),sub2ind(size_mat, i-1,j,r,t+1)) = t;
                        end
                        
                        if i + 1 <= size(x, 1)
                            Q_mat(sub2ind(size_mat, i,j,r,t),sub2ind(size_mat, i+1,j,r,t+1)) = t;
                        end
                        
                        if j - 1 >= 1
                            Q_mat(sub2ind(size_mat, i,j,r,t),sub2ind(size_mat, i,j-1,r,t+1)) = t;
                        end
                        
                        if j + 1 <= size(x, 2)
                            Q_mat(sub2ind(size_mat, i,j,r,t),sub2ind(size_mat, i,j+1,r,t+1)) = t;
                        end
                    end
                end
            end
        end
        Q_mat = Q_mat + Q_mat';
        toc
        save(strcat("Qmat/Qmat_", int2str(height),"_",int2str(length),"_",int2str(no_robot),"_",int2str(T), ".mat"), 'Q_mat');
    end
end