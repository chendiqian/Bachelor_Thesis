clc; clear;
% maximize
% x + y + z
% subject to
% x + y + z <= 3
% xy + yz + xz <= 2
% xy + xz >= 1
names = {'x', 'y', 'z', 'a1', 'a2', 'a3'};
model.varnames = names ;
model.vtype = 'B';
% Set objective : x
model.obj = [ 1 1 1 0 0 0];
model.modelsense = 'max';
model.A = [];
model.rhs = [];
model.sense = '';
% Add constraint : x + y + z = 3
model.A = [model.A; [1 1 1 0 0 0]];
model.rhs = [model.rhs; 3];
model.sense = strcat(model.sense, '<');
% Add rotated cone : xy + yz + xz <= 2
%{
model.quadcon(1).Qc = sparse([
0 1 0;
0 0 1;
1 0 0]);
model.quadcon(1).q = zeros(3 ,1);
model.quadcon(1).rhs = 2;
model.quadcon(1).sense = '<';
model.quadcon(1).name = 'rot_cone';
%}
% a1 = x * y, a2 = x * z, a3 = y * z
% a1 >= x + y - 1; a1 <= x; a1 <= y; a1 belongs to [0, 1]
model.A = [model.A; [1 1 0 -1 0 0]];
model.rhs = [model.rhs; 1];
model.sense = strcat(model.sense, '<');
model.A = [model.A; [1 0 0 -1 0 0]];
model.rhs = [model.rhs; 0];
model.sense = strcat(model.sense, '>');
model.A = [model.A; [0 1 0 -1 0 0]];
model.rhs = [model.rhs; 0];
model.sense = strcat(model.sense, '>');

model.A = [model.A; [1 0 1 0 -1 0]];
model.rhs = [model.rhs; 1];
model.sense = strcat(model.sense, '<');
model.A = [model.A; [1 0 0 0 -1 0]];
model.rhs = [model.rhs; 0];
model.sense = strcat(model.sense, '>');
model.A = [model.A; [0 0 1 0 -1 0]];
model.rhs = [model.rhs; 0];
model.sense = strcat(model.sense, '>');

model.A = [model.A; [0 1 1 0 0 -1]];
model.rhs = [model.rhs; 1];
model.sense = strcat(model.sense, '<');
model.A = [model.A; [0 1 0 0 0 -1]];
model.rhs = [model.rhs; 0];
model.sense = strcat(model.sense, '>');
model.A = [model.A; [0 0 1 0 0 -1]];
model.rhs = [model.rhs; 0];
model.sense = strcat(model.sense, '>');

model.A = [model.A; [0 0 0 1 1 1]];
model.rhs = [model.rhs; 2];
model.sense = strcat(model.sense, '<');

model.A = [model.A; [0 0 0 1 1 0]];
model.rhs = [model.rhs; 1];
model.sense = strcat(model.sense, '>');

model.A = sparse(model.A);
params.outputflag = 0;
result = gurobi(model, params);
disp(result.x)