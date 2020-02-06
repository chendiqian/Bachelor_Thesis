clc; clear;
names = {'x', 'y', 'z'};
model.varnames = names ;
model.Q = sparse([1 0.5 0; 0.5 1 0.5; 0 0.5 1]);
model.A = sparse([1 2 3; 1 1 0]);
model.obj = [2 0 0];
model.rhs = [4 1];
model.sense = '>';
results = gurobi(model);
for v=1: length(names)
fprintf("%s %e\n", names{v}, results.x(v));
end
fprintf("Obj: %e\n", results.objval);
model.vtype = 'B';
results = gurobi(model);
for v=1: length(names)
fprintf("%s %e\n", names{v}, results.x(v));
end
fprintf("Obj: %e\n", results.objval);
