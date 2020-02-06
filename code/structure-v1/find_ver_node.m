function [vernode1, vernode2] = find_ver_node(node)
cur_node = node;
while mod(cur_node(1)-1, 3) ~= 1
    cur_node(1) = cur_node(1) - 1;
end
vernode1 = cur_node;

cur_node = node;
while mod(cur_node(1)-1, 3) ~= 1
    cur_node(1) = cur_node(1) + 1;
end
vernode2 = cur_node;