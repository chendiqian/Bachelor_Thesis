function y = containsArray(big,small)
row_s = size(small, 1);
row_b = size(big, 1);
y = false;
for i = 1:row_b-row_s+1
    if big(i:i+row_s-1,:) == small
        y = true;
        break;
    end
end