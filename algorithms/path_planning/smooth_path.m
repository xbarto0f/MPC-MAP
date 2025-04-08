function [new_path] = smooth_path(old_path)
%SMOOTH_PATH Summary of this function goes here

new_path = old_path;

alpha = 0.8;
beta = 0.5;

for i = 2:size(new_path, 1) - 1
    new_path(i,:) = new_path(i,:) + alpha*(old_path(i,:) - new_path(i,:)) + beta*(new_path(i-1,:) + new_path(i+1,:) - 2*new_path(i, :));
end

end

