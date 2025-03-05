function [G, vG, public_vars] = get_target(P, public_vars)
    p_index = public_vars.path_index;
    path = public_vars.path;

    distances = vecnorm(path(p_index:end,:) - P, 2, 2);
    [~, index_offset] = min(distances);
    p_index = p_index + index_offset - 1;
    public_vars.path_index = p_index;

    if p_index >= length(path)
        G = path(end,:);
        G_vec = path(end,:) - path(end-1,:);
        vG = G_vec/norm(G_vec)* 1;
        return
    end

    G = path(p_index,:);
    G_vec = path(p_index+1,:) - path(p_index,:);
    vG = G_vec/norm(G_vec)* 1;
end

