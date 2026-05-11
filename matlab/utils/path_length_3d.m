function distance = path_length_3d(path)
%PATH_LENGTH_3D Compute Euclidean length of a 3D waypoint sequence.
    if isempty(path) || size(path, 1) < 2
        distance = 0;
        return;
    end

    delta = diff(path, 1, 1);
    distance = sum(sqrt(sum(delta.^2, 2)));
end
