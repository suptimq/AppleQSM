function density = compute_density(npts, pts)
    num = round(npts * 0.012);
    kdtree = KDTreeSearcher(pts);
    [index, distance_] = knnsearch(kdtree, pts, 'K', num);

    % approximate density by computing the distance to the nearest neighbor
    % reference https://www.cloudcompare.org/doc/wiki/index.php?title=Density
    % https://www.mathworks.com/matlabcentral/answers/563603-how-to-compute-the-density-of-a-3d-point-cloud
    distance_sum = sum(distance_, 2);
    density = num ./ (4 * pi * distance_sum.^3/3);
end