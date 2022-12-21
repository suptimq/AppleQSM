% classify point into 1-dimension. return xyz_location+class.
function xyz_location = coordinate2distance(xyz_location, pc, hilbertcurve)
    count = pc.Count;
    for i = 1:count
        x = xyz_location(i,1) * (2^hilbertcurve.p-1);
        y = xyz_location(i,2) * (2^hilbertcurve.p-1);
        z = xyz_location(i,3) * (2^hilbertcurve.p-1);
        x = fix(x);
        y = fix(y);
        z = fix(z);
        coordinate = [x,y,z];
        distance = hilbertcurve.distance_from_point(coordinate);
        distance = distance.int64;
        xyz_location(i,4) = distance;
    end
end