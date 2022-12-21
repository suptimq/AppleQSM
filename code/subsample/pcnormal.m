%normalize a ply type tree into [0,1]^3 and return the xyz coordinate.

function xyz = pcnormal(pc)
    xyz = pc.Location;
    count = pc.Count;
    Limits = [pc.XLimits, pc.YLimits, pc.ZLimits];
    for i = 1:count
        xyz(i,1) = (xyz(i,1)-Limits(1))/(Limits(2)-Limits(1));
        xyz(i,2) = (xyz(i,2)-Limits(3))/(Limits(4)-Limits(3));
        xyz(i,3) = (xyz(i,3)-Limits(5))/(Limits(6)-Limits(5));
    end
end