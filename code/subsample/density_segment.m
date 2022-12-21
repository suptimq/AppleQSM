function [density, segment] = density_segment(CoordinateAndDistance, b, P, pc)
    Color = pc.Color;
    count = pc.Count;
    Limits = [pc.XLimits, pc.YLimits, pc.ZLimits];
    num = fix(2^(3*P)/b)+1;
    bin = cell(num, 2);
    
    for i = 1:num
        bin{i, 2} = [];
    end
    for i = 1:num
        bin{i, 1}=0;
    end
    for i = 1:count
        location = fix(CoordinateAndDistance(i,4)/b)+1;
        bin{location, 1} = bin{location, 1} + 1;
        bin{location, 2} = [bin{location, 2}, i];
    end

    for i = 1:count
        location = fix(CoordinateAndDistance(i,4)/b)+1;
        density(i,1) = bin{location, 1}/count;
    end
    data(:, 1) = CoordinateAndDistance(:, 1); 
    data(:, 2) = CoordinateAndDistance(:, 2); 
    data(:, 3) = CoordinateAndDistance(:, 3); 
    for i = 1:count
        distance = CoordinateAndDistance(i,4);
        bin_location = fix(distance/b)+1;
        if(bin{bin_location,1}>=2000)
            segment(i,1) = 0;
        else
            segment(i,1) = 1;
    end
    
end