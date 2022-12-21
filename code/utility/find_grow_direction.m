function grow_info = find_grow_direction(pc, maximum_length)
    %% find branch grow direction (either more of X axis or Y axis)

    XLimits = pc.XLimits;
    YLimits = pc.YLimits;
    XRange = abs(XLimits(1) - XLimits(2));
    YRange = abs(YLimits(1) - YLimits(2));

    Segment = true;

    if YRange > XRange
        start = YLimits(1);
        end_ = YLimits(2);
        segment_dimension = 2;

        if YRange <= maximum_length
            Segment = false;
        end

    else
        start = XLimits(1);
        end_ = XLimits(2);
        segment_dimension = 1;

        if XRange <= maximum_length
            Segment = false;
        end

    end

    grow_info.start = start;
    grow_info.end = end_;
    grow_info.dimension = segment_dimension;
    grow_info.Seg = Segment;
    grow_info.ZLimits = pc.ZLimits;

end
