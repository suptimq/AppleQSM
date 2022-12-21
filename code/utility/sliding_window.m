function segment_pts = sliding_window(pts, start, kernel_size, stride)
    num_pts = size(pts, 1);
    num_segment = floor((num_pts - kernel_size) / stride) + 1;
    segment_pts = cell(num_segment, 1);

    for i = 1:num_segment
        segment_pts{i} = pts(start:start + kernel_size - 1, :);
        start = start + stride;
    end

end
