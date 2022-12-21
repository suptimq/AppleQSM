function subsampled_pointcloud = Hilbertcurve_method(P, bin_size, subsample_num, pc)
%  Hilbertcurve_method use space-filling curve to map a 3-dimension
%  coordinate to [0,1].
%  Parameters
%  -----------
%     --P: Iteration number to generate a hilbert curve.
%       bin_size: the size of each bin in a histogram.
%       subsample_num: the size of the data that you want after sampling.
%       path: the location of your point cloud data.
%  Example
%  -------
%     --subsampled_pointcloud = Hilbertcurve_method(4, 5, 10000, 'Normal_Tree\tree1.ply')
    tic
    fprintf('Begin subsampling')
    hilbert_curve = pyrunfile('hilbertcurve.py','hilbert_curve', p=P, n=3);

    pt_location = pcnormal(pc);%pt

    CoordinateAndDistance = coordinate2distance(pt_location, pc, hilbert_curve);%pt
    
    subsampled_pointcloud = downsample(CoordinateAndDistance, bin_size, subsample_num, P, pc);%pt

    fprintf('Subsample has been done...')
    toc
end