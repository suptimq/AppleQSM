%% combinate downsample density pre-segmentation functions
bin_size = 20;
subsample_num = 10000;
path = 'Normal_Tree/tree1.ply';
P = 4;
pc = pcread(path);

%% downsampling...
downsampled = Hilbertcurve_method(P, bin_size, subsample_num, pc);

%% combinate density calculation and pre-segmentation functions
[density,class] = density_and_segment(P, bin_size, pc);




