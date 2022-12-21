function [density,segment] = density_and_segment(P, bin_size, pc)
    hilbert_curve = pyrunfile('hilbertcurve.py','hilbert_curve', p=P, n=3);
    pt_location = pcnormal(pc);
    CoordinateAndDistance = coordinate2distance(pt_location, pc, hilbert_curve);
    [density, segment] = density_segment(CoordinateAndDistance, bin_size, P, pc);
end