function y = parameterfun(vs, surface_pts, lambda_s)
y = sum(pdist2(surface_pts, vs)) + lambda_s*var(pdist2(surface_pts, vs), 1);