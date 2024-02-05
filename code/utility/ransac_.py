import os
os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"
import numpy as np
import pyransac3d as pyrsc

from skimage.measure import ransac, LineModelND, EllipseModel, CircleModel


def ransac_(value, model, min_samples=5, residual_threshold=0.05, max_trials=100):

    # not working right now
    if model == 'Cylinder':
        fitting_model = pyrsc.Cylinder()
        center, axis, radius, inliers = fitting_model.fit(value, thresh=residual_threshold, maxIteration=1000)

        return center, radius

    if model == 'Ellipse':
        fitting_model = EllipseModel
    elif model == 'Circle':
        fitting_model = CircleModel
    elif model == '3D_Line':
        fitting_model = LineModelND

    try:
        model_robust, inliers = ransac(value, fitting_model, min_samples=min_samples,
                                       residual_threshold=residual_threshold, max_trials=max_trials)
        outliers = inliers == False
    except:
        print(f"-----------------------------!!! Failed to fit {model} using RANSAC in Python !!!-----------------------------")
        return None, None, None

    if model == 'Ellipse':
        vector = np.round(model_robust.params, 10)
    elif model == 'Circle':
        vector = np.round(model_robust.params, 10)
    elif model == '3D_Line':
        vector = np.concatenate((model_robust.params[0], model_robust.params[1]))

    return vector, inliers, outliers


vector, inliers, outliers = ransac_(value, model, min_samples, residual_threshold, max_trials)


# if __name__ == '__main__':
    
#     import scipy.io

#     data = scipy.io.loadmat('test.mat')
#     value = data['ori_branch_pts']
#     a = ransac_(value, 'Cylinder', 3, 0.03, 1000)
#     import pdb; pdb.set_trace()
