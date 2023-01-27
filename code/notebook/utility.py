import os

import numpy as np
import matplotlib.pyplot as plt

import seaborn as sns
import statsmodels.api as sm

from sklearn.metrics import mean_absolute_error, mean_squared_error
from sklearn.linear_model import RANSACRegressor, HuberRegressor, LinearRegression
from scipy.stats import pearsonr


def mean_absolute_percentage_error(y_true, y_pred): 
    y_true, y_pred = np.array(y_true), np.array(y_pred)
    return np.mean(np.abs((y_true - y_pred) / y_true)) * 100


def simple_regplot(
    x, y, n_std=2, n_pts=100, ax=None, scatter_kws=None, line_kws=None, ci_kws=None
):
    """ Draw a regression line with error interval. """
    ax = plt.gca() if ax is None else ax

    # calculate best-fit line and interval
    x_fit = sm.add_constant(x)
    fit_results = sm.OLS(y, x_fit).fit()

    eval_x = sm.add_constant(np.linspace(np.min(x), np.max(x), n_pts))
    pred = fit_results.get_prediction(eval_x)

    # draw the fit line and error interval
    ci_kws = {} if ci_kws is None else ci_kws
    ax.fill_between(
        eval_x[:, 1],
        pred.predicted_mean - n_std * pred.se_mean,
        pred.predicted_mean + n_std * pred.se_mean,
        alpha=0.5,
        **ci_kws,
    )
    line_kws = {} if line_kws is None else line_kws
    h = ax.plot(eval_x[:, 1], pred.predicted_mean, **line_kws)

    # draw the scatterplot
    scatter_kws = {} if scatter_kws is None else scatter_kws
    ax.scatter(x, y, c=h[0].get_color(), **scatter_kws)

    return fit_results    


def calibrate_branch_trait(field_measurement_df, sensor_measurement_df):

    valid_sensor_measurement_df = sensor_measurement_df.copy()
    for row in field_measurement_df.iterrows():
        section_idx, color, vertical_angle, branch_diameter = row[1][['Section Index', 'Color', 'Crotch Angle/°', 'Diameter/mm']]
        valid_sensor_measurement_df.loc[(valid_sensor_measurement_df['Section_Index']==section_idx)&(valid_sensor_measurement_df['Color']==color), ['Manual_Vertical_Crotch_Angle-Degree']] = vertical_angle
        valid_sensor_measurement_df.loc[(valid_sensor_measurement_df['Section_Index']==section_idx)&(valid_sensor_measurement_df['Color']==color), ['Manual_Branch_Diameter-mm']] = branch_diameter

    return valid_sensor_measurement_df


def plot_best_ransac_fit(x, y):
    
    x = x.values.reshape(-1, 1)
    y = y.values.reshape(-1, 1)

    ransac = RANSACRegressor()
    ransac.fit(x, y)

    line_X = np.arange(x.min(), x.max())[:, np.newaxis]
    line_y_ransac = ransac.predict(line_X)

    return line_X, line_y_ransac, ransac.estimator_.coef_, ransac.estimator_.intercept_, ransac.score(x, y)


def plot_linear_fit(x, y, fit_intercept=True):
    
    x = x.values.reshape(-1, 1)
    y = y.values.reshape(-1, 1)

    reg = LinearRegression(fit_intercept=fit_intercept)
    reg.fit(x, y)

    line_X = np.arange(x.min(), x.max())[:, np.newaxis]
    line_y = reg.predict(line_X)

    return line_X, line_y, reg.coef_, reg.intercept_, reg.score(x, y)  


def calibrate_branch_trait_from_other_excel(sensor_measurement_df, field_measurement_df):

    tmp_df = sensor_measurement_df.loc[~sensor_measurement_df['Label'].isnull(), :]
    valid_sensor_measurement_df = tmp_df.copy()
    valid_sensor_measurement_df['Section_Index'] = valid_sensor_measurement_df['Label'].apply(lambda x: int(x.split('_')[0][-1]))
    valid_sensor_measurement_df['Color'] = valid_sensor_measurement_df['Label'].apply(lambda x: x.split('_')[1])
    calibrated_sensor_measurement_df = calibrate_branch_trait(field_measurement_df, valid_sensor_measurement_df)
    
    return calibrated_sensor_measurement_df  


def evaluation(sensor_measurement, x, y):
    estimation = sensor_measurement[x]
    gt = sensor_measurement[y]

    mae = round(mean_absolute_error(gt, estimation), 2)
    mape = round(mean_absolute_percentage_error(gt, estimation), 2)
    rmse = round(mean_squared_error(gt, estimation, squared=False), 2)
    lr_x, lr_y, lr_coef, lr_intercept, lr_score = plot_linear_fit(estimation, gt, fit_intercept=True)
    ransac_lr_x, ransac_lr_y, coef, intercept, score = plot_best_ransac_fit(estimation, gt)

    return {
        'MAE': mae,
        'MAPE': mape,
        'RMSE': rmse,
        'LR': [lr_x, lr_y, lr_coef, lr_intercept, lr_score],
        'RLR': [ransac_lr_x, ransac_lr_y, coef, intercept, score]
    }