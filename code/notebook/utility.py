import os

import numpy as np
import matplotlib.pyplot as plt

import statsmodels.api as sm

from math import ceil, floor
from sklearn.metrics import r2_score
from sklearn.metrics import mean_absolute_error, mean_squared_error
from sklearn.linear_model import HuberRegressor, LinearRegression


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


def calibrate_branch_trait_from_other_excel(sensor_measurement_df, field_measurement_df):

    tmp_df = sensor_measurement_df.loc[~sensor_measurement_df['Label'].isnull(), :]
    valid_sensor_measurement_df = tmp_df.copy()
    valid_sensor_measurement_df['Section_Index'] = valid_sensor_measurement_df['Label'].apply(lambda x: int(x.split('_')[0][-1]))
    valid_sensor_measurement_df['Color'] = valid_sensor_measurement_df['Label'].apply(lambda x: x.split('_')[1])
    calibrated_sensor_measurement_df = calibrate_branch_trait(field_measurement_df, valid_sensor_measurement_df)
    
    return calibrated_sensor_measurement_df  


def plot_best_ransac_fit(x, y, fit_intercept):
    
    x = x.values.reshape(-1, 1)
    y = y.values.reshape(-1, 1)

    # ransac = RANSACRegressor()
    ransac = HuberRegressor(fit_intercept=fit_intercept)
    ransac.fit(x, y)

    line_X = np.arange(x.min(), x.max())[:, np.newaxis]
    line_y_ransac = ransac.predict(line_X)

    r2_ransac = r2_score(y, ransac.predict(x))

    return line_X, line_y_ransac, ransac.coef_, ransac.intercept_, r2_ransac


def plot_linear_fit(x, y, fit_intercept=True):
    
    x = x.values.reshape(-1, 1)
    y = y.values.reshape(-1, 1)

    reg = LinearRegression(fit_intercept=fit_intercept)
    reg.fit(x, y)

    line_X = np.arange(x.min(), x.max())[:, np.newaxis]
    line_y = reg.predict(line_X)

    r2_linear = r2_score(y, reg.predict(x))

    return line_X, line_y, reg.coef_, reg.intercept_, r2_linear


def evaluation(sensor_measurement, x, y, fit_intercept=True):
    estimation = sensor_measurement[x]
    gt = sensor_measurement[y]

    mae = round(mean_absolute_error(gt, estimation), 2)
    mape = round(mean_absolute_percentage_error(gt, estimation), 2)
    rmse = round(mean_squared_error(gt, estimation, squared=False), 2)
    lr_x, lr_y, lr_coef, lr_intercept, lr_score = plot_linear_fit(estimation, gt, fit_intercept=fit_intercept)
    ransac_lr_x, ransac_lr_y, coef, intercept, score = plot_best_ransac_fit(estimation, gt, fit_intercept=fit_intercept)

    return {
        'MAE': mae,
        'MAPE': mape,
        'RMSE': rmse,
        'LR': [lr_x, lr_y, lr_coef, lr_intercept, lr_score],
        'RLR': [ransac_lr_x, ransac_lr_y, coef, intercept, score]
    }


def plot_yx_line(df, x_axis, y_axis, ax, x_interval=5, y_interval=5):
    axs_xticks = df[x_axis].tolist()
    axs_yticks = df[y_axis].tolist()

    axs_xticks_min = floor(min(axs_xticks))
    axs_xticks_max = ceil(max(axs_xticks))
    axs_yticks_min = floor(min(axs_yticks))
    axs_yticks_max = ceil(max(axs_yticks))

    left_end = min(axs_xticks_min, axs_yticks_min)
    right_end = max(axs_xticks_max, axs_yticks_max)
    xtick = list(range(left_end, right_end, x_interval))
    ytick = list(range(left_end, right_end, y_interval))
    
    # set ticks and tick labels
    ax.set_xticks(xtick)
    ax.set_xticklabels(xtick)
    ax.set_yticks(ytick)
    ax.set_yticklabels(ytick)
    
    ref_line_limit = [left_end, right_end]
    x = np.arange(ref_line_limit[0], ref_line_limit[1])
    ax.plot(x, x, color='black', linewidth=3, linestyle='dashed')