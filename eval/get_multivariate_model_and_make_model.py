import pandas as pd
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
import scipy.io
import json


def get_stats(stat,values):
    if stat == 'max':
        re = np.round(np.nanmax(values),2)
    elif stat == 'min':
        re = np.round(np.nanmin(values),2)
    elif stat == 'std':
        re = np.round(np.nanstd(values),2)
    elif stat == 'mean':
        re = np.round(np.nanmean(values),2)
    return stat, re

def estimate_ab(a_part, b_part,df):
    ab = a_part + b_part
    sdf = df[df[ab].notna().all(axis=1)][ab]
    mu_a = np.mean(sdf[a_part].values,0)
    mu_b = np.mean(sdf[b_part].values,0)
    cov_all = np.cov(sdf.values.T)
    cov_aa = cov_all[:len(a_part),:len(a_part)]
    cov_bb = cov_all[len(a_part):,len(a_part):]
    cov_bb_inv = np.linalg.inv(cov_bb)
    cov_ab = cov_all[:len(a_part),len(a_part):]
    pred = mu_a+((sdf[b_part].values-mu_b)@cov_bb_inv.T)@cov_ab.T
    for i in range(len(a_part)):
        print(a_part[i])
        error = np.sum(abs(pred[:,i]-sdf[a_part].values[:,i]))/pred.shape[0]
        print('Error',round(error,3))
    return mu_a, cov_ab, cov_bb_inv, mu_b

def predict_a_from_b(df, a_part, b_part, b_vector):
    mu_a, cov_ab, cov_bb_inv, mu_b = estimate_ab(a_part, b_part,df)
    print(mu_a.shape, cov_ab.shape, cov_bb_inv.shape, (b_vector-mu_b).shape)
    re = mu_a+cov_ab@cov_bb_inv@(b_vector-mu_b)
    print(re)
    return re

def get_car_database(path, list_of_years):
    frames = []
    num_fields = ['MYR', 'OL', 'OW', 'OH ', 'WB', 'CW ', 'A', 'B', 'C','D', 'E', 'F', 'G', 'TWF', 'TWR ', 'YEAR']
    for i in list_of_years:
        try:
            with open(Path(__file__).resolve().parents[1]/f'priors/cars_database/{i}_en.csv', 'r') as f:
                df = pd.read_csv(f)
                df['YEAR']  = i
                for field in num_fields:
                    df[field] = df[field].astype(float)
            df.columns = ['MAKE', 'MODEL', 'MYR', 'OL', 'OW', 'OH', 'WB', 'CW', 'A', 'B', 'C','D', 'E', 'FH', 'RH', 'TWF', 'TWR', 'WDIST', 'YEAR']
            frames.append(df)
        except:
            print(i, 'has encoding error')
    df = pd.concat(frames)
    df['TW'] = np.round((df.TWF+df.TWR)/2,1)
    return df

def print_info(df):
    out_df = df[['OH', 'OW', 'WB','RH','FH','TW']].corr(method='pearson')
    out_df.to_latex(index=True,
                          formatters={"name": str.upper},
                          float_format="{:.2f}".format,
                          multirow=True,
                          multicolumn=True,
                          multicolumn_format='c',
                          position='h',
                         bold_rows=True
            )
    print(df.head())
    print(len(df))
    print(df.keys())  

    for i in ['OH', 'OW', 'WB','RH','FH']:
        print("Dimension",i)
        for k in ['mean', 'min', 'max','std']:
            print(get_stats(k,df[i].values))

    print(np.round(np.mean(df.OW - df.TW),3))
    print(df.OW.describe())
    print(df.OH.describe())
    print(df.OL.describe())
    print(df.WB.describe())
    print(np.nanpercentile(df.WB.values, 90))
    print(df.FH.describe())
    print(np.nanpercentile(df.FH.values, 90))
    print(df.RH.describe())
    print(np.nanpercentile(df.RH.values, 90))

    plt.hist(df.FH, bins=100, color='green', edgecolor='black')
    plt.show() 
    plt.hist(df.RH, bins=100, color='green', edgecolor='black')
    plt.show()

def get_multivariate_model(df, scenarios):
    car_model_param = dict()
    flat_car_model = dict()

    for scenario in scenarios:
        a_part = scenario[0]
        b_part = scenario[1]
        s_name = f"{'_'.join(a_part)}__{'_'.join(b_part)}"
        print(s_name)
        mu_a, cov_ab, cov_bb_inv, mu_b = estimate_ab(a_part, b_part, df)
        params = [mu_a, cov_ab, cov_bb_inv, mu_b]
        car_model_param[s_name] = params
        flat_car_model[s_name] = np.concatenate([param.flatten() for param in params], axis=0)
        print("---")
    return flat_car_model

def save_car_param(matlab_path, car_model_param, filename):
    scipy.io.savemat(f'{matlab_path}/{filename}', car_model_param)

def get_tracks_dims(df, all_vehicles,all_tracks, critical_dims):
    tracks_dims = dict()
    for m,i in enumerate(all_vehicles):
        i = i.replace('\n', '')
        splitted = i.split()
        make = splitted[0]
        model = " ".join(splitted[1:])
        model = model.replace('\n', '')
        # print(splitted,model)
        filtered_df = df[df.MODEL.str.contains(model, case=False, na=False) & df.MAKE.str.contains(make, case=False, na=False)]
        if len(filtered_df) > 0:
            same_dims = True
            for d in critical_dims:
                deviation = np.nanstd(filtered_df[d].values)/np.nanmean(filtered_df[d].values)
                if deviation >= 0.05: #If standard deviation is more than 5% from the mean
                    same_dims = False
                    break
            if same_dims:
                avg_dims = np.round(np.mean(filtered_df[critical_dims].values,0),3)
                dims_dict = dict(zip(critical_dims,avg_dims))
                tracks_dims[all_tracks[m]] = {i:dims_dict}
            else:
                print(make,model,"NOT same dims")
        else:
            print(make,model,'NOT found')
    return tracks_dims

def get_GT_make_model(path, seg_cam, critical_dims):
    all_cars_make_model = dict()
    with open(path/f"make_model_data/{seg_cam}.csv",'r') as f:
        mdf = pd.read_csv(f)
        mdf = mdf[mdf["Google image search result"].notnull()]
        mdf = mdf[mdf["Google image search result"].apply(len) > 2]
        all_vehicles = mdf['Google image search result'].values
        all_tracks = mdf.Track.values
        track_dims = get_tracks_dims(all_vehicles, all_tracks, critical_dims)
        print(track_dims)
        all_cars_make_model[seg_cam] = track_dims
        print(seg_cam, 'num valid tracks', len(track_dims), 'as a percentage', round(len(track_dims)/len(all_tracks)*100,2))
        print("=====")
    return all_cars_make_model

def save_make_model(gen_path, seg_cam, all_cars_make_model):
    with open(gen_path/f'saved_make_model/{seg_cam}_make_model_2025.json', 'w') as f:
        json.dump(all_cars_make_model, f)

def main():
    ROOT = Path(__file__).resolve().parents[1]
    path = ROOT / 'data/raw'
    gen_path = ROOT / 'data/gen'
    matlab_path = 'matlab_geometry/raster'

    list_of_years = list(range(2011,2024))
    df = get_car_database(path, list_of_years)

    ### GET CAR DIMENSIONS MULTIVARIATE MODEL
    scenarios = [[['RH', 'FH', 'OH'],['OW', 'WB']],
                [['RH', 'FH', 'OH', 'WB'],['OW']],
                [['OW'],['WB']],
                [['OW'],['TW']],
                ]
    car_model_param = get_multivariate_model(df, scenarios)
    print(car_model_param)
    filename = 'car_model_param_2025.mat'
    save_car_param(matlab_path, car_model_param, filename)    # save car_model_param.mat

    ### GET GT CAR DIMENSIONS FOR THE TRACKS 
    critical_dims = ['OL', 'OW', 'OH', 'WB', 'TW']
    vis_dims = ['MAKE','MODEL','YEAR','OL', 'OW', 'OH', 'WB','TW']
    seg = "23"
    cams = ["sc1", "sc2", "sc3", "sc4"]
    for cam in cams:
        seg_cam = f'{seg}_{cam}'
        all_cars_make_model = get_GT_make_model(path, seg_cam, critical_dims)
        save_make_model(gen_path, seg_cam, all_cars_make_model)
    

if __name__ == "__main__":
    main()