import pandas as pd
import numpy as np
from pathlib import Path

path = Path(__file__).resolve().parent
list_of_years = list(range(2011,2024))
frames = []
num_fields = ['MYR', 'OL', 'OW', 'OH ', 'WB', 'CW ', 'A', 'B', 'C','D', 'E', 'F', 'G', 'TWF', 'TWR ', 'YEAR']
for i in list_of_years:
    try:
        with open(path/f'cars_database/{i}_en.csv', 'r') as f:
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
parameters = ['OL', 'OW', 'OH', 'WB', 'A', 'C','D', 'E', 'FH', 'RH', 'TW']
# old = ['OH', 'OW', 'WB','RH','FH','TW']
out_df = df[parameters].corr(method='pearson')

import seaborn as sns
import matplotlib.pyplot as plt

plt.figure(figsize=(10, 8))  # Adjust size for poster printing (e.g., make larger for high-res export)
sns.heatmap(out_df, annot=True, fmt='.2f', cmap='coolwarm', vmin=-1, vmax=1, linewidths=0.5, square=True)
plt.title('Pearson Correlation Matrix of Vehicle Parameters (2011-2023 CVS Data)', fontsize=16)
plt.xticks(rotation=45, ha='right', fontsize=12)
plt.yticks(rotation=0, fontsize=12)
plt.tight_layout()
plt.savefig('correlation_heatmap.png', dpi=300)  # Export high-res for poster
plt.show()


def predict_a_from_b(b_vector):
    mu_a, cov_ab, cov_bb_inv, mu_b = estimate_ab(a_part, b_part,df)
    print(mu_a.shape, cov_ab.shape, cov_bb_inv.shape, (b_vector-mu_b).shape)
    re = mu_a+cov_ab@cov_bb_inv@(b_vector-mu_b)
    print(re)
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

scenarios = [[['RH', 'FH', 'OH'],['OW', 'WB']],
             [['RH', 'FH', 'OH', 'WB'],['OW']],
             [['OW'],['WB']],
             [['OW'],['TW']],
            ]
car_model_param = dict()
for i in scenarios:
    a_part = i[0]
    b_part = i[1]
    s_name = f"{'_'.join(a_part)}__{'_'.join(b_part)}"
    print(s_name)
    mu_a, cov_ab, cov_bb_inv, mu_b = estimate_ab(a_part, b_part,df)
    car_model_param[s_name] = [mu_a, cov_ab, cov_bb_inv, mu_b]
    print("---")