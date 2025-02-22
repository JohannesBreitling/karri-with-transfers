import pandas as pd
# import numpy as np


# Count the number of rows and the number of rows where the cost is infinite
n_legs_wot = df_leg_wot.index.size
n_legs_wt = df_leg_wt.index.size

# Only keep the rows where the cost for transfers is not infinite
for i in range(n_legs_wot):
    if df_leg_wot.loc[i, 'infty_transfer'] == True:
        df_leg_wot.drop(i, inplace=True)

for i in range(n_legs_wt):
    if df_leg_wt.loc[i, 'infty_transfer'] == True:
        df_leg_wt.drop(i, inplace=True)

n_better = df.index.size 

p_better = "{:.2f}".format(n_better * 100 / n_rows)

print(n_rows)
print(n_better)

print("Verbessert: " + repr(n_better) + "/" + repr(n_rows) + " " + repr(p_better) + "%")

mn_total = df['total_transfer'].mean()
mn_walking = df['walking_cost_transfer'].mean()

print(format_float(mn_total, 0))
print(format_float(mn_walking, 0))