import pandas as pd
# import numpy as np
from decimal import *


def format_float(x, n=2):
    return ("{:." + repr(n) + "f}").format(x)


# Configure options
getcontext().prec = 2

# Read the data
df = pd.read_csv('./outputs/Berlin-1pct_v-200_r-1000.assignmentcost.csv')

# Count the number of rows and the number of rows where the cost is infinite
n_rows = df.index.size

# Only keep the rows where the cost for transfers is not infinite
for i in range(n_rows):
    if df.loc[i, 'infty_transfer'] == True:
        df.drop(i, inplace=True)

n_better = df.index.size 

p_better = "{:.2f}".format(n_better * 100 / n_rows)

print(n_rows)
print(n_better)

print("Verbessert: " + repr(n_better) + "/" + repr(n_rows) + " " + repr(p_better) + "%")

mn_total = df['total_transfer'].mean()
mn_walking = df['walking_cost_transfer'].mean()

print(format_float(mn_total, 0))
print(format_float(mn_walking, 0))


# print(df.to_string())
