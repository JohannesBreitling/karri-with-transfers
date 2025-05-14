import pandas as pd

def calculate_sizes():
    df_ell = pd.read_csv('./sizes/ellipses.csv')['size']
    df_ell_inter = pd.read_csv('./sizes/ellipse-intersection.csv')['size']
    df_dijk = pd.read_csv('./sizes/dijkstra.csv')['size']

    sum_ell = df_ell.sum()
    num_ell = df_ell.where(df_ell > 0).count()

    sum_ell_inter = df_ell_inter.sum()
    num_ell_inter = df_ell_inter.where(df_ell_inter > 0).count()

    sum_dijk = df_dijk.sum()
    num_dijk = df_dijk.where(df_dijk > 0).count()

    
    print("Dijkstra Search Space:", int(sum_dijk / num_dijk))
    print("Ellipse:", int(sum_ell / num_ell))
    print("Ellipse Intersection:", int(sum_ell_inter / num_ell_inter))





calculate_sizes()