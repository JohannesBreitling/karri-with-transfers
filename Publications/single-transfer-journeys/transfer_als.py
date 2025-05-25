import sys
import pandas as pd

from utilities import *

print("Evaluating the perf stats of transfers ALS.....")

INSTANCE_NAME = sys.argv[1]
SOURCE_DIR = sys.argv[2]
TARGET_DIR = sys.argv[3]


PATH = SOURCE_DIR + INSTANCE_NAME + ".perf_transf_als_"
df_pveh = pd.read_csv(PATH + "pveh.csv")
df_dveh = pd.read_csv(PATH + "dveh.csv")

def eval_running_times():
    # ALS PVEH
    p_avg_total = from_nano_seconds(df_pveh["total_time"].mean())
    p_avg_try_assignments = from_nano_seconds(df_pveh["try_assignments_time"].mean())
    
    p_avg_st_pickup_als = from_nano_seconds(df_pveh["search_time_pickup_als"].mean())
    p_avg_st_dropoff_als = from_nano_seconds(df_pveh["search_time_dropoff_als"].mean())
    p_avg_st_last_stop_to_transfer = from_nano_seconds(df_pveh["search_time_last_stop_to_transfer"].mean())
    p_avg_st_pickup_to_transfer = from_nano_seconds(df_pveh["search_time_pickup_to_transfer"].mean())

    result = ""
    result += "# TRANSFER ALS PVEH\n"
    result += "- Total:                        " + p_avg_total + "mic_sec\n"
    result += "- Try Assignments:              " + p_avg_try_assignments + "mic_sec\n"
    result += "- Pickup ALS Search:            " + p_avg_st_pickup_als + "mic_sec\n"
    result += "- Dropoff ALS Search:           " + p_avg_st_dropoff_als + "mic_sec\n"
    result += "- Last Stop to Transfer Search: " + p_avg_st_last_stop_to_transfer + "mic_sec\n"
    result += "- Pickup to Transfer Search:    " + p_avg_st_pickup_to_transfer + "mic_sec\n"
    
    # ALS DVEH
    d_avg_total = from_nano_seconds(df_dveh["total_time"].mean())
    d_avg_try_assignments = from_nano_seconds(df_dveh["try_assignments_time"].mean())

    d_avg_st_dropoff_als = from_nano_seconds(df_dveh["search_time_dropoff_als"].mean())
    d_avg_st_last_stop_to_transfer = from_nano_seconds(df_dveh["search_time_last_stop_to_transfer"].mean())
    d_avg_st_transfer_to_dropoff = from_nano_seconds(df_dveh["search_time_transfer_to_dropoff"].mean())

    result += "# TRANSFER ALS DVEH\n"
    result += "- Total:                        " + d_avg_total + "mic_sec\n"
    result += "- Try Assignments:              " + d_avg_try_assignments + "mic_sec\n"
    result += "- Dropoff ALS Search:           " + d_avg_st_dropoff_als + "mic_sec\n"
    result += "- Last Stop to Transfer Search: " + d_avg_st_last_stop_to_transfer + "mic_sec\n"
    result += "- Transfer to Dropoff Search:   " + d_avg_st_transfer_to_dropoff + "mic_sec\n"

    print(result)
    writeToFile(TARGET_DIR + "rt_transfer_als.md", result)

# * Call the evaluation methods
eval_running_times()