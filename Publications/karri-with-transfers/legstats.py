
import pandas as pd


def print_hours_minutes(x):
    hours = int(x / (3600 * 10))
    minutes = int((x % 3600 * 10) / 60)
    print(f"{hours}:{minutes}")

# Read the data
df = pd.read_csv('./outputs/Berlin-1pct_v-200_r-1000.legstats.csv')

n_vehicles = df["vehicle_id"].max() + 1
n_rows = df.index.size

# Stats per vehicle
total_driving_time = [0] * n_vehicles
total_stop_time = [0] * n_vehicles
weighted_occupancy = [0] * n_vehicles
total_operation_time= [0] * n_vehicles

for i in range(n_rows):
    veh_id = df.loc[i, 'vehicle_id']
    driving_time = df.loc[i, 'drive_time']
    stop_time = df.loc[i, 'stop_time']
    occupancy = df.loc[i, 'occupancy']
    total_driving_time[veh_id] += driving_time
    total_stop_time[veh_id] += stop_time
    total_operation_time[veh_id] += stop_time + driving_time
    weighted_occupancy[veh_id] += driving_time * occupancy


# Average occupancy
average_occupancy = [0] * n_vehicles
# total_occupancy = 
for i in range(n_vehicles):
    average_occupancy[i] = 0 if total_driving_time[i] == 0 else weighted_occupancy[i] / total_driving_time[i]

overall_average_occupancy = sum(weighted_occupancy) / sum(total_driving_time)
total_driving_time = sum(total_driving_time)
total_stop_time = sum(total_stop_time)
average_operation_time = sum(total_operation_time) / n_vehicles



print(overall_average_occupancy)
print_hours_minutes(average_operation_time)
