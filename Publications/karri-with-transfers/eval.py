import pandas as pd

def format_float(x, n=2):
    return ("{:." + repr(n) + "f}").format(x)

def format_hh_mm(x):
    h = int(x / 36000)
    m = int(x % 36000 / 600)
    return ("{:02d}:{:02d}").format(h, m)

def format_mm_ss(x):
    m = x % 6000 / 60
    s = x % 6000 / 60
    return ("{:02f}:{:02f}").format(m, s)

def assignmentquality(name, path):
    print(name)
    df_asq_wt = pd.read_csv(path + '/wt/Berlin-1pct-wt.assignmentquality.csv')
    df_asq_wot = pd.read_csv(path + '/wot/Berlin-1pct-wot.assignmentquality.csv')

    # Get the mean wait and trip times
    avg_wait_time_wt = df_asq_wt['wait_time'].mean()
    avg_trip_time_wt = df_asq_wt['trip_time'].mean()
    avg_wait_time_wot = df_asq_wot['wait_time'].mean()
    avg_trip_time_wot = df_asq_wot['trip_time'].mean()

    print("Average wait time with transfers:", int(avg_wait_time_wt))
    print("Average wait time without transfers:", int(avg_wait_time_wot))
    print("Average trip time with transfers:", int(avg_trip_time_wt))
    print("Average trip time without transfers:", int(avg_trip_time_wot))
    print("")


def legstats(name, path):
    print(name)
    df_leg_wt = pd.read_csv(path + '/wt/Berlin-1pct-wt.legstats.csv')
    df_leg_wot = pd.read_csv(path + '/wot/Berlin-1pct-wot.legstats.csv')

    # Calculate average occupancy and total vehicle operation time
    n_vehicles_wot = df_leg_wot["vehicle_id"].max() + 1
    n_vehicles_wt = df_leg_wt["vehicle_id"].max() + 1


    n_legs_wot = df_leg_wot.index.size
    n_legs_wt = df_leg_wt.index.size

    # Stats per vehicle
    total_driving_time_wt = [0] * n_vehicles_wt
    total_stop_time_wt = [0] * n_vehicles_wt
    weighted_occupancy_wt = [0] * n_vehicles_wt
    total_operation_time_wt = [0] * n_vehicles_wt

    total_driving_time_wot = [0] * n_vehicles_wot
    total_stop_time_wot = [0] * n_vehicles_wot
    weighted_occupancy_wot = [0] * n_vehicles_wot
    total_operation_time_wot = [0] * n_vehicles_wot

    for i in range(n_legs_wt):
        veh_id = df_leg_wt.loc[i, 'vehicle_id']
        driving_time = df_leg_wt.loc[i, 'drive_time']
        stop_time = df_leg_wt.loc[i, 'stop_time']
        occupancy = df_leg_wt.loc[i, 'occupancy']
        total_driving_time_wt[veh_id] += driving_time
        total_stop_time_wt[veh_id] += stop_time
        total_operation_time_wt[veh_id] += stop_time + driving_time
        weighted_occupancy_wt[veh_id] += driving_time * occupancy

    for i in range(n_legs_wot):
        veh_id = df_leg_wot.loc[i, 'vehicle_id']
        driving_time = df_leg_wot.loc[i, 'drive_time']
        stop_time = df_leg_wot.loc[i, 'stop_time']
        occupancy = df_leg_wot.loc[i, 'occupancy']
        total_driving_time_wot[veh_id] += driving_time
        total_stop_time_wot[veh_id] += stop_time
        total_operation_time_wot[veh_id] += stop_time + driving_time
        weighted_occupancy_wot[veh_id] += driving_time * occupancy

    # Average occupancy
    avg_occupancy_wt = [0] * n_vehicles_wt
    avg_occupancy_wot = [0] * n_vehicles_wot

    for i in range(n_vehicles_wt):
        avg_occupancy_wt[i] = 0 if total_driving_time_wt[i] == 0 else weighted_occupancy_wt[i] / total_driving_time_wt[i]

    for i in range(n_vehicles_wot):
        avg_occupancy_wot[i] = 0 if total_driving_time_wot[i] == 0 else weighted_occupancy_wot[i] / total_driving_time_wot[i]

    total_avg_occupancy_wt = sum(avg_occupancy_wt) / n_vehicles_wt
    total_avg_occupancy_wot = sum(avg_occupancy_wot) / n_vehicles_wot

    total_avg_operation_time_wt = sum(total_operation_time_wt) / n_vehicles_wt
    total_avg_operation_time_wot = sum(total_operation_time_wot) / n_vehicles_wot

    print("Average occupancy with transfers:", format_float(total_avg_occupancy_wt))
    print("Average occupancy without transfers:", format_float(total_avg_occupancy_wot))
    print("Total operation time with transfers:", format_hh_mm(total_avg_operation_time_wt))
    print("Total operation time without transfers:", format_hh_mm(total_avg_operation_time_wot))
    print("")
    

# assignmentquality('AQ Berlin 1pct, R all, V 500', './outputs/server/karri-with-transfers/v-500')
# legstats('LS Berlin 1pct, R all, V 500', './outputs/server/karri-with-transfers/v-500')

assignmentquality('AQ Berlin 1pct, R1000, V200', './outputs/local/v-200_r-1000')
legstats('LS Berlin 1pct, R1000, V200', './outputs/local/v-200_r-1000')




