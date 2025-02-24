import pandas as pd

INFTY = 1073741823

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
    df_asq_wt = pd.read_csv(path + '/wt/wt.assignmentquality.csv')
    df_asq_wot = pd.read_csv(path + '/wot/wot.assignmentquality.csv')

    # Get the mean wait and trip times
    avg_wait_time_wt = df_asq_wt['wait_time'].mean()
    avg_trip_time_wt = df_asq_wt['trip_time'].mean()
    avg_wait_time_wot = df_asq_wot['wait_time'].mean()
    avg_trip_time_wot = df_asq_wot['trip_time'].mean()

    print("Avg wait time with transfers:", int(avg_wait_time_wt))
    print("Avg wait time without transfers:", int(avg_wait_time_wot))
    print("Avg trip time with transfers:", int(avg_trip_time_wt))
    print("Avg trip time without transfers:", int(avg_trip_time_wot))
    print("")


def legstats(name, path):
    print(name)
    df_leg_wt = pd.read_csv(path + '/wt/wt.legstats.csv')
    df_leg_wot = pd.read_csv(path + '/wot/wot.legstats.csv')

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

    # Avg occupancy
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

    print("Avg occupancy with transfers:", format_float(total_avg_occupancy_wt))
    print("Avg occupancy without transfers:", format_float(total_avg_occupancy_wot))
    print("Total operation time with transfers:", format_hh_mm(total_avg_operation_time_wt))
    print("Total operation time without transfers:", format_hh_mm(total_avg_operation_time_wot))
    print("")


def assignmentcost(name, path):
    print(name)
    
    # Calculate the mean cost of all assignments
    # Without transfer this is straightforward
    df_cost_wot = pd.read_csv(path + '/wot/wot.assignmentcost.csv')
    n_assignments_wot = df_cost_wot.index.size
    
    # Drop all lines, where the cost is infinite
    for i in range(n_assignments_wot):
        if (df_cost_wot.loc[i, 'infty_no_transfer']):
            df_cost_wot.drop(i, inplace=True)

    avg_total_cost_wot = df_cost_wot['total_no_transfer'].mean()
    avg_trip_cost_wot = df_cost_wot['trip_cost_no_transfer'].mean()
    avg_wait_cost_wot = df_cost_wot['wait_time_violation_cost_no_transfer'].mean()
    avg_trip_others_cost_wot = df_cost_wot['change_in_trip_costs_of_others_no_transfer'].mean()
    avg_veh_cost_wot = df_cost_wot['veh_cost_no_transfer'].mean()
    avg_walking_cost_wot = df_cost_wot['walking_cost_no_transfer'].mean()

    # With transfers we now have to differentiate between costs for improved assignments and the costs without transfers
    df_cost_wt = pd.read_csv(path + '/wt/wt.assignmentcost.csv')
    
    n_assignments = df_cost_wt.index.size
    # Calculate the mean costs
    total_total_cost_wt = 0
    total_trip_cost_wt = 0
    total_wait_cost_wt = 0
    total_trip_others_cost_wt = 0
    total_veh_cost_wt = 0
    total_walking_cost_wt = 0
    # Count the # of assignments that are improved
    n_improved_assignments = 0
    
    for i in range(n_assignments):
        if df_cost_wt.loc[i, 'transfer_improves'] == 1 and not df_cost_wt.loc[i, 'infty_transfer']:
            n_improved_assignments += 1
            total_total_cost_wt += df_cost_wt.loc[i, 'total_transfer']
            total_trip_cost_wt += df_cost_wt.loc[i, 'trip_cost_transfer']
            total_wait_cost_wt += df_cost_wt.loc[i, 'wait_time_violation_cost_transfer']
            total_trip_others_cost_wt += df_cost_wt.loc[i, 'change_in_trip_costs_of_others_transfer']
            total_veh_cost_wt += df_cost_wt.loc[i, 'veh_cost_transfer']
            total_walking_cost_wt += df_cost_wt.loc[i, 'walking_cost_transfer']
        elif (df_cost_wt.loc[i, 'infty_no_transfer'] == 0):
            total_total_cost_wt += df_cost_wt.loc[i, 'total_no_transfer']
            total_trip_cost_wt += df_cost_wt.loc[i, 'trip_cost_no_transfer']
            total_wait_cost_wt += df_cost_wt.loc[i, 'wait_time_violation_cost_no_transfer']
            total_trip_others_cost_wt += df_cost_wt.loc[i, 'change_in_trip_costs_of_others_no_transfer']
            total_veh_cost_wt += df_cost_wt.loc[i, 'veh_cost_no_transfer']
            total_walking_cost_wt +=df_cost_wt.loc[i, 'walking_cost_no_transfer']

    avg_total_cost_wt = total_total_cost_wt / n_assignments
    avg_trip_cost_wt = total_trip_cost_wt / n_assignments
    avg_wait_cost_wt = total_wait_cost_wt / n_assignments
    avg_trip_others_cost_wt = total_trip_others_cost_wt / n_assignments
    avg_veh_cost_wt = total_veh_cost_wt / n_assignments
    avg_walking_cost_wt = total_walking_cost_wt / n_assignments

    # Calculate the mean costs wt / wot of the assignments that have been improved
    for i in range(n_assignments):
        if df_cost_wt.loc[i, 'transfer_improves'] == 0 or df_cost_wt.loc[i, 'infty_transfer'] or df_cost_wt.loc[i, 'infty_no_transfer']:
            df_cost_wt.drop(i, inplace=True)
    
    avg_imp_total_cost_wt = df_cost_wt['total_transfer'].mean()
    avg_imp_trip_cost_wt = df_cost_wt['trip_cost_transfer'].mean()
    avg_imp_wait_cost_wt = df_cost_wt['wait_time_violation_cost_transfer'].mean()
    avg_imp_trip_others_cost_wt = df_cost_wt['change_in_trip_costs_of_others_transfer'].mean()
    avg_imp_veh_cost_wt = df_cost_wt['veh_cost_transfer'].mean()
    avg_imp_walking_cost_wt = df_cost_wt['walking_cost_transfer'].mean()

    avg_imp_total_cost_wot = df_cost_wt['total_no_transfer'].mean()
    avg_imp_trip_cost_wot = df_cost_wt['trip_cost_no_transfer'].mean()
    avg_imp_wait_cost_wot = df_cost_wt['wait_time_violation_cost_no_transfer'].mean()
    avg_imp_trip_others_cost_wot = df_cost_wt['change_in_trip_costs_of_others_no_transfer'].mean()
    avg_imp_veh_cost_wot = df_cost_wt['veh_cost_no_transfer'].mean()
    avg_imp_walking_cost_wot = df_cost_wt['walking_cost_no_transfer'].mean()


    print("# total assignments: " + repr(n_assignments))
    print("# improved assignments: " + repr(n_improved_assignments))
    print("Improved: " + format_float(n_improved_assignments / n_assignments * 100) + "%")
    
    print("Avg total cost with transfers:", int(avg_total_cost_wt))
    print("Avg total cost without transfers:", int(avg_total_cost_wot))
    print("Avg trip cost with transfers:", int(avg_trip_cost_wt))
    print("Avg trip cost without transfers:", int(avg_trip_cost_wot))
    print("Avg wait cost with transfers:", int(avg_wait_cost_wt))
    print("Avg wait cost without transfers:", int(avg_wait_cost_wot))
    print("Avg change in trip cost with transfers:", int(avg_trip_others_cost_wt))
    print("Avg change in trip cost without transfers:", int(avg_trip_others_cost_wot))
    print("Avg veh cost with transfers:", int(avg_veh_cost_wt))
    print("Avg veh cost without transfers:", int(avg_veh_cost_wot))
    print("Avg walking cost with transfers:", int(avg_walking_cost_wt))
    print("Avg walking cost without transfers:", int(avg_walking_cost_wot))

    print("Avg total cost for improved with transfers:", int(avg_imp_total_cost_wt))
    print("Avg total cost for improved without transfers:", int(avg_imp_total_cost_wot))
    print("Avg trip cost for improved with transfers:", int(avg_imp_trip_cost_wt))
    print("Avg trip cost for improved without transfers:", int(avg_imp_trip_cost_wot))
    print("Avg wait cost for improved with transfers:", int(avg_imp_wait_cost_wt))
    print("Avg wait cost for improved without transfers:", int(avg_imp_wait_cost_wot))
    print("Avg change in trip cost for improved with transfers:", int(avg_imp_trip_others_cost_wt))
    print("Avg change in trip cost for improved without transfers:", int(avg_imp_trip_others_cost_wot))
    print("Avg veh cost for improved with transfers:", int(avg_imp_veh_cost_wt))
    print("Avg veh cost for improved without transfers:", int(avg_imp_veh_cost_wot))
    print("Avg walking cost for improved with transfers:", int(avg_imp_walking_cost_wt))
    print("Avg walking cost for improved without transfers:", int(avg_imp_walking_cost_wot))

    print("")


# assignmentquality('AQ Berlin 1pct, R all, V 500', './outputs/server/karri-with-transfers/v-500')
# legstats('LS Berlin 1pct, R all, V 500', './outputs/server/karri-with-transfers/v-500')

assignmentquality('AQ Berlin 1pct, R1000, V200', './outputs/local/v-200_r-1000')
legstats('LS Berlin 1pct, R1000, V200', './outputs/local/v-200_r-1000')
assignmentcost('AC Berlin 1pct, R1000, V200', './outputs/local/v-200_r-1000')




