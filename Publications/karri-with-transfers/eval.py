import pandas as pd

INFTY = 1073741823

def format_float(x, n=2):
    return ("{:." + repr(n) + "f}").format(x)

def format_hh_mm(x):
    h = int(x / 36000)
    m = int(x % 36000 / 600)
    return ("{:02d}:{:02d}").format(h, m)

def format_hh_mm_ss(x):
    h = int(x / 36000)
    m = int((x % 36000) / 600)
    s = int((x % 600) / 10)
    return ("{:02d}:{:02d}:{:02d}").format(h, m, s)

def format_mm_ss(x):
    m = x % 6000 / 60
    s = x % 6000 / 60
    return ("{:02f}:{:02f}").format(m, s)




def request_density_over_time(name, path):
    df_req = pd.read_csv(path)
    n_req = df_req.index.size
    min_time = df_req['req_time'].min()
    max_time = df_req['req_time'].max()

    # Accumulate the results per 5 minutes
    n_intervals = int((max_time - min_time) / 3000) + 1
    req_density = [0] * n_intervals

    for i in range(n_req):
        interval = int((df_req.loc[i, 'req_time'] - min_time) / 3000)
        req_density[interval] += 1

    print(name)
    print("Total duration:", format_hh_mm(max_time - min_time))
    print(req_density)
    print("")





def assignmentquality(name, path):
    print(name)
    df_asq_wt = pd.read_csv(path + '/wt/wt.assignmentquality.csv')
    df_asq_wot = pd.read_csv(path + '/wot/wot.assignmentquality.csv')

    # Get the mean wait and trip times
    avg_wait_time_wt = df_asq_wt['wait_time'].mean()
    avg_trip_time_wt = df_asq_wt['trip_time'].mean()
    avg_wait_time_wot = df_asq_wot['wait_time'].mean()
    avg_trip_time_wot = df_asq_wot['trip_time'].mean()

    print("Avg wait time with transfers:", format_hh_mm_ss(int(avg_wait_time_wt)))
    print("Avg wait time without transfers:", format_hh_mm_ss(int(avg_wait_time_wot)))
    print("Avg trip time with transfers:", format_hh_mm_ss(int(avg_trip_time_wt)))
    print("Avg trip time without transfers:", format_hh_mm_ss(int(avg_trip_time_wot)))
    print("")


def legstats(name, path):
    print(name)
    df_leg_wt = pd.read_csv(path + '/wt/wt.legstats.csv')
    df_leg_wot = pd.read_csv(path + '/wot/wot.legstats.csv')

    # Calculate average occupancy and total vehicle operation time
    n_vehicles_wot = df_leg_wot["vehicle_id"].max() + 1
    n_vehicles_wt = df_leg_wt["vehicle_id"].max() + 1

    print("Anzahl Fahrzeuge WOT: " + repr(int(n_vehicles_wot)))
    print("Anzahl Fahrzeuge WT: " + repr(int(n_vehicles_wt)))

    n_legs_wot = df_leg_wot.index.size
    n_legs_wt = df_leg_wt.index.size

    # print("Anzahl Legs WOT: " + repr(int(n_legs_wot)))
    # print("Anzahl Legs WT: " + repr(int(n_legs_wt)))

    # for i in range(n_legs_wot):
    #     if (df_leg_wot.loc[i, 'drive_time'] == 0):
    #         df_leg_wot.drop(i, inplace=True)

    # for i in range(n_legs_wt):
    #     if (df_leg_wt.loc[i, 'drive_time'] == 0):
    #         df_leg_wt.drop(i, inplace=True)
            
    # n_legs_wot = df_leg_wot.index.size
    # n_legs_wt = df_leg_wt.index.size

    # print("Anzahl Legs > 0 WOT: " + repr(int(n_legs_wot)))
    # print("Anzahl Legs > 0 WT: " + repr(int(n_legs_wt)))

    # print(df_leg_wot)
    # print(df_leg_wt)


    # Stats per vehicle
    total_driving_time_wt = [0] * n_vehicles_wt
    total_stop_time_wt = [0] * n_vehicles_wt
    weighted_occupancy_wt = [0] * n_vehicles_wt
    total_operation_time_wt = [0] * n_vehicles_wt
    total_number_stops_wt = [0] * n_vehicles_wt

    total_driving_time_wot = [0] * n_vehicles_wot
    total_stop_time_wot = [0] * n_vehicles_wot
    weighted_occupancy_wot = [0] * n_vehicles_wot
    total_operation_time_wot = [0] * n_vehicles_wot
    total_number_stops_wot = [0] * n_vehicles_wot

    for i in range(n_legs_wt):
        veh_id = df_leg_wt.loc[i, 'vehicle_id']
        driving_time = df_leg_wt.loc[i, 'drive_time']
        stop_time = df_leg_wt.loc[i, 'stop_time']
        occupancy = df_leg_wt.loc[i, 'occupancy']
        total_driving_time_wt[veh_id] += driving_time
        total_stop_time_wt[veh_id] += stop_time
        total_operation_time_wt[veh_id] += stop_time + driving_time
        weighted_occupancy_wt[veh_id] += driving_time * occupancy
        total_number_stops_wt[veh_id] += 1

    for i in range(n_legs_wot):
        veh_id = df_leg_wot.loc[i, 'vehicle_id']
        driving_time = df_leg_wot.loc[i, 'drive_time']
        stop_time = df_leg_wot.loc[i, 'stop_time']
        occupancy = df_leg_wot.loc[i, 'occupancy']
        total_driving_time_wot[veh_id] += driving_time
        total_stop_time_wot[veh_id] += stop_time
        total_operation_time_wot[veh_id] += stop_time + driving_time
        weighted_occupancy_wot[veh_id] += driving_time * occupancy
        total_number_stops_wot[veh_id] += 1

    # # Avg occupancy
    # avg_occupancy_wt = [0] * n_vehicles_wt
    # avg_occupancy_wot = [0] * n_vehicles_wot

    # for i in range(n_vehicles_wt):
    #     avg_occupancy_wt[i] = 0 if total_driving_time_wt[i] == 0 else weighted_occupancy_wt[i] / total_driving_time_wt[i]

    # for i in range(n_vehicles_wot):
    #     avg_occupancy_wot[i] = 0 if total_driving_time_wot[i] == 0 else weighted_occupancy_wot[i] / total_driving_time_wot[i]

    # total_avg_occupancy_wt = sum(avg_occupancy_wt) / n_vehicles_wt
    # total_avg_occupancy_wot = sum(avg_occupancy_wot) / n_vehicles_wot
    
    # total_avg_driving_time_wt = sum(total_driving_time_wt) / n_vehicles_wt
    # total_avg_driving_time_wot = sum(total_driving_time_wot) / n_vehicles_wot

    total_avg_operation_time_wt = sum(total_operation_time_wt) / (n_vehicles_wt * 10)
    total_avg_operation_time_wot = sum(total_operation_time_wot) / (n_vehicles_wot * 10)

    # avg_stops_wt = sum(total_number_stops_wt) / n_vehicles_wt
    # avg_stops_wot = sum(total_number_stops_wot) / n_vehicles_wot

    # print("Avg occupancy with transfers:", format_float(total_avg_occupancy_wt, 4))
    # print("Avg occupancy without transfers:", format_float(total_avg_occupancy_wot, 4))
    print("Total operation time with transfers:", format_hh_mm_ss(total_avg_operation_time_wt))
    print("Total operation time without transfers:", format_hh_mm_ss(total_avg_operation_time_wot))
    # print("Total driving time with transfers:", format_hh_mm_ss(total_avg_driving_time_wt))
    # print("Total driving time without transfers:", format_hh_mm_ss(total_avg_driving_time_wot))
    # print("Total num stops with transfers:", repr(int(avg_stops_wt)))
    # print("Total num stops without transfers:", repr(int(avg_stops_wot)))
    # print("")

def writeCostStructure(name, total, walking, trip, trip_others, wait, veh):
    f_ac = open("./results/results.txt", "a")
    f_ac.write("Coordinates for " + name + "\n")
    coords = "coordinates {"
    coords += "(total," + repr(int(total)) + ") "
    coords += "(walking," + repr(int(walking)) + ") "
    coords += "(trip," + repr(int(trip)) + ") "
    coords += "(trip change," + repr(int(trip_others)) + ") "
    coords += "(wait," + repr(int(wait)) + ") "
    coords += "(vehicle," + repr(int(veh)) + ") "
    coords += "(empty,0)};\n"
    f_ac.write(coords)
    f_ac.close()

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
    n_total = 0
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
            n_total += 1
            total_total_cost_wt += df_cost_wt.loc[i, 'total_transfer']
            total_trip_cost_wt += df_cost_wt.loc[i, 'trip_cost_transfer']
            total_wait_cost_wt += df_cost_wt.loc[i, 'wait_time_violation_cost_transfer']
            total_trip_others_cost_wt += df_cost_wt.loc[i, 'change_in_trip_costs_of_others_transfer']
            total_veh_cost_wt += df_cost_wt.loc[i, 'veh_cost_transfer']
            total_walking_cost_wt += df_cost_wt.loc[i, 'walking_cost_transfer']
        elif (df_cost_wt.loc[i, 'infty_no_transfer'] == 0):
            n_total += 1
            total_total_cost_wt += df_cost_wt.loc[i, 'total_no_transfer']
            total_trip_cost_wt += df_cost_wt.loc[i, 'trip_cost_no_transfer']
            total_wait_cost_wt += df_cost_wt.loc[i, 'wait_time_violation_cost_no_transfer']
            total_trip_others_cost_wt += df_cost_wt.loc[i, 'change_in_trip_costs_of_others_no_transfer']
            total_veh_cost_wt += df_cost_wt.loc[i, 'veh_cost_no_transfer']
            total_walking_cost_wt +=df_cost_wt.loc[i, 'walking_cost_no_transfer']


    avg_total_cost_wt = total_total_cost_wt / n_total
    avg_trip_cost_wt = total_trip_cost_wt / n_total
    avg_wait_cost_wt = total_wait_cost_wt / n_total
    avg_trip_others_cost_wt = total_trip_others_cost_wt / n_total
    avg_veh_cost_wt = total_veh_cost_wt / n_total
    avg_walking_cost_wt = total_walking_cost_wt / n_total

    # f_ac = open("./results/results.txt", "a")
    # f_ac.write(name + "\n")
    # f_ac.write("Improved assignments" + "\n")
    # f_ac.write(repr(n_assignments) + " & " + repr(n_improved_assignments) + " & " + format_float(n_improved_assignments / n_assignments * 100) + "%" + "\n")
    # f_ac.close()

    # writeCostStructure("cost structure wt", avg_total_cost_wt, avg_walking_cost_wt, avg_trip_cost_wt, avg_trip_others_cost_wt, avg_wait_cost_wt, avg_veh_cost_wt)
    # writeCostStructure("cost structure wot", avg_total_cost_wot, avg_walking_cost_wot, avg_trip_cost_wot, avg_trip_others_cost_wot, avg_wait_cost_wot, avg_veh_cost_wot)
    
    
    print("# total assignments: " + repr(n_assignments))
    print("# improved assignments: " + repr(n_improved_assignments))
    print("Improved: " + format_float(n_improved_assignments / n_assignments * 100) + "%")
    print("")
    
    # print("Avg total cost with transfers:", int(avg_total_cost_wt))
    # print("Avg total cost without transfers:", int(avg_total_cost_wot))
    # print("Avg trip cost with transfers:", int(avg_trip_cost_wt))
    # print("Avg trip cost without transfers:", int(avg_trip_cost_wot))
    # print("Avg wait cost with transfers:", int(avg_wait_cost_wt))
    # print("Avg wait cost without transfers:", int(avg_wait_cost_wot))
    # print("Avg change in trip cost with transfers:", int(avg_trip_others_cost_wt))
    # print("Avg change in trip cost without transfers:", int(avg_trip_others_cost_wot))
    # print("Avg veh cost with transfers:", int(avg_veh_cost_wt))
    # print("Avg veh cost without transfers:", int(avg_veh_cost_wot))
    # print("Avg walking cost with transfers:", int(avg_walking_cost_wt))
    # print("Avg walking cost without transfers:", int(avg_walking_cost_wot))
    
    # Calculate the mean costs wt / wot of the assignments that have been improved
    for i in range(n_assignments):
        if df_cost_wt.loc[i, 'transfer_improves'] == 0 or df_cost_wt.loc[i, 'infty_transfer'] or df_cost_wt.loc[i, 'infty_no_transfer']:
            df_cost_wt.drop(i, inplace=True)
    
    if df_cost_wt.index.size == 0:
        print("Only assignments wiht former cost infty were improved")
        return

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

    # writeCostStructure("cost structure improved wt", avg_imp_total_cost_wt, avg_imp_walking_cost_wt, avg_imp_trip_cost_wt, avg_imp_trip_others_cost_wt, avg_imp_wait_cost_wt, avg_imp_veh_cost_wt)
    # writeCostStructure("cost structure improved wot", avg_imp_total_cost_wot, avg_imp_walking_cost_wot, avg_imp_trip_cost_wot, avg_imp_trip_others_cost_wot, avg_imp_wait_cost_wot, avg_imp_veh_cost_wot)
    
    # print("Avg total cost for improved with transfers:", int(avg_imp_total_cost_wt))
    # print("Avg total cost for improved without transfers:", int(avg_imp_total_cost_wot))
    # print("Avg trip cost for improved with transfers:", int(avg_imp_trip_cost_wt))
    # print("Avg trip cost for improved without transfers:", int(avg_imp_trip_cost_wot))
    # print("Avg wait cost for improved with transfers:", int(avg_imp_wait_cost_wt))
    # print("Avg wait cost for improved without transfers:", int(avg_imp_wait_cost_wot))
    # print("Avg change in trip cost for improved with transfers:", int(avg_imp_trip_others_cost_wt))
    # print("Avg change in trip cost for improved without transfers:", int(avg_imp_trip_others_cost_wot))
    # print("Avg veh cost for improved with transfers:", int(avg_imp_veh_cost_wt))
    # print("Avg veh cost for improved without transfers:", int(avg_imp_veh_cost_wot))
    # print("Avg walking cost for improved with transfers:", int(avg_imp_walking_cost_wt))
    # print("Avg walking cost for improved without transfers:", int(avg_imp_walking_cost_wot))

    # print("")

BASE_PATH_SERVER = './outputs/server/karri-with-transfers'

PATH_V_100_R_HOUR_1 = BASE_PATH_SERVER + '/v-100_r-hour-1'
PATH_V_150_R_HOUR_1 = BASE_PATH_SERVER + '/v-150_r-hour-1'
PATH_V_200_R_HOUR_1 = BASE_PATH_SERVER + '/v-200_r-hour-1'
PATH_V_250_R_HOUR_1 = BASE_PATH_SERVER + '/v-250_r-hour-1'
PATH_V_500_R_HOUR_1 = BASE_PATH_SERVER + '/v-500_r-hour-1'

PATH_V_500_R_ALL = BASE_PATH_SERVER + '/v-500_r-all'

PATH_V_100_R_HOUR_3 = BASE_PATH_SERVER + '/v-100_r-hour-3'
PATH_V_150_R_HOUR_3 = BASE_PATH_SERVER + '/v-150_r-hour-3'
PATH_V_175_R_HOUR_3 = BASE_PATH_SERVER + '/v-175_r-hour-3'

PATH_V_200_R_HOUR_3 = BASE_PATH_SERVER + '/v-200_r-hour-3'
NAME_V_200_R_HOUR_3 = 'Vehicles: 200 Request: Hour 3'
PATH_V_200_R_HOUR_3_NO_PSG = BASE_PATH_SERVER + '/v-200_r-hour-3_psg_cost_0'
NAME_V_200_R_HOUR_3_NO_PSG = 'Vehicles: 200 Request: Hour 3, Psg Cost Factor: 0'

PATH_V_200_R_HOUR_3_WAIT_600 = BASE_PATH_SERVER + '/v-200_r-hour-3_wait_600'
NAME_V_200_R_HOUR_3_WAIT_600 = 'Vehicles: 200 Request: Hour 3, Wait Time: 600'
PATH_V_200_R_HOUR_3_NO_PSG_WAIT_600 = BASE_PATH_SERVER + '/v-200_r-hour-3_psg_cost_0_wait_600'
NAME_V_200_R_HOUR_3_NO_PSG_WAIT_600 = 'Vehicles: 200 Request: Hour 3, Psg Cost Factor: 0, Wait Time: 600'


PATH_V_225_R_HOUR_3 = BASE_PATH_SERVER + '/v-225_r-hour-3'
PATH_V_250_R_HOUR_3 = BASE_PATH_SERVER + '/v-250_r-hour-3'
PATH_V_500_R_HOUR_3 = BASE_PATH_SERVER + '/v-500_r-hour-3'


PATH_V_200_R_FOURTH = BASE_PATH_SERVER + '/v-200_r-fourth-dens'
NAME_V_200_R_FOURTH = 'Vehicles: 200 Request: 1/4 Berlin 1pct'

PATH_V_400_R_FOURTH = BASE_PATH_SERVER + '/v-400_r-fourth-dens'
NAME_V_400_R_FOURTH = 'Vehicles: 400 Request: 1/4 Berlin 1pct'

PATH_V_400_R_HALF = BASE_PATH_SERVER + '/v-400_r-half-dens'
NAME_V_400_R_HALF = 'Vehicles: 400 Request: 1/2 Berlin 1pct'


# f_res = open("./results/results.txt", "w")
# f_res.write("")

# assignmentquality('Assignment Quality ' + NAME, PATH)
# legstats('Leg Stats ' + NAME, PATH)
# assignmentcost('Assignment Cost ' + NAME, PATH)

# request_density_over_time('Berlin 1pct', './inputs/Berlin-1pct-all.csv')
# request_density_over_time('Berlin 1pct half dens', './inputs/Berlin-1pct-half-density.csv')

# f_res.close()





# assignmentquality('Assignment Quality ' + NAME_V_200_R_FOURTH, PATH_V_200_R_FOURTH)
# legstats('Leg Stats ' + NAME_V_200_R_FOURTH, PATH_V_200_R_FOURTH)
# assignmentcost('Assignment Cost ' + NAME_V_200_R_FOURTH, PATH_V_200_R_FOURTH)

# assignmentquality('Assignment Quality ' + NAME_V_400_R_FOURTH, PATH_V_400_R_FOURTH)
# legstats('Leg Stats ' + NAME_V_400_R_FOURTH, PATH_V_400_R_FOURTH)
# assignmentcost('Assignment Cost ' + NAME_V_400_R_FOURTH, PATH_V_400_R_FOURTH)

# assignmentquality('Assignment Quality ' + NAME_V_400_R_HALF, PATH_V_400_R_HALF)
# legstats('Leg Stats ' + NAME_V_400_R_HALF, PATH_V_400_R_HALF)
# assignmentcost('Assignment Cost ' + NAME_V_400_R_HALF, PATH_V_400_R_HALF)





# Bisher am besten : V 200 Hour 3 -> 1 Minuten op gespart, 0.02 mehr Auslastung, 4.44% verbesserte Assignments
# assignmentquality('Assignment Quality ' + NAME_V_200_R_HOUR_3, PATH_V_200_R_HOUR_3)
# legstats('Leg Stats ' + NAME_V_200_R_HOUR_3, PATH_V_200_R_HOUR_3)
# assignmentcost('Assignment Cost ' + NAME_V_200_R_HOUR_3, PATH_V_200_R_HOUR_3)

# assignmentquality('Assignment Quality ' + NAME_V_200_R_HOUR_3_WAIT_600, PATH_V_200_R_HOUR_3_WAIT_600)
# legstats('Leg Stats ' + NAME_V_200_R_HOUR_3_WAIT_600, PATH_V_200_R_HOUR_3_WAIT_600)
# assignmentcost('Assignment Cost ' + NAME_V_200_R_HOUR_3_WAIT_600, PATH_V_200_R_HOUR_3_WAIT_600)

# assignmentquality('Assignment Quality ' + NAME_V_200_R_HOUR_3_NO_PSG_WAIT_600, PATH_V_200_R_HOUR_3_NO_PSG_WAIT_600)
# legstats('Leg Stats ' + NAME_V_200_R_HOUR_3_NO_PSG_WAIT_600, PATH_V_200_R_HOUR_3_NO_PSG_WAIT_600)
# assignmentcost('Assignment Cost ' + NAME_V_200_R_HOUR_3_NO_PSG_WAIT_600, PATH_V_200_R_HOUR_3_NO_PSG_WAIT_600)


# assignmentquality('Assignment Quality ' + NAME_V_200_R_HOUR_3_NO_PSG, PATH_V_200_R_HOUR_3_NO_PSG)
# legstats('Leg Stats ' + NAME_V_200_R_HOUR_3_NO_PSG, PATH_V_200_R_HOUR_3_NO_PSG)
# assignmentcost('Assignment Cost ' + NAME_V_200_R_HOUR_3_NO_PSG, PATH_V_200_R_HOUR_3_NO_PSG)

# assignmentquality('Assignment Quality ' + "Berlin 1pct, V500", PATH_V_500_R_ALL)


# legstats('Leg Stats ' + "Berlin 1pct, V500", PATH_V_500_R_ALL)
# assignmentcost('Assignment Cost ' + "Berlin 1pct, V500", PATH_V_500_R_ALL)


BASE_PATH_HOPE = './outputs/hope'
PATH_V_400_R_HOUR_2 = BASE_PATH_HOPE + '/v-400-h2_r-hour-2'
PATH_V_400_R_HOUR_2_PSG_0 = BASE_PATH_HOPE + '/v-400-h2_r-hour-2_psg_0'
PATH_V_200_R_HOUR_2 = BASE_PATH_HOPE + '/v-200-h2_r-hour-2'
PATH_V_200_R_HOUR_2_PSG_0 = BASE_PATH_HOPE + '/v-200-h2_r-hour-2_psg_0'


legstats('AQ ' + "V 500 V ALL", PATH_V_500_R_ALL)

legstats('AQ ' + "V 400 V H2", PATH_V_400_R_HOUR_2)
legstats('AQ ' + "V 400 V H2 PSG 0", PATH_V_400_R_HOUR_2_PSG_0)

legstats('AQ ' + "V 200 V H2", PATH_V_200_R_HOUR_2)
legstats('AQ ' + "V 200 V H2 PSG 0", PATH_V_200_R_HOUR_2_PSG_0)


# legstats('AQ ' + "V 400 V H2", PATH_V_400_R_HOUR_2)
# legstats('AQ ' + "V 400 V H2", PATH_V_400_R_HOUR_2)
# legstats('AQ ' + "V 500 V ALL", PATH_V_500_R_ALL)
# legstats('AQ ' + "V 400 V HALF DENS", PATH_V_400_R_HALF)
# legstats('AQ ' + "V 400 V FOURTH DENS", PATH_V_400_R_FOURTH)

# legstats('AQ ' + "V 500 R ALL", PATH_V_500_R_ALL)

# assignmentquality('AQ ' + "V 200 V H3", PATH_V_500_R_HOUR_3)
# assignmentquality('AQ ' + "V 200 V H3", PATH_V_500_R_HOUR_3)