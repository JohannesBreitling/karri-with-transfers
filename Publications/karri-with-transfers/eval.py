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
    m = int((x % 36000) / 600)
    s = int((x % 600) / 10)
    return ("{:02d}:{:02d}").format(m, s)

def get_table_row(vals):
    result = ""
    result += "\\texttt{" + vals[0] + "}"

    for i in range(1, len(vals)):
        result += " & "
        result += vals[i]
    result += "\\\\"
    return result

def generate_coordinates(keys, vals):
    result = "coordinates {"
    sep = ""
    for i in range(len(keys)):
        result += sep + "(" + keys[i] + "," + repr(int(vals[i])) + ")"
        sep = " "
    result += "};"
    return result


def clear_files():
    print("Clearing files.....", end='')
    
    f_rs = open("./results/request_sets.txt", "w")
    f_rs.write("Evaluation of Request Sets\n")
    f_rs.close()
    
    f_dq = open("./results/dispatch_quality.txt", "w")
    f_dq.write("Dispatch Quality\n")
    f_dq.close()

    f_tq = open("./results/transfer_quality.txt", "w")
    f_tq.write("Transfer Quality\n")
    f_tq.close()

    f_tp = open("./results/transfer_perf.txt", "w")
    f_tp.write("Transfer Performance\n")
    f_tp.close()
    
    print(" done")

def evaluate_request_set(name, path):
    print("Evaluate request set.....", end='')
    df_req = pd.read_csv(path)
    n_req = df_req.index.size
    min_time = df_req['req_time'].min()
    max_time = df_req['req_time'].max()

    # Accumulate the request density over time
    n_intervals = int((max_time - min_time) / 3000) + 1
    req_density = [0] * n_intervals
    time = [0] * n_intervals

    curr_time = 0
    for i in range(n_intervals):
        # time[i] = format_hh_mm(curr_time)
        time[i] = str(int(curr_time / 600))
        curr_time += 3000
    
    for i in range(n_req):
        interval = int((df_req.loc[i, 'req_time'] - min_time) / 3000)
        req_density[interval] += 1

    # Write the request data to the output file
    f_rs = open("./results/request_sets.txt", "a")
    f_rs.write("\n")
    f_rs.write(name + "\n")

    f_rs.write("#requests: " + repr(n_req) + "\n")
    f_rs.write("From: " + format_hh_mm(min_time) + " ")
    f_rs.write("To: " + format_hh_mm(max_time) + "\n")
    f_rs.write("Duration: " + format_hh_mm(max_time - min_time) + "\n")

    coords = generate_coordinates(time, req_density)
    f_rs.write(coords + "\n")
    f_rs.close()
    print(" done")


def dispatch_quality(name, path):
    print("Determine dispatch quality.....", end='')
    df_leg_wt = pd.read_csv(path + '/wt/wt.legstats.csv')
    df_leg_wot = pd.read_csv(path + '/wot/wot.legstats.csv')

    df_aq_wt = pd.read_csv(path + '/wt/wt.assignmentquality.csv')
    df_aq_wot = pd.read_csv(path + '/wot/wot.assignmentquality.csv')

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
    total_number_stops_wt = [0] * n_vehicles_wt

    total_driving_time_wot = [0] * n_vehicles_wot
    total_stop_time_wot = [0] * n_vehicles_wot
    weighted_occupancy_wot = [0] * n_vehicles_wot
    total_operation_time_wot = [0] * n_vehicles_wot
    total_number_stops_wot = [0] * n_vehicles_wot

    for i in range(n_legs_wt):
        veh_id = df_leg_wt.loc[i, 'vehicle_id']
        driving_time = df_leg_wt.loc[i, 'drive_time']
        if (driving_time == 0):
            continue
        
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
        if (driving_time == 0):
            continue

        stop_time = df_leg_wot.loc[i, 'stop_time']
        occupancy = df_leg_wot.loc[i, 'occupancy']
        total_driving_time_wot[veh_id] += driving_time
        total_stop_time_wot[veh_id] += stop_time
        total_operation_time_wot[veh_id] += stop_time + driving_time
        weighted_occupancy_wot[veh_id] += driving_time * occupancy
        total_number_stops_wot[veh_id] += 1

    # Avg occupancy
    avg_occupancy_wt = [0] * n_vehicles_wt
    avg_occupancy_wot = [0] * n_vehicles_wot

    for i in range(n_vehicles_wt):
        avg_occupancy_wt[i] = 0 if total_driving_time_wt[i] == 0 else weighted_occupancy_wt[i] / total_driving_time_wt[i]

    for i in range(n_vehicles_wot):
        avg_occupancy_wot[i] = 0 if total_driving_time_wot[i] == 0 else weighted_occupancy_wot[i] / total_driving_time_wot[i]

    total_avg_occupancy_wt = sum(avg_occupancy_wt) / n_vehicles_wt
    total_avg_occupancy_wot = sum(avg_occupancy_wot) / n_vehicles_wot
    
    total_avg_driving_time_wt = sum(total_driving_time_wt) / (n_vehicles_wt * 10)
    total_avg_driving_time_wot = sum(total_driving_time_wot) / (n_vehicles_wot * 10)

    total_avg_operation_time_wt = sum(total_operation_time_wt) / (n_vehicles_wt * 10)
    total_avg_operation_time_wot = sum(total_operation_time_wot) / (n_vehicles_wot * 10)

    avg_stops_wt = sum(total_number_stops_wt) / n_vehicles_wt
    avg_stops_wot = sum(total_number_stops_wot) / n_vehicles_wot

    # Get the mean wait and trip times
    avg_wait_time_wt = df_aq_wt['wait_time'].mean()
    avg_trip_time_wt = df_aq_wt['trip_time'].mean()
    avg_wait_time_wot = df_aq_wot['wait_time'].mean()
    avg_trip_time_wot = df_aq_wot['trip_time'].mean()

    avg_cost_wt = df_aq_wt['cost'].mean()
    avg_cost_wot = df_aq_wot['cost'].mean()
    
    vals_wt = []
    vals_wt.append(name)
    vals_wt.append('KT')

    vals_wot = []
    vals_wot.append(name)
    vals_wot.append('K')

    vals_wt.append(format_mm_ss(int(avg_wait_time_wt)))
    vals_wt.append(format_mm_ss(int(avg_trip_time_wt)))
    
    vals_wot.append(format_mm_ss(int(avg_wait_time_wot)))
    vals_wot.append(format_mm_ss(int(avg_trip_time_wot)))
    
    vals_wt.append(format_hh_mm_ss(int(total_avg_operation_time_wt)))
    vals_wot.append(format_hh_mm_ss(int(total_avg_operation_time_wot)))

    vals_wt.append(format_hh_mm_ss(int(total_avg_driving_time_wt)))
    vals_wot.append(format_hh_mm_ss(int(total_avg_driving_time_wot)))

    vals_wt.append(format_float(total_avg_occupancy_wt, 3))
    vals_wot.append(format_float(total_avg_occupancy_wot, 3))

    vals_wt.append(repr(int(avg_stops_wt)))
    vals_wot.append(repr(int(avg_stops_wot)))

    vals_wt.append(repr(int(avg_cost_wt)))
    vals_wot.append(repr(int(avg_cost_wot)))

    f_dq = open("./results/dispatch_quality.txt", "a")
    
    row_wt = get_table_row(vals_wt)
    row_wot = get_table_row(vals_wot)
    
    f_dq.write(row_wt + "\n")
    f_dq.write(row_wot + "\n")
    f_dq.close()
    print(" done")


def transfer_quality(name, path):
    df_imp = pd.read_csv(path + '/wt/wt.assignmentcost.csv')
    
    n_assignments = df_imp.index.size
    
    n_improved = 0
    n_improved_no_walk = 0
    
    # Calculate the distribution of cost improvements
    quantiles = ["0", "5", "10", "15", "20", "25", "30", "35", "40", "45", "50", "55", "60", "65", "70", "75", "80", "85", "90", "95", "100"]
    # quantiles = [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100]
    quantiles_values = [0] * len(quantiles)
    
    for i in range(n_assignments):
        if df_imp.loc[i, 'transfer_improves'] == 0:
            df_imp.drop(i, inplace=True)    
            continue
        
        if df_imp.loc[i, 'infty_no_transfer'] == 1:
            df_imp.drop(i, inplace=True)
            n_improved += 1
            continue

        n_improved += 1
        n_improved_no_walk += 1

        cost_wt = df_imp.loc[i, 'total_transfer']
        cost_wot = df_imp.loc[i, 'total_no_transfer']

        delta = cost_wot - cost_wt
        p_delta = int(delta * 100 / cost_wot)
        quantile = int(p_delta / 5) + 1
        quantiles_values[quantile] += 1 


    quantiles_result = [0] * len(quantiles)
    n_left = n_improved_no_walk

    for i in range(len(quantiles_values)):
        quantiles_result[i] = int(((n_left - quantiles_values[i])))
        n_left -= quantiles_values[i]

    for i in range(len(quantiles_result)):
        quantiles_result[i] = round(quantiles_result[i] * 100 / n_improved_no_walk)
    
    total_cost_imp = df_imp['total_transfer'].mean()
    total_cost_unimp = df_imp['total_no_transfer'].mean()
    
    detour_imp = df_imp['veh_cost_transfer'].mean()
    detour_unimp = df_imp['veh_cost_no_transfer'].mean()

    trip_unimp = df_imp['trip_cost_no_transfer'].mean()
    trip_imp = df_imp['trip_cost_transfer'].mean()

    dist_coords = generate_coordinates(quantiles, quantiles_result)
    
    cost_delta = repr(int(total_cost_imp - total_cost_unimp)) 
    trip_delta = "-" + format_mm_ss(int((trip_unimp - trip_imp) / 10))
    detour_delta = "-" + format_mm_ss(int((detour_unimp - detour_imp) / 10))
        
    vals = []
    vals.append(name)

    p_improved = format_float(n_improved / n_assignments * 100) + "\%"
    p_improved_no_walk = format_float(n_improved_no_walk / n_assignments * 100) + "\%"
    vals.append(p_improved)
    vals.append(p_improved_no_walk)

    vals.append(cost_delta)
    vals.append(detour_delta)
    vals.append(trip_delta)


    row = get_table_row(vals)
    
    f_tq = open("./results/transfer_quality.txt", "a")
    f_tq.write(row + "\n")
    f_tq.write(dist_coords + "\n")
    f_tq.close()

def routelength(name, path):
    print(name)
    df_asg_wt = pd.read_csv(path + '/wt/wt.bestassignmentswithtransfer.csv')
    df_asg_nt = pd.read_csv(path + '/wt/wt.bestassignments.csv')

    df_asg_wot = pd.read_csv(path + '/wot/wot.bestassignments.csv')
    n_reqs = df_asg_wt.index.size

    n_vehicles = max(df_asg_wt['pickup_vehicle_id'].max(), df_asg_wt['dropoff_vehicle_id'].max(), df_asg_nt['vehicle_id'].max()) + 1

    total_route_length_wt = [0] * n_vehicles
    total_insertions_wt = [0] * n_vehicles

    total_route_length_wot = [0] * n_vehicles
    total_insertions_wot = [0] * n_vehicles

    for i in range(n_reqs):
        
        # Update the route for the case where transfers are turned off
        veh_id = df_asg_wot.loc[i, 'vehicle_id']
        stops = df_asg_wot.loc[i, 'num_stops']

        if (stops >= 0):
            total_route_length_wot[veh_id] += stops
            total_insertions_wot[veh_id] += 1
        
        
        cost_wt = df_asg_wt.loc[i, 'cost']
        cost_wot = df_asg_nt.loc[i, 'cost']
        
        if cost_wt > cost_wot:
            veh_id = df_asg_nt.loc[i, 'vehicle_id']
            stops = df_asg_nt.loc[i, 'num_stops']
            if stops > 0:
                total_route_length_wt[veh_id] += stops
                total_insertions_wt[veh_id] += 1

        else:
            pveh_id = df_asg_wt.loc[i, 'pickup_vehicle_id']
            stops = df_asg_wt.loc[i, 'num_stops_pveh']
            if stops > 0:
                total_route_length_wt[pveh_id] += stops
                total_insertions_wt[pveh_id] += 1

            dveh_id = df_asg_wt.loc[i, 'dropoff_vehicle_id']
            stops = df_asg_wt.loc[i, 'num_stops_dveh']
            if stops > 0:
                total_route_length_wt[dveh_id] += stops
                total_insertions_wt[dveh_id] += 1
        
        # if 

        # pveh_id = df_asg_wt.loc[i, 'pickup_vehicle_id']
        # dveh_id = df_asg_wt.loc[i, 'dropoff_vehicle_id']


        # total_route_length_wt[veh_id] += df_asg_wt.loc[i, 'route_length']
        # total_insertions_wt[veh_id] += 1

        # veh_id = df_asg_nt.loc[i, 'vehicle_id']
        # total_route_length_wot[veh_id] += df_asg_nt.loc[i, 'route_length']
        # total_insertions_wot[veh_id] += 1

    for i in range(0, n_vehicles):
        total_route_length_wt[i] = 0 if total_insertions_wt[i] == 0 else total_route_length_wt[i] / total_insertions_wt[i]
        total_route_length_wot[i] = 0 if total_insertions_wot[i] == 0 else total_route_length_wot[i] / total_insertions_wot[i]

    avg_route_length_wt = sum(total_route_length_wt) / n_vehicles
    avg_route_length_wot = sum(total_route_length_wot) / n_vehicles
    print(avg_route_length_wt)
    print(avg_route_length_wot)


def overall_perf(name, path):
    df_perf_wt = pd.read_csv(path + '/wt/wt.perf_overall.csv')
    df_perf_wot = pd.read_csv(path + '/wot/wot.perf_overall.csv')

    avg_t_ord = df_perf_wt['transf_ord_time'].mean()
    avg_t_als_p = df_perf_wt['transf_als_pveh_time'].mean()
    avg_t_als_d = df_perf_wt['transf_als_dveh_time'].mean()
    
    avg_total_rest = df_perf_wt['total_time'].mean()

    avg_total_wt = avg_t_ord + avg_t_als_p + avg_t_als_d + avg_total_rest
    avg_total_wot = df_perf_wot['total_time'].mean()

    
    
    vals = []
    vals.append(name)
    vals.append(format_float(avg_total_wot / 1000000, 3))
    vals.append(format_float(avg_total_wt / 1000000, 2))
    vals.append(format_float(avg_t_ord / 1000000, 2))
    vals.append(format_float(avg_t_als_p / 1000000, 2))
    vals.append(format_float(avg_t_als_d / 1000000, 2))

    row = get_table_row(vals)
    f_tp = open("./results/transfer_perf.txt", "a")
    f_tp.write("Overall Performance (Total WOT, Total WT, TORD, TALS P, TALS D)\n")
    f_tp.write(row + "\n")
    f_tp.close()


def transfer_perf(name, path):
    
    df_ord_perf = pd.read_csv(path + '/wt/wt.perf_transf_ord.csv')
    df_als_perf_p = pd.read_csv(path + '/wt/wt.perf_transf_als_pveh.csv')
    df_als_perf_d = pd.read_csv(path + '/wt/wt.perf_transf_als_dveh.csv')

    df_aq = pd.read_csv(path + '/wt/wt.assignmentquality.csv')
    df_asg = pd.read_csv(path + '/wt/wt.bestassignmentswithtransfer.csv') 
    n_asg = df_asg.index.size

        

    t_ord = 0
    t_als_pveh = 0
    t_als_dveh = 0

    p_bns = 0
    p_ord = 0
    p_als = 0
    d_bns = 0
    d_ord = 0
    d_als = 0

    n_total = 0

    for i in range(n_asg):
        if str(df_asg.loc[i, 'cost']) == 'inf':
            continue
        
        n_total += 1
        cost = int(df_asg.loc[i, 'cost'])

        t_pveh = df_asg.loc[i, 'transfer_type_pveh'].strip()
        t_dveh = df_asg.loc[i, 'transfer_type_dveh'].strip()

        t_p = df_asg.loc[i, 'pickup_type'].strip()
        t_d = df_asg.loc[i, 'dropoff_type'].strip()

        if ((t_pveh == "ORD" or t_pveh == "BNS") and (t_dveh == "ORD" or t_dveh == "BNS")):
            t_ord += 1
        
        if (t_pveh == "ALS" and (t_dveh == "ORD" or t_dveh == "BNS")):
            t_als_pveh += 1

        if ((t_pveh == "ORD" or t_pveh == "BNS") and t_dveh == "ALS"):
            t_als_dveh += 1

        if (t_p == "BNS"):
            p_bns += 1
        if (t_p == "ORD"):
            p_ord += 1
        if (t_p == "ALS"):
            p_als += 1
        
        if (t_d == "BNS"):
            d_bns += 1
        if (t_d == "ORD"):
            d_ord += 1
        if (t_d == "ALS"):
            d_als += 1


    f_tp = open("./results/transfer_perf.txt", "a")
    f_tp.write(name + "\n")

    vals = []
    vals.append(name)
    vals.append(format_float(t_ord * 100 / n_total) + "\%")
    vals.append(format_float(t_als_pveh * 100 / n_total) + "\%")
    vals.append(format_float(t_als_dveh * 100 / n_total) + "\%")

    vals.append(format_float(p_bns * 100 / n_total) + "\%")
    vals.append(format_float(p_ord * 100 / n_total) + "\%")
    vals.append(format_float(p_als * 100 / n_total) + "\%")

    vals.append(format_float(d_bns * 100 / n_total) + "\%")
    vals.append(format_float(d_ord * 100 / n_total) + "\%")
    vals.append(format_float(d_als * 100 / n_total) + "\%")

    row = get_table_row(vals)
    f_tp.write("Distrubution of Transfer Types and Pickup and Dropoff Types (TORD, TALS P, TALS D, PBNS, PORD, PALS, DBNS, DORD, DALS)\n")
    f_tp.write(row + "\n")


def evaluate_instance(name, path):
    dispatch_quality(name, path)
    transfer_quality(name, path)
    transfer_perf(name, path)
    routelength(name, path)
    overall_perf(name, path)






#############################################################################################
##
## FINAL EVALUAION DONE IN THIS SECTION
##
#############################################################################################

# Final Instances that will be evaluated in the written thesis work

FINAL_BASE = './outputs/final'
FINAL_IN = './inputs/final'

TP_SCEN_ALL = FINAL_IN + '/Berlin-1pct-all.csv'
TP_SCEN_HD = FINAL_IN + '/Berlin-1pct-hd.csv'
TP_SCEN_H2 = FINAL_IN + '/Berlin-1pct-h2.csv'
TP_SCEN_H3 = FINAL_IN + '/Berlin-1pct-h3.csv'

TN_SCEN_ALL = 'B1% All Requests'
TN_SCEN_HD = 'B1% Half Density'
TN_SCEN_H2 = 'B1% Hour 2'
TN_SCEN_H3 = 'B1% Hour 3'

TP_400_ALL = FINAL_BASE + '/v-400_r-all' 
TN_400_ALL = "B1\%-400-all"

TP_400_HD = FINAL_BASE + '/v-400_r-half-density' 
TN_400_HD = "B1\%-400-hd"

TP_200_H3 = FINAL_BASE +'/v-200-h3_r-hour-3'
TN_200_H3 = "B1\%-200-h3"

TP_400_H2 = FINAL_BASE +'/v-400-h2_r-hour-2'
TN_400_H2 = "B1\%-400-h2"

clear_files()

evaluate_request_set(TN_SCEN_ALL, TP_SCEN_ALL)
evaluate_request_set(TN_SCEN_HD, TP_SCEN_HD)
evaluate_request_set(TN_SCEN_H2, TP_SCEN_H2)
evaluate_request_set(TN_SCEN_H3, TP_SCEN_H3)


# Evaluate the instances:
# evaluate_instance(TN_200_H3, TP_200_H3)
# evaluate_instance(TN_400_H2, TP_400_H2)
evaluate_instance(TN_400_HD, TP_400_HD)
evaluate_instance(TN_400_ALL, TP_400_ALL)

