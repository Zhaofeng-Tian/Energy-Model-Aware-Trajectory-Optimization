from sim import Sim
from param.param import Param, TruckParam
import pickle

# if __name__ == '__main__':

def run_experiment(car_type, cycle_list, solver_list, gd_profile_list, p_time_list,weights_list, if_use_gd):
    # cycle_list = ['HWFET', 'INDIA_HWY', 'NYCC', 'NYCTRUCK', 'MANHATTAN' ,'INDIA_URBAN' ]
    # # loop over:
    # solver_list = ['QP_CA', 'SQP', 'NLP']
    # gd_profile_list = ['flat', 'normal', 'steep']
    # p_time_list = [2, 5, 10] # prediction time, different N number
    # weights_list = [[0.1, 10, 10], [0.1, 5, 2], [0.1, 5, 5],[0.1, 5, 10] ]
    # if_use_gd = [True, False]

    cycle_list = cycle_list
    solver_list = solver_list
    gd_profile_list = gd_profile_list
    p_time_list = p_time_list
    weights_list = weights_list
    if_use_gd = if_use_gd


    num_run = 0
    # recording
    file_name_list = []
    skipped_sim_list = []

    for cycle in cycle_list:

        if car_type =='car':
            param = Param(cycle=cycle)
        elif car_type == 'truck':
            param = TruckParam(cycle=cycle)
        else:
            raise ValueError("Got run car type, cannot initialize the param")
        
        for solver_name in solver_list:
            param.solver_type = solver_name
            if solver_name=='SQP':
                param.dmax = 300
            for gd_name in gd_profile_list:
                param.gd_profile_type = gd_name
                for p_time in p_time_list:
                    param.prediction_time = p_time
                    param.N = int(param.prediction_time / param.dt)
                    for weight in weights_list:
                        param.w1, param.w2, param.w3 = weight
                        for use_gd_flag in if_use_gd:
                            param.use_gd_prediction = use_gd_flag
                            if param.solver_type == 'QP_CA' and use_gd_flag == True:
                                continue
                            # ************ Inner Loop Run Here **********************
                            num_run += 1
                            file_name = f"{param.car_type}_{param.cycle}_{param.gd_profile_type}_{param.solver_type}_{param.w1}_{param.w2}_{param.w3}_gd_{param.use_gd_prediction}_ptime_{param.prediction_time}.pkl"
                            print("********** In loop #",num_run, " ****************" )
                            print("running: ", file_name)
                            print(" solver: ", param.solver_type, " use_gd_flag: ", use_gd_flag)
                            sim = Sim(param)
                            try: 

                                sim.run(if_plot=False, if_save=True)
                                file_name_list.append((num_run, file_name))
                            except Exception as e:
                                skipped_sim_list.append((num_run, file_name, str(e)))
                                print(f"Skipping simulation with param {file_name} due to error: {e}")

                            with open("data/file_name_list.txt", "w") as file:
                                for num_run, fn in file_name_list:
                                    file.write(f"Num_run: {num_run}, file_name: {fn} \n")                                
                            
                            with open("data/skipped_simulations.txt", "w") as file:
                                for num_run, fn, error in skipped_sim_list:
                                    file.write(f"Num_run: {num_run}, Param: {fn}, Error: {error}\n")

    print(" How many expermients were run? ",)                       


def run_experiment_weights(car_type, cycle_list, solver_list, gd_profile_list, p_time_list,weights_list, if_use_gd):
    cycle_list = cycle_list
    solver_list = solver_list
    gd_profile_list = gd_profile_list
    p_time_list = p_time_list
    weights_list = weights_list
    if_use_gd = if_use_gd


    num_run = 0
    # recording
    file_name_list = []
    skipped_sim_list = []

    for cycle in cycle_list:

        if car_type =='car':
            param = Param(cycle=cycle)
        elif car_type == 'truck':
            param = TruckParam(cycle=cycle)
        else:
            raise ValueError("Got run car type, cannot initialize the param")
        
        for solver_name in solver_list:
            param.solver_type = solver_name
            if solver_name == 'QP_CA':
                param.w1, param.w2, param.w3 = weights_list[0]
            elif solver_name == 'SQP':
                param.dmax = 300
                param.dmin = 20
                param.dinit = 60
                param.w1, param.w2, param.w3 = weights_list[1]
            elif solver_name == 'NLP':
                param.w1, param.w2, param.w3 = weights_list[2]
            else:
                raise ValueError("Solver type not found!")
            # print("solver type:", param.solver_type)
            # print(" weights: ", param.w1, param.w2, param.w3)
            # assert 1==2 , "fdf"
            for gd_name in gd_profile_list:
                param.gd_profile_type = gd_name
                for p_time in p_time_list:
                    param.prediction_time = p_time
                    param.N = int(param.prediction_time / param.dt)
                    for use_gd_flag in if_use_gd:
                        param.use_gd_prediction = use_gd_flag
                        # if param.solver_type == 'QP_CA' and use_gd_flag == True:
                        #     continue
                        # ************ Inner Loop Run Here **********************
                        num_run += 1
                        file_name = f"{param.car_type}_{param.cycle}_{param.gd_profile_type}_{param.solver_type}_{param.w1}_{param.w2}_{param.w3}_gd_{param.use_gd_prediction}_ptime_{param.prediction_time}.pkl"
                        print("********** In loop #",num_run, " ****************" )
                        print("running: ", file_name)
                        print(" solver: ", param.solver_type, " use_gd_flag: ", use_gd_flag)
                        sim = Sim(param)
                        try: 

                            sim.run(if_plot=False, if_save=True)
                            file_name_list.append((num_run, file_name))
                        except Exception as e:
                            skipped_sim_list.append((num_run, file_name, str(e)))
                            print(f"Skipping simulation with param {file_name} due to error: {e}")

                        with open("data/file_name_list.txt", "w") as file:
                            for num_run, fn in file_name_list:
                                file.write(f"Num_run: {num_run}, file_name: {fn} \n")                                
                        
                        with open("data/skipped_simulations.txt", "w") as file:
                            for num_run, fn, error in skipped_sim_list:
                                file.write(f"Num_run: {num_run}, Param: {fn}, Error: {error}\n")

    print(" How many expermients were run? ",)  


