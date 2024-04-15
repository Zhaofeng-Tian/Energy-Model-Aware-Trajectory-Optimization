import pickle
from util.recorder import Recorder
from util.util import *



def get_data(file_name):
    # file_name = 'data/car_HWFET_flat_QP_CA_0.1_10_2.0_gd_False_ptime_2.pkl'
    # Deserialize the data from the file
    with open(file_name, 'rb') as file:
        data_loaded = pickle.load(file)

    # Extract the recorder and additional parameters from the loaded data
    recorder = data_loaded['recorder']
    try:
        param = data_loaded['additional_params']
    except Exception as e:
        param = data_loaded['param']
    return recorder, param

def get_file_name(car_type, cycle, gd_profile_type, solver_type, w1,w2,w3,use_gd_prediction, prediction_time, route_head = 'data/'):
    file_name = f"{car_type}_{cycle}_{gd_profile_type}_{solver_type}_{w1}_{w2}_{w3}_gd_{use_gd_prediction}_ptime_{prediction_time}.pkl"
    # print(file_name)
    return route_head + file_name



# if __name__ == "__main__":
#     # file = "data/car_HWFET_flat_SQP_0.1_5_5_gd_True_ptime_10.pkl"
#     # rder, param = get_data(file)
#     weights_test()