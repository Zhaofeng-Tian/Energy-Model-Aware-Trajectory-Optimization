from render import Render
from experiments import run_experiment, run_experiment_weights

def template_test():
    cl = ['HWFET', 'INDIA_HWY', 'NYCC', 'NYCTRUCK', 'MANHATTAN' ,'INDIA_URBAN' ]
    sl = ['QP_CA', 'SQP', 'NLP']
    gl = ['flat', 'normal', 'steep']
    tl = [2, 5, 10] # prediction time, different N number
    wl = [[0.1, 10, 10], [0.1, 5, 2], [0.1, 5, 5],[0.1, 5, 10] ]
    ul = [True, False]
    # car_type = 'car'; cycle = "HWFET" ; slope_type = 'flat'
    car_type = 'truck'; cycle = "HWFET" ; slope_type = 'steep'
    render = Render(car_type, cycle,slope_type,sl,tl,wl,ul)
    # render.plot()
    render.print()

def weights_test():
    """
    QP_CA best weights [0.1, 5]
    SQP [0.1,5,2]
    NLP [0.1, 10, 10]
    """
    car_type = 'truck'
    cl = ['HWFET', 'INDIA_HWY', 'NYCC', 'NYCTRUCK', 'MANHATTAN' ,'INDIA_URBAN' ]
    sl = ['QP_CA', 'SQP', 'NLP']
    gl = ['flat', 'normal', 'steep']
    tl = [2, 5, 10] # prediction time, different N number
    wl = [[0.1, 10, 10], [0.1, 5, 2], [0.1, 5, 5],[0.1, 5, 10] ]
    ul = [True, False]

    cl = ['HWFET', 'INDIA_HWY']
    sl = ['SQP']
    gl = ['flat', 'steep']
    tl = [5, 10]
    # wl = [[0.1, 10, 10], [0.1, 5, 2], [0.1, 5 , 20]]
    wl = [[0.1, 1, 1], [0.1, 1, 5], [0.1, 10 , 100], [0.1, 10, 0], [0.1, 0.1, 1], [0.1, 3, 10]]
    wl = [[0.1,1,1],[0.1,5,2]]
    ul = [True]

    run_experiment(car_type, cl, sl, gl, tl, wl, ul)

    for cycle in cl:
        for slope_type in gl:
            render = Render(car_type, cycle,slope_type,sl,tl,wl,ul, weights_fixed=False)
            render.print(route_head="test_data/weights3/")
            render.plot()
            
def gd_test():

    car_type = 'truck'
    cl = ['HWFET', 'INDIA_HWY', 'NYCC']
    sl = ['SQP', 'NLP']
    gl = ['normal', 'steep']
    tl = [5]
    wl = [[0.1, 8, 2], [0.1, 5, 2], [0.1, 10 ,10]]
    ul = [True, False]

    run_experiment_weights(car_type, cl, sl, gl, tl, wl, ul)

    for cycle in cl:
        for slope_type in gl:
            render = Render(car_type, cycle,slope_type,sl,tl,wl,ul,weights_fixed = True)
            render.print(route_head="test_data/gd_test/")
            render.plot()

def cycle_test():

    car_type = 'car'
    # cl = ['HWFET', 'INDIA_HWY', 'NYCC']
    # sl = ['SQP', 'NLP']
    # gl = ['normal', 'steep']
    # tl = [5]
    # wl = [[0.1, 5, 2], [0.1, 5, 2], [0.1, 10 ,10]]
    # ul = [True, False]

    cl = ['HWFET', 'INDIA_HWY', 'NYCC', 'NYCTRUCK', 'MANHATTAN' ,'INDIA_URBAN' ]
    sl = ['QP_CA', 'SQP', 'NLP']
    gl = ['flat', 'normal', 'steep']
    tl = [5] # prediction time, different N number
    wl = [[0.01, 0.3, 1], [0.01, 1, 1], [0.01, 1 ,1]]
    ul = [True]
    run_experiment_weights(car_type, cl, sl, gl, tl, wl, ul)

    for cycle in cl:
        for slope_type in gl:
            render = Render(car_type, cycle,slope_type,sl,tl,wl,ul,weights_fixed = True)
            render.print(route_head="test_data/cycle_test/")
            render.plot()

if __name__ == "__main__":
    # file = "data/car_HWFET_flat_SQP_0.1_5_5_gd_True_ptime_10.pkl"
    # rder, param = get_data(file)
    # weights_test()
    cycle_test()
