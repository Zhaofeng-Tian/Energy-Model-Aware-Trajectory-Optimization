import numpy as np
from sqp import SQP
from util.recorder import Recorder
from util.util import *
from param.param import Param, TruckParam
from car import Car
from qp import QP, QP_CA
from nlp import NLP
from sqp import SQP
import pickle

class Sim:
    def __init__(self, param):
        self.param= param
        self.recorder = Recorder()
        self.save_route = 'data/'
        # speed profile: 'HWFET', 'EUDC'
        self.profile_leading = get_leading_profile(self.param.cycle)
        # 'flat' 'normal' 'steep'
        self.profile_altitude, self.profile_gradient = get_gradient_profile(feature=self.param.gd_profile_type)

        # TO DO:
        # self.profile_gradient= get_gradient_profile()
        self.global_time = 0.
        self.sim_time = self.param.ts

        self.update_traj_l()
        self.init_cars()
        if self.param.solver_type == "QP":
            self.solver = QP(self.param,self.get_traj_l(),x0 = self.car_e.get_x())
        elif self.param.solver_type == "NLP":
            self.solver = NLP(self.param,self.get_traj_l(),xc = self.car_e.get_x())
        elif self.param.solver_type == "SQP":
            self.solver = SQP(self.param,self.get_traj_l(),xc = self.car_e.get_x())
        elif self.param.solver_type == "QP_CA":
            self.solver = QP_CA(self.param,self.get_traj_l(),xc = self.car_e.get_x())
        else:
            raise ValueError("Solver Type Not Found!")
        
        if self.param.use_gd_prediction:
            if self.param.solver_type == "NLP":
                self.solver.set_gd_profile(self.profile_gradient)
            elif self.param.solver_type == "SQP":
                self.solver.set_gd_profile(self.profile_gradient)

    def init_cars(self):
        self.car_l = Car(self.param)
        self.car_e = Car(self.param)
      
        # TO DO:
        # get gradient profile from s coords
        self.car_l.set_state(self.traj_sl[0],self.traj_vl[1], self.traj_al[2], theta= self.get_theta(self.traj_sl[0]), fc = 0.,)
        self.car_e.set_state(self.traj_sl[0] - self.param.dinit,self.traj_vl[1], self.traj_al[2], theta= self.get_theta(self.traj_sl[0]- self.param.dinit), fc = 0.,)

    def update_traj_l(self):
        self.traj_sl, self.traj_vl, self.traj_al = local_approx(self.profile_leading,
                                                time_target=self.sim_time, 
                                                delta_time=self.param.prediction_time,
                                                dt=self.param.dt, 
                                                acc_approx = True, 
                                                plot=False)

    def sim_step(self):
        # Solve the optimization problem given the car states
        xc = self.car_e.get_x()
        # print("check get traj sl: ", self.get_traj_sl())
        self.solver.update(xc, self.get_traj_l())
        print(" In sim_step, solver_type: ", self.param.solver_type)
        if self.param.solver_type == "QP":
            av = self.solver.solve()
            self.car_e.set_states_with_av(av)
        elif self.param.solver_type == "QP_CA":
            av = self.solver.solve()
            self.car_e.set_states_with_av(av) 
        elif self.param.solver_type == "SQP":
            at, av , ab = self.solver.solve()
            self.car_e.set_at_av_ab(at, av, ab)
        elif self.param.solver_type == "NLP":
            at, av , ab = self.solver.solve()
            self.car_e.set_at_av_ab(at, av, ab)

        print(" ******************** solve time: {} *******************" .format(self.solver.get_solve_info()['solve_time']))
        if self.param.solver_type == 'SQP':
            print("SQP iteration number: ",self.solver.get_solve_info()['executed_its']  )
        # Record
        self.recorder.record(self.car_l.get_state(), self.car_e.get_state(),self.sim_time)
        self.recorder.record_solve_info(self.solver.get_solve_info())
        print("******************************* In Sim Step **************************")
        print("car_l state: ", self.car_l.get_state())
        print("car_e state: ", self.car_e.get_state())
        # assert self.car_e.v < 27.0, "wrong speed!"
        # Time Step
        self.time_step()
        # Car Step
        self.leading_step()
        self.ego_step()


    def run(self, if_plot=False, if_save=False, route_head = "data/"):
        route_head = route_head
        for i in  range(int(10* self.param.ts), int(10*self.param.te) ):
            print("*********************** THis is step ", i, " **********************************************")
            self.sim_step()

        if if_plot:
            self.recorder.plot_trajectory()
        if if_save:
            self.save_data(route_head)

    def ego_step(self):
        self.car_e.step()
        theta_new = self.get_theta(self.car_e.get_s())
        self.car_e.set_theta(theta_new)

    def leading_step(self):
        self.car_l.step()
        self.update_traj_l()
        self.car_l.set_state(self.traj_sl[0],self.traj_vl[0], self.traj_al[0], theta= self.get_theta(self.traj_sl[0]), fc = self.car_l.fc)
        

    def time_step(self):
        self.global_time += self.param.dt
        self.sim_time += self.param.dt

    def get_traj_l(self):
        return self.traj_sl, self.traj_vl, self.traj_al

    def get_theta(self, s):
        # return self.profile_gradient[round(s)]
        return self.profile_gradient[np.int(s)]
    
    def save_data(self, route_head):
    # def save_data(recorder, additional_params, file_name='data.pkl'):
        # Combine the recorder and additional parameters into a single dictionary
        data_to_save = {
            'recorder': self.recorder,
            'param': self.param
        }
        file_name = f"{self.param.car_type}_{self.param.cycle}_{self.param.gd_profile_type}_{self.param.solver_type}_{self.param.w1}_{self.param.w2}_{self.param.w3}_gd_{self.param.use_gd_prediction}_ptime_{self.param.prediction_time}.pkl"
        # Serialize the data and save it to a file
        # file_name = self.save_route + file_name
        file_name = route_head + file_name
        with open(file_name, 'wb') as file:
            pickle.dump(data_to_save, file)
    


if __name__ == "__main__":
    param=TruckParam(cycle='HWFET')
    param.solver_type = 'SQP'
    param.gd_profile_type = 'flat'
    param.use_gd_prediction = True
    param.w1 = 0.1
    param.w2 = 5
    param.w3 = 2
    param.dmax = 200
    
    # param=Param()
    print(" In main program, solver type: ", param.solver_type)
    # assert 1==2, "check solver type"
    sim = Sim(param)
    sim.run()


