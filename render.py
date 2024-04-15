import matplotlib.pyplot as plt
import numpy as np
import pickle
from process_data import get_data, get_file_name

class Render:
    def __init__(self,car_type, cycle, slope_type,sl,tl,wl,ul, weights_fixed = True):
        # cl = ['HWFET', 'INDIA_HWY', 'NYCC', 'NYCTRUCK', 'MANHATTAN' ,'INDIA_URBAN' ]
        # sl = ['QP_CA', 'SQP', 'NLP']
        # gl = ['flat', 'normal', 'steep']
        # tl = [2, 5, 10] # prediction time, different N number
        # wl = [[0.1, 10, 10], [0.1, 5, 2], [0.1, 5, 5],[0.1, 5, 10] ]
        # ul = [True, False]
        self.wf = weights_fixed
        self.car_type = car_type
        self.cycle = cycle
        self.slope_type = slope_type
        self.sl = sl
        self.tl = tl
        self.wl = wl
        self.ul = ul

        self.Tt = []

        # Leading car's trajectory
        self.Tsl = None # s coords leading car 
        self.Tvl =  None # v profile leading car
        self.Tavl = None # apparent acceleration leading car
        self.Tatl = None # traction acceleration leading car
        self.Tarl = None
        self.Tfrl = None # fuel rate leading car ml/s
        self.Tfcl = None # fuel consumption leading car ml
        self.Tthetal = None
    
        # Ego car's trajectory
        self.l_Tse = [] # s coords ego car 
        self.l_Tve = [] # v profile ego car
        self.l_Tave = [] # apparent acceleration ego car
        self.l_Tate = [] # traction acceleration ego car
        self.l_Tare = []
        self.l_Tfre = [] # fuel rate ego car ml/s
        self.l_Tfce = [] # fuel consumption ego car ml
        self.l_Tthetae = []

        self.l_Tst = []
        self.l_Teit = []
        self.l_label  =[]

        self.total_t = None
        self.fcl =[]
        self.dl = []
        self.feffl = None
        self.aspdl = None
        self.aatl = None
        self.afrl = None

        self.l_fce = []
        self.l_de = []
        
        self.l_feff = []
        self.l_aspd = []
        self.l_aat = []
        self.l_afr = []
        self.l_improve = []
        self.l_ast = []

        if self.wf:
            for pt in tl:
                for solver_type in sl:
                    if solver_type == 'QP_CA':
                        w = self.wl[0]
                    elif solver_type == 'SQP':
                        w = self.wl[1]
                    elif solver_type == 'NLP':
                        w = self.wl[2]
                    else:
                        raise ValueError("Solver type not found!")
                    # print("solver type:", param.solver_type)
                    for uf in ul:
                        file_name = get_file_name(car_type, cycle, slope_type,solver_type,\
                                                w[0], w[1], w[2],uf, pt)
                        try:
                            recorder, param = get_data(file_name)
                            print("Success", file_name)
                            if self.Tsl == None:
                                self.Tt = recorder.Tt
                                self.Tsl = recorder.Tsl
                                self.Tvl = recorder.Tvl
                                self.Tatl = recorder.Tatl
                                self.Tfrl = recorder.Tfrl
                                self.Tfcl = recorder.Tfcl

                                self.total_t = self.Tt[-1] - self.Tt[0]
                                self.fcl = self.Tfcl[-1] - self.Tfcl[0]
                                self.dl = self.Tsl[-1] - self.Tsl[0]
                                self.feffl = 100 * self.fcl / self.dl # L/100km
                                self.aspdl = self.dl / self.total_t
                                self.aatl = sum(self.Tatl)/len(self.Tatl)
                                self.afrl = self.fcl / self.total_t
                            self.l_Tse.append(recorder.Tse)
                            self.l_Tve.append(recorder.Tve)
                            self.l_Tate.append(recorder.Tate)
                            self.l_Tfre.append(recorder.Tfre)
                            self.l_Tfce.append(recorder.Tfce)
                            self.l_Tst.append(recorder.Tst)
                            elabel = "{}_{}_{}_{}".format(solver_type, w, pt, uf)
                            self.l_label.append(elabel)
                            fce = recorder.Tfce[-1]-recorder.Tfce[0]
                            de = recorder.Tse[-1] - recorder.Tse[0]
                            feffe = 100*fce/de
                            aspde = de/self.total_t
                            aate = sum(recorder.Tate) / len(recorder.Tate)
                            afre = fce / self.total_t
                            self.l_fce.append(fce)
                            self.l_de.append(de)
                            self.l_feff.append(feffe)
                            self.l_aspd.append(aspde)
                            self.l_aat.append(aate)
                            self.l_afr.append(afre)
                            self.l_improve.append((self.feffl - feffe)*100 / self.feffl)
                            self.l_ast.append( sum(recorder.Tst) / len(recorder.Tst)         )
                        except Exception as e:
                            print("Exception for ", file_name)
        else:
            for pt in tl:
                for solver_type in sl:
                    for uf in ul:
                        for w in wl:
                            file_name = get_file_name(car_type, cycle, slope_type,solver_type,\
                                                    w[0], w[1], w[2],uf, pt)
                            try:
                                recorder, param = get_data(file_name)
                                print("Success", file_name)
                                if self.Tsl == None:
                                    self.Tt = recorder.Tt
                                    self.Tsl = recorder.Tsl
                                    self.Tvl = recorder.Tvl
                                    self.Tatl = recorder.Tatl
                                    self.Tfrl = recorder.Tfrl
                                    self.Tfcl = recorder.Tfcl

                                    self.total_t = self.Tt[-1] - self.Tt[0]
                                    self.fcl = self.Tfcl[-1] - self.Tfcl[0]
                                    self.dl = self.Tsl[-1] - self.Tsl[0]
                                    self.feffl = 100 * self.fcl / self.dl # L/100km
                                    self.aspdl = self.dl / self.total_t
                                    self.aatl = sum(self.Tatl)/len(self.Tatl)
                                    self.afrl = self.fcl / self.total_t
                                self.l_Tse.append(recorder.Tse)
                                self.l_Tve.append(recorder.Tve)
                                self.l_Tate.append(recorder.Tate)
                                self.l_Tfre.append(recorder.Tfre)
                                self.l_Tfce.append(recorder.Tfce)
                                self.l_Tst.append(recorder.Tst)
                                elabel = "{}_{}_{}_{}".format(solver_type, w, pt, uf)
                                self.l_label.append(elabel)
                                fce = recorder.Tfce[-1]-recorder.Tfce[0]
                                de = recorder.Tse[-1] - recorder.Tse[0]
                                feffe = 100*fce/de
                                aspde = de/self.total_t
                                aate = sum(recorder.Tate) / len(recorder.Tate)
                                afre = fce / self.total_t
                                self.l_fce.append(fce)
                                self.l_de.append(de)
                                self.l_feff.append(feffe)
                                self.l_aspd.append(aspde)
                                self.l_aat.append(aate)
                                self.l_afr.append(afre)
                                self.l_improve.append((self.feffl - feffe)*100 / self.feffl)
                                self.l_ast.append( sum(recorder.Tst) / len(recorder.Tst)         )




                                assert len(self.l_label) == len(self.l_Tse)
                            
                            except Exception as e:
                                print("Exception for ", file_name)
        # self.l_fce = []
        # self.l_de = []
        
        # self.l_feff = []
        # self.l_aspd = []
        # self.l_aat = []
        # self.l_afr = []
        # self.l_improve = []
    def print(self, route_head = "test_data/"):
        print_file =  route_head + f"{self.car_type}_{self.cycle}_{self.slope_type}.txt"
        with open(print_file, "w") as file:
            file.write(f" label: Leading, t:{self.total_t}, eff: {self.feffl}, d: {self.dl}, fc: {self.fcl},  ipv: {0}, aspd: {self.aspdl}, aat: {self.aatl}, afr: {self.afrl}  \n")
            for i in range(len(self.l_aat)):
                file.write(f" label:{self.l_label[i]}, eff: {self.l_feff[i]}, ipv: {self.l_improve[i]}, ast:{self.l_ast[i]}, d: {self.l_de[i]}, fc: {self.l_fce[i]},  aspd: {self.l_aspd[i]}, aat: {self.l_aat[i]}, afr: {self.l_afr[i]}  \n")



    def plot(self):
        # print("Tsl check: ", self.Tsl)
        plt.figure(figsize=(12, 8))
        nr = 3; nc = 2
        # Plotting
        plt.subplot(nr, nc, 1)
        plt.plot(self.Tt, self.Tsl, label="Leader's Position")
        # plt.plot(self.Tt, self.Tse, label="Ego's Position")
        for i in range(len(self.l_Tse)):
            print( "len l_Tse {}, len_label {} i {}".format(len(self.l_Tse), len(self.l_label), i)  )
            plt.plot(self.Tt, self.l_Tse[i], label=self.l_label[i])
        plt.ylabel('Position')
        plt.legend()

        plt.subplot(nr, nc, 2)
        plt.plot(self.Tt, self.Tvl, label="Leader's Velocity")
        # plt.plot(self.Tt, self.Tve, label="Ego's Velocity")
        for i in range(len(self.l_Tve)):
            plt.plot(self.Tt, self.l_Tve[i], label=self.l_label[i])
        plt.ylabel('Velocity')
        plt.legend()

        # plt.subplot(nr, nc, 3)
        # plt.plot(self.Tt, self.Tavl, label="Leader's Apparent Acceleration")
        # plt.plot(self.Tt, self.Tave, label="Ego's Apparent Acceleration")
        # plt.ylabel('Acceleration')
        # plt.legend()

        plt.subplot(nr, nc, 3)
        # plt.plot(self.Tt, self.Tate, label="Ego's Traction Acceleration")
        plt.plot(self.Tt, self.Tatl, label="Leading's Traction Acceleration")
        for i in range(len(self.l_Tate)):
            plt.plot(self.Tt, self.l_Tate[i], label=self.l_label[i])
        plt.ylabel('Actual Acceleration')
        plt.legend()

        plt.subplot(nr, nc, 4)
        # plt.plot(self.Tt, self.Tfre, label="Ego Fuel Rate")
        plt.plot(self.Tt, self.Tfrl, label="Leading Fuel Rate")
        for i in range(len(self.l_Tfre)):
            plt.plot(self.Tt, self.l_Tfre[i], label=self.l_label[i])
        plt.ylabel('Fuel Rate (ml/s)')
        plt.legend()

        plt.subplot(nr, nc, 5)
        # plt.plot(self.Tt, self.Tfce, label="Ego Fuel Consumption")
        plt.plot(self.Tt, self.Tfcl, label="Leading Fuel Consumption")
        for i in range(len(self.l_Tfce)):
            plt.plot(self.Tt, self.l_Tfce[i], label=self.l_label[i])
        plt.ylabel('Total Fuel Consumption (ml)')
        plt.legend()

        plt.subplot(nr, nc, 6)
        # plt.plot(self.Tt, self.Tfce, label="Ego Fuel Consumption")
        # plt.plot(self.Tt, self.Tfcl, label="Leading Fuel Consumption")
        for i in range(len(self.l_Tst)):
            plt.plot(self.Tt, self.l_Tst[i], label=self.l_label[i])
        plt.ylabel('Solving time (s)')
        plt.legend()

        # plt.subplot(nr, nc, 7)
        # plt.plot(self.Tt, self.Tare, label="Ego Resistance Acceleration")
        # plt.plot(self.Tt, self.Tarl, label="Leading Resistance Acceleration")
        # plt.ylabel('Resistance Acceleration')
        
        # plt.subplot(nr, nc, 8)
        # plt.plot(self.Tt, self.Tthetae, label="Ego Gradient")
        # plt.plot(self.Tt, self.Tthetal, label="Leading Gradient")
        # plt.ylabel('Gradient')

        plt.tight_layout()
        plt.show()   

