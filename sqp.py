import numpy as np
import osqp
from scipy import sparse
import casadi as ca
from param.param import Param
from util.util import *
import time

class SQP:
    def __init__(self, param, traj_l, xc):
        self.nx = 2 ; self.nu = 1
        self.vr = param.vr ; self.xr = param.xr
        self.xmin = param.xmin.copy() ; self.xmax = param.xmax.copy()
        self.umin = param.umin ; self.umax = param.umax
        self.vmin = param.vmin; self.vmax = param.vmax
        self.bmin = param.bmin; self.bmax = param.bmax
        self.w1 = param.w1 ;self.w2 = param.w2 ;self.w3 = param.w3
        self.T = param.T ; self.dmin = param.dmin; self.dmax = param.dmax
        self.N = param.N ; self.dt = param.dt  ;self.dinit= param.dinit
        self.max_iteration = param.max_iteration

        self.k1,self.k2,self.k3 = param.k
        self.c0,self.c1,self.c2 = param.c
        self.o0,self.o1,self.o2,self.o3,self.o4 = param.o

        self.traj_sl, self.traj_vl, self.traj_al = traj_l
        self.xc = xc; 
        self.theta = xc[6]

        self.solve_time = 0.
        self.executed_its  = 0 # number of iterations executed
        # self.SL = ca.MX.sym('SL', self.N)
        # self.VL = ca.MX.sym('VL', self.N)
        # self.p = ca.vertcat(self.SL, self.VL)
        # self.S = ca.MX.sym('S', self.N)
        # self.U = ca.MX.sym('U', self.N)  # Control input vector
        # self.V = ca.MX.sym('V', self.N)  # Velocity vector
        # self.A = ca.MX.sym('A', self.N)  # Apparent Acceleration
        # self.B = ca.MX.sym('B', self.N)

        # self.p_value = ca.vertcat(self.traj_sl[:self.N], self.traj_vl[:self.N])

        # self.J = 0.
        # for i in range(self.N):
        #     fu = self.o0 + self.o1 * self.V[i] + self.o2 * self.V[i]**2 + \
        #                 self.o3 * self.V[i]**3 +self. o4 * self.V[i]**4 + \
        #                     (self.c0 + self.c1 * self.V[i] + self.c2 * self.V[i]**2) * self.U[i]
        #     self.J += self.w1 * (self.V[i] - self.p[self.N+i])**2 + self.w2 * self.A[i]**2 + self.w3 * fu
        #     # self.J += self.w1 * (self.V[i] - self.vr)**2 + self.w2 * self.A[i]**2 
        # # print(" self.fu: ", self.fu)
        # print("self. J: ", self.J)
        # print("self.J shape: ", self.J.shape)
        # assert 1==2, " check j shape"
        """
        Now try to use casadi taylor expansion,
        to expand fuel consumption over guessed 
        states.       
        
        """
        self.SL = ca.SX.sym('SL', self.N)
        self.VL =ca.SX.sym('VL', self.N)
        self.VE = ca.SX.sym('VE', self.N)
        self.UE = ca.SX.sym("UE", self.N)
        self.sec = ca.SX.sym("sc",1)
        self.vec = ca.SX.sym("vc",1)
        self.gdec = ca.SX.sym("gdc",1) # gradient (gd) ego (e) current (c)


        self.p = ca.vertcat(self.SL, self.VL,self.VE, self.UE, self.sec, self.vec, self.gdec)
        self.S = ca.SX.sym('S', self.N)
        self.U = ca.SX.sym('U', self.N)  # Control input vector
        self.V = ca.SX.sym('V', self.N)  # Velocity vector
        self.A = ca.SX.sym('A', self.N)  # Apparent Acceleration
        self.B = ca.SX.sym('B', self.N)

        self.fu = self.o0 + self.o1 * self.V + self.o2 * self.V**2 + \
                        self.o3 * self.V**3 +self. o4 * self.V**4 + \
                            (self.c0 + self.c1 * self.V + self.c2 * self.V**2) * self.U
        
        # J_orig = self.w1*(self.V-self.VL)**2 + self.w2*self.A**2 + self.w3 * self.fu
        # J_exp = ca.mtaylor(J_orig, ca.vertcat(self.V, self.U), ca.vertcat(self.VE, self.UE), 1)
        # self.J = ca.sum1(J_exp)

        self.fu_expansion = ca.mtaylor(self.fu, ca.vertcat(self.V, self.U), ca.vertcat(self.VE, self.UE), 1)
        self.J = ca.sum1(self.w1*(self.V-self.VL)**2 + self.w2*self.A**2 + self.w3 * self.fu_expansion)
        # self.J = ca.sum1(self.w1*(self.V-self.vr)**2 + self.w2*self.A**2 + self.w3 * self.fu_expansion)


        # self.init_guess = np.concatenate((self.traj_sl[:self.N]- self.dinit,self.traj_vl[:self.N], 1.1*np.ones(self.N), 0.2*np.ones(self.N), np.zeros(self.N) ))
        self.res = None


        # assert 1==2, " assert"
        # Equation n ,n-1, n-1, 2
        self.gacc = self.SL - (self.S + self.T * self.V + self.dmin)
        self.gfa = self.A-   ( self.U -self.B- self.k1 * self.V**2 - self.k2 * ca.cos(self.gdec) - self.k3 * ca.sin(self.gdec) ) 
        # self.gfa_exp = ca.mtaylor(self.gfa, ca.vertcat(self.V, self.U, self.A, self.B), self.init_guess[self.N:],2)
        self.gfa_exp = ca.mtaylor(self.gfa, self.V, self.VE,1) # g fa expansion
        print("gfa exp: ", self.gfa_exp.shape)
        # assert 1==2, "checking gfa"
        self.gfv = [] ; self.gfd = []
        for i in range(self.N-1):
            self.gfv.append(self.V[i+1] - (self.V[i] + self.A[i] * self.dt))
        for i in range(self.N-1):
            self.gfd.append(self.S[i+1]-(self.S[i] + self.V[i]*self.dt + 0.5*self.A[i]* self.dt**2))
        self.gd0 = self.S[0] - self.sec
        self.gv0 = self.V[0] - self.vec
        self.g = ca.vertcat(self.gacc, self.gfa,*self.gfv,*self.gfd, self.gd0, self.gv0)


        self.qp = {'x': ca.vertcat(self.S, self.V,self.U,self.A, self.B), 'f': self.J, 'g': self.g, 'p':self.p}
        # self.opt = {}       
        self.opt = {'error_on_fail': False}
        
        # self.solver = ca.qpsol('S', 'qpoases', self.qp,self.opt)
        self.solver = ca.qpsol('S', 'osqp', self.qp, self.opt)
        # Initial guess and bounds
        # Lower bounds for v and u
        self.lb_s = -np.inf * np.ones(self.N)
        self.lb_v = np.zeros(self.N)  # v >= 0
        self.lb_u = self.umin * np.ones(self.N)  # u >= -1
        self.lb_a = -3.0 * np.ones(self.N)
        self.lb_b = self.bmin * np.ones(self.N)
        self.lbx = np.concatenate((self.lb_s, self.lb_v, self.lb_u, self.lb_a, self.lb_b))
        # self.lbx = ca.vertcat(self.lb_s, self.lb_v, self.lb_u, self.lb_a, self.lb_b)

        self.ub_s = np.inf * np.ones(self.N)
        self.ub_v = 27.* np.ones(self.N)  # v >= 0
        self.ub_u = self.umax * np.ones(self.N)  # u >= -1
        self.ub_a = 2.0 * np.ones(self.N)
        self.ub_b = self.bmax * np.ones(self.N)
        self.ubx = np.concatenate((self.ub_s, self.ub_v, self.ub_u, self.ub_a, self.ub_b))
        # self.ubx = ca.vertcat(self.ub_s, self.ub_v, self.ub_u, self.ub_a, self.ub_b)

        self.lbg = np.zeros(4*self.N)  # Lower bounds of g
        self.ubg = np.concatenate(( self.dmax * np.ones(self.N), np.zeros(3*self.N)))  # Upper bounds of g
    
    def solve(self):
        # res = self.solver(x0=self.init_guess, lbx=self.lbx, ubx=self.ubx,\
        #                   lbg = self.lbg, ubg = self.ubg, p = self.p_value)
        t1 = time.time()
        self.res = self.solver(lbx=self.lbx, ubx=self.ubx,\
                        lbg = self.lbg, ubg = self.ubg, p = self.p_value)
        t2 = time.time()
        st = t2 - t1
        # S, V, U, A, B
        at = np.array(self.res['x'][2*self.N:3*self.N]).flatten()[0]
        av = np.array(self.res['x'][3*self.N:4*self.N]).flatten()[0]
        ab = np.array(self.res['x'][4*self.N:5*self.N]).flatten()[0]
        # print("res: ", self.res['x'])
        # print("self res obj: ", self.res['f'])
        # print("at: ", at, " av: ", av , "ab: ", ab)
        if self.max_iteration == 1:
            self.solve_time = st
            self.executed_its = 1
            return
        for i in range(self.max_iteration-1):
            resx = self.get_res()
            self.p_value = ca.vertcat( self.traj_sl[:self.N], self.traj_vl[:self.N], resx[self.N: 2 *self.N], resx[2*self.N: 3*self.N],\
                                       self.xc[0],self.xc[1],self.xc[6] )
            t1 = time.time()
            res = self.solver(lbx=self.lbx, ubx=self.ubx,\
                            lbg = self.lbg, ubg = self.ubg, p = self.p_value) 
            t2 = time.time(); st += t2 -t1
            self.solve_time = st
            self.executed_its = 1+i+1
            # print("************* iteration{} *******************".format(i))
            # print("res: ", res['x'])
            # print("res obj: ", res['f'])
            # print("at: ", at, " av: ", av , "ab: ", ab)
            if abs(self.res['f']-res['f']) <= 1e-1:
                print(" *********** Tolerance reached*****************")
                # print(" self.res:  {},   res: {} , self.res-res = {} ".format(self.res['f'], res['f'], self.res['f']-res['f']))
                at = np.array(self.res['x'][2*self.N:3*self.N]).flatten()[0]
                av = np.array(self.res['x'][3*self.N:4*self.N]).flatten()[0]
                ab = np.array(self.res['x'][4*self.N:5*self.N]).flatten()[0]
                break;
            else:
                self.res = res

        # assert 1==2, "check obj"
        return at, av, ab

    def update(self, xc, traj_l, if_dynamic_vr = True):
        self.xc = xc
        self.traj_sl, self.traj_vl, self.traj_al = traj_l
        
        if self.res != None:
            resx = self.get_res()
            # print("resx: *****************")
            # print("resx size: ",resx.shape)
            # print(resx)
            # assert 1==2,"size"
            self.p_value = ca.vertcat( self.traj_sl[:self.N], self.traj_vl[:self.N], resx[self.N: 2 *self.N], resx[2*self.N: 3*self.N],\
                                      self.xc[0],self.xc[1],self.xc[-1]   )
        else:
            self.p_value = ca.vertcat(self.traj_sl[:self.N], self.traj_vl[:self.N], self.traj_vl[:self.N], 1.1*np.ones(self.N),\
                                      self.xc[0],self.xc[1],self.xc[-1] )
        
        # self.gfa_exp = ca.mtaylor(self.gfa, ca.vertcat(self.V, self.U, self.A, self.B), self.init_guess[self.N:],2)
        # self.gfa_exp = ca.mtaylor(self.gfa, self.V, self.init_guess[self.N: 2*self.N],2)
            
        # self.gd0 = self.S[0] - self.xc[0]
        # self.gv0 = self.V[0] - self.xc[1]
        # self.g = ca.vertcat(self.gacc, self.gfa_exp,*self.gfv,*self.gfd, self.gd0, self.gv0)

        # self.qp = {'x': ca.vertcat(self.S, self.V,self.U,self.A, self.B), 'f': self.J, 'g': self.g, 'p':self.p}
        # self.solver = ca.qpsol('S', 'osqp', self.qp)


        # self.nlp = {'x': ca.vertcat(self.S, self.V,self.U,self.A, self.B), 'f': self.J, 'g': self.g, 'p':self.p}
        # self.solver = ca.nlpsol('solver', 'ipopt', self.nlp, self.opts)     

    def get_res(self):
        return self.res['x']
    
    def get_solve_time(self):
        return self.solve_time
    

class SQP:
    def __init__(self, param, traj_l, xc):
        self.nx = 2 ; self.nu = 1
        self.vr = param.vr ; self.xr = param.xr
        self.xmin = param.xmin.copy() ; self.xmax = param.xmax.copy()
        self.umin = param.umin ; self.umax = param.umax
        self.vmin = param.vmin; self.vmax = param.vmax
        self.bmin = param.bmin; self.bmax = param.bmax
        self.w1 = param.w1 ;self.w2 = param.w2 ;self.w3 = param.w3
        self.T = param.T ; self.dmin = param.dmin; self.dmax = param.dmax
        self.N = param.N ; self.dt = param.dt  ;self.dinit= param.dinit
        self.max_iteration = param.max_iteration

        self.k1,self.k2,self.k3 = param.k
        self.c0,self.c1,self.c2 = param.c
        self.o0,self.o1,self.o2,self.o3,self.o4 = param.o

        self.traj_sl, self.traj_vl, self.traj_al = traj_l
        self.xc = xc; 
        self.theta = xc[6]

        self.solve_time = 0.
        self.executed_its  = 0 # number of iterations executed
        self.gd_profile = None
        self.use_gd_prediction = param.use_gd_prediction

        """
        Now try to use casadi taylor expansion,
        to expand fuel consumption over guessed 
        states.       
        
        """
        self.SL = ca.SX.sym('SL', self.N)
        self.VL =ca.SX.sym('VL', self.N)
        self.VE = ca.SX.sym('VE', self.N)
        self.UE = ca.SX.sym("UE", self.N)
        self.sec = ca.SX.sym("sc",1)
        self.vec = ca.SX.sym("vc",1)
        if self.use_gd_prediction:
            self.gdec = ca.SX.sym('gdec', self.N)
        else:
            self.gdec = ca.SX.sym('gdec', 1)


        self.p = ca.vertcat(self.SL, self.VL,self.VE, self.UE, self.sec, self.vec, self.gdec)
        self.S = ca.SX.sym('S', self.N)
        self.U = ca.SX.sym('U', self.N)  # Control input vector
        self.V = ca.SX.sym('V', self.N)  # Velocity vector
        self.A = ca.SX.sym('A', self.N)  # Apparent Acceleration
        self.B = ca.SX.sym('B', self.N)

        self.fu = self.o0 + self.o1 * self.V + self.o2 * self.V**2 + \
                        self.o3 * self.V**3 +self. o4 * self.V**4 + \
                            (self.c0 + self.c1 * self.V + self.c2 * self.V**2) * self.U
        
        # J_orig = self.w1*(self.V-self.VL)**2 + self.w2*self.A**2 + self.w3 * self.fu
        # J_exp = ca.mtaylor(J_orig, ca.vertcat(self.V, self.U), ca.vertcat(self.VE, self.UE), 1)
        # self.J = ca.sum1(J_exp)

        self.fu_expansion = ca.mtaylor(self.fu, ca.vertcat(self.V, self.U), ca.vertcat(self.VE, self.UE), 1)
        self.J = ca.sum1(self.w1*(self.V-self.VL)**2 + self.w2*self.A**2 + self.w3 * self.fu_expansion)
        # self.J = ca.sum1(self.w1*(self.V-self.vr)**2 + self.w2*self.A**2 + self.w3 * self.fu_expansion)


        # self.init_guess = np.concatenate((self.traj_sl[:self.N]- self.dinit,self.traj_vl[:self.N], 1.1*np.ones(self.N), 0.2*np.ones(self.N), np.zeros(self.N) ))
        self.res = np.concatenate((self.traj_sl[:self.N]-self.dinit ,self.traj_vl[:self.N], 0.1*np.ones(self.N), 0.2*np.ones(self.N), np.zeros(self.N) ))



        # assert 1==2, " assert"
        # Equation n ,n-1, n-1, 2
        self.gacc = self.SL - (self.S + self.T * self.V + self.dmin)
        self.gfa = self.A-   ( self.U -self.B- self.k1 * self.V**2 - self.k2 * ca.cos(self.gdec) - self.k3 * ca.sin(self.gdec) ) 
        # self.gfa_exp = ca.mtaylor(self.gfa, ca.vertcat(self.V, self.U, self.A, self.B), self.init_guess[self.N:],2)
        self.gfa_exp = ca.mtaylor(self.gfa, self.V, self.VE,1) # g fa expansion
        print("gfa exp: ", self.gfa_exp.shape)
        # assert 1==2, "checking gfa"
        self.gfv = [] ; self.gfd = []
        for i in range(self.N-1):
            self.gfv.append(self.V[i+1] - (self.V[i] + self.A[i] * self.dt))
        for i in range(self.N-1):
            self.gfd.append(self.S[i+1]-(self.S[i] + self.V[i]*self.dt + 0.5*self.A[i]* self.dt**2))
        self.gd0 = self.S[0] - self.sec
        self.gv0 = self.V[0] - self.vec
        self.g = ca.vertcat(self.gacc, self.gfa,*self.gfv,*self.gfd, self.gd0, self.gv0)


        self.qp = {'x': ca.vertcat(self.S, self.V,self.U,self.A, self.B), 'f': self.J, 'g': self.g, 'p':self.p}
        self.opt = {}       
        # self.opt = {'error_on_fail': False}
        
        # self.solver = ca.qpsol('S', 'qpoases', self.qp,self.opt)
        self.solver = ca.qpsol('S', 'osqp', self.qp, self.opt)
        # Initial guess and bounds
        # Lower bounds for v and u
        self.lb_s = -np.inf * np.ones(self.N)
        self.lb_v = np.zeros(self.N)  # v >= 0
        self.lb_u = self.umin * np.ones(self.N)  # u >= -1
        self.lb_a = -3.0 * np.ones(self.N)
        self.lb_b = self.bmin * np.ones(self.N)
        self.lbx = np.concatenate((self.lb_s, self.lb_v, self.lb_u, self.lb_a, self.lb_b))
        # self.lbx = ca.vertcat(self.lb_s, self.lb_v, self.lb_u, self.lb_a, self.lb_b)

        self.ub_s = np.inf * np.ones(self.N)
        self.ub_v = 27.* np.ones(self.N)  # v >= 0
        self.ub_u = self.umax * np.ones(self.N)  # u >= -1
        self.ub_a = 2.0 * np.ones(self.N)
        self.ub_b = self.bmax * np.ones(self.N)
        self.ubx = np.concatenate((self.ub_s, self.ub_v, self.ub_u, self.ub_a, self.ub_b))
        # self.ubx = ca.vertcat(self.ub_s, self.ub_v, self.ub_u, self.ub_a, self.ub_b)

        self.lbg = np.zeros(4*self.N)  # Lower bounds of g
        self.ubg = np.concatenate(( self.dmax * np.ones(self.N), np.zeros(3*self.N)))  # Upper bounds of g
    
    def solve(self):
        self.solve_time = 0
        self.executed_its = 0
        t1 = time.time()
        res = self.solver(lbx=self.lbx, ubx=self.ubx,\
                        lbg = self.lbg, ubg = self.ubg, p = self.p_value)
        t2 = time.time()
        self.res = np.array(res['x']).flatten()
        obj_value = res['f']
        t2 = time.time()
        st = t2 - t1
        # S, V, U, A, B

        at = self.res[2*self.N]
        av = self.res[3*self.N]
        ab = self.res[4*self.N]

        if self.max_iteration == 1:
            self.solve_time = st
            self.executed_its = 1
            return
        for i in range(self.max_iteration-1):
            resx = self.get_res()
            if self.use_gd_prediction:
                s_index = np.concatenate((resx[1:self.N], [resx[self.N-1]]))
                index_gd = np.round(s_index).astype(np.int)
                assert len(index_gd) == self.N
                p_gd = self.gd_profile[index_gd]
                self.p_value = ca.vertcat( self.traj_sl[:self.N], self.traj_vl[:self.N], resx[self.N: 2 *self.N], resx[2*self.N: 3*self.N],\
                                        self.xc[0],self.xc[1],p_gd )
            else:
                self.p_value = ca.vertcat( self.traj_sl[:self.N], self.traj_vl[:self.N], resx[self.N: 2 *self.N], resx[2*self.N: 3*self.N],\
                                        self.xc[0],self.xc[1],self.xc[-1] )
            t1 = time.time()
            res = self.solver(lbx=self.lbx, ubx=self.ubx,\
                            lbg = self.lbg, ubg = self.ubg, p = self.p_value) 
            t2 = time.time(); st += t2 -t1
            self.solve_time = st
            self.executed_its = 1+i+1

            if abs(obj_value-res['f']) <= 1e-2:
                print(" *********** Tolerance reached*****************")
                # print(" self.res:  {},   res: {} , self.res-res = {} ".format(self.res['f'], res['f'], self.res['f']-res['f']))
                self.res = np.array(res['x']).flatten()
                at = self.res[2*self.N]
                av = self.res[3*self.N]
                ab = self.res[4*self.N]
                break;
            else:
                self.res = np.array(res['x']).flatten()
                obj_value = res['f']

        return at, av, ab

    # def update(self, xc, traj_l, if_dynamic_vr = True):
    #     self.xc = xc
    #     self.traj_sl, self.traj_vl, self.traj_al = traj_l
        
    #     if self.res != None:
    #         resx = self.get_res()
    #         self.p_value = ca.vertcat( self.traj_sl[:self.N], self.traj_vl[:self.N], resx[self.N: 2 *self.N], resx[2*self.N: 3*self.N],\
    #                                   self.xc[0],self.xc[1],self.xc[-1]   )
    #     else:
    #         self.p_value = ca.vertcat(self.traj_sl[:self.N], self.traj_vl[:self.N], self.traj_vl[:self.N], 1.1*np.ones(self.N),\
    #                                   self.xc[0],self.xc[1],self.xc[-1] )
            
    def update(self, xc, traj_l, if_dynamic_vr = True):
        self.xc = xc
        self.traj_sl, self.traj_vl, self.traj_al = traj_l
        resx = self.get_res()
        self.init_guess = resx
        if self.use_gd_prediction:
            s_index = np.concatenate((resx[1:self.N], [resx[self.N-1]]))
            index_gd = np.round(s_index).astype(np.int)
            assert len(index_gd) == self.N
            p_gd = self.gd_profile[index_gd]
            self.p_value = ca.vertcat( self.traj_sl[:self.N], self.traj_vl[:self.N], resx[self.N: 2 *self.N], resx[2*self.N: 3*self.N],\
                                    self.xc[0],self.xc[1],p_gd )
        else:
            self.p_value = ca.vertcat( self.traj_sl[:self.N], self.traj_vl[:self.N], resx[self.N: 2 *self.N], resx[2*self.N: 3*self.N],\
                                    self.xc[0],self.xc[1],self.xc[-1] )       

    def get_res(self):
        return self.res
    
    def get_solve_time(self):
        return self.solve_time

    def get_solve_info(self):
        return {'solve_time': self.solve_time, 'executed_its': self.executed_its}

    def set_gd_profile(self,gd_profile):
        self.gd_profile = gd_profile

if __name__ == "__main__":
    prediction_time = 10; dt = 0.1;  dmin=10; dinit = 100; ts = 50; te = 730
    profile = get_leading_profile()
    traj_sl, traj_vl, traj_al = local_approx(profile,time_target=ts, delta_time=prediction_time,dt=0.1, acc_approx = True, plot=False)
    x0 = np.array([traj_sl[0] - dinit, traj_vl[0]])
    param = Param()
    solver = SQP(param,(traj_sl, traj_vl, traj_al), x0)
