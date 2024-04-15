import numpy as np
import osqp
from scipy import sparse
import casadi as ca
import time

class QP:
    def __init__(self, param, traj_l, x0):
        self.nx = 2
        self.nu = 1
        self.traj_sl, self.traj_vl, self.traj_al = traj_l
        self.x0 = x0
        self.vr = param.vr
        self.xr = param.xr
        self.xmin = param.xmin.copy()
        self.xmax = param.xmax.copy()
        self.umin = param.umin
        self.umax = param.umax
        self.w1 = param.w1 
        self.w2 = param.w2
        self.T = param.T
        self.dmin = param.dmin; self.dmax =  param.dmax
        self.N = param.N
        self.dt = param.dt  

        self.Ad = sparse.csc_matrix([[1, self.dt],
                            [0, 1]])
        self.Bd = sparse.csc_matrix([[0.5 * self.dt**2 ],
                            [   self.dt     ]])

        self.Q = sparse.diags([0., self.w1]) 
        self.QN = sparse.diags([0., self.w1])  
        self.R = self.w2* sparse.eye(1) 
        self.P = sparse.block_diag([sparse.kron(sparse.eye(self.N), self.Q), self.QN,
                            sparse.kron(sparse.eye(self.N), self.R)], format='csc')
        self.q = np.hstack([np.kron(np.ones(self.N), -self.Q@self.xr), -self.QN@self.xr, np.zeros(self.N*self.nu)])

        self.Ax = sparse.kron(sparse.eye(self.N+1),-sparse.eye(self.nx)) + sparse.kron(sparse.eye(self.N+1, k=-1), self.Ad)
        self.Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, self.N)), sparse.eye(self.N)]), self.Bd)
        self.Aeq = sparse.hstack([self.Ax, self.Bu])
        self.leq = np.hstack([-self.x0, np.zeros(self.N*self.nx)])
        self.ueq = self.leq

        # - input and state constraints
        self.Aineq = sparse.eye((self.N+1)*self.nx + self.N*self.nu)
        self.lineq = np.hstack([np.kron(np.ones(self.N+1), self.xmin), np.kron(np.ones(self.N), self.umin)])
        self.uineq = np.hstack([np.kron(np.ones(self.N+1), self.xmax), np.kron(np.ones(self.N), self.umax)])

        self.Aa = sparse.csc_matrix([1,self.T])
        self.ua = self.traj_sl[0:self.N+1]-self.dmin
        # self.la = np.zeros(self.N+1)
        self.la = self.traj_sl[0:self.N+1]-self.dmax

        self.Aacc = sparse.hstack([sparse.kron(sparse.eye(self.N+1), self.Aa), np.zeros((self.N+1, self.N*self.nu))])
                        
        self.A = sparse.vstack([self.Aeq, self.Aineq, self.Aacc], format='csc')
        self.l = np.hstack([self.leq, self.lineq, self.la])
        self.u = np.hstack([self.ueq, self.uineq, self.ua])

        self.prob = osqp.OSQP()
        self.prob.setup(self.P, self.q, self.A, self.l, self.u, verbose=True)

        self.solve_time = 0.
        # self.res = []

    def solve(self):
        t1 = time.time()
        res = self.prob.solve()
        t2 = time.time()
        self.solve_time = t2 - t1
        av = res.x[-self.N*self.nu:-(self.N-1)*self.nu].copy()[0]
        return av
    
    def update(self, x0, traj_l, if_dynamic_vr = True):
        self.traj_sl, self.traj_vl, self.traj_al = traj_l
        
        self.l[:self.nx] = -x0
        self.u[:self.nx] = -x0
        self.u[-len(self.ua):] = self.traj_sl[:self.N+1]-self.dmin
        
        if if_dynamic_vr:
            self.xr = np.array([0.,self.traj_vl[0]])
            self.q = np.hstack([np.kron(np.ones(self.N), -self.Q@self.xr), -self.QN@self.xr, np.zeros(self.N*self.nu)])
            print("len self.q: ", len(self.q))
            xr_array = np.array([-self.Q@np.array([0, vr]) for vr in self.traj_vl[0:self.N]]).flatten()
            print("len of xr_array: ", len(xr_array))
            assert len(xr_array) == 2*self.N, " length should equal to 2 * N"
            self.q =np.hstack([xr_array, -self.Q @ np.array([0. ,self.traj_vl[-1]]), np.zeros(self.N*self.nu)])
            # print(self.q)
            # print(xr_array)
            # assert 1==2, "stop to view! "
            self.prob.update(q=self.q, l=self.l, u=self.u)
        else:
            self.prob.update(l=self.l, u=self.u)
        
    def get_solve_info(self):
        return {'solve_time': self.solve_time, 'executed_its': 1}



class QP_CA:
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
        self.theta = xc[-1]

        self.solve_time = 0.
        self.executed_its  = 0 # number of iterations executed

        """
        Now try to use casadi taylor expansion,
        to expand fuel consumption over guessed 
        states.       
        
        """
        self.SL = ca.SX.sym('SL', self.N)
        self.VL =ca.SX.sym('VL', self.N)
        self.sec = ca.SX.sym("sc",1)
        self.vec = ca.SX.sym("vc",1)

        self.p = ca.vertcat(self.SL, self.VL, self.sec, self.vec)
        self.S = ca.SX.sym('S', self.N)
        self.V = ca.SX.sym('V', self.N)  # Velocity vector
        self.A = ca.SX.sym('A', self.N)  # Apparent Acceleration


        self.J = ca.sum1(self.w1* (self.V-self.VL)**2 + self.w2 * self.A**2 )

        # self.init_guess = np.concatenate((self.traj_sl[:self.N]- self.dinit,self.traj_vl[:self.N], 1.1*np.ones(self.N), 0.2*np.ones(self.N), np.zeros(self.N) ))
        self.res = None

        self.gacc = self.SL - (self.S + self.T * self.V + self.dmin)
        self.gfv = [] ; self.gfd = []
        for i in range(self.N-1):
            self.gfv.append(self.V[i+1] - (self.V[i] + self.A[i] * self.dt))
        for i in range(self.N-1):
            self.gfd.append(self.S[i+1]-(self.S[i] + self.V[i]*self.dt + 0.5*self.A[i]* self.dt**2))
        self.gd0 = self.S[0] - self.sec
        self.gv0 = self.V[0] - self.vec
        self.g = ca.vertcat(self.gacc,*self.gfv,*self.gfd, self.gd0, self.gv0)


        self.qp = {'x': ca.vertcat(self.S, self.V,self.A), 'f': self.J, 'g': self.g, 'p':self.p}
        # self.opt = {}       
        self.opt = {'error_on_fail': False}

        self.solver = ca.qpsol('S', 'osqp', self.qp, self.opt)
        # Initial guess and bounds
        # Lower bounds for v and u
        self.lb_s = -np.inf * np.ones(self.N)
        self.lb_v = np.zeros(self.N)  # v >= 0
        self.lb_a = -3.0 * np.ones(self.N)
        self.lbx = np.concatenate((self.lb_s, self.lb_v, self.lb_a))
        # self.lbx = ca.vertcat(self.lb_s, self.lb_v, self.lb_u, self.lb_a, self.lb_b)

        self.ub_s = np.inf * np.ones(self.N)
        self.ub_v = 27.* np.ones(self.N)  # v >= 0
        self.ub_a = 2.0 * np.ones(self.N)
        self.ubx = np.concatenate((self.ub_s, self.ub_v, self.ub_a))
        # self.ubx = ca.vertcat(self.ub_s, self.ub_v, self.ub_u, self.ub_a, self.ub_b)

        self.lbg = np.zeros(3*self.N)  # Lower bounds of g
        self.ubg = np.concatenate(( self.dmax * np.ones(self.N), np.zeros(2*self.N)))  # Upper bounds of g
    
    def solve(self):
        # res = self.solver(x0=self.init_guess, lbx=self.lbx, ubx=self.ubx,\
        #                   lbg = self.lbg, ubg = self.ubg, p = self.p_value)
        t1 = time.time()
        self.res = self.solver(lbx=self.lbx, ubx=self.ubx,\
                        lbg = self.lbg, ubg = self.ubg, p = self.p_value)
        t2 = time.time()
        self.solve_time = t2-t1
        # S, V, U, A, B
        av = np.array(self.res['x'][2*self.N:3*self.N]).flatten()[0]
        # assert 1==2, " check main loop try and exemption"
        return  av

    def update(self, xc, traj_l, if_dynamic_vr = True):
        self.xc = xc
        self.traj_sl, self.traj_vl, self.traj_al = traj_l
        self.p_value = ca.vertcat( self.traj_sl[:self.N], self.traj_vl[:self.N], \
                                      self.xc[0],self.xc[1] )

    def get_res(self):
        return self.res['x']
    
    def get_solve_info(self):
        return {'solve_time': self.solve_time, 'executed_its': 1}


