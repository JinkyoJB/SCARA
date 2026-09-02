import numpy as np
from dataclasses import dataclass, field
from copy import deepcopy

TG_MODE_TRAPEZOIDAL = "TG_MODE_TRAPEZOIDAL"
TG_MODE_POLY5TRAJ = "TG_MODE_POLY5TRAJ"


'''
/////////////////////////////////////////////////////////////////////////////
#                            Path Planner
/////////////////////////////////////////////////////////////////////////////
'''
@dataclass
class Poly5Coeff:
    a0:float = 0.0
    a1:float = 0.0
    a2:float = 0.0
    a3:float = 0.0
    a4:float = 0.0
    a5:float = 0.0

class P2P_TrapeZoidal:

    def __init__(self, s0, s1, ds_max, dds_max, control_freq, xd_arm:list=None):

        self.control_freq = control_freq
        self.dt = 1/control_freq
        self.s0 = s0
        self.s1 = s1
        self.ds_max = ds_max
        self.dds_max = dds_max

        self.t_acc = 0.0
        self.t_dec = 0.0
        self.t_fin = 0.0

        self.t = 0.0
        self.step:int = 0
        
        self.EPS_PRECISION = 4
        self.EPS_TIME = 10**(-self.EPS_PRECISION)

        self.num_samples = 0

        self.S = []
        self.SD = []
        self.SDD = []

        self.PathPlanner()

        self.cur_pos = deepcopy(self.s0)
        self.cur_vel = np.zeros_like(self.s0)
        self.cur_acc = np.zeros_like(self.s0)
        
        self.xd_arm = xd_arm
        
    def PathPlanner(self):

        ret = self.GenTrapeProfileSyncAxis()

        self.num_samples = int(np.round(self.t_fin*self.control_freq))
        self.t = 0.0
        self.sstp = 0

    def GenTrapeProfileSyncAxis(self):
        """
        Generates equalized trapezoidal profiles for several joints

        Args:
            q0 (list of float): List of initial positions of all joints
            q1 (list of float): List of desired positions of all joints
            n (int): Number of points to sample

        Returns:
            np.ndarray, np.ndarray, np.ndarray, float, float:
                qs - Joint positions
                dqs - Joint velocities
                ddqs - Joint accelerations
                ts - Corresponding times
        """
        # variables
        s0=self.s0; s1=self.s1; ds_max=self.ds_max; dds_max=self.dds_max;control_freq=self.control_freq

        # Pregenerate coefficients
        tacc_list, tdec_list = [], []

        n = len(s0)
        for i in range(n):
            _tacc, _tdec = self.GenTrapeProfile(s0[i], s1[i], ds_max[i], dds_max[i], control_freq)
            tacc_list.append(_tacc)
            tdec_list.append(_tdec)

        # Equalize profiles
        tacc_max = max(tacc_list)
        tdec_max = max(tdec_list)

        # get time
        self.t_acc = tacc_max # t_0 ~ t_acc
        self.t_dec = tdec_max     # ~ t_dec
        self.t_fin = tdec_max+tacc_max


        # print("t_acc : ", self.t_acc)
        # print("t_dec : ", self.t_dec)
        # print("t_fin : ", self.t_fin)
        
        return True

    @staticmethod
    def GenTrapeProfile(s0, s1, vmax, amax, control_freq=0):
        """
        Generates trapezoidal profile

        Args:
            s0 (float): Current position
            s1 (float): Desired position
            vmax (float): Maximum velocity
            amax (float): Maximum acceleration
            control_freq (int, optional): Frequency of a controller. If it is 0
                will not take the controller frequency into account

        Returns:
            float, float:
                t_1 - acceleration stop time
                tau - deceleration start time

                    /----------\
                / |        | \
                /  |        |  \
                    t_1       tau
        """
        dtheta = abs(s1 - s0)
        vp = np.sqrt(dtheta * amax)
        """
        (Derivation)
        If Triangular profile.
        area of triangular is.. 
            dtheta = delta_t * vmax / 2  ...(1)
        The acc is slope of triangular
            amax = vmax / (1/2 delta_t)
            delta_t = vmax / (1/2 amax) ... (2)
        substitute (2) to (1)
            dtheta = vmax / (1/2 amax) * vmax / 2 = vmax^2 / amax
            vmax = sqrt(dtheta x amax)
        """

        # Triangular profile
        if vp < vmax:
            t_1 = vp / amax
            tau = t_1
        # Trapezoidal profile
        else:
            t_1 = vmax / amax
            tau = dtheta / vmax
        # Consider controller frequency
        if control_freq != 0:
            n = np.ceil(t_1 * control_freq)
            m = np.ceil(tau * control_freq)

            t_1 = n / control_freq
            tau = m / control_freq

        return t_1, tau

    def isDone(self):
        return self.t > self.t_fin - self.EPS_TIME

    def get_state_traj(self):

        if self.isDone():
            # print("[ERR] exceed")
            return False
        
        _t = self.step*self.dt
        _t = np.round(_t, self.EPS_PRECISION)

        self.t = _t

        delta_s = self.s1 - self.s0
        ds_cur = delta_s / self.t_dec # (deriv) t_1*dq_cur + (tau-t_1)*dq_cur = delta_s
        dds_cur = ds_cur / self.t_acc
        
        s0 = self.s0
        t = self.t

        if t < self.t_acc: # 가속    
            s = s0 + 1/2 * dds_cur * t**2
            sdd = dds_cur
            sd = dds_cur * t
        elif t >= self.t_acc and t < self.t_dec: # 등속 (안들어갈 수도 있다)
            s = s0 + 1/2 * dds_cur * self.t_acc**2 + ds_cur * (t-self.t_acc)
            sdd = np.zeros_like(s)
            sd = ds_cur
        elif t <= self.t_fin+self.EPS_TIME:
            s = s0\
                + 1/2 * dds_cur * self.t_acc**2 \
                + ds_cur * (self.t_dec-self.t_acc) \
                + (self.t_fin-self.t_dec)*ds_cur/2 - 1/2*dds_cur*(self.t_fin-t)**2
            sdd = -dds_cur
            sd = ds_cur - dds_cur * (t-self.t_dec)
        else:
            raise Exception("t is over tf")

        self.cur_pos = np.array(s)
        self.cur_vel = np.array(sd)
        self.cur_acc = np.array(sdd)

        self.step += 1

        return True

    def GenSamples(self):

        S = []
        SD= []
        SDD = []
        
        while(1):
            
            ret = self.get_state_traj()
            S.append(self.cur_pos)
            SD.append(self.cur_vel)
            SDD.append(self.cur_acc)

            if not ret:
                print("done")
                break

        self.S = np.array(S)
        self.SD = np.array(SD)
        self.SDD = np.array(SDD)

        return self.S, self.SD, self.SDD

    def reset(self):
        self.t = 0.0
        self.step = 0

class P2P_Poly5:

    def __init__(self, s0, s1, v0, v1, a0, a1, t0, t1=None, ds_max=None, control_freq=1000, xd_arm:list=None):

        self.control_freq = control_freq
        self.dt = 1/control_freq

        self.t0 = t0;        self.t1 = t1
        self.s0 = s0;        self.s1 = s1
        self.v0 = v0;        self.v1 = v1
        self.a0 = a0;        self.a1 = a1

        self.ds_max = ds_max

        if t1 is None:
            self.t1 = self.get_shortest_time(s0, s1, ds_max)
        
        self.t_fin = self.t1

        self.t = 0.0
        self.step:int = 0
        
        self.EPS_PRECISION = 4
        self.EPS_TIME = 10**(-self.EPS_PRECISION)


        self.num_samples = 0

        self.S = []
        self.SD = []
        self.SDD = []

        self.U = []

        self.PathPlanner()

        self.cur_pos = deepcopy(self.s0)
        self.cur_vel = deepcopy(self.v0)
        self.cur_acc = deepcopy(self.a0)
        
        self.xd_arm=xd_arm
        
    @staticmethod
    def Poly5Coefficient(t0, q0, v0, a0, t1, q1, v1, a1):
        TINY = 1e-6
        
        T = t1-t0
        h = q1 - q0
        
        u = Poly5Coeff()

        if T < TINY:
            u.a0 = q0
            u.a1 = 0.0
            u.a2 = 0.0
            u.a3 = 0.0
            u.a4 = 0.0
            u.a5 = 0.0
            return u
        
        T2 = T**2
        T3 = T**3
        T4 = T**4
        T5 = T**5
        
        
        u.a0 = q0;
        u.a1 = v0;
        u.a2 = 0.5*a0;
        u.a3 = 1/(2*T3)*(20*h-(8*v1+12*v0)*T-(3*a0-a1)*T2);
        u.a4 = 1/(2*T4)*(-30*h+(14*v1+16*v0)*T+(3*a0-2*a1)*T2);
        u.a5 = 1/(2*T5)*(12*h-6*(v1+v0)*T+(a1-a0)*T2);
        
        return u

    @staticmethod
    def _get_state_traj(t:float, t0:float, u:Poly5Coeff):

        T = t - t0
        T2 = T**2
        T3 = T**3
        T4 = T**4
        T5 = T**5
        
        a0, a1, a2, a3, a4, a5 = u.a0, u.a1, u.a2, u.a3, u.a4, u.a5
        
        th = a0 + a1*T + a2*T2 + a3*T3 + a4*T4 + a5*T5
        th_d = a1 + 2*a2*T + 3*a3*T2 + 4*a4*T3 + 5*a5*T4
        th_dd = 2*a2 + 6*a3*T + 12*a4*T2 + 20*a5*T3
        
        return th, th_d, th_dd
    
    @staticmethod
    def get_shortest_time(s0, s1, ds_max):
        t_list=[]
        for i in range(len(s0)):
            t_list.append(15*(abs(s1[i]-s0[i]))/(8*ds_max[i]))
        t1=max(t_list)
        return t1

    def PathPlanner(self):
        for i in range(len(self.s0)):
            self.U.append(self.Poly5Coefficient(self.t0, self.s0[i], self.v0[i], self.a0[i],
                                           self.t1, self.s1[i], self.v1[i], self.a1[i])  )

        self.num_samples = int(np.round(self.t1*self.control_freq))
        self.t = self.t0
        self.step = 0

    def isDone(self):
        return self.t > self.t_fin - self.EPS_TIME
    
    def get_state_traj(self):

        if self.isDone():
            # print("[ERR] exceed")
            return False
        
        _t = self.step*self.dt
        _t = np.round(_t, self.EPS_PRECISION)

        self.t = _t

        pos=[]; vel=[]; acc=[]
        for i in range(len(self.U)):
            _pos, _vel, _acc = self._get_state_traj(self.t, self.t0, self.U[i])
            pos.append(_pos); vel.append(_vel); acc.append(_acc)


        self.cur_pos = np.array(pos)
        self.cur_vel = np.array(vel)
        self.cur_acc = np.array(acc)

        self.step += 1

        return True

    
    def GenSamples(self):

        S = []
        SD= []
        SDD = []
        
        while(1):
            
            ret = self.get_state_traj()
            S.append(self.cur_pos)
            SD.append(self.cur_vel)
            SDD.append(self.cur_acc)

            if not ret:
                print("done")
                break

        self.S = np.array(S)
        self.SD = np.array(SD)
        self.SDD = np.array(SDD)

        return self.S, self.SD, self.SDD
        
    def reset(self):
        self.t = 0.0
        self.step = 0

def GenTrajCoeff_conv(s_start, s_goal, Mv, At, vel_rate=100, Tg_step2sec = 0.02, tg_mode = TG_MODE_POLY5TRAJ, coord = "JOINT", xd_arm:list=None):

    # -- code -- #
    ds_max = deepcopy(Mv)
    dds_max = deepcopy(Mv/At)

    ds_max *= vel_rate * 0.01
    s_init = deepcopy(s_start)
    control_freq = int(1/Tg_step2sec)

    dof = len(s_start)

    # -- TG -- #
    if tg_mode == TG_MODE_TRAPEZOIDAL:
        Ux = P2P_TrapeZoidal(s_init, s_goal, ds_max, dds_max, control_freq)
    else:
        t0 = 0;                                          t1 = None  # t1 is obtained as shortest time considering ds_max
        s0 = s_init;                                     s1 = s_goal
        

        v0 = np.zeros(dof);   v1 = np.zeros(dof)
        a0 = np.zeros(dof);   a1 = np.zeros(dof)

        Ux = P2P_Poly5(s0, s1, v0, v1, a0, a1, t0, t1=t1,ds_max=ds_max, control_freq=control_freq, xd_arm=xd_arm)
    return Ux