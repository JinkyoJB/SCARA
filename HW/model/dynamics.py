
import numpy as np
from robot import ToolClass, LinkClass
from copy import deepcopy
from define import DEFAULT_GRAV_ACC_VEC, METER2MM, MM2METER, RAD2DEG, DEG2RAD
from dataclasses import dataclass, field
import pandas as pd
from pandas.core.frame import DataFrame
from define import RAD2DEG, DEG2RAD, DPS2RPM, RPM2DPS, MM2METER, METER2MM

def rne2(links:LinkClass, q, qd, qdd,
        tool:ToolClass = None,
        fext = np.zeros(6),
        grav = DEFAULT_GRAV_ACC_VEC):
    """ Recursive Newton-Euler Inward Dynamics
    The tool is considered as 7-th link
    The return value is reaction force

    TODO:
    - DH 외에 다른 모델
    - tool의 tensor는 이 전 링크의 frame (즉, end-effector frame)에 대해 표현되야함
    """
    # ===============================================
    #               PARAMETER SETTING
    # ===============================================
    react_grav = -grav
    # LinkChain_ = deepcopy(LinkChain)
    # LinkChain_[:,0] += q
    z0 = np.array([0., 0., 1.0])

    dof = len(links)
    m = dof

    q_ = q
    qd_ = qd
    qdd_ = qdd

    
    T_list = []
    p_inv_list = []
    mass_list = []
    COM_list = []
    tensor_list = []
    
    def skew3x3(a: np.ndarray or list):
            """ return a 3x3 skew-symmetric matrix corresponding to 3x1 input vector a
            The order should be x-y-z
            """
            x, y, z = a
            return np.array([[0, -z,  y],
                            [z,  0, -x],
                            [-y,  x,  0]])
    
    for i in range(dof):
        T_list.append(links[i].kine.tform(q[i]))
        p_inv_list.append(links[i].kine.invPos * MM2METER) # i-1 coord to i coord represented in i coord
        mass_list.append(links[i].massProp.mass)
        COM_list.append(links[i].massProp.COM * MM2METER)   # for unit consistency
        tensor_list.append(links[i].massProp.tensor) # kg.m^2
        
    if tool is not None:
        T_list.append(np.eye(4)) #Tool coord를 6번 coord와 일치시켯기 때문에 이렇게 함
        T_temp = T_list[-1]
        p_inv_list.append(np.zeros(3))
        mass_list.append(tool.massProp.mass)
        COM_list.append(tool.massProp.COM * MM2METER)  # for unit consistency
        tensor_list.append(tool.massProp.tensor)

        q_ = np.hstack((q, 0))
        qd_ = np.hstack((qd, 0))
        qdd_ = np.hstack((qdd, 0))
        m = dof+1
    # ===============================================
    #               FORWARD PROPAGATION
    # ===============================================
    '''
    Forward propagatoin to calculate velocity and acceleration
    we don't need to calculate linear velocity of link i
    becuase it's not used actually
    we calculate angular velocity at each link coordinate
    and linear and angular accerelation at each link coordinate
    and linear accerelation at COM
    '''
    # ---------------------
    # Initial setting
    # ---------------------
    w_pre = np.zeros(3);     vd_pre = react_grav;     wd_pre = np.zeros(3)
    W = [];     Wd = [];     Vd = [];     Vdc = []
    F = [];     N = []

    for i in range(m):  # 0, 1, 2, 3, 4, 5 (6개), if tool_flag == True then,  until 6 (7개)
        
        T = T_list[i]
        R = T[0:3,0:3]
        RT = R.T # i to i-1
        p_inv = p_inv_list[i]

        
        mass = mass_list[i]
        pc = COM_list[i]
        tensor = tensor_list[i]

        '''
        z0 : i-1 th z vector wrt i-1-th coord so it's just [0, 0, 1]
        RT does transform z vector to i th coord  (i-1)z(i-1) -->   (i)z(i-1)
        '''
        # ---------------------
        # Angular acceleration
        # ---------------------
        
        '''
        angular acc at each link coord wrt each link coord  (i)wd(i)
        wd = (i)wd(i)
        wd_pre = (i-1)w(i-1)
        '''
        wd = RT @ (wd_pre + qdd_[i]*z0 + qd_[i]* skew3x3(w_pre) @ z0) # i_wd_i
        # wd = RT @ (wd_pre + qdd[i]*z0 + qd[i]* np.cross(w_pre, z0)) # i_wd_i
        # wd = RT @ wd_pre + qdd[i] * RT @ z + RT @ np.cross(w_pre, qd[i] * z)
        
        # ---------------------
        # Angular Velocity
        # ---------------------
        '''
        angular velocity at each link wrt each coord
        w = (i)w(i)
        w_pre = (i-1)w(i-1)
        '''
        w = RT @ (w_pre + qd_[i]*z0)
        # ---------------------
        # Linear acceleration
        # ---------------------
        '''
        linear acc at each link coord wrt each link coord
        it's better to add gravitational term to this
        but not nessessary. you can add it to other equation
        vd = (i)vd(i)
        '''
        vd = RT @ vd_pre + np.cross(wd, p_inv) + skew3x3(w) @ skew3x3(w) @ p_inv
        # vd = RT @ vd_pre + np.cross(wd, p) + np.cross(w, np.cross(w, p))
        
        # ---------------------
        # Linear acceleration at COM
        # ---------------------
        '''
        # linear acc at COM wrt each link coord
        # pc = (i)pc(i)
        # vdc = (i)vdc(i)
        '''
        
        '''
        i번째 coord에서 구한 i번째 link의 COM; 좌표 변환 할 필요 없음. i_pc_i
        reference cooridnate를 여기서는 TCP지점으로 잡았다.
        그러나 다른 좌표계에서 쓰이는 경우를 생각해야 함. 그 경우 변환해줘야 함
        TOOL의 COM및 TENSOR에 해당
        '''
        vdc = vd + np.cross(wd, pc) + skew3x3(w) @ skew3x3(w) @ pc
        # vdc = vd + np.cross(wd, pc) + np.cross(w, np.cross(w, pc))
        
        # ---------------------
        # Force and Moment
        # ---------------------
        
        F.append(mass * vdc)
        N.append(tensor @ wd + skew3x3(w) @  tensor @ w)

        # ---------------------
        # append and for loop
        # ---------------------
        W.append(w); Wd.append(wd); Vd.append(vd); Vdc.append(vdc)
        wd_pre = wd; w_pre = w; vd_pre = vd;


    # ===============================================
    #               BACKWARD PROPAGATION
    # ===============================================
    '''
    Inward propagataion to calculate force and moment
    외력 없을 경우 0으로 초기화
    '''
    # ---------------------
    # INITIAL SETTING
    # ---------------------
    f_nxt = fext[0:3] # extern form, # loop 상 이전 링크의 force
    n_nxt = fext[3:6] # extern moment, # loop 상 이전 링크의 moment

    ff = [] # force
    nn = [] # moment
    tt = [] # torque
    for i in range(m-1, -1, -1): # with tool : i = 6 5 4 3 2 1 0 (7개), # without tool : i = 5, 4, 3, 2, 1, 0 (6개)
        if i == m-1:
            T = np.eye(4)
        else:
            T = T_list[i+1]
        
        R = T[0:3,0:3]
        
        # ---------------------
        # p = (i)p(i-1 -> i)
        # ---------------------
        p_inv = p_inv_list[i]
    
        # ---------------------
        # pc = (i)p(i->ci)
        # ---------------------
        pc = COM_list[i] # 6 5 4 3 2 1 0
        
        # -----------------------------
        # force and moment and torque
        # -----------------------------
        f = R @ f_nxt + F[i]
        n = -skew3x3(f) @ p_inv - skew3x3(F[i]) @ pc + R @ n_nxt + N[i]
        # n = -np.cross(f, p) - np.cross(F[i], pc) + R @ n_nxt + N[i]

        # Torque Calculation
        Rt = T_list[i][0:3,0:3]

        n_out = Rt @ n
        f_out = Rt @ f
        t = n_out[2]
        
        # append and for loop
        tt.append(t); ff.append(f_out); nn.append(n_out)
        f_nxt = f; n_nxt = n


    tt.reverse(); ff.reverse(); nn.reverse()
    tt = np.array(tt); ff = np.array(ff); nn = np.array(nn)

    return tt, ff, nn

""" ///////////////////////////////////////////////////////////////////////////
                            Reducer Analysis Function
//////////////////////////////////////////////////////////////////////////"""

@dataclass(frozen = False)
class sPARAM_REDUCER_ANALYSIS_DATA():

    Q:np.ndarray = None
    QD:np.ndarray = None
    QDD:np.ndarray = None

    FF:np.ndarray = None
    NN:np.ndarray = None
    TAU:np.ndarray = None
    
    RadialForce:np.ndarray = None
    AxialForce:np.ndarray = None
    Moment:np.ndarray = None
    
    # -- Speed -- #
    MaxInputSpeed:np.ndarray = None 
    AveInputSpeed:np.ndarray = None
    
    # -- torque -- #
    AveOutputTorque:np.ndarray = None
    MaxTorque:np.ndarray = None

    # -- moment -- #
    AveMoment:np.ndarray = None
    MaxMoment:np.ndarray = None
    
    # -- life -- #
    ReducerLife = None
    BearingLife = None
    
    df:DataFrame = None

def WrenchCoord(links:LinkClass, Q:np.ndarray, QD:np.ndarray, QDD:np.ndarray,
                tool:ToolClass = None, grav:np.ndarray = DEFAULT_GRAV_ACC_VEC, fext:np.ndarray= np.zeros(6)):
    TAU = []; F = []; N = []

    dof = len(links)
    for i in range(len(Q)):
        tt, ff, nn = rne2(links, Q[i], QD[i], QDD[i], tool=tool, fext=fext, grav=grav)
        nn = -nn # ?
        tt = tt[:dof]
        ff = ff[:dof]
        nn = nn[:dof]
        TAU.append(tt); F.append(ff); N.append(nn)
    
    return np.array(TAU), np.array(F), np.array(N)

def _CRB_Fr_Fa_NN(f:np.ndarray, n:np.ndarray, jnt_conn_offs:np.ndarray):
    """
    :param f            : 3x1 force vector on the link coordinate [N]
    :param n            : 3x1 moment vector on the link coordinate [N]
    :param jnt_conn_offs: joint connection offset [mm]
    """
    CRB_force = np.abs(np.array(f))
    CRB_moment = np.cross(jnt_conn_offs*1e-3, f) + n

    fr = np.sqrt(CRB_force[0]**2 + CRB_force[1]**2)   # RaidalForce
    fa = CRB_force[2]                                 # AxialForce
    nn = np.sqrt(CRB_moment[0]**2 + CRB_moment[1]**2) # Moment
    
    return fr, fa, nn

def LoadBearing(FF, NN, links:LinkClass):
    RadialForce = []; AxialForce = []; Moment = []
    dataNum = np.shape(FF)[0]
    for j in range(len(links)):
        joffs = links[j].jntConfig.jntConnOffs
        Fr = []; Fa = []; M = []
        for i in range(dataNum):
            fr, fa, nn = _CRB_Fr_Fa_NN(FF[i,j], NN[i,j], joffs)
            Fr.append(fr); Fa.append(fa); M.append(nn)
        RadialForce.append(Fr); AxialForce.append(Fa); Moment.append(M)

    return np.array(RadialForce).T, np.array(AxialForce).T, np.array(Moment).T

def _EquivalentRadialLoadCRB(fr_av, fa_av, nn_av, dp, X, Y):
    """
    Get Equivalent Radial Load(동등가하중)
    Refer to Harmonic Drive manual
    
    :param fr_av: Average of radial load (평균레이디얼하중) [N]
    :param fa_av: Average of axial load (평균엑셜하중) [N]
    :param nn_av: Average of Moment (평균모멘트) -내가 만든 변수 [Nm]
    :param dp: 코로의 피치원경 [m]
    :param X: coefficient of radial load
    :param Y: coefficient of axial load
    :returns: Equivalent raidal load [N]
    """
    Pc = X * (fr_av + 2 * nn_av / dp) + Y * fa_av
    return Pc

def _LoadCoefficientCRB(fr_av, fa_av, nn_av, dp):
    """
    Get cofficient of radial load and axial load
    
    :param Fr_av: 평균레이디얼하중[N]
    :param Fa_av: 평균엑셜하중[N]
    :param NN_av: 평균모멘트[N]
    :param dp: 코로의 피치원경 [m]
    :return X,Y: 레이디얼하중계수, 스러스트하중계수
    """
    val = fr_av / (fa_av + 2 * nn_av / dp)
    if val <= 1.5:
        X = 1
        Y = 0.45
    else:
        X = 0.67
        Y = 0.67
    
    return X, Y
    
# -- BEARING LIFE -- #
def _LifeCRB(Fr, Fa, NN, QD, dp, C, fw):
    '''
    :param Fr : raidial force at the bearing [N]
    :param Fa : axial force at the bearing [N]
    :param NN : moment at the bearing  [Nm]
    :param QD : output speed [rad/s]
    :param dp : 코로의 피치원경 [m]
    :param C : 기본동정격하중 [N]
    :param fw : 하중계수
    '''
    powSumPc = 0.0
    aveVel = 0.0;    sumVel = 0.0
    oldVel = QD[0]; newVel = deepcopy(QD[0])
    
    t_sum = 0.0
    for i in range(1, len(QD)):
        oldVel = deepcopy(newVel)
        newVel = deepcopy(QD[i])
        aveVel  = np.abs((newVel + oldVel) / 2)
        sumVel += aveVel
        
        # fr, fa, nn = Bearing_Fr_Fa_NN(F[i], N[i], joffs)
        X, Y = _LoadCoefficientCRB(Fr[i], Fa[i], NN[i], dp)
        Pc = _EquivalentRadialLoadCRB(Fr[i], Fa[i], NN[i], dp, X, Y)
        
        powSumPc = powSumPc + (Pc**(10/3)) * aveVel
        
        t_sum += 1.0
        
    avePc = (powSumPc/sumVel)**(3/10)
    N_av = sumVel / t_sum  * RAD2DEG * DPS2RPM
    
    res = 1e+6/(60.0*N_av)*(C/(avePc*fw))**(10/3)
    
    return res

def _LifeBB(NN, QD, M0, N0, Km): # NOT VERIFIED YET
    '''
    :param NN : moment at the bearing [Nm]
    :param QD : output speed [rad/s]
    :param M0 : 허용모멘트하중, Mc [Nm]
    :param N0 : 정격속도 [RPM]
    :param Km : 서비스 라이프 [hr]
    '''

    # Pc = X * (fr_av + 2 * nn_av / dp) + Y * fa_av 
    factor = 3 # not verified

    powSumM = 0.0
    aveVel = 0.0;    sumVel = 0.0
    oldVel = QD[0]; newVel = deepcopy(QD[0])
    
    t_sum = 0.0
    
    for i in range(1, len(QD)):
        oldVel = deepcopy(newVel)
        newVel = deepcopy(QD[i])
        aveVel  = np.abs((newVel + oldVel) / 2)
        sumVel += aveVel
        
        powSumM = powSumM + (NN[i]**factor) * aveVel
        
        t_sum += 1.0
        
    M_av = (powSumM/sumVel)**(1/factor)
    N_av = sumVel / t_sum  * RAD2DEG * DPS2RPM
    
    # res = 1e+6/(60.0*N_av)*(C/(M_av*fw))**(10/3)
    
    res = Km * (N0/N_av) * (M0/M_av)**factor
    
    return res

def LifeBearingRobot(links:LinkClass, Q, QD, QDD, tool:ToolClass=None, grav=DEFAULT_GRAV_ACC_VEC, fext=np.zeros(6)):
    
    # -- Get Wrench at the link coordinate -- #
    _TAU, FF, NN = WrenchCoord(links, Q, QD, QDD, tool=tool,grav=grav,fext=fext)
    RadialForce, AxialForce, Moment = LoadBearing(FF, NN, links)

    # -- Get Bearing Life for All Joints -- #
    life_all = []
    dof = len(links)
    for i in range(dof):
        Reducer:sREDUCER_PARAM = links[i].Reducer
        if   Reducer.bearing_type == 'CRB':  life = _LifeCRB(RadialForce[:,i], AxialForce[:,i], Moment[:,i], QD[:,i], Reducer.dp, Reducer.C, Reducer.fw)
        elif Reducer.bearing_type == 'BB':   life = _LifeBB(Moment[:,i], QD[:,i], Reducer.Mc, Reducer.rated_speed, Reducer.rated_service_life)
        else:                                raise Exception("NOT DEIFNED BEARING TYPE")
        life_all.append(life)
    return life_all

# -- SPEED -- #
def GetAveInputSpeedRobot(QD, links:LinkClass):
    return np.array([_AveInputSpeed(QD[:,i], links[i].Reducer.reducing_ratio) for i in range(len(links))])

def _AveInputSpeed(QD_out, gamma):
    """
    :param QD_out            : output velocity of joint[rad/s]
    :param gamma         : reducing ratio
    """
    aveVel = 0.0;    sumVel = 0.0
    oldVel = QD_out[0]; newVel = deepcopy(QD_out[0])
    t_sum = 0.0
    
    QD_in = QD_out * gamma
    for j in range(1, len(QD_in)):
        oldVel = deepcopy(newVel)
        newVel = deepcopy(QD_in[j])
        aveVel  = np.abs((newVel + oldVel) / 2)
        sumVel += aveVel
        t_sum += 1
    N_av = sumVel / t_sum  * RAD2DEG * DPS2RPM
    return N_av

def AveInputSpeedRobot(QD, links:LinkClass):
    return np.array([np.max(np.abs(QD[:,i]))*links[i].Reducer.reducing_ratio for i in range(len(links))])*RAD2DEG*DPS2RPM

# -- TORQUE -- #
def MaxTorque(TAU):
    if 0:
        return np.array([np.max(TAU[:,i]) for i in range(np.shape(TAU)[1])])
    else:
        MaxTorque = []
        for i in range(len(TAU[0,:])):
            MaxTorque.append(np.max(TAU[:,i]))
        return np.array(MaxTorque)

def _AveOutputTorque(Q, QD, TAU, reducer_type = 'HD'):
    
    sumVel = 0.0; aveVel = 0.0
    oldVel =0.0; newVel = deepcopy(QD[0])
    
    newT = TAU[0]
    powSumT = 0.0
    t_sum = 0.0
    
    if reducer_type == 'HD':
        factor = 3
    elif reducer_type == 'RV':
        factor = 10/3
    else:
        raise Exception("NOT DEFINED REDUCER TYPE")
    
    for i in range(1, len(Q)):
        oldVel = deepcopy(newVel)
        oldT = deepcopy(newT)
        
        newVel = deepcopy(QD[i])
        aveVel = np.abs((newVel + oldVel) / 2)   
        sumVel += aveVel
        
        newT = TAU[i]
        aveT = np.abs((newT + oldT) / 2)

        powSumT = powSumT +  np.power(aveT,factor) * aveVel
        
        t_sum += 1
    
    return np.power(powSumT / sumVel, 1.0/factor)
    
def AveOutputTorqueRobot(Q, QD, TAU, links:LinkClass):
    return np.array([_AveOutputTorque(Q[:,i], QD[:,i], TAU[:,i], links[i].Reducer.type) for i in range(len(links))])

# -- MOMENT -- #
def MaxMoment(Moment):
    return np.array([np.max(np.abs(Moment[:,i])) for i in range(np.shape(Moment)[1])])

# -- LIFE -- #
def _LifeWaveGen(Q, QD, TAU,
                      Ln, Tr, Nr,
                      reducing_ratio,
                      reducer_type = 'HD'):
    '''
    :param reducer_type : 'HD' / 'RV'
    :param Ln : service life of reducer in rated torque and speed
    :param Tr : rated torque
    :param Nr : rated speed
    '''
    sumVel = aveVel = oldVel = 0.0
    newVel = deepcopy(QD[0])
    
    newT = TAU[0]
    powSumT = t_sum = 0.0
    
    if reducer_type == 'HD':    factor = 3
    elif reducer_type == 'RV':  factor = 10/3
    else:                       raise Exception("NOT DEFINED REDUCER TYPE")
    
    for i in range(1, len(Q)):
        oldVel = deepcopy(newVel)
        oldT = deepcopy(newT)
        
        newVel = deepcopy(QD[i])
        aveVel = np.abs((newVel + oldVel) / 2)   #TODO:aveVel = (aveVel + oldVel)/2 가 맞지 않은지? 검토 필요
        sumVel += aveVel
        
        newT = TAU[i]
        aveT = np.abs((newT + oldT) / 2)

        powSumT = powSumT +  np.power(aveT,factor) * aveVel
        
        t_sum += 1
    
    aveInSp = sumVel / t_sum * reducing_ratio
    avePowT = (powSumT / sumVel) ** (1/factor)
    Lh =  Ln * (Tr/avePowT)**factor * (Nr / aveInSp)
    
    return Lh

def LifeWaveGenRobot(Q, QD, TAU, links:LinkClass):
    '''
    :param reducer_type : 'HD' / 'RV'
    :param Ln : service life of reducer in rated torque and speed
    :param Tr : rated torque
    :param Nr : rated speed
    '''
    Lh = []
    for j in range(len(links)):
        
        Ln = links[j].Reducer.rated_service_life
        Tr = links[j].Reducer.rated_torque
        Nr = links[j].Reducer.rated_speed
        reducing_ratio = links[j].Reducer.reducing_ratio
        reducer_type = links[j].Reducer.type
        
        Lh.append(_LifeWaveGen(Q[:,j], QD[:,j], TAU[:,j], 
                                              Ln, Tr, Nr, 
                                              reducing_ratio, reducer_type))
    return Lh

# -- DataFrame -- #
def make_df(links:LinkClass, data:sPARAM_REDUCER_ANALYSIS_DATA):

    TABLE_HEADER = ["Ave.Torque[Nm]", "Ave.Input Speed[RPM]",
                "Max. Input Speed[RPM]", "A/D Torque[Nm]",
                "Max. Moment[Nm]", "ReducerLife[Hour]", "BearingLife[Hour]", ]
    
    cols = TABLE_HEADER
    index = ["J{}".format(i+1) for i in range(len(links)) ]

    df = pd.DataFrame(columns = cols)
    
    for i in range(len(links)):
        j = 0
        
        val = "%.2f(%.2f)"%(data.AveOutputTorque[i], links[i].Reducer.rated_torque)
        df.loc[i,cols[j]] = val
        j += 1
        
        val = "%.2f(%.2f)"%(data.AveInputSpeed[i], links[i].Reducer.rated_speed)
        df.loc[i,cols[j]] = val
        j += 1
        
        val = "%.2f(%.2f)"%(data.MaxInputSpeed[i], links[i].Reducer.max_input_speed)
        df.loc[i,cols[j]] = val
        j += 1
        
        
        val = "%.2f(%.2f)"%(data.MaxTorque[i], links[i].Reducer.ad_torque)
        df.loc[i,cols[j]] = val
        j += 1
        
        val = "%.2f(%.2f)"%(data.MaxMoment[i], links[i].Reducer.Mc)
        df.loc[i,cols[j]] = val
        j += 1
        
        try:
            tmpStr = "%d"%data.ReducerLife[i]
        except:
            tmpStr = "None"
        val = "%s(%d)"%(tmpStr, links[i].Reducer.rated_service_life)
        df.loc[i,cols[j]] = val
        j += 1

        try:
            tmpStr = "%d"%data.BearingLife[i]
        except:
            tmpStr = "None"
        val = "%s(%s)"%(tmpStr, "None")
        df.loc[i,cols[j]] = val
        j += 1
        
        
    df.index = index
    
    return df
    
# -- Main Analysis -- #
def calculate_dynamics(links:LinkClass, Q, QD, QDD, tool:ToolClass=None, grav=DEFAULT_GRAV_ACC_VEC, fext=np.zeros(6)):

    data = sPARAM_REDUCER_ANALYSIS_DATA()
    data.Q = Q
    data.QD = QD
    data.QDD = QDD

    # -- Get Wrench at the link coordinate -- #
    data.TAU, data.FF, data.NN = WrenchCoord(links, data.Q, data.QD, data.QDD, tool=tool,
                                                      grav=grav,fext=fext)
    
    # -- Get Wrench at the bearing -- #
    data.RadialForce, data.AxialForce, data.Moment = LoadBearing(data.FF, data.NN, links)
    
    # WAVE GENERATOR
    # -- Get max Moment -- #
    data.MaxMoment = MaxMoment(data.Moment)

    # -- 허용최고입력회전속도 -- # 
    data.MaxInputSpeed = AveInputSpeedRobot(data.QD, links)
    
    # -- 허용평균입력회전속도 -- #
    data.AveInputSpeed = GetAveInputSpeedRobot(data.QD, links)

    # -- 기동정지시허용피크토크 -- #
    data.MaxTorque = MaxTorque(data.TAU)
    
    # -- 평균허용토크(등가 평균 출력토크(Teq)) -- #
    data.AveOutputTorque = AveOutputTorqueRobot(data.Q, data.QD, data.TAU, links)

    # -- Reducer life -- #
    data.ReducerLife = LifeWaveGenRobot(data.Q, data.QD, data.TAU, links)

    # BEARING
    # -- Bearing Life -- #
    data.BearingLife = LifeBearingRobot(links, Q, QD, QDD, tool=tool, grav=grav,fext=fext)

    # -- Result df -- #
    data.df = make_df(links, data)

    return data
