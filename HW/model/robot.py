from define import *
from mathematics import tr2pose, pose2tr, inv_tform

from dataclasses import dataclass, field
import numpy as np
from numpy import arctan2, cos, sin, tan, pi
import yaml
import os

# 파라미터 yaml 루트. 이 파일은 SCARA/HW/model/ 에 있고,
# scara.yaml / motors/ / reducers/ 는 한 단계 위인 SCARA/HW/ 에 있다.
HW_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

class EmptyClass():
    pass

def isNone(value):
    return type(value).__name__ == "NoneType"

def get_link_tform_dh(link, ang):
    """ Compute a tform matrix from link to link with DH parameter
    """
    cos_th = cos(link[0]+ang)
    sin_th = sin(link[0]+ang)
    
    cos_al = cos(link[3])
    sin_al = sin(link[3])
    
    d = link[1]
    a = link[2]
    
    return np.array([[cos_th,  -sin_th*cos_al,  sin_th*sin_al,   a*cos_th],
                     [sin_th,   cos_th*cos_al, -cos_th*sin_al,   a*sin_th],
                     [     0,          sin_al,         cos_al,          d],
                     [     0,               0,              0,          1]])

def inv_pos_from_dh(link):
    '''
    NOTE:
    - independent to joint variable
    '''    
    d = link[1]
    a = link[2]
    alpha = link[3]
    p = np.array([a, d*sin(alpha), d*cos(alpha)]) # i-1 coord to i coord represented in i coord
    return p

def get_kine_table(arg1, arg2):
    # TODO: 빈칸 채우기

    KINE_TABLE =  {
                    DH            : { IDX_PARA_NUM : 4,     IDX_LINK_FUNC : get_link_tform_dh,                        IDX_INV_FUNC   : inv_pos_from_dh                                  },
                    XYZ_EULXYZ    : { IDX_PARA_NUM : 6,     IDX_LINK_FUNC : lambda vec: pose2tr(vec, eulType=EUL_XYZ),    IDX_INV_FUNC   : lambda vec:tr2pose(inv_tform(pose2tr(vec, eulType=EUL_XYZ)),eulType=EUL_XYZ)  },
                    XYZ_EULRPY    : { IDX_PARA_NUM : 6,     IDX_LINK_FUNC : lambda vec: pose2tr(vec, eulType=EUL_RPY),    IDX_INV_FUNC   : lambda vec:tr2pose(inv_tform(pose2tr(vec, eulType=EUL_RPY)),eulType=EUL_RPY)  },
                }
    
    return KINE_TABLE[arg1][arg2]

@dataclass(frozen = False)
class sMOTOR_PARAM:
    sn:str = UNDEFINED
    type:str = UNDEFINED
    rated_torque:float = 0.0 # Nm
    max_torque:float = 0.0   # Nm
    rated_speed:float = 0.0  # rad/s
    max_speed:float = 0.0    # rad/s
    efficiency:float = 1.0
    power:float = 0.0

@dataclass(frozen = False)
class sREDUCER_PARAM:
    sn:str = UNDEFINED
    type:str = HD
    efficiency:float = 1.0
    size:int = 0
    reducing_ratio:float = 0.0
    rated_speed:float = 0.0
    rated_torque:float = 0.0
    ad_torque:float = 0.0
    ave_torque:float = 0.0
    max_input_speed:float = 0.0
    ave_input_speed:float = 0.0
    rated_service_life:float = 0.0
    two_mupf:float = 0.0
    flexsplineFixed:bool = False 

    bearing_type:str = CRB
    dp:float = 0.0 # 코로의 피치원경
    C:float = 0.0  # 기본동정격하중
    C0:float = 0.0 # 기본정정격하중
    fw:float = 0.0 # 하중계수
    Mc:float = 0.0 # 허용모멘트하중
    
@dataclass(frozen = False)
class sLINK_INFO:
    name = UNDEFINED
    stlFiles = None
    linkNum = 0

@dataclass(frozen=False)
class sLINK_KINE:
    origin: np.ndarray = field(default_factory=lambda: np.zeros(6))
    kineType: str = DH
    tform = None
    invPos = None
    numPara = None

@dataclass(frozen = False)
class sLINK_JOINT_CONFIG:
    type = None
    stiff = np.inf
    minRange = None
    maxRange =None
    maxVel = None
    ratedVel = None
    gearRatio = None
    used = None
    jntAccTime = None
    jntDecTime = None
    jntConnOffs:np.ndarray = None

@dataclass(frozen=False)
class sLINK_MASS_PROP:
    mass: float = 0
    COM: np.ndarray = field(default_factory=lambda: np.zeros(3))
    tensor: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))

class LinkClass:
    def __init__(self,
                 name:str = UNDEFINED,
                 stlFiles:str = None,
                 linkNum:int = 0,

                 origin:np.ndarray = np.zeros(6),
                 kineType:str = DH,
                 
                 jntType:str = REV,
                 jntStiff:float = np.inf,
                 jntMinRange:float = None,
                 jntMaxRange:float = None,
                 jntMaxVel:float = None,
                 jntRatedVel:float = None,
                 jntAccTime:float = None,
                 jntDecTime:float = None,              
                 jntGearRatio:float = 1.0,
                 jntUsed:bool = True,
                 jntConnOffs:np.ndarray = None,

                 mass:float = 0.0,
                 COM:np.ndarray = np.zeros(3),
                 tensor:np.ndarray = np.zeros((3,3)),

                 Motor:sMOTOR_PARAM = None,
                 Reducer:sREDUCER_PARAM = None,
                ):

        # ===========================================
        #               INFORMATION
        # ===========================================
        self.info = sLINK_INFO()
        self.info.name = name
        self.info.stlFiles = stlFiles # None, 
        self.info.linkNum = linkNum

        # ===========================================
        #               COMPONENTS
        # ===========================================
        self.Motor = Motor
        self.Reducer = Reducer

        # ===========================================
        #               KINEMATICS
        # ===========================================
        self.kine = sLINK_KINE()
        self.kine.origin = np.hstack((origin, np.zeros(6-len(origin))))
        self.kine.kineType = kineType # DH HAYATI SIX_PARA, XYZ_eulRPY, XYZ_eulXYZ, ....

        self.kine.tform = lambda ang : get_kine_table(kineType, IDX_LINK_FUNC)(self.kine.origin, ang)
        self.kine.invPos = get_kine_table(kineType, IDX_INV_FUNC)(origin)
        self.kine.numPara = get_kine_table(kineType, IDX_PARA_NUM)
        
        # ===========================================
        #               JOINT CONFIG
        # ===========================================
        self.jntConfig = sLINK_JOINT_CONFIG()
        self.jntConfig.type = REV # PRIS # FIX
        self.jntConfig.stiff = jntStiff

        if isNone(jntMinRange):
            print("WARNING! joint min range is not defined (set to -PI_TWO)")
            jntMinRange = -PI_TWO if jntType == REV else -np.inf
        if isNone(jntMaxRange):
            print("WARNING! joint max range is not defined (set to PI_TWO)")
            jntMaxRange = PI_TWO if jntType == REV else np.inf
        
        if isNone(jntMaxVel):
            if self.Motor is not None:
                jntMaxVel = self.Motor.max_speed / jntGearRatio * RPM2DPS * DEG2RAD
                print("WARNING! joint max velocity is not defined (estimated by motor max speed and gear ratio)")
                print("joint max velocity : ", jntMaxVel)
            else:
                jntMaxVel = 6000.0 / 100 * RPM2DPS * DEG2RAD
                print("WARNING! joint max velocity is not defined (set to default value")
                print("joint max velocity : ", jntMaxVel)

        if isNone(jntRatedVel):
            if self.Motor is not None:
                jntRatedVel = self.Motor.rated_speed / jntGearRatio * RPM2DPS * DEG2RAD
                print("WARNING! joint rated velocity is not defined (estimated by motor rated speed and gear ratio)")
                print("joint max velocity : ", jntMaxVel)
            else:
                jntRatedVel = 3000.0 / 100 * RPM2DPS * DEG2RAD
                print("WARNING! joint rated velocity is not defined (set to default value")
                print("joint rated velocity : ", jntRatedVel)

        if isNone(jntAccTime):
            jntAccTime = 0.2 # sec
        
        if isNone(jntDecTime):
            jntDecTime = 0.1 # sec

        self.jntConfig.minRange = jntMinRange # RAD
        self.jntConfig.maxRange = jntMaxRange # RAD
        self.jntConfig.maxVel = jntMaxVel # RAD/S
        self.jntConfig.ratedVel = jntRatedVel # RAD/S
        self.jntConfig.jntAccTime = jntAccTime # sec
        self.jntConfig.jntDecTime = jntDecTime # sec
        self.jntConfig.gearRatio = jntGearRatio
        self.jntConfig.used = jntUsed
        self.jntConfig.jntConnOffs = jntConnOffs # for bearing

        # ===========================================
        #               MASS PROPERTY
        # ===========================================
        self.massProp = sLINK_MASS_PROP()
        self.massProp.mass = mass
        self.massProp.COM = COM
        self.massProp.tensor = tensor

    def set_kineType(self, kineType):
        self.kine.kineType = kineType # DH HAYATI SIX_PARA, XYZ_eulRPY, XYZ_eulXYZ, ....
        self.kine.tform = lambda ang : get_kine_table(kineType, IDX_LINK_FUNC)(self.kine.origin, ang)
        self.kine.invPos = get_kine_table(kineType, IDX_INV_FUNC)(self.kine.origin)
        self.kine.numPara = get_kine_table(kineType, IDX_PARA_NUM)

    def update(self):
        self.__init__(
            name = self.info.name,
            stlFiles = self.info.stlFiles,
            linkNum = self.info.linkNum,

            origin = self.kine.origin,
            kineType = self.kine.kineType,
            
            jntType = self.jntConfig.type,
            jntStiff = self.jntConfig.stiff,
            jntMinRange = self.jntConfig.minRange,
            jntMaxRange = self.jntConfig.maxRange,
            jntMaxVel = self.jntConfig.maxVel,
            jntRatedVel = self.jntConfig.ratedVel,
            jntGearRatio = self.jntConfig.gearRatio,
            jntUsed = self.jntConfig.used,

            mass = self.massProp.mass,
            COM = self.massProp.COM,
            tensor = self.massProp.tensor,

            Motor = self.Motor,
            Reducer = self.Reducer,
        )

class ToolClass:
    '''
    기능이 있는 tool은 상속받아 사용
    '''
    def __init__(self,
                 name:str = UNDEFINED,
                 stlFiles:str = None,

                 origin:np.ndarray = np.zeros(6),
                 kineType:str = XYZ_EULXYZ,

                 mass:float = 0.0,
                 COM:np.ndarray = np.zeros(3),
                 tensor:np.ndarray = np.zeros((3,3)),

                 sensorType = None
                ):

        # INFORMATION ======================================        
        self.info = EmptyClass()
        self.info.name = name
        self.info.stlFiles = stlFiles # None, 
        self.info.sensorType = sensorType # 

        # KINEMATICS  ======================================
        self.kine = EmptyClass()
        self.kine.origin = origin
        self.kine.kineType = kineType # DH HAYATI SIX_PARA, XYZ_eulRPY, XYZ_eulXYZ, ....



        self.kine.tform = get_kine_table(kineType, IDX_LINK_FUNC)(origin)
        self.kine.invTform = inv_tform(self.kine.tform)
        self.kine.invPos = get_kine_table(kineType, IDX_INV_FUNC)(origin)
        self.kine.numPara = get_kine_table(kineType, IDX_PARA_NUM)
        
        # jntConfig  ======================================

        # MASS PROPERTY ======================================
        self.massProp = EmptyClass()
        self.massProp.mass = mass
        self.massProp.COM = COM
        self.massProp.tensor = tensor

        # COMPONENT =====================================

    def set_kineType(self, kineType):
        self.kine.kineType = kineType # DH HAYATI SIX_PARA, XYZ_eulRPY, XYZ_eulXYZ, ....
        self.kine.tform = get_kine_table(kineType, IDX_LINK_FUNC)(self.kine.origin)
        self.kine.invPos = get_kine_table(kineType, IDX_INV_FUNC)(self.kine.origin)
        self.kine.numPara = get_kine_table(kineType, IDX_PARA_NUM)
    

@dataclass(frozen = False)
class sMOTOR_PARAM:
    sn:str = UNDEFINED
    type:str = UNDEFINED
    rated_torque:float = 0.0 # Nm
    max_torque:float = 0.0   # Nm
    rated_speed:float = 0.0  # rad/s
    max_speed:float = 0.0    # rad/s
    efficiency:float = 1.0
    power:float = 0.0

@dataclass(frozen = False)
class sREDUCER_PARAM:
    sn:str = UNDEFINED
    type:str = HD
    efficiency:float = 1.0
    size:int = 0
    reducing_ratio:float = 0.0
    rated_speed:float = 0.0
    rated_torque:float = 0.0
    ad_torque:float = 0.0
    ave_torque:float = 0.0
    max_input_speed:float = 0.0
    ave_input_speed:float = 0.0
    rated_service_life:float = 0.0
    two_mupf:float = 0.0
    flexsplineFixed:bool = False 

    bearing_type:str = CRB
    dp:float = 0.0 # 코로의 피치원경
    C:float = 0.0  # 기본동정격하중
    C0:float = 0.0 # 기본정정격하중
    fw:float = 0.0 # 하중계수
    Mc:float = 0.0 # 허용모멘트하중
    
def load_param_motor(sn):

    motor = sMOTOR_PARAM()
    file = os.path.join(HW_ROOT, 'motors', sn + '.yaml')

    with open(file) as f:
        film = yaml.load(f, Loader=yaml.FullLoader)
    
    motor.sn                   = film['sn']
    motor.type                 = film['type']
    motor.power                = film['power']
    motor.rated_torque         = film['rated_torque']
    motor.max_torque           = film['max_torque']
    motor.rated_speed          = film['rated_speed']
    motor.max_speed            = film['max_speed']
    motor.efficiency           = film['efficiency']
    
    return motor

def load_param_reducer(sn):

    reducer = sREDUCER_PARAM()
    file = os.path.join(HW_ROOT, 'reducers', sn + '.yaml')
    with open(file) as f:
        film = yaml.load(f, Loader=yaml.FullLoader)
    
    reducer.sn                   = film['sn']
    reducer.type                 = film['type']
    reducer.efficiency           = film['efficiency']
    reducer.size                 = film['size']
    reducer.reducing_ratio       = film['reducing_ratio']
    reducer.rated_speed          = film['rated_speed']
    reducer.rated_torque         = film['rated_torque']
    reducer.ad_torque            = film['ad_torque']
    reducer.ave_torque           = film['ave_torque']
    reducer.max_input_speed      = film['max_input_speed']
    reducer.ave_input_speed      = film['ave_input_speed']
    reducer.rated_service_life   = film['rated_service_life']
    reducer.two_mupf             = film['two_mupf']
    reducer.flexsplineFixed      = film['flexsplineFixed']
    
    reducer.bearing_type         = film['bearing_type']
    reducer.dp                   = film['dp']
    reducer.C                    = film['C']
    reducer.C0                   = film['C0']
    reducer.fw                   = film['fw']
    reducer.Mc                   = film['Mc']

    return reducer

def load_param_robot(sn, basePose, grav, file):

    # 파일명만 넘어오면 HW_ROOT(=SCARA/HW) 기준으로 해석한다.
    # 절대경로/상대경로가 그대로 유효하면 그것을 우선 사용.
    if not os.path.isfile(file):
        file = os.path.join(HW_ROOT, file)
    with open(file) as f:
        film = yaml.load(f, Loader=yaml.FullLoader)

    name                = film['name']
    dof                 = film['dof']
    axisConfig          = film['axisConfig']
    kineType            = film['kineType']
    eulType             = film['eulType']
    stlFiles            = film['stlFiles']
    LinkChain           = np.array(film['LinkChain']).astype(float)

    LinkChain[:,0] *= DEG2RAD # theta
    LinkChain[:,3] *= DEG2RAD # alpha
    if np.shape(LinkChain)[1] > 4: LinkChain[:,4] *= DEG2RAD  # beta
    
    mass                = np.array(film['mass'])
    COM                 = np.array(film['COM'])
    tensor              = np.array(film['tensor'])
    jntStiff            = np.array(film['jntStiff'])
    jntRange            = np.array(film['jntRange']) * DEG2RAD
    jntMaxVel           = np.array(film['jntMaxVel']) * DEG2RAD
    jntRatedVel         = np.array(film['jntRatedVel']) * DEG2RAD
    jntGearRatio        = np.array(film['jntGearRatio'])
    ovs_ratio           = np.array(film['ovs_ratio'])
    joint_ovs           = jntMaxVel * ovs_ratio
    motor_ovs           = joint_ovs * jntGearRatio
    jntConnOffs         = np.array(film['jntConnOffs'])

    task_rated_speed    = np.array(film['task_rated_speed']) * DEG2RAD_POSE
    task_acc_time       = film['task_acc_time']
    task_dec_time       = film['task_dec_time']

    motors = [None for _ in range(dof)]
    reducers = [None for _ in range(dof)]

    f_motors=film['motors']
    f_reducers=film['reducers']
    for i in range(dof):
        if f_motors[i] == False:
            motors[i] = load_param_motor('DEFAULT_MOTOR')
        else:
            try   : motors[i] = load_param_motor(f_motors[i])
            except: motors[i] = load_param_motor('DEFAULT_MOTOR')
    
        if f_reducers[i] == False:
            motors[i] = load_param_reducer('DEFAULT_REDUCER')
        else:
            try   : reducers[i] = load_param_reducer(f_reducers[i])
            except: reducers[i] = load_param_reducer('DEFAULT_REDUCER')
    
    # BASE 생성 ======================================
    base = LinkClass(name = 'LINK0',
                    stlFiles = None,
                    origin = basePose,
                    kineType = XYZ_EULXYZ,
                    linkNum = 0,
                    )
    
    # LINK 생성 ======================================
    links = [None for _ in range(dof)]

    for i in range(dof):
        links[i] = LinkClass(name = 'LINK{}'.format(i+1),
                            stlFiles = stlFiles[i],
                            linkNum = i+1,

                            origin = LinkChain[i],
                            kineType = kineType,

                            jntType = REV,
                            jntStiff = jntStiff[i],
                            jntMinRange = jntRange[i,0],
                            jntMaxRange = jntRange[i,1],
                            jntMaxVel = jntMaxVel[i],
                            jntRatedVel = jntRatedVel[i],          
                            jntGearRatio = jntGearRatio[i],
                            jntUsed = True,
                            jntConnOffs = jntConnOffs[i],

                            mass=mass[i],
                            COM=COM[i],
                            tensor=tensor[i],

                            Motor = motors[i],
                            Reducer = reducers[i],
                            )
    
    return base,links,kineType,name,axisConfig,basePose,grav,eulType,\
                      task_rated_speed, task_acc_time, task_dec_time