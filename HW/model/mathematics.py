
from define import *
""" 
////////////////////// MODULES //////////////////////
"""
import numpy as np
from numpy import arctan2, cos, sin, tan, pi

'''
/////////////////////////////////////////////////////////////////////////////
#                            ROTATION
/////////////////////////////////////////////////////////////////////////////
'''

def rotx(ang: float, unit='rad'):
    """
    get a matrix in SO(3) for rotation w.r.t x-axis
    
    :param ang: angle with respec to x-axis
    :param unit: unit
    :return: ndarray  with the (3,3) shape
    
    np.array([[1,        0,         0],
              [0, cos(ang), -sin(ang)],
              [0, sin(ang),  cos(ang)]])  
              
   """
    if unit == 'deg':
        ang = np.deg2rad(ang)

    return np.array([[1,        0,         0],
                     [0, cos(ang), -sin(ang)],
                     [0, sin(ang),  cos(ang)]])

def roty(ang: float, unit='rad'):
    """
    get a matrix in SO(3) for rotation w.r.t y-axis
    
    :param ang: angle with respec to y-axis
    :param unit: unit
    :return: ndarray  with the (3,3) shape
    
    np.array([[ cos(ang),  0,  sin(ang)],
              [        0,  1,         0],
              [-sin(ang),  0,  cos(ang)]])
              
   """
    if unit == 'deg':
        ang = np.deg2rad(ang)

    return np.array([[cos(ang),  0,  sin(ang)],
                     [0,  1,         0],
                     [-sin(ang),  0,  cos(ang)]])

def rotz(ang: float, unit='rad'):
    """
    get a matrix in SO(3) for rotation w.r.t z-axis
    
    :param ang: angle with respec to z-axis
    :param unit: unit
    :return: ndarray  with the (3,3) shape
    
    np.array([[cos(ang), -sin(ang),  0],
              [sin(ang),  cos(ang),  0],
              [       0,         0,  1]])
              
   """

    if unit == 'deg':
        ang = np.deg2rad(ang)

    return np.array([[cos(ang), -sin(ang),  0],
                     [sin(ang),  cos(ang),  0],
                     [0,         0,  1]])

def trotx(ang: float, unit='rad'):
    """
    get a matrix in SE(3) for rotation w.r.t x-axis
    
    :param ang: angle with respec to x-axis
    :param unit: unit
    :return: ndarray  with the (3,3) shape
    
    np.array([[1,        0,         0, 0],
              [0, cos(ang), -sin(ang), 0],
              [0, sin(ang),  cos(ang), 0],
              [0,        0,         0, 1]])
              
   """
    if unit == 'deg':
        ang = np.deg2rad(ang)

    mat = np.eye(4)
    mat[0:3, 0:3] = rotx(ang, unit)
    return mat

def troty(angle: float, unit='rad'):
    """
    get a matrix in SE(3) for rotation w.r.t y-axis
    
    :param ang: angle with respec to x-axis
    :param unit: unit
    :return: ndarray  with the (3,3) shape
    
    np.array([[ cos(ang),  0,  sin(ang), 0],
              [        0,  1,         0, 0],
              [-sin(ang),  0,  cos(ang), 0],
              [        0,  0,         0, 1]])
              
   """

    if unit == 'deg':
        angle = np.deg2rad(angle)

    mat = np.eye(4)
    mat[0:3, 0:3] = roty(angle, unit)
    return mat

def trotz(angle: float, unit='rad'):
    """
    get a matrix in SE(3) for rotation w.r.t z-axis
    
    :param ang: angle with respec to x-axis
    :param unit: unit
    :return: ndarray  with the (3,3) shape
    
    np.array([[cos(ang), -sin(ang),  0, 0],
              [sin(ang),  cos(ang),  0, 0],
              [       0,         0,  1, 0],
              [       0,         0,  0, 1]])
   """

    if unit == 'deg':
        angle = np.deg2rad(angle)

    mat = np.eye(4)
    mat[0:3, 0:3] = rotz(angle, unit)
    return mat

'''
/////////////////////////////////////////////////////////////////////////////
#                            TRANSLATION
/////////////////////////////////////////////////////////////////////////////
'''

def translx(dx: float):
    """ get a matrix in SE(3) for displacement x direction
    """
    mat = np.eye(4)
    mat[0, 3] = dx
    return mat

def transly(dy: float):
    """ get a matrix in SE(3) for displacement y direction
    """
    mat = np.eye(4)
    mat[1, 3] = dy
    return mat

def translz(dz: float):
    """ get a matrix in SE(3) for displacement z direction
    """
    mat = np.eye(4)
    mat[2, 3] = dz
    return mat

def transl(dvec: np.ndarray or list):
    """ get a matrix in SE(3) for displacement x, y, z direction
    """
    mat = np.eye(4)
    mat[0:3, 3] = np.array(dvec)
    return mat


'''
/////////////////////////////////////////////////////////////////////////////
#                            TRANFORMATION
/////////////////////////////////////////////////////////////////////////////
'''
def inv_tform(T: np.ndarray):
    """ Compute invese matrix of a transformation matrix in SE(3)
    
    Inv T =  | R_T, -R_T *d |
             |   0,       1 |
    """
    RT = T[0:3, 0:3].T
    d = T[0:3, 3]

    res = np.eye(4)
    res[0:3, 0:3] = RT
    res[0:3, 3] = -RT @ d

    return res

def t2r(T: np.ndarray):
    """ return rotation matrix from T
    """
    return T[0:3, 0:3]


'''
/////////////////////////////////////////////////////////////////////////////
#                            TF <-> VECTOR
/////////////////////////////////////////////////////////////////////////////
'''
def vex(S: np.ndarray):
    """ Compute a vector corresponding skew-symmetric matrix S
    For example,
    S =np.array([[  0, -vz,  vy],
                [ vz,   0, -vx],
                [-vy,  vx,   0]])

    Note:
        - This is the inverse of the function Skew()
        - Only rudimentary checking (zero diagonal) is done to ensure that the matrix is actually skew-symmetric
        - The funciton takes the mean of the two elements that correspond to each unique element of the matrix
    Reference: 
        - Robotics, Vision & Control: Second Edition, Chap 2, P. Corke, Springer 2016.
    """
    v = 0.5*np.array([S[2, 1]-S[1, 2], S[0, 2]-S[2, 0], S[1, 0]-S[0, 1]])
    return v

def tr2tvec(T: np.ndarray):
    """ return translation component of T matrix
    """
    return T[0:3, 3]

def delta2tr(d: np.ndarray or list):
    """ Convert differential motion  to a homogeneous transform

    T = DELTA2TR(D) is a homogeneous transform (4x4) representing differential 
    translation and rotation. The vector D=(dx, dy, dz, dRx, dRy, dRz)
    represents an infinitessimal motion, and is an approximation to the spatial 
    velocity multiplied by time.

    """
    eye = np.eye(4)

    delta = np.zeros((4, 4))
    delta[0:3, 0:3] = skew3x3(d[3:6])
    delta[0:3, 3] = d[0:3]

    delta += eye

    return delta

def tr2delta(T1: np.ndarray, T2: np.ndarray):
    """ Convert homogeneous transform to differential motion in the T1 frame

    D = TR2DELTA(T0, T1) is the differential motion (6x1) corresponding to 
    infinitessimal motion (in the T0 frame) from pose T0 to T1 which are homogeneous 
    transformations (4x4) or SE3 objects. D=(dx, dy, dz, dRx, dRy, dRz). 

    D = TR2DELTA(T) as above but the motion is with respect to the world frame.

    Notes::
    - D is only an approximation to the motion T, and assumes
    that T0 ~ T1 or T ~ eye(4,4).
    - can be considered as an approximation to the effect of spatial velocity over a
    a time interval, average spatial velocity multiplied by time.

    사실 단위행렬 안빼도 된다. 원래 이론적으로는 단위행렬 뺀 행렬에서 뽑는 게 맞는데,
    하던 안하던 결과는 똑같다. 어차피 diagonal term은 안가져감.

    """
    # Td = np.linalg.inv(T1) @ T2
    Td = inv_tform(T1) @ T2

    delt = tr2tvec(Td)
    delv = vex(t2r(Td) - np.eye(3))
    delta = np.hstack([delt, delv])

    return delta

def skew3x3(a: np.ndarray or list):
    """ return a 3x3 skew-symmetric matrix corresponding to 3x1 input vector a
    The order should be x-y-z
    """
    x, y, z = a
    return np.array([[0, -z,  y],
                     [z,  0, -x],
                     [-y,  x,  0]])

'''
/////////////////////////////////////////////////////////////////////////////
#                            EULER EUL_XYZ
/////////////////////////////////////////////////////////////////////////////
'''

def set_in_180(x):
    while(1):
        if x > PI or x <= -PI:
            if x <= -PI:
                x += PI_TWO
            elif x > PI:
                x -= PI_TWO
        else:
            break
    return x

def eulXYZ2r(eul):
    """ convert a 3x1 euler vector into 3x3 rotation matrix
    """
    rx = rotx(eul[0])
    ry = roty(eul[1])
    rz = rotz(eul[2])

    return rx @ ry @ rz

def eulXYZ2tr(eul):
    """ convert a 3x1 euler vector into 4x4 transfomr matrix
    """
    rx = trotx(eul[0])
    ry = troty(eul[1])
    rz = trotz(eul[2])

    return rx @ ry @ rz

def tr2eulXYZ(R, in180=True, TINY=1e-4):
    """ convert a 3x3 rotation matrix into 3x1 EUL_XYZ euler vector
    TODO:
    - Check SO(3) Functions
    """
    try:
        if R[0, 2] < 1 and R[0, 2] > -1:
            phi = np.arctan2(-R[1, 2], R[2, 2])
            theta = np.arctan2(R[0, 2], np.sqrt(R[0, 0]**2 + R[0, 1]**2))
            psi = np.arctan2(-R[0, 1], R[0, 0])

        elif np.abs(R[0, 2] + 1) < TINY:
            theta = -np.pi/2
            psi = 0.0
            phi = np.arctan2(-R[1, 0], R[1, 1])

        elif np.abs(R[0, 2]-1) < TINY:
            theta = np.pi/2
            phi = np.arctan2(R[1, 0], R[1, 1])
            psi = 0.0
        else:
            raise Exception("R[0,2] is out of range (1, -1), R[0,2] :", R[0, 2])

        if in180 == True:
            phi = set_in_180(phi)
            theta = set_in_180(theta)
            psi = set_in_180(psi)

    except:
        print("R : ", R)
        print("in180 : ", in180)
        print("TINY : ", TINY)

    return np.array([phi, theta, psi])


'''
/////////////////////////////////////////////////////////////////////////////
#                            EULER EUL_RPY (SW)
/////////////////////////////////////////////////////////////////////////////
'''

def eulRPY2r(eul):
    """ convert a 3x1 euler vector into 3x3 rotation matrix
    eul = [r, p, y]
    where,
      r : rotation angle w.r.t x axis of fixed frame
      p : rotation angle w.r.t y axis of fixed frame
      y : rotation angle w.r.t z axis of fixed frame
    """
    eul = list(eul)
    eul[0] = float(eul[0])
    eul[1] = float(eul[1])
    eul[2] = float(eul[2])
    r, p, y = eul
    
    return rotz(eul[2]) @ roty(eul[1]) @ rotx(eul[0])

def eulRPY2tr(eul):
    """ convert a 3x1 euler vector into 4x4 rotation matrix
    eul = [r, p, y]
    where,
      r : rotation angle w.r.t z axis of fixed frame
      p : rotation angle w.r.t y axis of fixed frame
      y : rotation angle w.r.t x axis of fixed frame
    """
    eul = list(eul)
    eul[0] = float(eul[0])
    eul[1] = float(eul[1])
    eul[2] = float(eul[2])
    r, p, y = eul
    
    return trotz(eul[2]) @ troty(eul[1]) @ trotx(eul[0])

def tr2eulRPY(R, in180=True):
    """ convert a 3x3 rotation matrix into 3x1 XYZ euler vector
    TODO:
    - Check SO(3) Functions
    """
    
    try:
        if -R[2, 0] < 1 and -R[0, 2] > -1:
            r = np.arctan2(R[2, 1], R[2, 2])
            p = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
            y = np.arctan2(R[1, 0], R[0, 0])
        elif np.abs(-R[2, 0] + 1) < TINY:
            p = -np.pi/2
            r = 0.0
            y = np.arctan2(-R[1, 2], R[1, 1]) # 
        elif np.abs(-R[2, 0] - 1) < TINY:
            p = np.pi/2
            r = 0.0
            y = np.arctan2(R[1, 2], R[1, 1])
        else:
            raise Exception("R[0,2] is out of range (1, -1), R[0,2] :", R[0, 2])
        if in180 == True:
            r = set_in_180(r)
            p = set_in_180(p)
            y = set_in_180(y)

    except:
        print("R : ", R)
        print("in180 : ", in180)
        print("TINY : ", TINY)

    return np.array([r, p, y])

'''
/////////////////////////////////////////////////////////////////////////////
#                            EULER EUL_RPY (INDY7)
/////////////////////////////////////////////////////////////////////////////
'''
def eulINDY2r(eul):
    eul = list(eul)
    eul[0] = float(eul[0])
    eul[1] = float(eul[1])
    eul[2] = float(eul[2])
    r, p, y = eul
    
    return rotz(eul[2]) @ roty(eul[1]) @ rotx(eul[0])

def eulINDY2tr(eul):
    eul = list(eul)
    eul[0] = float(eul[0])
    eul[1] = float(eul[1])
    eul[2] = float(eul[2])
    r, p, y = eul
    
    return trotz(eul[2]) @ troty(eul[1]) @ trotx(eul[0])

def tr2eulINDY(R, in180=True, TINY=1e-4):
    """ convert a 3x3 rotation matrix into 3x1 INDY7 euler vector
    TODO:
    - Check SO(3) Functions
    """
    
    try:
        if -R[2, 0] < 1 and -R[0, 2] > -1:
            r = np.arctan2(R[2, 1], R[2, 2])
            p = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
            y = np.arctan2(R[1, 0], R[0, 0])
        elif np.abs(-R[2, 0] + 1) < TINY:
            p = -np.pi/2
            r = 0.0
            y = np.arctan2(-R[1, 2], R[1, 1]) # 
        elif np.abs(-R[2, 0] - 1) < TINY:
            p = np.pi/2
            r = 0.0
            y = np.arctan2(R[1, 2], R[1, 1])
        else:
            raise Exception("R[0,2] is out of range (1, -1), R[0,2] :", R[0, 2])
        if in180 == True:
            r = set_in_180(r)
            p = set_in_180(p)
            y = set_in_180(y)

    except:
        print("R : ", R)
        print("in180 : ", in180)
        print("TINY : ", TINY)

    return np.array([r, p, y])

'''
/////////////////////////////////////////////////////////////////////////////
#                            POSE <-> TF
/////////////////////////////////////////////////////////////////////////////
'''

def pose2tr(vec, eulType = EUL_XYZ, func = None):
    """ convert 6x1 pose vector into a homogeneous transform matrix
    The euler form EUL_XYZ and EUL_RPY are accepted
    """
    eul = vec[3:]
    pos = vec[0:3]
    
    if func is None:
        if eulType == EUL_XYZ:
            T = eulXYZ2tr(eul)
        elif eulType == EUL_RPY:
            T = eulRPY2tr(eul)
    else:
        if type(func) is not function:
            raise Exception("func argument must be function type")
        T = func(eul)
        
    T[0:3, 3] = pos

    return T

def tr2pose(T, in180=False, eulType=EUL_XYZ, func = None):
    """ convert a homogenous transform matrix into 6x1 pose vector
    The euler form EUL_XYZ and EUL_RPY are accepted
    """
    try:
        p = np.zeros(6)
        p[0:3] = T[0:3, 3]
        if func is None:
            if eulType == EUL_XYZ:
                p[3:] = tr2eulXYZ(T[0:3, 0:3], in180=in180)
            elif eulType == EUL_RPY:
                p[3:] = tr2eulRPY(T[0:3, 0:3], in180=in180)
            else:
                raise Exception("")
        else:
            if type(func) is not function:
                raise Exception("func argument must be function type")
            p[3:] = func(T[0:3, 0:3], in180=in180)
        
    except:
        print("T : ", T)
        print("in180 : ", in180)
        print("eulType : ", eulType)
    return p



def MatAng2EulXYZVel(eul):
    """ Get 3x3 matrix that converts angular velocity into time derivative of euler xyz
    Note that rz, eul[2] is not used
    
    old name : GetAngular2EulerXYZVel

    """
    sin_rx = sin(eul[0])
    sin_ry = sin(eul[1])
    cos_ry = cos(eul[1])
    cos_rx = cos(eul[0])

    return np.array([[1, (sin_rx*sin_ry) / cos_ry, -(cos_rx*sin_ry) / cos_ry],
                     [0,                   cos_rx,                    sin_rx],
                     [0,            -sin_rx/cos_ry,            cos_rx/cos_ry]])

def MatEulXYZ2AngVel(eul):
    """ Get 3x3 matrix that converts time derivative of euler xyz into angular velocity
    Note that rz, eul[2] is not used
    """
    sin_rx = sin(eul[0])
    sin_ry = sin(eul[1])
    cos_ry = cos(eul[1])
    cos_rx = cos(eul[0])

    return np.array([[1,      0,          sin_ry],
                     [0, cos_rx,  -sin_rx*cos_ry],
                     [0, sin_rx,   cos_rx*cos_ry]])

def MatAng2EulRPYVel(eul): #TODO: 검토 필요
    """ Get 3x3 matrix that converts angular velocity into time derivative of euler EUL_RPY
    Note that rx, eul[0] is not used
    """
    sin_ry = sin(eul[1])
    sin_rz = sin(eul[2])

    cos_ry = cos(eul[1])
    cos_rz = cos(eul[2])

    return np.array([[cos_rz/cos_ry,           sin_rz/cos_ry, 0],
                     [-sin_rz,                   cos_rz, 0],
                     [(cos_rz*sin_ry)/cos_ry, (sin_ry*sin_rz)/cos_ry, 1]])

def MatEulRPY2AngVel(eul): #TODO: 검토 필요
    """ Get 3x3 matrix that converts time derivative of euler EUL_RPY into angular velocity\n
    Note that rx, eul[0] is not used
    """
    sin_ry = sin(eul[1])
    sin_rz = sin(eul[2])

    cos_ry = cos(eul[1])
    cos_rz = cos(eul[2])

    return np.array([[cos_rz*cos_ry,     -sin_rz,  0],
                     [sin_rz*cos_ry,     cos_rz,  0],
                     [-sin_ry,                 0,  1]])

def convert_angvel_to_eulvel(v, eul, eulType = EUL_XYZ): #TODO: 검토 필요
    """
    INDY RPY VERSION NOT SUPPORTED YET
    """
    angvel = v[3:]
    if eulType == EUL_XYZ:
        E_inv = MatAng2EulXYZVel(eul)
    elif eulType == EUL_RPY:
        E_inv = MatAng2EulRPYVel(eul)
        
    eulvel = E_inv @ angvel
    
    res = np.zeros(6)
    res[0:3] = v[0:3]
    res[3:] = eulvel
    return res    

""" 
////////////////////// LIE GROUP //////////////////////
"""

def NearZero(z, tol = 1e-15):
    """Determines whether a scalar is small enough to be treated as zero

    :param z: A scalar input to check
    :return: True if z is close to zero, false otherwise

    Example Input:
        z = -1e-7
    Output:
        True
    """
    # v1.0
    # return abs(z) < 1e-6`
    # v1.1
    return abs(z) < tol

def VecToso3(omg):
    """Converts a 3-vector to an so(3) representation

    :param omg: A 3-vector
    :return: The skew symmetric representation of omg

    Example Input:
        omg = np.array([1, 2, 3])
    Output:
        np.array([[ 0, -3,  2],
                  [ 3,  0, -1],
                  [-2,  1,  0]])
    """
    return np.array([[0,      -omg[2],  omg[1]],
                     [omg[2],       0, -omg[0]],
                     [-omg[1], omg[0],       0]])
    
def so3ToVec(so3mat):
    """Converts an so(3) representation to a 3-vector

    :param so3mat: A 3x3 skew-symmetric matrix
    :return: The 3-vector corresponding to so3mat

    Example Input:
        so3mat = np.array([[ 0, -3,  2],
                           [ 3,  0, -1],
                           [-2,  1,  0]])
    Output:
        np.array([1, 2, 3])
    """
    return np.array([so3mat[2][1], so3mat[0][2], so3mat[1][0]])

def AxisAng3(expc3):
    """Converts a 3-vector of exponential coordinates for rotation into
    axis-angle form
    
        --> omgtheta -> omg, theta
    
    :param expc3: A 3-vector of exponential coordinates for rotation
    :return omghat: A unit rotation axis
    :return theta: The corresponding rotation angle

    Example Input:
        expc3 = np.array([1, 2, 3])
    Output:
        (np.array([0.26726124, 0.53452248, 0.80178373]), 3.7416573867739413)
    """
    theta = np.linalg.norm(expc3)
    omg = expc3 / theta
    
    return (omg, theta)

def MatrixExp3(so3mat):
    """Computes the matrix exponential of a matrix in so(3)
    
    refer to Rodrigues' Formula
    
    Rot(omghat, theta) = I +sin(theta)[omghat] + (1-cos(theta))[omghat].T @ [omghat] \inc SO(3)
    
    :param so3mat: A 3x3 skew-symmetric matrix
    :return: The matrix exponential of so3mat

    Example Input:
        so3mat = np.array([[ 0, -3,  2],
                           [ 3,  0, -1],
                           [-2,  1,  0]])
    Output:
        np.array([[-0.69492056,  0.71352099,  0.08929286],
                  [-0.19200697, -0.30378504,  0.93319235],
                  [ 0.69297817,  0.6313497 ,  0.34810748]])
    """
    omgtheta = so3ToVec(so3mat) # vector
    if NearZero(np.linalg.norm(omgtheta)):
        return np.eye(3)
    else:
        theta = AxisAng3(omgtheta)[1]
        omgmat = so3mat / theta
        
        res = np.eye(3) + \
              np.sin(theta) * omgmat + \
              (1 - np.cos(theta)) * omgmat @ omgmat
        
        return res
    
def MatrixLog3(R, tol = 1e-25):
    """Computes the matrix logarithm of a rotation matrix

    :param R: A 3x3 rotation matrix
    :return: The matrix logarithm of R

    Example Input:
        R = np.array([[0, 0, 1],
                      [1, 0, 0],
                      [0, 1, 0]])
    Output:
        np.array([[          0, -1.20919958,  1.20919958],
                  [ 1.20919958,           0, -1.20919958],
                  [-1.20919958,  1.20919958,           0]])
    """

    # print("LieGroupV01")
    acosinput = (np.trace(R) - 1.0) / 2.0 # which is from equation : tr R = r11 + r22 + r33 = 1+2cos(th)
    
    if acosinput >= 1.0: # R = I and others
        # print("R = I and others")
        # print(acosinput)
        return np.zeros((3, 3)) 
    elif acosinput <= -1.0: # if acosinput == -1 --> is the case : tr R = -1  then, theta is pi
        # and it's multiple solution there are three cases for it
        
        if not (NearZero(1 + R[2][2], tol = tol)): # R[2][2] = cos(th) + omega_3^2 * (1-cos(th))
            omg = (1.0 / np.sqrt(2 * (1 + R[2][2]))) \
                  * np.array([R[0][2], R[1][2], 1 + R[2][2]]) # eq 3.58
            
        elif not (NearZero(1 + R[1][1], tol = tol)):
            omg = (1.0 / np.sqrt(2 * (1 + R[1][1]))) \
                  * np.array([R[0][1], 1 + R[1][1], R[2][1]]) # eq 3.59
            
        else:
            omg = (1.0 / np.sqrt(2 * (1 + R[0][0]))) \
                  * np.array([1 + R[0][0], R[1][0], R[2][0]]) # eq 3.60
            
        return VecToso3(np.pi * omg) # sckew symmetric으로 바꾸기
    else:
        
        theta = np.arccos(acosinput)
        return theta / 2.0 / np.sin(theta) * (R - np.array(R).T)

def _MatrixLog3(R, tol = 1e-25):
    """Computes the matrix logarithm of a rotation matrix

    :param R: A 3x3 rotation matrix
    :return: The matrix logarithm of R

    Example Input:
        R = np.array([[0, 0, 1],
                      [1, 0, 0],
                      [0, 1, 0]])
    Output:
        np.array([[          0, -1.20919958,  1.20919958],
                  [ 1.20919958,           0, -1.20919958],
                  [-1.20919958,  1.20919958,           0]])
    """

    # print("LieGroupV01")
    acosinput = (np.trace(R) - 1.0) / 2.0 # which is from equation : tr R = r11 + r22 + r33 = 1+2cos(th)
    
    if acosinput >= 1.0: # R = I and others
        # print("R = I and others")
        # print(acosinput)
        return np.zeros((3, 3)) 
    elif acosinput <= -1.0: # if acosinput == -1 --> is the case : tr R = -1  then, theta is pi
        # and it's multiple solution there are three cases for it
        
        if not (NearZero(1 + R[2][2], tol = tol)): # R[2][2] = cos(th) + omega_3^2 * (1-cos(th))
            omg = (1.0 / np.sqrt(2 * (1 + R[2][2]))) \
                  * np.array([R[0][2], R[1][2], 1 + R[2][2]]) # eq 3.58
            
        elif not (NearZero(1 + R[1][1], tol = tol)):
            omg = (1.0 / np.sqrt(2 * (1 + R[1][1]))) \
                  * np.array([R[0][1], 1 + R[1][1], R[2][1]]) # eq 3.59
            
        else:
            omg = (1.0 / np.sqrt(2 * (1 + R[0][0]))) \
                  * np.array([1 + R[0][0], R[1][0], R[2][0]]) # eq 3.60
            
        return VecToso3(np.pi * omg) # sckew symmetric으로 바꾸기
    else:
        
        theta = np.arccos(acosinput)
        return theta / 2.0 / np.sin(theta) * (R - np.array(R).T)
def RpToTrans(R, p):
    """Converts a rotation matrix and a position vector into homogeneous
    transformation matrix

    :param R: A 3x3 rotation matrix
    :param p: A 3-vector
    :return: A homogeneous transformation matrix corresponding to the inputs

    Example Input:
        R = np.array([[1, 0,  0],
                      [0, 0, -1],
                      [0, 1,  0]])
        p = np.array([1, 2, 5])
    Output:
        np.array([[1, 0,  0, 1],
                  [0, 0, -1, 2],
                  [0, 1,  0, 5],
                  [0, 0,  0, 1]])
    """
    resMat = np.eye(4)
    resMat[0:3,0:3] = R; resMat[0:3,3] = p
    return resMat

def TransToRp(T):
    """Converts a homogeneous transformation matrix into a rotation matrix
    and position vector

    :param T: A homogeneous transformation matrix
    :return R: The corresponding rotation matrix,
    :return p: The corresponding position vector.

    Example Input:
        T = np.array([[1, 0,  0, 0],
                      [0, 0, -1, 0],
                      [0, 1,  0, 3],
                      [0, 0,  0, 1]])
    Output:
        (np.array([[1, 0,  0],
                   [0, 0, -1],
                   [0, 1,  0]]),
         np.array([0, 0, 3]))
    """
    T = np.array(T)
    return T[0: 3, 0: 3], T[0: 3, 3]

def VecTose3(V):
    """Converts a spatial velocity vector into a 4x4 matrix in se3

    The vector order : x y z wx wy wz
    
    :param V: A 6-vector representing a spatial velocity
    :return: The 4x4 se3 representation of V

    Example Input:
        V = np.array([1, 2, 3, 4, 5, 6])
    Output:
        np.array([[ 0, -3,  2, 4],
                  [ 3,  0, -1, 5],
                  [-2,  1,  0, 6],
                  [ 0,  0,  0, 0]])
    """
    
    return np.array([[    0,  -V[5],  V[4],  V[0]],
                     [ V[5],      0, -V[3],  V[1]],
                     [-V[4],   V[3],     0,  V[2]],
                     [    0,      0,     0,     1]])
def se3ToVec(se3mat):
    """ Converts an se3 matrix into a spatial velocity vector

    :param se3mat: A 4x4 matrix in se3
    :return: The spatial velocity 6-vector corresponding to se3mat

    Example Input:
        se3mat = np.array([[ 0, -3,  2, 4],
                           [ 3,  0, -1, 5],
                           [-2,  1,  0, 6],
                           [ 0,  0,  0, 0]])
    Output:
        np.array([1, 2, 3, 4, 5, 6])
    """
    lin = se3mat[0:3,3]
    rot = np.array([se3mat[2,1],se3mat[0,2],se3mat[1,0]])
    
    return np.hstack((lin,rot))
    
def adjMat(T):
    """Computes the adjMat representation of a homogeneous transformation
    matrix

    :param T: A homogeneous transformation matrix
    :return: The 6x6 adjMat representation [AdT] of T

    adjmat = [  R       0  ]
             [ [p]R     R  ]
             
    Js=adjMat(Tsb)@Jb
    Example Input:
        T = np.array([[1, 0,  0, 0],
                      [0, 0, -1, 0],
                      [0, 1,  0, 3],
                      [0, 0,  0, 1]])
    Output:
        np.array([[1, 0,  0, 0, 0,  0],
                  [0, 0, -1, 0, 0,  0],
                  [0, 1,  0, 0, 0,  0],
                  [0, 0,  3, 1, 0,  0],
                  [3, 0,  0, 0, 0, -1],
                  [0, 0,  0, 0, 1,  0]])
    """
    R = T[0:3,0:3]
    p = T[0:3,3]
    
    return np.r_[np.c_[R, np.dot(VecToso3(p), R)],
                 np.c_[np.zeros((3, 3)), R]]
    
def adjMat_s(V):
    """
    Computes the small adjoint representation (ad_V) of a twist vector V.
    
    :param V: 6x1 twist vector [v; w], 
              where v is linear velocity (3x1), w is angular velocity (3x1).
    :return: 6x6 matrix ad_V
    """
    v = V[0:3]
    w = V[3:6]

    admat = np.zeros((6,6))
    admat[0:3,0:3] = VecToso3(w)
    admat[3:6,0:3] = VecToso3(v)
    admat[3:6,3:6] = VecToso3(w)
    
    return admat

def ScrewToAxis(q, s, h):
    """Takes a parametric description of a screw axis and converts it to a
    normalized screw axis

    Sw = thetadot * shat
    Sv = (h*shat- shat X q) * thetadot
    
    :param q: A point lying on the screw axis
    :param s: A unit vector in the direction of the screw axis
    :param h: The pitch of the screw axis
    :return: A normalized screw axis described by the inputs

    Example Input:
        q = np.array([3, 0, 0])
        s = np.array([0, 0, 1])
        h = 2
    Output:
        np.array([0, 0, 1, 0, -3, 2])
    """
    res = np.zeros(6)
    res[0:3] = s
    res[3:] = h * s - np.cross(s, q)
    return res

def MatrixExp6(se3mat):
    """Computes the matrix exponential of an se3 representation of
    exponential coordinates
    
    :param se3mat: A matrix in se3
    :return: The matrix exponential of se3mat

    Example Input:
        se3mat = np.array([[0,          0,           0,          0],
                           [0,          0, -1.57079632, 2.35619449],
                           [0, 1.57079632,           0, 2.35619449],
                           [0,          0,           0,          0]])
    Output:
        np.array([[1.0, 0.0,  0.0, 0.0],
                  [0.0, 0.0, -1.0, 0.0],
                  [0.0, 1.0,  0.0, 3.0],
                  [  0,   0,    0,   1]])
    """
    se3mat = np.array(se3mat)
    
    omgtheta = so3ToVec(se3mat[0: 3, 0: 3])
    
    if NearZero(np.linalg.norm(omgtheta)):
        return np.array([[1, 0, 0, se3mat[0,3]],
                         [0, 1, 0, se3mat[1,3]],
                         [0, 0, 1, se3mat[2,3]],
                         [0, 0, 0,        1.0]])
    else:
        theta = AxisAng3(omgtheta)[1]
        omgmat = se3mat[0: 3, 0: 3] / theta
        
        resMat = np.eye(4)
        resMat[0:3,0:3] = MatrixExp3(se3mat[0:3,0:3])
        resMat[0:3,3] = (np.eye(3)*theta + \
                        (1-np.cos(theta))*omgmat + \
                        (theta-np.sin(theta)) * omgmat @ omgmat) @ se3mat[0:3,3]
        resMat[0:3,3] /= theta
        
        return resMat

        # return resMat

def MatrixLog6(T):
    """Computes the matrix logarithm of a homogeneous transformation matrix

    :param R: A matrix in SE3
    :return: The matrix logarithm of R

    Example Input:
        T = np.array([[1, 0,  0, 0],
                      [0, 0, -1, 0],
                      [0, 1,  0, 3],
                      [0, 0,  0, 1]])
    Output:
        np.array([[0,          0,           0,           0]
                  [0,          0, -1.57079633,  2.35619449]
                  [0, 1.57079633,           0,  2.35619449]
                  [0,          0,           0,           0]])
    """
    R, p = TransToRp(T)
    omgmat = MatrixLog3(R)
    if np.array_equal(omgmat, np.zeros((3, 3))):
        return np.array([[0, 0, 0, T[0,3]],
                         [0, 0, 0, T[1,3]],
                         [0, 0, 0, T[2,3]],
                         [0, 0, 0,      0]])
    else:
        theta = np.arccos((np.trace(R) - 1.0) / 2.0)
        resMat = np.zeros((4,4))
        resMat[0:3,0:3] = omgmat
        resMat[0:3,3] = (np.eye(3) - omgmat/2.0 \
                        + (1.0 / theta - 1.0 / np.tan(theta/2.0) / 2) * omgmat @ omgmat / theta) \
                        @ T[0:3,3]
        
                  
        return resMat    
      
def ProjectToSO3(mat):
    """Returns a projection of mat into SO(3)

    :param mat: A matrix near SO(3) to project to SO(3)
    :return: The closest matrix to R that is in SO(3)
    Projects a matrix mat to the closest matrix in SO(3) using singular-value
    decomposition (see
    http://hades.mech.northwestern.edu/index.php/Modern_Robotics_Linear_Algebra_Review).
    This function is only appropriate for matrices close to SO(3).

    Example Input:
        mat = np.array([[ 0.675,  0.150,  0.720],
                        [ 0.370,  0.771, -0.511],
                        [-0.630,  0.619,  0.472]])
    Output:
        np.array([[ 0.67901136,  0.14894516,  0.71885945],
                  [ 0.37320708,  0.77319584, -0.51272279],
                  [-0.63218672,  0.61642804,  0.46942137]])
    """
    U, s, Vh = np.linalg.svd(mat)
    R = np.dot(U, Vh)
    if np.linalg.det(R) < 0:
    # In this case the result may be far from mat.
        R[:, s[2, 2]] = -R[:, s[2, 2]]
    return R

def ProjectToSE3(mat):
    """Returns a projection of mat into SE(3)

    :param mat: A 4x4 matrix to project to SE(3)
    :return: The closest matrix to T that is in SE(3)
    Projects a matrix mat to the closest matrix in SE(3) using singular-value
    decomposition (see
    http://hades.mech.northwestern.edu/index.php/Modern_Robotics_Linear_Algebra_Review).
    This function is only appropriate for matrices close to SE(3).

    Example Input:
        mat = np.array([[ 0.675,  0.150,  0.720,  1.2],
                        [ 0.370,  0.771, -0.511,  5.4],
                        [-0.630,  0.619,  0.472,  3.6],
                        [ 0.003,  0.002,  0.010,  0.9]])
    Output:
        np.array([[ 0.67901136,  0.14894516,  0.71885945,  1.2 ],
                  [ 0.37320708,  0.77319584, -0.51272279,  5.4 ],
                  [-0.63218672,  0.61642804,  0.46942137,  3.6 ],
                  [ 0.        ,  0.        ,  0.        ,  1.  ]])
    """
    mat = np.array(mat)
    return RpToTrans(ProjectToSO3(mat[:3, :3]), mat[:3, 3])

def DistanceToSO3(mat):
    """Returns the Frobenius norm to describe the distance of mat from the
    SO(3) manifold

    :param mat: A 3x3 matrix
    :return: A quantity describing the distance of mat from the SO(3)
             manifold
    Computes the distance from mat to the SO(3) manifold using the following
    method:
    If det(mat) <= 0, return a large number.
    If det(mat) > 0, return norm(mat^T.mat - I).

    Example Input:
        mat = np.array([[ 1.0,  0.0,   0.0 ],
                        [ 0.0,  0.1,  -0.95],
                        [ 0.0,  1.0,   0.1 ]])
    Output:
        0.08835
    """
    if np.linalg.det(mat) > 0:
        return np.linalg.norm(np.dot(np.array(mat).T, mat) - np.eye(3))
    else:
        return 1e+9

def DistanceToSE3(mat):
    """Returns the Frobenius norm to describe the distance of mat from the
    SE(3) manifold

    :param mat: A 4x4 matrix
    :return: A quantity describing the distance of mat from the SE(3)
              manifold
    Computes the distance from mat to the SE(3) manifold using the following
    method:
    Compute the determinant of matR, the top 3x3 submatrix of mat.
    If det(matR) <= 0, return a large number.
    If det(matR) > 0, replace the top 3x3 submatrix of mat with matR^T.matR,
    and set the first three entries of the fourth column of mat to zero. Then
    return norm(mat - I).

    Example Input:
        mat = np.array([[ 1.0,  0.0,   0.0,   1.2 ],
                        [ 0.0,  0.1,  -0.95,  1.5 ],
                        [ 0.0,  1.0,   0.1,  -0.9 ],
                        [ 0.0,  0.0,   0.1,   0.98 ]])
    Output:
        0.134931
    """
    matR = np.array(mat)[0: 3, 0: 3]
    if np.linalg.det(matR) > 0:
        return np.linalg.norm(np.r_[np.c_[np.dot(np.transpose(matR), matR),
                                          np.zeros((3, 1))],
                              [np.array(mat)[3, :]]] - np.eye(4))
    else:
        return 1e+9

def TestIfSO3(mat, tol = 1e-3):
    """Returns true if mat is close to or on the manifold SO(3)

    :param mat: A 3x3 matrix
    :return: True if mat is very close to or in SO(3), false otherwise
    Computes the distance d from mat to the SO(3) manifold using the
    following method:
    If det(mat) <= 0, d = a large number.
    If det(mat) > 0, d = norm(mat^T.mat - I).
    If d is close to zero, return true. Otherwise, return false.

    Example Input:
        mat = np.array([[1.0, 0.0,  0.0 ],
                        [0.0, 0.1, -0.95],
                        [0.0, 1.0,  0.1 ]])
    Output:
        False
    """
    return abs(DistanceToSO3(mat)) < tol

def TestIfSE3(mat, tol = 1e-3):
    """Returns true if mat is close to or on the manifold SE(3)

    :param mat: A 3x3 matrix
    :return: True if mat is very close to or in SE(3), false otherwise
    Computes the distance d from mat to the SE(3) manifold using the
    following method:
    Compute the determinant of the top 3x3 submatrix of mat.
    If det(mat) <= 0, d = a large number.
    If det(mat) > 0, replace the top 3x3 submatrix of mat with mat^T.mat, and
    set the first three entries of the fourth column of mat to zero.
    Then d = norm(T - I).
    If d is close to zero, return true. Otherwise, return false.

    Example Input:
        mat = np.array([[1.0, 0.0,   0.0,  1.2],
                        [0.0, 0.1, -0.95,  1.5],
                        [0.0, 1.0,   0.1, -0.9],
                        [0.0, 0.0,   0.1, 0.98]])
    Output:
        False
    """
    return abs(DistanceToSE3(mat)) < tol

def FKinBody(M, Blist, thetalist):
    """Computes forward kinematics in the body frame for an open chain robot

    :param M: The home configuration (position and orientation) of the end-
              effector
    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param thetalist: A list of joint coordinates
    :return: A homogeneous transformation matrix representing the end-
             effector frame when the joints are at the specified coordinates
             (i.t.o Body Frame)

    Example Input:
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
        Blist = np.array([[0, 0, -1, 2, 0,   0],
                          [0, 0,  0, 0, 1,   0],
                          [0, 0,  1, 0, 0, 0.1]]).T
        thetalist = np.array([np.pi / 2.0, 3, np.pi])
    Output:
        np.array([[0, 1,  0,         -5],
                  [1, 0,  0,          4],
                  [0, 0, -1, 1.68584073],
                  [0, 0,  0,          1]])
    """
    T = np.array(M)
    for i in range(len(thetalist)):
        T = T @ MatrixExp6(VecTose3(Blist[:,i] * thetalist[i]))
    return T

def FKinSpace(M, Slist, thetalist):
    """Computes forward kinematics in the space frame for an open chain robot

    :param M: The home configuration (position and orientation) of the end-
              effector
    :param Slist: The joint screw axes in the space frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param thetalist: A list of joint coordinates
    :return: A homogeneous transformation matrix representing the end-
             effector frame when the joints are at the specified coordinates
             (i.t.o Space Frame)

    Example Input:
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
        Slist = np.array([[0, 0,  1,  4, 0,    0],
                          [0, 0,  0,  0, 1,    0],
                          [0, 0, -1, -6, 0, -0.1]]).T
        thetalist = np.array([np.pi / 2.0, 3, np.pi])
    Output:
        np.array([[0, 1,  0,         -5],
                  [1, 0,  0,          4],
                  [0, 0, -1, 1.68584073],
                  [0, 0,  0,          1]])
    """
    T = np.array(M)
    
    for i in range(len(thetalist) - 1, -1, -1):
        T = MatrixExp6(VecTose3(Slist[:,i] * thetalist[i])) @ T
    return T

'''
*** CHAPTER 5: VELOCITY KINEMATICS AND STATICS***
'''

def JacobianBody(Blist, thetalist):
    """Computes the body Jacobian for an open chain robot

    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param thetalist: A list of joint coordinates
    :return: The body Jacobian corresponding to the inputs (6xn real
             numbers)

    Example Input:
        Blist = np.array([[0, 0, 1,   0, 0.2, 0.2],
                          [1, 0, 0,   2,   0,   3],
                          [0, 1, 0,   0,   2,   1],
                          [1, 0, 0, 0.2, 0.3, 0.4]]).T
        thetalist = np.array([0.2, 1.1, 0.1, 1.2])
    Output:
        np.array([[-0.04528405, 0.99500417,           0,   1]
                  [ 0.74359313, 0.09304865,  0.36235775,   0]
                  [-0.66709716, 0.03617541, -0.93203909,   0]
                  [ 2.32586047,    1.66809,  0.56410831, 0.2]
                  [-1.44321167, 2.94561275,  1.43306521, 0.3]
                  [-2.06639565, 1.82881722, -1.58868628, 0.4]])
    """
    Jb = np.array(Blist).copy().astype(np.float)
    T = np.eye(4)
    for i in range(len(thetalist) - 2, -1, -1):
        T = T @ MatrixExp6(VecTose3(Blist[i,i+1] * -thetalist[i+1]))
        Jb[:,i] = adjMat(T) @ Blist[:,i]
    return Jb

def JacobianSpace(Slist, thetalist):
    """Computes the space Jacobian for an open chain robot

    :param Slist: The joint screw axes in the space frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param thetalist: A list of joint coordinates
    :return: The space Jacobian corresponding to the inputs (6xn real
             numbers)

    Example Input:
        Slist = np.array([[0, 0, 1,   0, 0.2, 0.2],
                          [1, 0, 0,   2,   0,   3],
                          [0, 1, 0,   0,   2,   1],
                          [1, 0, 0, 0.2, 0.3, 0.4]]).T
        thetalist = np.array([0.2, 1.1, 0.1, 1.2])
    Output:
        np.array([[  0, 0.98006658, -0.09011564,  0.95749426]
                  [  0, 0.19866933,   0.4445544,  0.28487557]
                  [  1,          0,  0.89120736, -0.04528405]
                  [  0, 1.95218638, -2.21635216, -0.51161537]
                  [0.2, 0.43654132, -2.43712573,  2.77535713]
                  [0.2, 2.96026613,  3.23573065,  2.22512443]])
    """
    Js = np.array(Slist).copy().astype(np.float)
    T = np.eye(4)
    for i in range(1, len(thetalist)):
        T = T @ MatrixExp6(VecTose3(Slist[:,i-1] * thetalist[i-1]))
        Js[:,i] = adjMat(T) @ Slist[:,i]
    return Js



'''
*** CHAPTER 6: INVERSE KINEMATICS ***
'''


""" =======================================================
                   # Functions for Mobile Manipulator
======================================================= """
def rotation_matrix_z(theta):
    
    return np.array([[1, 0, 0],
                    [0, np.cos(theta), -np.sin(theta)],
                    [0, np.sin(theta),  np.cos(theta)]])

def qbTose3(qb, z):
    return np.array([[np.cos(qb[0]), -np.sin(qb[0]),     0, qb[1]],
                    [np.sin(qb[0]),  np.cos(qb[0]),     0, qb[2]],
                    [            0,              0,     1,     z],
                    [            0,              0,     0,     1]]) 
    
def uToVb(u, F, dt=1.0):
    dtheta = u * dt
    return F @ dtheta  



""" =======================================================
                   # NOT USED YET
======================================================= """
def RELBASE(T, dist):
    raise Exception("NOT USED YET")
    T[0:3,0:3] = eulXYZ2r(dist[3:]) @ T[0:3,0:3]
    T[0:3,3] += dist[0:3]
    
def RELTOOL(T, dist):
    raise Exception("NOT USED YET")
    T[0:3,0:3] = T[0:3,0:3] @ eulXYZ2r(dist[3:])
    T[0:3,3] += dist[0:3]



""" =======================================================
                   Rotation Conversion
======================================================= """

def rodrigues(rvec):

    theta = np.linalg.norm(rvec)
    if theta < 1e-8:
        return np.eye(3)
    r = rvec / theta
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    R = cos_theta * np.eye(3) + (1 - cos_theta) * np.outer(r, r) + sin_theta * np.array([
        [0, -r[2], r[1]],
        [r[2], 0, -r[0]],
        [-r[1], r[0], 0]
    ])
    return R

def rmat2rvec(R):

    cos_theta = (np.trace(R) - 1) / 2.0
    sin_theta = np.sqrt(1 - cos_theta**2)

    theta = np.arccos(cos_theta)

    if np.abs(sin_theta) < 1e-8:
        return np.zeros(3)

    rvec = np.array([
        (R[2, 1] - R[1, 2]) / (2 * sin_theta),
        (R[0, 2] - R[2, 0]) / (2 * sin_theta),
        (R[1, 0] - R[0, 1]) / (2 * sin_theta)
    ]) * theta

    return rvec

def quat2rmat(q):
    w,x,y,z=q
    return np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
                    [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
                    [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]])

def rmat2quat(R):
    q = np.empty(4)
    q[0] = 0.5 * np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2])
    q[1] = (R[2, 1] - R[1, 2]) / (4 * q[0])
    q[2] = (R[0, 2] - R[2, 0]) / (4 * q[0])
    q[3] = (R[1, 0] - R[0, 1]) / (4 * q[0])
    return q

def rvec2quat(rvec):
    return rmat2quat(rodrigues(rvec))

""" =======================================================
                   mean rotation
======================================================= """

def mean_quat(q:np.ndarray):
    mean_q = np.array([np.mean(q[:,i]) for i in range(4)])
    return mean_q / np.linalg.norm(mean_q)  

def mean_rmat(R_array:np.ndarray):
    return quat2rmat(mean_quat(np.array([rvec2quat(rmat2rvec(R)) for R in R_array])))

def mean_tvec(t_array:np.ndarray):
    return np.array([np.mean(t_array[:,0]), np.mean(t_array[:,1]), np.mean(t_array[:,2])])

def mean_tform(T_list):
    R_array = np.array([ T[0:3,0:3] for T in T_list])
    t_array = np.array([ T[0:3,3] for T in T_list])
    return np.r_[np.c_[mean_rmat(R_array), mean_tvec(t_array)], [[0, 0, 0, 1]]]


""" =======================================================
                   computation of velocity 
======================================================= """
def compute_vel(x_cur, x_pre):
    bTe_cur = pose2tr(x_cur); bTe_pre = pose2tr(x_pre)
    dT = inv_tform(bTe_pre) @ bTe_cur
    return adjMat(inv_tform(dT)) @ se3ToVec(MatrixLog6(dT)) # equals to xd = -se3ToVec(MatrixLog6(inv_dT))

def dx2tform(vc):
    return MatrixExp6(VecTose3(vc))

""" =======================================================
                   Point Cloud
======================================================= """

def kabschUmeyama(X, Y):
    """
    두 점군 X(3×m), Y(3×m)의 대응점을 이용해
    최소제곱 rigid 변환 T=[R|t] (3D 회전+이동) 추정.
    Kabsch/Umeyama 알고리즘 구현.
    반환값: 4×4 동차변환행렬
    
    [SIMPLE EXAMPLE]
    # Example usage with dummy data (replace with actual data for the real calculation)
    # X and Y should be 3xm matrices where each column represents the position vectors x_j and y_j
    # For demonstration purposes, we will just use random points which should not be considered as real data
    
    X_dummy = np.random.rand(3, 10)  # Replace with real data
    Y_dummy = np.random.rand(3, 10)  # Replace with real data

    # Estimate the transformation matrix with dummy data
    T_estimated = kabschUmeyama(X_dummy, Y_dummy)

    [ROBOT]
    robot = RA004()
    T      = robot.get_fk(gen_random_joint_angles(6))                       # 기준 변환

    X_dummy, Y_dummy = [], []
    for _ in range(10):
        X_dummy.append(robot.get_fk(gen_random_joint_angles(6))[0:3,3])
        Y_dummy.append((T @ np.hstack((X_dummy[-1], 1)))[0:3])

    T_est = kabschUmeyama(np.array(X_dummy).T, np.array(Y_dummy).T)         # 추정 변환
    print(np.linalg.norm(tr2delta(T, T_est)))                               # 오차

    """
    
    # 1) 각 점군의 무게중심 계산 → 원점으로 이동
    centroid_X = np.mean(X, axis=1, keepdims=True)
    centroid_Y = np.mean(Y, axis=1, keepdims=True)
    X_centered = X - centroid_X
    Y_centered = Y - centroid_Y
    
        
    # 2) 상관행렬 H = Σ X_i' Y_i'^T (벡터화 없이 행렬계산)
    H = X_centered @ Y_centered.T
       
    
    # 3) SVD(H) = U Σ Vᵀ → 최적 회전 R = V Uᵀ
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    # det(R) < 0 이면 반사 포함 → 마지막 축 뒤집어 교정
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # 4) 이동벡터 t = μ_Y − R μ_X
    t = centroid_Y - R @ centroid_X
    
    # 5) Assemble the transformation matrix T
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t[:, 0]
    
    return T


def cutOutSO3(T):
    for ii in range(3):
        for jj in range(3):
            if np.abs(T[ii,jj]) > 1.0:
                T[ii,jj] = np.sign(T[ii,jj]) *1.0
    return T