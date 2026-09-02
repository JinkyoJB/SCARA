
# -*- coding: utf-8 -*-
import numpy as np

# THRESHOLD =====================================\
TINY = 1e-6

# Math =====================================\
PI_TWO = 2* np.pi

INF = np.inf

GRAV_ACC = -9.80665
DEFAULT_GRAV_ACC_VEC = np.array([0, 0, GRAV_ACC])

# UNIT CONVERSION ========================================
RAD2DEG = 180.0 / np.pi
DEG2RAD = np.pi / 180.0

DEG2RAD_POSE = np.array([1, 1, 1, DEG2RAD, DEG2RAD, DEG2RAD])

DPS2RPM = 60/360
RPM2DPS = 360/60

METER2MM = 1000.0
MM2METER = 0.001

# KINEMATIC MODEL ============================================

# --- ROBOT CONFIG TYPE
SPHERICAL_WRIST = 'spherical_wrist'
COBOT_WRIST = 'cobot_wrist'

# --- KINE TYPE
DH = 'DH'
HAYATI = 'HAYATI'
SIX_PARA = "SIX_PARA"
GEN_PARA = "GEN_PARA"
POE = "POE"
XYZ_EULXYZ = 'XYZ_EULXYZ'
XYZ_EULRPY = 'XYZ_EULRPY'

# --- EULER NOTATION
EUL_XYZ = 'EUL_XYZ'
EUL_RPY = 'EUL_RPY'

# --- KINE MODEL TABLE
IDX_LINK_FUNC = 0
IDX_INV_FUNC = 1
IDX_PARA_NUM = 2

# --- IK METHOD
IK_METHOD_DIFF_TFORM = "differential transform based ik"
IK_METHOD_EULER = "euler based ik"

# --- JACOBIAN
GEO_JAC = 'geometrical_jacobian'
ANL_JAC = 'analytical_jacobian'
BODY_JAC = 'body_jacobian'

# TRAJECTORY  ============================================
REF = 'reference'
FEED = 'feedback'

# JOINT TYPE  ============================================
REV = 'REV'
PRIS = 'PRIS'
FIX = 'FIX'

# COMPONENTS  ============================================
HD = 'HD'
RV = 'RV'
CRB = 'CRB'
BB = 'BB'


# ETC =======================================================
UNDEFINED = 'Undefined'


