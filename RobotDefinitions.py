import numpy as np
from GNKinematics import kinematics

Meca500_kin = kinematics.kinematics.from_home_positions(
    v0=np.array([0, 0, 135.0]),
    v1=np.array([0, 0, 270.0]),
    v2=np.array([60, 0, 308]),
    v3=np.array([120, 0, 308]),
    v4=np.array([190, 0, 308]),
    motor_limits=np.array([[-1750, 1750], [-700, 900], [-1350, 700],
                            [-1700, 1200], [-900, 1150], [-36000, 36000]]),
    centre_offset=[0, 0, 0],
    tool_offset=[0, 0, -13.35],
    strategy='minimum_movement',
    weighting=[6, 5, 4, 3, 2, 1],
)

GP225_kin = kinematics.kinematics.from_home_positions(
    v0=np.array([325, 0, 650]),
    v1=np.array([325, 0, 1800]),
    v2=np.array([1364.7, 0, 2100]),
    v3=np.array([1550.6, 0, 2100]),
    v4=np.array([1798, 0, 2100]),
    motor_limits=np.array([[-180, 180], [-60, 76], [-86, 90],
                            [-360, 360], [-125, 125], [-360, 360]]),
    centre_offset=[0, 0, 0],
    tool_offset=[0, 0, 0],
    strategy='minimum_movement',
    weighting=[6, 5, 4, 3, 2, 1],
)

GP180_120_kin = kinematics.kinematics.from_home_positions(
    v0=np.array([325, 0, 650]),
    v1=np.array([325, 0, 1800]),
    v2=np.array([1637.6, 0, 2100]),
    v3=np.array([1915, 0, 2100]),
    v4=np.array([2140, 0, 2100]),
    motor_limits=np.array([[-180, 180], [-60, 76], [-86, 90],
                            [-360, 360], [-130, 130], [-360, 360]]),
    centre_offset=[0, 0, 0],
    tool_offset=[0, 0, 0],
    strategy='minimum_movement',
    weighting=[6, 5, 4, 3, 2, 1],
)

GP280_kin = kinematics.kinematics.from_home_positions(
    v0=np.array([285, 0, 650]),
    v1=np.array([285, 0, 1800]),
    v2=np.array([1056.2, 0, 2050]),
    v3=np.array([1300, 0, 2050]),
    v4=np.array([1550, 0, 2050]),
    motor_limits=np.array([[-180, 180], [-60, 76], [-77.8, 197],
                            [-360, 360], [-125, 125], [-360, 360]]),
    centre_offset=[0, 0, 0],
    tool_offset=[0, 0, 0],
    strategy='minimum_movement',
    weighting=[6, 5, 4, 3, 2, 1],
)
