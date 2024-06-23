import numpy as np

def TransformIMUvalue(IMU_1: dict, IMU_2: dict, delta_t: float) -> dict:

    m_struct = 3

    def InertieMomentum(a, b, m_tot):
        I = (1/5) * m_tot * (np.square(a) + np.square(b))
        return I
    
    Ix = InertieMomentum(a=0.75, b=0.75, m_tot=m_struct)
    Iy = InertieMomentum(a=3, b=0.75, m_tot=m_struct)
    Iz = InertieMomentum(a=3, b=0.75, m_tot=m_struct)

    # TODO(1): faire delta omega en valeur absolue ? 

    # expression des moments (si c'est ok)
    M_roulis = Ix * (np.deg2rad(IMU_1['v_p']) - np.deg2rad(IMU_2['v_p']))/delta_t
    M_tangage = Iy * (np.deg2rad(IMU_1['v_q']) - np.deg2rad(IMU_2['v_q']))/delta_t
    M_lacet = Iz * (np.deg2rad(IMU_1['v_r']) - np.deg2rad(IMU_2['v_r']))/delta_t

    # expression des angles d'Euler
    a_x = np.mean([[IMU_1['a_x'], IMU_2['a_x']]])
    a_y = np.mean([[IMU_1['a_y'], IMU_2['a_y']]])
    a_z = np.mean([[IMU_1['a_z'], IMU_2['a_z']]])

    roll = np.arctan(a_y/a_z)
    pitch = np.arcsin(a_x/9.80665)

    # expression des forces
    Fx = m_struct * a_x
    Fy = m_struct * a_y
    Fz = m_struct * a_z

    dynamique = {

        'momentum': {
            'M_roulis': M_roulis,
            'M_tangage': M_tangage,
            'M_lacet': M_lacet,
        },

        'orientation': {
            'roll': roll,
            'pitch': pitch,
        },

        'force': {
            'Fx': Fx,
            'Fy': Fy,
            'Fz': Fz
        },
    }

    return dynamique