import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.signal import StateSpace, lsim
import matplotlib.pyplot as plt
import scipy.io
import os

class FlightControl():

    """
    Principe de base de la classe :
        Convergence des moments de tangage et de roulis.
    """

    def __init__(self, matrice, X_equilibre, data_dict, data_path) -> None:

        # matrice du modèle dynamique
        self.A = matrice['A']
        self.B = matrice['B']
        self.C = matrice['C']
        self.D = matrice['D']
        self.U = matrice['U']

        self.CG_total, self.CG_motor, self.CG_components = self.CG(data_path)

        # point d'équilibre à l'instant t
        self.X_eq = X_equilibre

        # caractéristique du modèle
            # inertie
        self.Inertie = data_dict['I']
            # masse
        self.mass_dirigeable = data_dict['mass_dirigeable']
        self.mass_motor = data_dict['mass_motor']
        self.mass_grappin = data_dict['mass_grappin']
        self.mass_ballast = data_dict['mass_ballast']
            # temps simulation
        self.T_sim = data_dict['T_sim']
            # gravity
        self.gravity = data_dict['gravity']

        # sortie du système
        try:
            self.T, self.yout, self.xout = self.LQR_opti()
            self.plotOutPut()
        except Exception as Error:
            print(f'ERROR : {Error}')

    def CG(self, data_path):
        CG_data = scipy.io.loadmat(os.path.join(data_path, 'CG_data.mat'))

        CG_total = CG_data['CG_total'][0].T
        CG_motor = CG_data['CG_motor'].T
        CG_components = CG_data['CG_components'].T

        return CG_total, CG_motor, CG_components

    # expression de la norme de propulsion
    def thrust_RPM(self, perc_RPM):
        return 1.1081*1e-4 * np.square(perc_RPM) + 0.092 * perc_RPM - 0.0934
            
    # méthode LQR : U(t) = -KX(t)
    def LQR_opti(self):

        Q = np.eye(self.A.shape[0])
        R = np.eye(self.B.shape[0])

        P = solve_continuous_are(self.A, self.B, Q, R)
        self.K = np.linalg.inv(R) @ self.B.T @ P

        # simulation en boucle fermé du modèle
        A_BF = self.A - self.B @ self.K

        # simulation de l'espace d'état
        StateSpace_sim = StateSpace(A_BF, self.B, self.C, self.D)
        T_sim, yout, xout = lsim(StateSpace_sim, U=np.ones((len(self.T_sim), 3)), T=self.T_sim, X0=self.X_eq)
        return T_sim, yout, xout
    
    # forces appliquées sur le système
    def force_expression(self, alpha, perc_RPM):
        forces = np.zeros((3, 13))
        
        # force de propulsion des moteurs
        for index, (ang, RPM) in enumerate(zip(alpha.values(), perc_RPM.values())):
            forces[:, index] = self.thrust_RPM(RPM) * np.array([np.cos(ang), 0, np.sin(ang)]).reshape((3, ))

        # force de poids des systèmes moteurs
        for index in range(len(alpha.values())):
            forces[:, index + 4] = - self.mass_motor * self.gravity * np.array([0, 0, 1]).reshape((3, ))
        
        # force de poids du grappin
        forces[:, 8] = - self.mass_grappin * self.gravity * np.array([0, 0, 1]).reshape((3, ))

        # force de poids des ballasts
        for index in range(len(alpha.values())):
            forces[:, index + 9] = - self.mass_ballast[index] * self.gravity * np.array([0, 0, 1]).reshape((3, ))

        return forces
    
    # moments des forces appliquées sur le système
    def moment_expression(self, forces):
        # distance entre le centre de poussée des moteurs et le centre de gravité du dirigeable
        d_CG_motor_CG = np.array([self.CG_total - self.CG_motor[:, index] for index in range(len(self.CG_motor) + 1)]).T

        # distance entre le centre de gravité des systèmes moteurs et le centre de gravité du dirigeable
        d_CG_sysmot_CG = np.array([self.CG_total - self.CG_components[:, index] for index in range(len(self.CG_components) + 1)]).T
        
        distance = np.concatenate((d_CG_motor_CG, d_CG_sysmot_CG), axis=1)
        try:
            self.moments = np.array([
                (-1 if index in [1, 2] else 1) * np.cross(distance[:, index], forces[:, index]) for index in range(distance.shape[1])
            ]).T
            return self.moments
        
        except Exception as error:
            print(error)

    # target M_roulis & M_theta
    def opt_Moment(self, X):
            # expression des moments via le PDF
        Mroulis_dyn = np.sum(self.moments[:, 1])
        Mtangage_dyn = np.sum(self.moments[:, 2])

            # construction des moments via le modèle MCEE
        U = - self.K @ X

        dp = self.A[0, :] @ X + self.B[0, :] * U
        dq = self.A[1, :] @ X + self.B[1, :] * U

        # reconstruction du moment de roulis
        Mroulis_MCEE = self.Inertie['I_x'] * dp[0] + (self.Inertie['I_z'] - self.Inertie['I_y']) * X[1] * X[2]
        # reconstruction du moment de tangage
        Mtangage_MCEE = self.Inertie['I_y'] * dq[1] + (self.Inertie['I_x'] - self.Inertie['I_z']) * X[0] * X[2]

        return np.array([np.abs(Mroulis_dyn + Mroulis_MCEE), np.abs(Mtangage_dyn + Mtangage_MCEE)]).T
    
    def objectif(self, params, X, alpha, perc_RPM):
        perc_RPM = {i: params[i] for i in range(4)}
        alpha = {i: params[i+4] for i in range(4)}

        forces = self.force_expression(alpha, perc_RPM)
        self.moment_expression(forces)
        moments = self.opt_Moment(X)
        return np.sum(moments)

    def plotOutPut(self):
        # dX(t)/dt
        dp = np.gradient(self.xout[:, 0], self.T)
        dq = np.gradient(self.xout[:, 1], self.T)
        dr = np.gradient(self.xout[:, 2], self.T)

        # X(t)
        p = self.xout[:, 0]
        q = self.xout[:, 1]
        r = self.xout[:, 2]
        
        # reconstruction du moment de roulis
        M_roulis = self.Inertie['I_x'] * dp + (self.Inertie['I_z'] - self.Inertie['I_y']) * q * r
        # reconstruction du moment de tangage
        M_tangage = self.Inertie['I_y'] * dq + (self.Inertie['I_x'] - self.Inertie['I_z']) * p * r

        fig, ax = plt.subplots(3, 1, figsize=(12, 10))

        # Tracé du moment de roulis
        ax0 = ax[0]
        ax0.plot(self.T, M_roulis, label=r'$M_{\phi}$', c='navy')
        ax0.legend()
        ax0.set_title('Évolution du moment de roulis en fonction du temps')
        ax0.set_ylabel('Moment [Nm]')
        ax0.set_xlabel('Temps [s]')
        ax0.grid('on')

        # Tracé du moment de tangage
        ax1 = ax[1]
        ax1.plot(self.T, M_tangage, label=r'$M_{\theta}$', c='navy')
        ax1.legend()
        ax1.set_title('Évolution du moment de tangage en fonction du temps')
        ax1.set_ylabel('Moment [Nm]')
        ax1.set_xlabel('Temps [s]')
        ax1.grid('on')

        # Tracé des deux moments ensemble
        ax2 = ax[2]
        ax2.plot(self.T, M_roulis, label=r'$M_{\phi}$', c='navy')
        ax2.plot(self.T, M_tangage, label=r'$M_{\theta}$', c='red')
        ax2.legend()
        ax2.set_ylabel('Moment [Nm]')
        ax2.set_xlabel('Temps [s]')
        ax2.set_title('Évolution des moments de roulis et de tangage en fonction du temps')
        ax2.grid('on')

        # Affichage de la figure
        plt.tight_layout()
        plt.show()
