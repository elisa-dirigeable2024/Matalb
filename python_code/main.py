import numpy as np
import os
import scipy.io
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import FlightControlClass

mass = 3

# expression des moments d'inerties
a = 3
b = 0.75 
c = 0.75

I_x = (1/5) * mass * (np.square(b) + np.square(c))
I_y = (1/5) * mass * (np.square(a) + np.square(c))
I_z = (1/5) * mass * (np.square(a) + np.square(b))

I = {
    'I_x': I_x,
    'I_y': I_y,
    'I_z': I_z
}

X_0 = np.array([0.1, 0.5, 0.1])

A = np.array([
    [0, X_0[2] * (I['I_y'] - I['I_z'])/I['I_x'], X_0[1] * (I['I_y'] - I['I_z'])/I['I_x']],
    [X_0[2] * (I['I_z'] - I['I_x'])/I['I_y'], 0, X_0[0] * (I['I_z'] - I['I_x'])/I['I_y']],
    [X_0[1] * (I['I_x'] - I['I_y'])/I['I_z'], X_0[0] * (I['I_x'] - I['I_y'])/I['I_z'], 0]
])

B = np.diag([1/I['I_x'], 1/I['I_y'], 1/I['I_z']])

C = np.array([
    [1, 0, 0],
    [0, 1, 0]
])

T_sim = np.linspace(0, 25, 500)

matrice = {
    'A': A,
    'B': B,
    'C': C,
    'D': np.zeros((2, 3)),
    'U': np.ones((len(T_sim), 1))
}

I = {
    'I_x': I_x,
    'I_y': I_y,
    'I_z': I_z
}

X_0 = np.array([0.1, 0.1, 0.1])

# angle de rotation des moteurs
alpha_1 = np.pi/2
alpha_2 = -np.pi/4
alpha_3 = -np.pi/2
alpha_4 = np.pi/4

angle_dict = {
    'alpha_1': alpha_1,
    'alpha_2': alpha_2,
    'alpha_3': alpha_3,
    'alpha_4': alpha_4,
}

perc_RPM = {
    'perc_RPM_1': 50,
    'perc_RPM_2': 50,
    'perc_RPM_3': 50,
    'perc_RPM_4': 50,
}

# ensemble des caractéristiques de la simulation
data_dict = {
    'I': I,
    'mass_dirigeable': 3,
    'mass_motor': 0.1415,
    'mass_grappin': 0.75,
    'mass_ballast': [0.25, 0.25, 0.25, 0.25],
    'T_sim': T_sim,
    'gravity': 9.80665
}

# dossier data
data_path = os.path.join(os.getcwd(), 'data')
CG_data = scipy.io.loadmat(os.path.join(data_path, 'CG_data.mat'))

# utilisation du modèle de simulation
FC = FlightControlClass.FlightControl(matrice, X_0, data_dict, data_path)

# TODO(1) mettre les calculs du PFD avec opt_Moment

# expression des forces
forces = FC.force_expression(angle_dict, perc_RPM)
moments = FC.moment_expression(forces)

# Simulation sur une période de temps

params_initial = np.array([50.0, 50.0, 50.0, 50.0, alpha_1, alpha_2, alpha_3, alpha_4])

time_steps = 100
dt = 0.1
X = X_0
params_values = []
M_roulis_values = []
M_tangage_values = []
time_values = []

for t in range(time_steps):
    # Optimisation des moments
    try:
        resultat = minimize(FC.objectif, params_initial, args=(X, None, None), bounds=[(0, 100)]*4 + [(0, 180)]*4)
    except Exception as Error:
        print(Error)

        break
    params_optimise = resultat.x
    params_values.append(params_optimise)

    # Moments optimisés
    moments = FC.opt_Moment(X)
    M_roulis_values.append(moments[0])
    M_tangage_values.append(moments[1])
    
    time_values.append(t * dt)

    # Mise à jour de l'état
    U = -FC.K @ X
    dX = A @ X + np.dot(B, U)
    X = X + dX * dt

# Conversion en tableaux numpy pour faciliter l'affichage
params_values = np.array(params_values)
rpm_values = params_values[:, :4]
angles_values = params_values[:, 4:]
M_roulis_values = np.array(M_roulis_values)
M_tangage_values = np.array(M_tangage_values)

# Affichage des résultats
plt.figure(figsize=(12, 10))

# Graphique du moment de roulis
plt.subplot(4, 1, 1)
plt.plot(time_values, M_roulis_values, label='Moment de Roulis', c='navy')
plt.xlabel('Temps (s)')
plt.ylabel('Mroulis')
plt.title('Évolution du Moment de Roulis')
plt.legend()

# Graphique du moment de tangage
plt.subplot(4, 1, 2)
plt.plot(time_values, M_tangage_values, label='Moment de Tangage', c='navy')
plt.xlabel('Temps (s)')
plt.ylabel('Mtangage')
plt.title('Évolution du Moment de Tangage')
plt.legend()

# Graphique des RPM par moteur
plt.subplot(4, 1, 3)
for i in range(rpm_values.shape[1]):
    plt.plot(time_values, rpm_values[:, i], label=f'Moteur {i+1}')
plt.xlabel('Temps (s)')
plt.ylabel('% RPM')
plt.title('Évolution des % RPM par Moteur')
plt.legend()

# Graphique des angles par moteur
plt.subplot(4, 1, 4)
for i in range(angles_values.shape[1]):
    plt.plot(time_values, angles_values[:, i], label=f'Angle Moteur {i+1}')
plt.xlabel('Temps (s)')
plt.ylabel('Angle (°)')
plt.title('Évolution des Angles par Moteur')
plt.legend()

plt.tight_layout()
plt.show()
