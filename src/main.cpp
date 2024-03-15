#include <iostream>
#include <armadillo>
#include <cmath>
#include <tuple>
#include <typeinfo>

using namespace std;

/* Paramètres d'entrée */
double rho = 1.225; // kg/m^3
double pi = M_PI;
double g = 9.81;  // m/s^2
double radius = (12.7/2)*1e-2; // m
double area = pi * radius * radius; // m^2
double M_struct = 3.0; // kg
double V_tot = 7.0; // m^3

/* Vecteur dans la base du dirigeable */
arma::colvec x_m = {1, 0, 0};
arma::colvec y_m = {0, 1, 0};
arma::colvec z_m = {0, 0, 1};

/* Vecteur dans la base fixe */
arma::colvec x_0 = {1, 0, 0};
arma::colvec y_0 = {0, 1, 0};
arma::colvec z_0 = {0, 0, 1};

/* Vecteur nul*/
arma::colvec zeros = {0, 0, 0};

/* Matrice de rotation */

arma::mat rotation_matrix(double phi, double theta, double psi){

    arma::mat R_x = {{1, 0, 0}, 
                     {0, cos(phi), sin(phi)},
                     {0, -sin(phi), cos(phi)}};

    arma::mat R_y = {{cos(theta), 0, sin(theta)},
                     {0, 1, 0},
                     {-sin(theta), 0, cos(theta)}};

    arma::mat R_z = {{cos(psi), sin(psi), 0},
                     {-sin(psi), cos(psi), 0},
                     {0, 0, 1}};

    arma::mat R = R_x * R_y * R_z;

    return R;

}

arma::mat rotation_matrix_motor(double alpha){

    arma::mat R_y = {{cos(alpha), 0, sin(alpha)},
                     {0, 1, 0},
                     {-sin(alpha), 0, cos(alpha)}};

    return R_y;
    
}

/* Déplacement du centre de gravité */

double moved_gravity_center(arma::rowvec vect_mass, arma::rowvec vect_pos){

    double total_mass = arma::accu(vect_mass);
    double somme = 0;

    for(unsigned int i =0; i < vect_pos.n_elem; i++){ // itération sur le nombre d'item du vecteur
        somme = somme + vect_mass(i)*vect_pos(i);
    }

    double new_position = somme / total_mass;
    return new_position;

}

/* Fonction expressoin des composantes des forces */

arma::colvec buoyancy_force(double rho, double V_tot, arma::colvec vector){
    double buoyancy_norm = rho * V_tot * g;
    arma::colvec F_b = buoyancy_norm * vector;
    return F_b;
}

arma::colvec weigth_force(double m, arma::colvec vector){
    double weight_norm = m * g;
    arma::colvec F_p = weight_norm * vector;
    return F_p;
}

arma::colvec prop_force(double constant, double rho, double RPM, double radius, arma::colvec vector){
    double d = 2 * radius;
    // constante étant le coeff de portance adimensionalisé de l'hélice
    double prop_norm = constant * rho * pow(RPM/60, 2) * pow(d, 4);
    arma::colvec F_prop = prop_norm * vector;
    return F_prop;

    /*
    Autre façon de calculer la force :
    F = b * (w_i)^2
    avec b   : coeff de portance (en kg/m/s ?)
         w_i : vitesse de rotation du moteur (en rad/s)
    */

}

/* Fonction pour étude du système moteur */

double rotor_speed(double radius, int RPM){
    double v_rotor = 2 * pi * radius * (RPM/60);
    return v_rotor;
}


double mass_flow(double rho, double area, double speed){
    double m_dot = rho * area * speed; // [m/s]
    return m_dot;
}


double cruise_speed(double area, double rho, double M_struct){
    double v_cruise = sqrt((M_struct * g)/ (2 * rho * area));
    return v_cruise;
}

/* Fonction pour changement de base */

arma::colvec change_of_base_force(arma::colvec force, arma::mat rotation_matrix){

    arma::mat inv_rotation_matrix = arma::inv(rotation_matrix);
    arma::colvec new_force = force * inv_rotation_matrix;

    /* Opération réaliser : M_{1*3} * M_{3*3} = M_{1*3} */

    return new_force;
}

arma::colvec change_of_base_moment(arma::colvec moment, arma::mat rotation_matrix){

    arma::mat inv_rotation_matrix = arma::inv(rotation_matrix);
    arma::colvec new_moment = moment * inv_rotation_matrix;

    /* Opération réaliser : M_{1*3} * M_{3*3} = M_{1*3} */

    return new_moment;
}

/* Déterminer la distance entre deux points */

arma::colvec distance(arma::colvec a, arma::colvec b){
    
    /*Vecteur AB = x_i_{B} - x_i_{A} */

    arma::colvec d = b - a;
    return  d;

}

arma::colvec moment_translation(arma::colvec Force, arma::colvec distance, arma::colvec Moment){
    /* Théorème du moment */
    arma::colvec new_moment = Moment + arma::cross(distance, Force);
    return new_moment;
}

/* Sous-Système moteur */
arma::mat sub_system_motor(double alpha, double RPM, arma::rowvec motor_position, arma::rowvec struct_position, arma::rowvec sys_position, arma::colvec gravity_center_position){
    /* Données du systèmes moteurs */

    double system_mass = 0.1;
    double motor_mass = 37*1e-3;
    double blade_const = 0.83;

    double Fx, Fy, Fz;
    double Mx, My, Mz;

    /* Appel de la matrice de rotation du moteur */
    arma::mat R = rotation_matrix_motor(alpha);
    arma::mat R_inv = arma::inv(R);

    /* Vecteur z_r et x_r */
    arma::colvec x_r = R_inv * x_m;
    arma::colvec z_r = R_inv * z_m;

    /* Forces appliquées sur le système */
    arma::colvec P_S = weigth_force(system_mass, -1 * z_m);
    arma::colvec P_M = weigth_force(motor_mass, -1 * z_m);
    arma::colvec F_prop = prop_force(blade_const, rho, RPM, radius, z_r);

    /* Expression des projections des forces */
    Fx = P_S(0) + P_M(0) + F_prop(0);
    Fy = P_S(1) + P_M(1) + F_prop(1);
    Fz = P_S(2) + P_M(2) + F_prop(2);

    /* Moments appliquées sur le système */

    arma::colvec position_motor = {motor_position(0), motor_position(1), motor_position(2)};
    arma::colvec position_sys = {sys_position(0), sys_position(1), sys_position(2)};
    arma::colvec position_struct = {struct_position(0), struct_position(1), struct_position(2)};

    /* Expression des moments au centre de gravité de la structure moteur [SM] */
    arma::colvec distance_motor_system = distance(position_motor, position_sys);
    arma::colvec distance_struct_system = distance(position_struct, position_sys);

    arma::colvec M_P_M = moment_translation(P_M, distance_motor_system, zeros);
    arma::colvec M_F_prop = moment_translation(F_prop, distance_motor_system, zeros);
    arma::colvec M_P_S = moment_translation(P_S, distance_struct_system, zeros);

    /* Expression des moments au centre de gravité du dirigeable [G] */
    arma::colvec distance_system_dirigible = distance(position_sys, gravity_center_position);

    arma::colvec M_P_M_G = moment_translation(P_M, distance_system_dirigible, M_P_M);
    arma::colvec M_F_prop_G = moment_translation(F_prop, distance_system_dirigible, M_F_prop);
    arma::colvec M_P_S_G = moment_translation(P_S, distance_system_dirigible, M_P_S);

    /* Expression des projections des moments */
    Mx = M_P_M_G(0) + M_F_prop_G(0) + M_P_S_G(0);
    My = M_P_M_G(1) + M_F_prop_G(1) + M_P_S_G(1);
    Mz = M_P_M_G(2) + M_F_prop_G(2) + M_P_S_G(2);

    arma::mat M_mot_in_G = {{Fx, Mx},
                            {Fy, My},
                            {Fz, Mz}};

   return M_mot_in_G;

}

/* Sous-Système grappin */

// tuple<arma::colvec, arma::colvec> grappling_hook(){
    
// }

/* Programme boucle principale */

int main() {
    /* Données */
    int RPM = 20000;
    arma::colvec motor_angle = {pi/4, pi/4, pi/4, pi/4}; // Valeur à entrer grâce à la télécommande
    arma::colvec motor_RPM = {10000, 15000, 20000, 17500}; // Valeur à entrer grâce à la télécommande

    /* Matrice de rotation */
    double phi = pi;
    double theta = 0;
    double psi = 0;
    arma::mat R = rotation_matrix(phi, theta, psi);

    /* Mise à jour du centre de gravité de la stucture */
    arma::rowvec mass = {3, 32*1e-3, 32*1e-3, 32*1e-3, 32*1e-3};
    arma::rowvec x_position = {0, 0.46, 0.46, -1.54, -1.54};
    arma::rowvec y_position = {0, 0, 0, 0, 0};
    arma::rowvec z_position = {0, 0, 0, 0, 0};

    double new_x_position = moved_gravity_center(mass, x_position);
    double new_y_position = moved_gravity_center(mass, y_position);
    double new_z_position = moved_gravity_center(mass, z_position);

    arma::colvec G = {new_x_position, new_y_position, new_z_position};

    /* Etude des blocs moteurs */
    double v_rotor = rotor_speed(radius, RPM);
    double m_dot = mass_flow(rho, area, v_rotor); // juste pour vérif la fonctionnalité
    double v_cruise = cruise_speed(area, rho, M_struct); 

    cout << "Voici la nouvelle position  du centre de gravité : " << endl;
    cout << G << endl;
    cout << "La vitesse du rotor est : " << v_rotor << " m/s." << endl;
    cout << "Débit massique : " << m_dot << " kg/s." << endl;
    cout << "Vitesse rotor de croisière (compensation Poids Poussée) : " << v_cruise << " m/s, soit " << v_cruise * 3.6 << " km/h." <<endl;

    /* Etude dirigeable à vide */
    arma::colvec B = {2.41, 0, 0.5}; // centre de flotabilité
    arma::colvec b2g = distance(B, G);

    arma::colvec F_b = buoyancy_force(rho, V_tot, z_m);
    arma::colvec P = weigth_force(M_struct, -1 * z_m);
    arma::colvec F_b_g = moment_translation(F_b, b2g, zeros);

    arma::mat Dirigible_Torsor = {{F_b(0) + P(0), F_b_g(0)},
                                  {F_b(1) + P(1), F_b_g(1)},
                                  {F_b(2) + P(2), F_b_g(2)}};

    cout << "\nTorseur du dirigeable à vide :\n" << endl;
    cout << Dirigible_Torsor << endl;

    /* Etude des systèmes-blocs moteur */

    arma::mat Motor_Torsor = {{0, 0},
                              {0, 0},
                              {0, 0}};

                        /* Positionnement dans l'espace */

    /* Position du centre de gravité du systèmes blocs-moteurs */
    arma::mat G_SM = {{0.46, 1, 0.5}, 
                      {0.46, -1, 0.5}, 
                      {-1.54, -1, 0.5}, 
                      {-1.54, 1, 0.5}};

    /* Position du centre de gravité de la structure des moteurs */
    arma::mat G_S = {{0.46, 1.05, 0},
                     {0.46, -1.05, 0},
                     {-1.54, -1.05, 0},
                     {-1.54, 1.05, 0}};

    /* Position des moteurs */
    arma::mat Ri = {{0.46, 1.5, 0}, 
                    {0.46, -1.5, 0}, 
                    {-1.54, -1.5, 0}, 
                    {-1.54, 1.5, 0}};


    for (int i=0; i<4; i++) {
            Motor_Torsor += sub_system_motor(motor_angle(i), motor_RPM(i), Ri.row(i), G_S.row(i), G_SM.row(i), G);
    }

    cout << "\n Torseur des actions extérieurs associées aux 4 systèmes moteurs exprimé en G\n" << endl; 
    cout << Motor_Torsor << endl;

    /* Expression du torseur des efforts intérieurs associés à la structure */

    arma::mat Torsor = Dirigible_Torsor + Motor_Torsor;
    cout << "Torseur des efforts extérieurs total :" << endl;
    cout << Torsor << endl;

    return 0;
}