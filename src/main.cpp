#include <iostream>
#include <armadillo>
#include <cmath>
#include <tuple>

using namespace std;

double pi = M_PI;
double g = 9.81;

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

/* Moment d'une force */

arma::colvec moment_of_force(arma::colvec force, arma::colvec position_vector){
    arma::colvec moment = arma::cross(position_vector, force);
    /* Force appliquée en P, exprimée en 0 : M = OP ^ F */
    return moment;
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

/* Sous-Système moteur */

tuple<arma::colvec, arma::colvec> sub_system_motor(arma::rowvec alpha){
    /* Donneés */
    double constant = 0.83;
    double rho = 1.225;
    arma::rowvec RPM = {10000, 20000, 15000, 17500};
    double radius = (12.7/2)*1e-2;

    /* TODO: ajouter le stabilisateur PID du sous-système moteur */
    double F_x, F_y, F_z;
    double M_x, M_y, M_z = 0;
    arma::colvec x_m = {1, 0, 0};
    arma::colvec y_m = {0, 1, 0};
    arma::colvec z_m = {0, 0, 1};

    double alpha_1 = alpha(0);
    double alpha_2 = alpha(1);
    double alpha_3 = alpha(2);
    double alpha_4 = alpha(3);

    arma::mat R_y_1 = rotation_matrix_motor(alpha_1);
    arma::mat R_y_2 = rotation_matrix_motor(alpha_2);
    arma::mat R_y_3 = rotation_matrix_motor(alpha_3);
    arma::mat R_y_4 = rotation_matrix_motor(alpha_4);

    arma::colvec z_r_1 = R_y_1 * z_m;
    arma::colvec z_r_2 = R_y_2 * z_m;
    arma::colvec z_r_3 = R_y_3 * z_m;
    arma::colvec z_r_4 = R_y_4 * z_m;

    /* Expression des forces de Poids du sous-stème moteur */
    double m_sys = 0.1;
    double m_mot = 0.037;
    arma::colvec P_m = weigth_force(m_sys, -z_m);
    arma::colvec P = weigth_force(m_mot, -z_m);

    /* Expression des forces de poussée du sous-système moteur */
    arma::colvec F_prop_1 = prop_force(constant, rho, RPM(0), radius, z_r_1);
    arma::colvec F_prop_2 = prop_force(constant, rho, RPM(1), radius, z_r_2);
    arma::colvec F_prop_3 = prop_force(constant, rho, RPM(2), radius, z_r_3);
    arma::colvec F_prop_4 = prop_force(constant, rho, RPM(3), radius, z_r_4);

    /* Equation des forces */
    F_x = 4*P_m(0) + 4*P(0) + F_prop_1(0) + F_prop_2(0) + F_prop_3(0) + F_prop_4(0); 
    F_y = 4*P_m(1) + 4*P(1) + F_prop_1(1) + F_prop_2(1) + F_prop_3(1) + F_prop_4(1); 
    F_z = 4*P_m(2) + 4*P(2) + F_prop_1(2) + F_prop_2(2) + F_prop_3(2) + F_prop_4(2); 

    arma::colvec F_mot = {F_x, F_y, F_z};
    arma::colvec M_mot = {M_x, M_y, M_z};
    
    return make_tuple(F_mot, M_mot);
}

/* Sous-Système grappin */

// tuple<arma::colvec, arma::colvec> grappling_hook(){
    
// }

/* Programme boucle principale */

int main() {
    /* Données */
    double radius = (12.7/2)*1e-2; // [m]
    double rho = 1.225; // [kg/m^3]
    double area = pi * radius * radius;
    double M_struct = 3;
    double V_tot = 7.0;
    int RPM = 20000;

    /* Déclaration des vecteurs dans la base mobile */
    arma::colvec x_m = {1, 0, 0};
    arma::colvec y_m = {0, 1, 0};
    arma::colvec z_m = {0, 0, 1};

    /* Déclaration des vecteurs dans la base fixe */
    arma::colvec x_0 = {1, 0, 0};
    arma::colvec y_0 = {0, 1, 0};
    arma::colvec z_0 = {0, 0, 1};

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

    /* Expression des forces appliquées sur la structure */
    arma::colvec F_b = buoyancy_force(rho, V_tot, z_m);
    cout << "Poussée d'Archimède (flotabilité), F_b : " << endl << F_b << endl; 

    /* Etude des blocs moteurs */
    double v_rotor = rotor_speed(radius, RPM);
    double m_dot = mass_flow(rho, area, v_rotor); // juste pour vérif la fonctionnalité
    double v_cruise = cruise_speed(area, rho, M_struct); 

    cout << "Voici la nouvelle position x du centre de gravité : " << new_x_position << endl;
    cout << "La vitesse du rotor est : " << v_rotor << " m/s." << endl;
    cout << "Débit massique : " << m_dot << " kg/s." << endl;
    cout << "Vitesse rotor de croisière (compensation Poids Poussée) : " << v_cruise << " m/s, soit " << v_cruise * 3.6 << " km/h." <<endl;

    arma::rowvec alpha = {pi/4, pi/4, -pi/4, -pi/4};
    auto [F_mot, M_mot] = sub_system_motor(alpha);
    cout << F_mot << endl;

    return 0;
}