#include <iostream>
#include <armadillo>
#include <cmath>

using namespace std;

double pi = M_PI;
double g = 9.81;

double moved_gravity_center(arma::rowvec vect_mass, arma::rowvec vect_pos){

    double total_mass = arma::accu(vect_mass);
    double somme = 0;

    for(unsigned int i =0; i < vect_pos.n_elem; i++){ // itération sur le nombre d'item du vecteur
        somme = somme + vect_mass(i)*vect_pos(i);
    }

    double new_position = somme / total_mass;
    return new_position;

}


arma::colvec buoyancy_force(double rho, double V_tot, arma::colvec vector){
    double buoyancy_norme = rho * V_tot * g;
    arma::colvec F_b = buoyancy_norme * vector;
    return F_b;

}


double rotor_speed(double radius, int RPM){

    double v_rotor = 2 * pi * radius * (RPM/60);
    return v_rotor;

}


double mass_flow(double rho, double area, double speed){

    double m_dot = rho * area * speed; // [m/s]
    return m_dot;

}


double cruise_speed(double area, double rho, double M_struct){

    double g = 9.81;
    double v_cruise = sqrt((M_struct * g)/ (2 * rho * area));
    return v_cruise;

}


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

    return 0;
}