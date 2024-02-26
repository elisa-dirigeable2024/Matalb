#include <iostream>
#include <armadillo>
#include <cmath>

using namespace std;

double pi = M_PI;

double moved_gravity_center(arma::rowvec vect_mass, arma::rowvec vect_pos){

    double total_mass = arma::accu(vect_mass);
    double somme = 0;

    for(unsigned int i =0; i < vect_pos.n_elem; i++){ // itération sur le nombre d'item du vecteur
        somme = somme + vect_mass(i)*vect_pos(i);
    }

    double new_position = somme / total_mass;
    return new_position;

}


double rotor_speed(double radius, int RPM){

    double v_rotor = 2 * pi * radius * (RPM/60);
    return v_rotor;

}


double mass_flow(double rho, double area, double speed){

    double m_dot = rho * area * speed; // [m/s]
    return m_dot;

}


int main() {

    double radius = (12.7/2)*1e-2; // [m]
    double rho = 1.225; // [kg/m^3]
    int RPM = 20000;
    double area = pi * radius * radius;

    arma::rowvec mass = {3, 32*1e-3, 32*1e-3, 32*1e-3, 32*1e-3};
    arma::rowvec x_position = {0, 0.46, 0.46, -1.54, -1.54};

    double new_x_position = moved_gravity_center(mass, x_position);
    double v_rotor = rotor_speed(radius, RPM);
    double m_dot = mass_flow(rho, area, v_rotor); // juste pour vérif la fonctionnalité

    cout << "Voici la nouvelle position x du centre de gravité : " << new_x_position << endl;
    cout << "La vitesse du rotor est : " << v_rotor << " m/s." << endl;
    cout << "Débit massique : " << m_dot << " kg/s." << endl;

    return 0;
}
