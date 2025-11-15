#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include "particle/particle_filter.h"
using namespace std;

static  default_random_engine gen;

/*
* TODO
* This function initialize randomly the particles
* Input:
*  boundaries of the map
*  nParticles - number of particles
*  Tolto il rumore dato che non ha senso visto che è già randomico
*/
void ParticleFilter::init_random(int nParticles,double min_x, double max_x, double min_y, double max_y)
{
    num_particles = nParticles;
    double range_x = max_x - min_x;
    double range_y = max_y - min_y;

    if (range_x <= 0 || range_y <= 0) {     //Controllo per sicurezza
        std::cerr << "Error: invalid map bounds" << std::endl;
        return;
    }

    for (int i = 0; i < num_particles; ++i) {
        Particle p;
        p.id = i;

        // Genera posizione uniforme nei limiti specificati  (variante intera rand()%range+min)
        // Es: xmin=10 xmax=20 rand%10.000 = 7850 / 10.000 => 0,7850* 10 + 10 = 17,850  
        // (rand() % 10000) / 10000.0  genera un numero random [0,1)  
        p.x = (rand() % 10000) / 10000.0 * range_x + min_x;
        p.y = (rand() % 10000) / 10000.0 * range_y + min_y;

        // radiante randomico [-pi, pi]
        // (rand() % 10000) / 10000.0 * 2PI ==> numero random [0,2PI) - pi => [-pi,pi]
        p.theta = ((rand() % 10000) / 10000.0) * 2.0 * M_PI - M_PI;

        p.weight = 1.0;
        particles.push_back(p);
        weights.push_back(p.weight);
    }

    is_initialized = true;
}

/*
* TODO
* This function initialize the particles using an initial guess
* Input:
*  x,y,theta - position and orientation
*  std - noise that might be added to the position
*  nParticles - number of particles
*/ 
void ParticleFilter::init(double x, double y, double theta, double std[], int nParticles)
{
    num_particles = nParticles;

    //rappresenta la distribuzione normale con mu,std
    std::normal_distribution<double> genera_x(x, std[0]);
    std::normal_distribution<double> genera_y(y, std[1]);
    std::normal_distribution<double> genera_theta(theta, std[2]);

    // Creo le n particelle
    for (int i = 0; i < num_particles; i++)
    {
        Particle p;
        p.id = i;
        p.x = genera_x(gen);  // x+ rumore gaussiano con std[0]      
        p.y = genera_y(gen);       
        p.theta = genera_theta(gen); 
        p.weight = 1.0;          

        particles.push_back(p);
        weights.push_back(p.weight);
    }

    is_initialized = true;
}

//La creo per visualizzare e debuggare il metodo
pcl::PointCloud<pcl::PointXYZ>::Ptr ParticleFilter::asPointCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(particles.size());
    
    for (const auto& p : particles)
    {
        pcl::PointXYZ point;
        point.x = p.x;
        point.y = p.y;
        point.z = 0.0; //tutte sul piano xy
        cloud->points.push_back(point);
    }
    return cloud;
}



/*
* TODO
* The predict phase uses the state estimate from the previous timestep to produce an estimate of the state at the current timestep
* Input:
*  delta_t  - time elapsed beetween measurements
*  std_pos  - noise that might be added to the position
*  velocity - velocity of the vehicle
*  yaw_rate - current orientation
* Output:
*  Updated x,y,theta position
*/
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocita, double yaw_rate) {
    normal_distribution<double> dist_x(0.0, std_pos[0]);
    normal_distribution<double> dist_y(0.0, std_pos[1]);
    normal_distribution<double> dist_theta(0.0, std_pos[2]);

    // aggiorno pos e tetha di ogni particella secondo movimento dato dall'odometria
    for (auto &p : particles) {

        double x_prec = p.x;
        double y_prec = p.y;
        double theta_prec = p.theta;
        double x_nuovo, y_nuovo, theta_nuovo;
        // Controllo cambio di direzione
        if (fabs(yaw_rate) < 0.00001) 
        {
            x_nuovo = x_prec + velocita * delta_t * cos(theta_prec);
            y_nuovo = y_prec + velocita * delta_t * sin(theta_prec);
            theta_nuovo = theta_prec;
        } 
        else   //Se la yaw è variato ==> formula slide
        {
            x_nuovo = x_prec + (velocita / yaw_rate) * (sin(theta_prec + yaw_rate * delta_t) - sin(theta_prec));
            y_nuovo = y_prec + (velocita / yaw_rate) * (cos(theta_prec) - cos(theta_prec + yaw_rate * delta_t));
            theta_nuovo = theta_prec + yaw_rate * delta_t;
        }
        p.x = x_nuovo + dist_x(gen);    //Per aggiungere rumore gaussiano
        p.y = y_nuovo + dist_y(gen);
        p.theta = theta_nuovo + dist_theta(gen);
    }
}


/*
* TODO
* This function associates the landmarks from the MAP to the landmarks from the OBSERVATIONS
* Input:
*  mapLandmark   - landmarks of the map
*  observations  - observations of the car
* Output:
*  Associated observations to mapLandmarks (perform the association using the ids)
*  Associamo il landmark più vicino ad una osservazione
*/
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> mappa_landmarks, std::vector<LandmarkObs>& osservazioni) {

    for (int i = 0; i < osservazioni.size(); i++) {
        double distanza_min = 1e9;
        int id_min = -1;

        for (int j = 0; j < mappa_landmarks.size(); j++) {
            //dist == distanza euclidea (in helper_functions)
            double distanza = dist(osservazioni[i].x, osservazioni[i].y,mappa_landmarks[j].x, mappa_landmarks[j].y);
            if (distanza < distanza_min) {
                distanza_min = distanza;
                id_min = mappa_landmarks[j].id;
            }
        }

        osservazioni[i].id = id_min;  // associa ID del landmark più vicino
    }
}


/*
* TODO
* This function transform a local (vehicle) observation into a global (map) coordinates
* Input:
*  observation   - A single landmark observation
*  p             - A single particle
* Output:
*  local         - transformation of the observation from local coordinates to global
                 Proietta ciò che vede il "Robot" in ciò che vedrebbe la particella
*/         
LandmarkObs transformation(LandmarkObs osservazione, Particle p){
    LandmarkObs global;
    global.id = osservazione.id;

    // corrisponde alla roto traslazione 
    global.x = p.x + (cos(p.theta) * osservazione.x) - (sin(p.theta) * osservazione.y);
    global.y = p.y + (sin(p.theta) * osservazione.x) + (cos(p.theta) * osservazione.y);

    return global;
}

/*
* TODO
* This function updates the weights of each particle
* Input:
*  std_landmark   - Sensor noise
*  observations   - Sensor measurements
*  map_landmarks  - Map with the landmarks
* Output:
*  Updated particle's weight (particles[i].weight *= w)
* best particle
*/
void ParticleFilter::updateWeights(double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks,Particle &best_particle) {

    //Creates a vector that stores tha map (this part can be improved) 
    // MIGLIORAMENTO:
    // Creo il vettore mettendo gli id in modo ordinato in modo che viene automatico la associazione con le 
    // transformed observations

    //ottengo max_id così da allocare il numero giusto di spazio nel vettore
    int max_id = -1;
    for (auto &lm : map_landmarks.landmark_list) {
        if (lm.id_i > max_id)
            max_id = lm.id_i;
    }
    std::vector<LandmarkObs> landmarks_mappa(max_id + 1);
    //inserisco nella posizione corretta
    for (auto &lm : map_landmarks.landmark_list) {
        landmarks_mappa[lm.id_i] = LandmarkObs{lm.id_i, lm.x_f, lm.y_f};
    }

    for(int i=0;i<particles.size();i++){

        // Before applying the association we have to transform the observations in the global coordinates
        std::vector<LandmarkObs> transformed_observations;
        //TODO: for each observation transform it (transformation function), uso la transformation creata sopra
        for (auto &obs : observations) {
            transformed_observations.push_back(transformation(obs, particles[i]));
        }
        
        //TODO: perform the data association (associate the landmarks to the observations)
        dataAssociation(landmarks_mappa, transformed_observations);
        //Adesso  ogni obs ha l'id che identifica a quale reale landmark è associato

        particles[i].weight = 1.0;
        // Compute the probability
		//The particles final weight can be represented as the product of each measurement’s Multivariate-Gaussian probability density
		//We compute basically the distance between the observed landmarks and the landmarks in range from the position of the particle
        for(int k=0;k<transformed_observations.size();k++){
            double obs_x,obs_y,l_x,l_y;
            obs_x = transformed_observations[k].x;
            obs_y = transformed_observations[k].y;
            int id = transformed_observations[k].id;

            if (id < 0 || id >= landmarks_mappa.size()) {
                cout<<"[WARNING] id landmark non valido, anormale\n";
                continue;
            }
            //get the associated landmark (costo O(1))  [ottimizzato]
            l_x = landmarks_mappa[id].x;
            l_y = landmarks_mappa[id].y;

            
			// How likely a set of landmarks measurements are, given a prediction state of the car 
            double w = exp( -( pow(l_x-obs_x,2)/(2*pow(std_landmark[0],2)) + pow(l_y-obs_y,2)/(2*pow(std_landmark[1],2)) ) ) / ( 2*M_PI*std_landmark[0]*std_landmark[1] );
            particles[i].weight *= w;
        }

    }  
    double somma=0.0;
    
    for (int i=0; i<num_particles ; i++)
        somma+=particles[i].weight;
    cout<<"SOMMA DEI PESI: "<<somma<<"\n"; 
    /*
    if (somma < 10){
       for (int i=0; i<num_particles ; i++)
        particles[i].weight += 0.2; 
    }*/

    //Calcolo la best particle per mostrarla (faccio ora prima di aggiungere rumore)
    double highest_weight = -1.0;
    for (const auto &pt : particles) {
        if (pt.weight > highest_weight) {
            highest_weight = pt.weight;
            best_particle = pt;
        }
    }
    
}


/*
* TODO
* This function resamples the set of particles by repopulating the particles using the weight as metric

void ParticleFilter::resample() {

    // Distribuzione uniforme per l'indice iniziale da cui partire
    uniform_int_distribution<int> dist_index(0, num_particles - 1);
    int index = dist_index(gen);

    std::vector<double> weights;
    for (int i = 0; i < num_particles; i++) {
        weights.push_back(particles[i].weight);
    }
    double max_w = *max_element(weights.begin(), weights.end());

    // Distribuzione uniforme per la variabile beta
    uniform_real_distribution<double> dist_beta(0.0, 2.0 * max_w);

    double beta = 0.0;
    std::vector<Particle> new_particles;
    new_particles.reserve(num_particles); 

    // --- Distribuzioni per aggiungere rumore ---
    
    //normal_distribution<double> dist_x(0.0, 0.1);     // Rumore in metri
    //normal_distribution<double> dist_y(0.0, 0.1);
    //normal_distribution<double> dist_theta(0.0, 0.03); // Rumore in radianti
    

    // --- Resampling Wheel ---
    for (int i = 0; i < num_particles; i++) {

        beta += dist_beta(gen);

        // Finché beta è maggiore del peso corrente, scorri in avanti
        while (beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;  // ciclico
        }
         // Copia della particella selezionata
        Particle p = particles[index];

        // --- Aggiungi piccolo rumore casuale ---
        
        //p.x += dist_x(gen);
        //p.y += dist_y(gen);
        //p.theta += dist_theta(gen);
        
        new_particles.push_back(p);
    }

    // Aggiorna le particelle con quelle campionate
    particles = new_particles;
}
*/


 //   Versione Systematic "deterministic" + noise per maggiore esplorazione

void ParticleFilter::resample() {

    normal_distribution<double> dist_x(0.0, 0.03);     // Rumore in metri
    normal_distribution<double> dist_y(0.0, 0.03);
    normal_distribution<double> dist_theta(0.0, 0.01); // Rumore in radianti

    int N = num_particles;

    // estraggo il peso tot per calcolarmi il passo, e creo vettore cumulativo
    std::vector<double> cumulative(N);
    cumulative[0] = particles[0].weight;
    double sum_w = 0.0;

    for (int i = 1; i < N; i++) 
        cumulative[i] = cumulative[i-1] + particles[i].weight;
    
    sum_w = cumulative[N-1];

    // calcolo step size
    double step = sum_w / N;

    std::vector<Particle> new_particles;
    new_particles.reserve(N);

    double target = step;
    int idx = 0;

    for (int i = 0; i < N; i++) {

        //trovo idx della particella
        while (idx < N-1 && cumulative[idx] <= target) {
            idx++;
        }

        particles[idx].x += dist_x(gen);
        particles[idx].y += dist_y(gen);
        particles[idx].theta += dist_theta(gen);

        new_particles.push_back(particles[idx]);
        target += step;
    }
    particles = new_particles;
}
