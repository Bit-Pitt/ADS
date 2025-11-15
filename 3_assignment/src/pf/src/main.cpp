#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <math.h>
#include "particle/particle_filter.h"
#include <pcl/common/transforms.h>
#include "Renderer.hpp"
#include <pcl/filters/voxel_grid.h>
#include <particle/helper_cloud.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>


#define circleID "circle_id"
#define reflectorID "reflector_id"

using namespace std;
using namespace lidar_obstacle_detection;

// Qui i parametri per modificare il comportamento del particle filter 
namespace config {          //Sigma pos ==> 0.07, land 0.2   PUOI NOTARE DAI CERCHI BIANCHI (ASSOCIANO I LANDMARK) CHE LA SOLUZIONE DIVERGE
    #define NPARTICLES 500
    double sigma_init [3] = {3, 3, 3};
    double sigma_pos [3]  = {0.2, 0.2, 0.2};     //se abbassi questo bad result (diverge)   
    double sigma_landmark [2] = {0.2, 0.2};         //se abbassi a 0.05 ==> risultato PESSIMO
    bool gps_init = false;          //true se vuoi usare il gps come initial guess
    bool bounding_precalcolati=true;      //true se vuoi usare i bounding box del garage per random_init     

}

Map map_mille;
ParticleFilter pf;
bool init_odom=false;
Renderer renderer;
vector<Particle> best_particles;
std::ofstream myfile;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_particles(new pcl::PointCloud<pcl::PointXYZ>);


std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1)};
control_s odom;

// mostra particelle 
void showPCstatus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const vector<Particle> &particles){
    cloud->points.resize(particles.size());
    for (size_t i = 0; i < particles.size(); ++i) {
        cloud->points[i].x = particles[i].x;
        cloud->points[i].y = particles[i].y;
        cloud->points[i].z = 0.0f;
    }
    cloud->width = (uint32_t)cloud->points.size();
    cloud->height = 1;
    renderer.updatePointCloud(cloud,"particles");
}

void updateViewerReflector(const pcl::PointCloud<pcl::PointXYZI> &reflectorCenter){
    if (best_particles.empty()){
        return;   //skip se non abbiamo best_particle
    }
    const Particle &best = best_particles.back();

    for (size_t i = 0; i < reflectorCenter.size(); ++i)
    {
        pcl::PointXYZI pt = reflectorCenter[i];
        float gx = pt.x * cos(best.theta) - pt.y * sin(best.theta) + best.x;
        float gy = pt.x * sin(best.theta) + pt.y * cos(best.theta) + best.y;
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << gx, gy, 0;
        renderer.updatePose(reflectorID + std::to_string(i), transform);
        renderer.updateShape(reflectorID + std::to_string(i), 1.0);
    }
}

// Prediction - odometry callback
void OdomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    static std::chrono::time_point<std::chrono::high_resolution_clock> t_start ;
    static std::chrono::time_point<std::chrono::high_resolution_clock> t_end ;
    odom.velocity = msg->twist.twist.linear.x;
    odom.yawrate = msg->twist.twist.angular.z;
    t_start = std::chrono::high_resolution_clock::now();

    if(!init_odom){
        pf.prediction(0, config::sigma_pos, odom.velocity, odom.yawrate);
        init_odom=true;
    } else {
        double delta_t = (std::chrono::duration<double, std::milli>(t_start - t_end).count())/1000.0;
        if (delta_t < 0) delta_t = 0;
        pf.prediction(delta_t, config::sigma_pos, odom.velocity, odom.yawrate);
    }
    t_end = std::chrono::high_resolution_clock::now();
}

// Update - pointcloud callback (con protezioni)
void PointCloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){
    static std::chrono::time_point<std::chrono::high_resolution_clock> t_start, t_end;
    t_start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    try {
        pcl::fromROSMsg(*cloud_msg, *cloud);
    } catch (const std::exception &e) {
        std::cerr << "pcl::fromROSMsg threw: " << e.what() << std::endl;
        return;
    }

    if (cloud->empty()) {
        std::cerr << "Converted cloud is empty, skipping." << std::endl;
        return;
    }


    pcl::PointCloud<pcl::PointXYZI> reflectorCenter = extractReflectors(cloud);
    // Nascondi tutti i reflectors
    for(int i = 0; i < nReflectors; ++i)
        renderer.updateShape(reflectorID + std::to_string(i), 0.0);

    updateViewerReflector(reflectorCenter);

    // Costruisci osservazioni rumorose
    vector<LandmarkObs> noisy_observations;
    for (size_t i = 0; i < reflectorCenter.size(); ++i) {
        LandmarkObs obs;
        obs.x = reflectorCenter[i].x;
        obs.y = reflectorCenter[i].y;
        noisy_observations.push_back(obs);
    }

    // controllo che il filtro sia inizializzato e abbia particelle
    if (pf.particles.empty()) {
        std::cerr << "ParticleFilter has zero particles, skipping update." << std::endl;
        return;
    }

    Particle best_particle;
    //  per catchare errori imprevisti
    try {
        pf.updateWeights(config::sigma_landmark, noisy_observations, map_mille,best_particle);
        pf.resample();
    } catch (const std::exception &e) {
        std::cerr << "Exception during update/resample: " << e.what() << std::endl;
        return;
    }

    /*
    Particle best_particle = pf.particles.front();
    double highest_weight = -1.0;
    for (const auto &pt : pf.particles) {
        if (pt.weight > highest_weight) {
            highest_weight = pt.weight;
            best_particle = pt;
        }
    }*/
    best_particles.push_back(best_particle);

    showPCstatus(cloud_particles, pf.particles);

    renderer.removeShape(circleID + std::to_string(NPARTICLES + 1));
    renderer.addCircle(best_particles.back().x, best_particles.back().y, circleID + std::to_string(NPARTICLES + 1), 0.3, 1, 0, 0);

    // Log tempo esecuzione
    t_end = std::chrono::high_resolution_clock::now();
    double delta_t = (std::chrono::duration<double, std::milli>(t_end - t_start).count())/1000.0;
    myfile << best_particle.x << " " << best_particle.y << " " << delta_t << '\n';

    renderer.SpinViewerOnce();
}


int main(int argc,char **argv)
{
    // Carica mappa (path relativo alla directory del pacchetto, es. src/pf/data)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMap (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudReflectors (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> vg;

    if (pcl::io::loadPCDFile ("data/map_reflector.pcd", *cloudReflectors) < 0) {
        std::cerr << "Error: Could not read data/map_reflector.pcd" << std::endl;
        return -1;
    }
    if (pcl::io::loadPCDFile ("data/map_pepperl.pcd", *cloudMap) < 0) {
        std::cerr << "Error: Could not read data/map_pepperl.pcd" << std::endl;
        return -1;
    }

    remove("./res.txt");
    createMap(cloudReflectors, "data/map_data.txt", map_mille);

    if (!read_map_data("data/map_data.txt", map_mille)) {
        cout << "Error: Could not open map file" << endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_map (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloudMap);
    vg.setLeafSize (1.0f, 1.0f, 1.0f);
    vg.filter (*cloud_filtered_map);

    renderer.InitCamera(CameraAngle::XY);
    renderer.ClearViewer();
    renderer.RenderPointCloud(cloud_filtered_map,"originalCloud",colors[2]);
    renderer.RenderPointCloud(cloudReflectors,"reflectorCloud",colors[0]);

    for(int i=0;i<nReflectors;i++)
        renderer.addCircle(0, 0, reflectorID+std::to_string(i), 0.2,1,1,1);

    double GPS_x = 2;
    double GPS_y = 1;
    double GPS_theta = -1;

    Particle p(GPS_x,GPS_y,GPS_theta);
    best_particles.push_back(p);

    if (config::gps_init)
        pf.init(GPS_x, GPS_y, GPS_theta, config::sigma_init,NPARTICLES);
    else
    {
        double min_x = 1e10, min_y = 1e10, max_x = -1e10, max_y = -1e10;
        if (config::bounding_precalcolati)
        {
            min_x = -9,97788;   max_x = 10,4361; max_y = 34,3804;
            min_y = -35;        //Questa informazione non data dai landmark
        }
        else
        {      //Computo il bounding box della mappa (l'attuale cloud filtered)
            for (const auto& pt : cloud_filtered_map->points) //Questo in generale se non sappiamo la mappa a piori
            {
                if (pt.x < min_x) min_x = pt.x;
                if (pt.x > max_x) max_x = pt.x;
                if (pt.y < min_y) min_y = pt.y;
                if (pt.y > max_y) max_y = pt.y;
            }
        }
        pf.init_random(NPARTICLES, min_x, max_x, min_y, max_y);
    }

    /**
    // Prepara cloud_particles con la misura corretta
    cloud_particles->points.clear();
    cloud_particles->points.resize(max( (size_t)1, (size_t)NPARTICLES ));
    renderer.RenderPointCloud(cloud_particles,"particles",colors[0]);
    */
    // Visualizza le particelle inizializzate (in sostituzione alle 3 righe sopra)
    auto cloud_particles = pf.asPointCloud();
    renderer.RenderPointCloud(cloud_particles, "particles", colors[1]);


    renderer.addCircle(GPS_x, GPS_y, circleID+std::to_string(NPARTICLES+1), 0.4,0,1,1);
    renderer.SpinViewerOnce();

    std::cout << "Map loaded, waiting for the rosbag (give ~2s) ..." << std::endl;
    myfile.open("./res.txt", std::ios_base::app);

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("particle");

    // Piccolo delay per permettere alla GUI di inizializzarsi prima di ricevere messaggi
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "/auriga_id0_odom", 10, OdomCb);

    auto pc_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/pepperl_id0_cloud", 10, PointCloudCb);

    rclcpp::Rate rate(30);
    while (rclcpp::ok() && !renderer.WasViewerStopped()) {
        rclcpp::spin_some(node);
        renderer.SpinViewerOnce();
        rate.sleep();
    }

    myfile.close();
    rclcpp::shutdown();
    return 0;
}
