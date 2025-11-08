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

#define NPARTICLES 100
#define circleID "circle_id"
#define reflectorID "reflector_id"

using namespace std;
using namespace lidar_obstacle_detection;

Map map_mille;
ParticleFilter pf;
bool init_odom=false;
Renderer renderer;
vector<Particle> best_particles;
std::ofstream myfile;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_particles(new pcl::PointCloud<pcl::PointXYZ>);

double sigma_init [3] = {1, 1, 1};
double sigma_pos [3]  = {0.07, 0.07, 0.07};
double sigma_landmark [2] = {0.2, 0.2};
std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1)};
control_s odom;

// mostra particelle - ora ridimensiona safe la cloud
void showPCstatus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const vector<Particle> &particles){
    // Assicuriamoci che la cloud abbia tanti punti quanti le particelle
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

// aggiorna reflectors in viewer (protetto)
void updateViewerReflector(const pcl::PointCloud<pcl::PointXYZI> &reflectorCenter){
    if (best_particles.empty()){
        // non abbiamo ancora il best particle: skip
        return;
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
        pf.prediction(0, sigma_pos, odom.velocity, odom.yawrate);
        init_odom=true;
    } else {
        double delta_t = (std::chrono::duration<double, std::milli>(t_start - t_end).count())/1000.0;
        if (delta_t < 0) delta_t = 0;
        pf.prediction(delta_t, sigma_pos, odom.velocity, odom.yawrate);
    }
    t_end = std::chrono::high_resolution_clock::now();
}

// Update - pointcloud callback (con protezioni)
void PointCloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){
    static std::chrono::time_point<std::chrono::high_resolution_clock> t_start, t_end;
    t_start = std::chrono::high_resolution_clock::now();

    if (!cloud_msg) {
        std::cerr << "[WARN] Received null cloud message, skipping." << std::endl;
        return;
    }
    if (cloud_msg->width == 0 || cloud_msg->height == 0) {
        std::cerr << "[WARN] Received empty PointCloud2 (width/height == 0), skipping." << std::endl;
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    try {
        pcl::fromROSMsg(*cloud_msg, *cloud);
    } catch (const std::exception &e) {
        std::cerr << "[ERROR] pcl::fromROSMsg threw: " << e.what() << std::endl;
        return;
    }

    if (cloud->empty()) {
        std::cerr << "[WARN] Converted cloud is empty, skipping." << std::endl;
        return;
    }

    // Debug: numero di punti ricevuti
    std::cout << "[INFO] Received cloud with " << cloud->size() << " points." << std::endl;

    // Estrai i reflectors (funzione esistente)
    pcl::PointCloud<pcl::PointXYZI> reflectorCenter = extractReflectors(cloud);

    // Nascondi tutti i reflectors
    for(int i = 0; i < nReflectors; ++i)
        renderer.updateShape(reflectorID + std::to_string(i), 0.0);

    // Aggiorna viewer (protetto)
    updateViewerReflector(reflectorCenter);

    // Costruisci osservazioni rumorose
    vector<LandmarkObs> noisy_observations;
    for (size_t i = 0; i < reflectorCenter.size(); ++i) {
        LandmarkObs obs;
        obs.x = reflectorCenter[i].x;
        obs.y = reflectorCenter[i].y;
        noisy_observations.push_back(obs);
    }

    // Protezione: assicurati che il filtro sia inizializzato e abbia particelle
    if (pf.particles.empty()) {
        std::cerr << "[WARN] ParticleFilter has zero particles, skipping update." << std::endl;
        return;
    }

    // Update weights e resample dentro try per sicurezza
    try {
        pf.updateWeights(sigma_landmark, noisy_observations, map_mille);
        pf.resample();
    } catch (const std::exception &e) {
        std::cerr << "[ERROR] Exception during update/resample: " << e.what() << std::endl;
        return;
    }

    // Seleziona best particle in modo sicuro
    Particle best_particle = pf.particles.front();
    double highest_weight = -1.0;
    for (const auto &pt : pf.particles) {
        if (pt.weight > highest_weight) {
            highest_weight = pt.weight;
            best_particle = pt;
        }
    }
    best_particles.push_back(best_particle);

    // Mostra particelle (assicurati che cloud_particles abbia la dimensione giusta)
    showPCstatus(cloud_particles, pf.particles);

    // Update circle
    renderer.removeShape(circleID + std::to_string(NPARTICLES + 1));
    renderer.addCircle(best_particles.back().x, best_particles.back().y, circleID + std::to_string(NPARTICLES + 1), 0.3, 1, 0, 0);

    // Log tempo esecuzione
    t_end = std::chrono::high_resolution_clock::now();
    double delta_t = (std::chrono::duration<double, std::milli>(t_end - t_start).count())/1000.0;
    myfile << best_particle.x << " " << best_particle.y << " " << delta_t << '\n';

    // Aggiorna viewer (singolo frame)
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

    pf.init(GPS_x, GPS_y, GPS_theta, sigma_init, NPARTICLES);

    // Prepara cloud_particles con la misura corretta
    cloud_particles->points.clear();
    cloud_particles->points.resize(max( (size_t)1, (size_t)NPARTICLES ));
    renderer.RenderPointCloud(cloud_particles,"particles",colors[0]);

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
