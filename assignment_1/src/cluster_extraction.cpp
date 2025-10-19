#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include "../include/Renderer.hpp"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <chrono>
#include <unordered_set>
#include "../include/tree_utilities.hpp"
#include <boost/filesystem.hpp>
#include <stack>   //per opt versione di proximity
#include <queue>
namespace fs = boost::filesystem;


using namespace lidar_obstacle_detection;

// Config generale
namespace Config {

    //define USE_PCL_LIBRARY 
    //#define MY_PCL 
    #define MY_DBSCAN 

    constexpr float LEAF_VOXEL_GRID = 0.15f;

    constexpr int RANSAC_ITERATIONS = 700;
    constexpr float RANSAC_THRESHOLD = 0.2f;
    constexpr float REMAINING_CLOUD_RATIO = 0.4f;

    constexpr float CLUSTER_TOLERANCE = 0.4f;
    constexpr int MIN_CLUSTER_SIZE = 30;
    constexpr int MAX_CLUSTER_SIZE = 15000;

    //per configurare il renderer
    constexpr bool SHOW_VOXEL_FILTERED = false;
    constexpr bool SHOW_AFTER_CROP = false;
    constexpr bool SHOW_PLANES = false;
    constexpr bool SHOW_NON_PLANAR = false;

    //per la versione opt della "proximity"
    constexpr bool OPT_PROXIMITY = false;
}



typedef std::unordered_set<int> my_visited_set_t;

//This function sets up the custom kdtree using the point cloud
void setupKdtree(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, int dimension)
{
    //insert point cloud points into tree
    for (int i = 0; i < cloud->size(); ++i)
    {
        tree->insert({cloud->at(i).x, cloud->at(i).y, cloud->at(i).z}, i);
    }
}

/*
OPTIONAL
This function computes the nearest neighbors and builds the clusters
    - Input:
        + cloud: Point cloud to be explored
        + target_ndx: i-th point to visit
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + visited: Visited points --> typedef std::unordered_set<int> my_visited_set_t;
        + cluster: Here we add points that will represent the cluster
        + max: Max cluster size
    - Output:
        + visited: already visited points
        + cluster: at the end of this function we will have one cluster
*/
void proximity(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int target_ndx, my_pcl::KdTree* tree, float distanceTol, my_visited_set_t& visited, std::vector<int>& cluster, int max)
{
	if (cluster.size() < max)
    {
        cluster.push_back(target_ndx);
        visited.insert(target_ndx);

        std::vector<float> point {cloud->at(target_ndx).x, cloud->at(target_ndx).y, cloud->at(target_ndx).z};
    
        // get all neighboring indices of point
        std::vector<int> neighborNdxs = tree->search(point, distanceTol);

        for (int neighborNdx : neighborNdxs)
        {
            // if point was not visited
            if (visited.find(neighborNdx) == visited.end())
            {
                proximity(cloud, neighborNdx, tree, distanceTol, visited, cluster, max);
            }

            if (cluster.size() >= max)
            {
                return;
            }
        }
    }
}


// Versione ottimizzata, resa la funzione iterativa evitando lo stack di ricorsione, e look-up diretto dei visited con array [invece che set per cui hai overhead dell'hash]
void proximity_opt_iterative(
                        typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                        int start_ndx, 
                        my_pcl::KdTree* tree, 
                        float distanceTol, 
                        std::vector<bool>& visited, 
                        std::vector<int>& cluster, 
                        int max)
{
    std::stack<int> toVisit;      //in questo modo simulo la ricorsione senza farla eff
    toVisit.push(start_ndx);

    while (!toVisit.empty() && cluster.size() < max)        //continuo finchè quelli da visitare non sono finiti oppure ho raggiunto max_size
    {
        int current = toVisit.top();
        toVisit.pop();

        if (visited[current])
            continue;

        visited[current] = true;
        cluster.push_back(current);

        std::vector<float> point = {cloud->at(current).x, cloud->at(current).y, cloud->at(current).z};
        std::vector<int> neighbors = tree->search(point, distanceTol);

        // Aggiungi tutti i vicini non visitati nello stack anzichè una recursive call
        for (int n : neighbors)
        {
            if (!visited[n])
                toVisit.push(n);
        }
    }
}


/*
OPTIONAL
This function builds the clusters following a euclidean clustering approach
    - Input:
        + cloud: Point cloud to be explored
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + setMinClusterSize: Minimum cluster size
        + setMaxClusterSize: Max cluster size
    - Output:
        + cluster: at the end of this function we will have a set of clusters
TODO: Complete the function
*/
std::vector<pcl::PointIndices> euclideanCluster(
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
    my_pcl::KdTree* tree, 
    float distanceTol, 
    int setMinClusterSize, 
    int setMaxClusterSize)
{
    std::vector<bool> visited(cloud->size(), false); // lookup diretto per punti visitati 
    std::vector<pcl::PointIndices> clusters;         // vettore finale dei cluster

    for (int i = 0; i < cloud->size(); ++i)
    {
        if (!visited[i]) // se il punto non è stato ancora visitato
        {
            std::vector<int> clusterIndices;

            // chiama la versione iterativa della proximity
            proximity_opt_iterative(cloud, i, tree, distanceTol, visited, clusterIndices, setMaxClusterSize);

            // aggiungi il cluster se rispetta i limiti di dimensione
            if (clusterIndices.size() >= setMinClusterSize && clusterIndices.size() <= setMaxClusterSize)
            {
                pcl::PointIndices pclCluster;
                pclCluster.indices = clusterIndices;
                clusters.push_back(pclCluster);
            }
        }
    }

    return clusters;
}


/*          DBSCAN
 Clustering usando l'euristica "Density-based Scan"  (discussa in molti paper)
Oltre alla distanza è importante la densità locale in un punto per essere considerato "core point"
 Funzionamento:
classifichi ogni punto come:
 - core point	(se ha tot vicini ← qui si vede la density based)
 - border point
 - noise
⇒cluster sarà una somma di core point + border vicini tra loro
*/
std::vector<pcl::PointIndices> DBSCAN(
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    my_pcl::KdTree* tree,
                    float eps,
                    int minPts)
{
    //Using DBScan
    int N = cloud->size();
    std::vector<int> labels(N, -1);  // Per lo stato di ogni punto: -1 = unvisited, 0 = noise, >0 cluster id
    int clusterId = 0;

    for (int i = 0; i < N; ++i)
    {
        if (labels[i] != -1) continue; // già visitato

        std::vector<float> point = {cloud->at(i).x, cloud->at(i).y, cloud->at(i).z};
        std::vector<int> neighbors = tree->search(point, eps);

        if (neighbors.size() < minPts)      //Considero punti "noise" se i vicini in un eps < minimoPunti
        {
            labels[i] = 0; // punto classificato come rumore
            continue;
        }

        // punto core → nuovo cluster
        clusterId++;
        labels[i] = clusterId;

        std::queue<int> Q;          //metto in coda tutti i vicini
        for (int n : neighbors)
        {
            if (n != i)
                Q.push(n);
        }

        while (!Q.empty())      //ciclo i vicini del punto core:
        {
            int idx = Q.front();
            Q.pop();

            if (labels[idx] == 0) labels[idx] = clusterId; // punto classificato da rumore a cluster
            if (labels[idx] != -1) continue;               // già visitato/core

            labels[idx] = clusterId;

            //ottengo i vicini del vicino
            std::vector<float> np = {cloud->at(idx).x, cloud->at(idx).y, cloud->at(idx).z};
            std::vector<int> n_neighbors = tree->search(np, eps);           

            //se è un core --> porto i vicini in coda (quindi aggiungo ricorsivamente solo se è un core il vicino)
            if (n_neighbors.size() >= minPts)
            {
                for (int nn : n_neighbors)
                    if (labels[nn] == -1) Q.push(nn);
            }
        }
    }

    // Converti labels in vettore di pcl::PointIndices per creare il vettore di cluster
    std::vector<pcl::PointIndices> clusters;
    std::unordered_map<int, pcl::PointIndices> cluster_map;
    for (int i = 0; i < N; ++i)
    {
        int l = labels[i];      //se label[punto:10]= 4  -->appartiene al cluster 4 quindi lo metto nel vettore cluster_map[4]
        if (l <= 0) continue; // rumore
        cluster_map[l].indices.push_back(i);
    }

    for (auto &kv : cluster_map)       //aggiungo singolarmente i cluster [cluster_map[0] , ... ]
        clusters.push_back(kv.second);

    return clusters;
}


void ProcessAndRenderPointCloud (Renderer& renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    using namespace Config;

    //donwsample

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_before_crop(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(LEAF_VOXEL_GRID, LEAF_VOXEL_GRID, LEAF_VOXEL_GRID); 
    vg.filter(*cloud_filtered);       

    if (SHOW_VOXEL_FILTERED) {
        Color red(1.0, 0.0, 0.0);
        renderer.RenderPointCloud(cloud_filtered, "voxel_filtered", red);
    }
    

    // crop
    pcl::CropBox<pcl::PointXYZ> cb(true);
    cb.setInputCloud(cloud_filtered);
    cb.setMin(Eigen::Vector4f(-20, -6, -2, 1));
    cb.setMax(Eigen::Vector4f(30, 7, 5, 1));
    cb.filter(*cloud_filtered); 

    if (SHOW_AFTER_CROP) {
        Color white(1.0, 1.0, 1.0);
        renderer.RenderPointCloud(cloud_filtered, "croppedCloud", white);
    }

    // Segmentation w RANSAC
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(RANSAC_ITERATIONS);
    seg.setDistanceThreshold(RANSAC_THRESHOLD); 

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    int i = 0;
    int nr_points = (int) cloud_filtered->size();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux(new pcl::PointCloud<pcl::PointXYZ>);

    while (cloud_filtered->size() > REMAINING_CLOUD_RATIO * nr_points)
    {
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            std::cerr << "[WARN] Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*cloud_plane);


        if (SHOW_PLANES) {
            Color green(0.0, 1.0, 0.0);
            renderer.RenderPointCloud(cloud_plane, "plane_" + std::to_string(i), green);
        }

        extract.setNegative(true);
        extract.filter(*cloud_aux);
        cloud_filtered.swap(cloud_aux);

        i++;
    }

    std::cout << "[INFO] Remaining cloud after removing planes: " 
              << cloud_filtered->size() << " points." << std::endl;

    
    
    if (SHOW_NON_PLANAR) {
        Color yellow(1.0, 1.0, 0.0);
        renderer.RenderPointCloud(cloud_filtered, "nonPlanarCloud", yellow);
    }
    


    // Clustering
    
    std::vector<pcl::PointIndices> cluster_indices;

    #ifdef USE_PCL_LIBRARY                      //already implemented in PCL 

        std::cout << "[CLUSTERING DEFAULT] "<< std::endl;
        // --- PCL Euclidean clustering ---
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_filtered); 

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(CLUSTER_TOLERANCE);        
        ec.setMinClusterSize(MIN_CLUSTER_SIZE);          
        ec.setMaxClusterSize(MAX_CLUSTER_SIZE);       
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);
    #endif
    #ifdef   MY_PCL            //mia versione di opt euclidian clustering

        my_pcl::KdTree treeM;
        treeM.set_dimension(3);
        setupKdtree(cloud_filtered, &treeM, 3);
        cluster_indices = euclideanCluster(cloud_filtered, &treeM, CLUSTER_TOLERANCE, MIN_CLUSTER_SIZE, MAX_CLUSTER_SIZE);

    #endif
    #ifdef MY_DBSCAN

        my_pcl::KdTree treeM;
        treeM.set_dimension(3);
        setupKdtree(cloud_filtered, &treeM, 3);

        // Parametri DBSCAN
        float eps = 0.5f;          // distanza massima per considerare i punti vicini (in metri)
        int minPts = 10;           // numero minimo di punti per considerare un punto core

        cluster_indices = DBSCAN(cloud_filtered, &treeM, eps, minPts);

    #endif


    // Rosso: distanza < 3m, giallo <6, verde altrimenti
    const Color RED(1.0f, 0.0f, 0.0f);
    const Color YELLOW(1.0f, 1.0f, 0.0f);
    const Color GREEN(0.0f, 1.0f, 0.0f);
    const float RED_DIST = 3.0f;
    const float YELLOW_DIST = 6.0f;

    //std::cout << cluster_indices.size() << std::endl;
    int clusterId = 0;

    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (int idx : it->indices)
            cloud_cluster->push_back((*cloud_filtered)[idx]);

        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // Cerco il punto più vicino dall'ego vehicle (si trova in (0,0,0)) per cui corrisponde alla norma del punto
        float minDistance = 9999999;
        for (const auto& pt : cloud_cluster->points)
        {
            float dist = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);          //la norma (distanza dall'origine)
            if (dist < minDistance)
                minDistance = dist;

            if (minDistance < RED_DIST)         //in questo caso ho già deciso il colore rosso posso fermarmi
                break;
        }

        // Decido quindi il colore del cluster
        const Color* clusterColor = nullptr;
        if (minDistance < RED_DIST)
            clusterColor = &RED;
        else if (minDistance < YELLOW_DIST)
            clusterColor = &YELLOW;
        else
            clusterColor = &GREEN;

        renderer.RenderPointCloud(cloud_cluster, "cluster_" + std::to_string(clusterId), *clusterColor);

        // render del box rosso attorno al cluster
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);
        Box box{minPt.x, minPt.y, minPt.z, maxPt.x, maxPt.y, maxPt.z};
        renderer.RenderBox(box, clusterId);

        clusterId++;
    }



}


int main(int argc, char* argv[])
{
    //gestione dinamica del dataset tramite parametro
    if (argc < 2)
    {
        std::cerr << "Errore: immetti dataset come argomento.\n";
        return -1;
    }

    std::string dataset_path = argv[1];

    if (!boost::filesystem::exists(dataset_path) || 
        !boost::filesystem::is_directory(dataset_path))
    {
        std::cerr << "Errore: il percorso \"" << dataset_path << "\" non esiste o non è una directory.\n";
        return -1;
    }


    Renderer renderer;
    renderer.InitCamera(CameraAngle::XY);
    // Clear viewer
    renderer.ClearViewer();

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{dataset_path},
                                                boost::filesystem::directory_iterator{});

    // sort files in ascending (chronological) order
    std::sort(stream.begin(), stream.end());

    auto streamIterator = stream.begin();

    while (not renderer.WasViewerStopped())
    {
        renderer.ClearViewer();

        pcl::PCDReader reader;
        reader.read (streamIterator->string(), *input_cloud);
        auto startTime = std::chrono::steady_clock::now();

        ProcessAndRenderPointCloud(renderer,input_cloud);
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "[PointCloudProcessor<PointT>::ReadPcdFile] Loaded "
        << input_cloud->points.size() << " data points from " << streamIterator->string() <<  "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        renderer.SpinViewerOnce();
    }
}



