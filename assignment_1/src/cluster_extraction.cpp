#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
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
namespace fs = boost::filesystem;
#define USE_PCL_LIBRARY             //disattiva
using namespace lidar_obstacle_detection;

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
std::vector<pcl::PointIndices> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, float distanceTol, int setMinClusterSize, int setMaxClusterSize)
{
	my_visited_set_t visited{};                                                          //already visited points
	std::vector<pcl::PointIndices> clusters;                                             //vector of PointIndices that will contain all the clusters
    std::vector<int> cluster;                                                            //vector of int that is used to store the points that the function proximity will give me back
	//for every point of the cloud
    //  if the point has not been visited (use the function called "find")
    //    find clusters using the proximity function
    //
    //    if we have more clusters than the minimum
    //      Create the cluster and insert it in the vector of clusters. You can extract the indices from the cluster returned by the proximity funciton (use pcl::PointIndices)   
    //    end if
    //  end if
    //end for
	return clusters;	
}

void ProcessAndRenderPointCloud (Renderer& renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{

    //Configuration
    float LeafVoxelGrid = 0.1;

    int RANSAC_it = 1000;
    float RANSAC_threashold = 0.1;
    float remainingCloudRatio = 0.4f;  

    float cluster_tol = 0.3;       //30cm
    int min_cluster_size = 100;
    int max_cluster_size = 25000;


    ////////////////////////////////////////////////////////////////////////////////////////
    // TODO: 1) Downsample the dataset 
    ////////////////////////////////////////////////////////////////////////////////////////

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_before_crop(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

    //Qui aggiungo il downsample Voxel Grid
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(LeafVoxelGrid, LeafVoxelGrid, LeafVoxelGrid); // dimensione del voxel (modificabile)
    vg.filter(*cloud_filtered);        //questo dopo va mostrata tramite renderer

    // Salva una copia prima del crop (altrimenti viene sovrascritta)
    *cloud_before_crop = *cloud_filtered;

    std::cout << "[INFO] PointCloud after filtering: " 
              << cloud_filtered->width * cloud_filtered->height << " points." << std::endl;

    /*  per debug
    Color white(1.0, 1.0, 1.0);
    renderer.RenderPointCloud(cloud_before_crop, "onlyVoxel_filteredCloud", white);
    */

    // 2) here we crop the points that are far away from us, in which we are not interested
    pcl::CropBox<pcl::PointXYZ> cb(true);
    cb.setInputCloud(cloud_filtered);
    cb.setMin(Eigen::Vector4f (-20, -6, -2, 1));
    cb.setMax(Eigen::Vector4f ( 30, 7, 5, 1));
    cb.filter(*cloud_filtered); 

    /* per debug
    Color red(1.0, 0.0, 0.0);
    renderer.RenderPointCloud(cloud_filtered, "filteredCloud", red);
    */

    ////////////////////////////////////////////////////////////////////////////////////////
    // TODO: 3) Segmentation and apply RANSAC
    // TODO: 4) iterate over the filtered cloud, segment and remove the planar inliers 
    ////////////////////////////////////////////////////////////////////////////////////////
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // Set RANSAC parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(RANSAC_it);
    seg.setDistanceThreshold(RANSAC_threashold); 

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    int i = 0;
    int nr_points = (int) cloud_filtered->size();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux(new pcl::PointCloud<pcl::PointXYZ>);

    while (cloud_filtered->size() > remainingCloudRatio * nr_points)
    {
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            std::cerr << "[WARN] Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Estraggo il piano
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*cloud_plane);

        std::cout << "[INFO] Plane " << i 
                  << " extracted with " << cloud_plane->size() << " points." << std::endl;

        // Render del piano in verde (DEBUG)
        /*
        Color green(0.0, 1.0, 0.0);
        renderer.RenderPointCloud(cloud_plane, "plane_" + std::to_string(i), green);
        */

        // Rimuovo il piano dal cloud principale
        extract.setNegative(true);
        extract.filter(*cloud_aux);
        cloud_filtered.swap(cloud_aux);

        i++;
    }

    std::cout << "[INFO] Remaining cloud after removing planes: " 
              << cloud_filtered->size() << " points." << std::endl;

    
        
    //Ora cloud_filtered ha SOLO i punti NON planari
    //debug:
    /*
    Color yellow(1.0, 1.0, 0.0);
    renderer.RenderPointCloud(cloud_filtered, "nonPlanarCloud", yellow);
    */


    /////////
    // TODO: 5) Create the KDTree and the vector of PointIndices
    /////////
    // TODO: 6) Set the spatial tolerance for new cluster candidates (pay attention to the tolerance!!!)
    std::vector<pcl::PointIndices> cluster_indices;


    #ifdef USE_PCL_LIBRARY                      //Qui uso di funzioni già fatte da PCL

        std::cout << "[CLUSTERING DEFAULT] "<< std::endl;
        // --- PCL Euclidean clustering ---
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_filtered); 

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tol);        
        ec.setMinClusterSize(min_cluster_size);          
        ec.setMaxClusterSize(max_cluster_size);       
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);

    #else               //QUI MIA IMPLEMENTAZIONE

        my_pcl::KdTree treeM;
        treeM.set_dimension(3);
        setupKdtree(cloud_filtered, &treeM, 3);
        cluster_indices = euclideanCluster(cloud_filtered, &treeM, clusterTolerance, setMinClusterSize, setMaxClusterSize);

    #endif


    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1)};


    /**Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices. 

    To separate each cluster out of the vector<PointIndices> we have to iterate through cluster_indices, create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
    Compute euclidean distance
    
    int j = 0;
    int clusterId = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->push_back ((*cloud_filtered)[*pit]); 
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        renderer.RenderPointCloud(cloud,"originalCloud"+std::to_string(clusterId),colors[2]);
        // TODO: 7) render the cluster and plane without rendering the original cloud 
        //<-- here
        //----------

        //Here we create the bounding box on the detected clusters
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

        //TODO: 8) Here you can plot the distance of each cluster w.r.t ego vehicle
        Box box{minPt.x, minPt.y, minPt.z,
        maxPt.x, maxPt.y, maxPt.z};
        //TODO: 9) Here you can color the vehicles that are both in front and 5 meters away from the ego vehicle
        //please take a look at the function RenderBox to see how to color the box
        renderer.RenderBox(box, j);

        ++clusterId;
        j++;
    }  **/
    std::cout << cluster_indices.size() << std::endl;
    int clusterId = 0;
    // "it --> iteratore dei cluster"
    // it-->indices per accedere a tutti i punti del cluster attuale
    // cloud_cluster perchè si crea ad ogni iterazione la nuvola del cluster con colori che ciclano
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (int idx : it->indices)
            cloud_cluster->push_back((*cloud_filtered)[idx]);

        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // --- Render cluster ---
        Color clusterColor = colors[clusterId % colors.size()];
        renderer.RenderPointCloud(cloud_cluster, "cluster_" + std::to_string(clusterId), clusterColor);

        // --- Bounding box ---
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);
        Box box{minPt.x, minPt.y, minPt.z, maxPt.x, maxPt.y, maxPt.z};

        // TODO: opzionale: colorare box in base alla distanza dall'ego vehicle [da fare]
        renderer.RenderBox(box, clusterId);

        ++clusterId;
    }


}


// SOLO di partenza
void ProcessAndRenderPointCloud_def(Renderer& renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // Mostra semplicemente la point cloud originale
    if (cloud->empty()) {
        std::cout << "[WARNING] Empty point cloud, nothing to render!" << std::endl;
        return;
    }

    // Colore rosso per la nuvola  [r,g,b]
    Color red(1.0, 0.0, 0.0);

    // Render della nuvola originale
    renderer.RenderPointCloud(cloud, "originalCloud", red);

    // Aggiungiamo anche la strada per riferimento (opzionale)
    //renderer.RenderHighway();


    std::cout << "[INFO] Rendered point cloud with " 
              << cloud->size() << " points." << std::endl;
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



