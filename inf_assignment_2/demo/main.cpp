#include <fstream>

#include "viewer/Renderer.h"
#include "tracker/Tracker.h"
#include "CloudManager.h"
#include "tracker/post_track_analysis.h"



namespace Config{
    
    //Per uscire dalla fase Online dopo "MAX_FRAME" e passare all'analisi offline 
    const int MAX_FRAMES = 600;   // es 300 fram = 30 secondi
    //const int MAX_FRAMES = std::numeric_limits<int>::max(); se non si vuole uscire
 
    bool debug_storia = false;          //per stampare i "passi" di ogni track


    // Decidi un area per visualizzare le track che ci sono passate e per quanti step
    double x_min = -5,  x_max = 0;
    double y_min = -2 , y_max = 0;
}

int main(int argc, char *argv[])
{
    cout<<"Rurring config:\n";
    cout<<"MAX_FRAMES: "<<Config::MAX_FRAMES<<endl;
    cout<<"Debug cammini: "<<Config::debug_storia <<endl;


    int64_t freq = 100;            // Frequency of the thread dedicated to process the point cloud
    std::string log_path = "../build/log";  // TODO: define the path to the log folder

    std::ifstream dataFile(log_path, std::ios::in | std::ios::binary);
    if (!dataFile)
    {
        std::cerr << "ERROR: The file '" << log_path << "' does not exist. Exiting.\n";
        return 1;
    }

    // Init renderer
    viewer::Renderer renderer;
    renderer.initCamera(viewer::CameraAngle::XY);
    renderer.clearViewer();

    // Instantiate the tracker
    Tracker tracker;


    // Spawn the thread that process the point cloud and performs the clustering
    CloudManager lidar_cloud(log_path, freq, renderer,Config::MAX_FRAMES);
    std::thread t(&CloudManager::startCloudManager, &lidar_cloud);
    int frame_count = 0;


    while (true)
    {
        frame_count++;
        if (frame_count >= Config::MAX_FRAMES)
        {
            std::cout << "\n[INFO] Raggiunto il numero massimo di frame. Esco...\n";
            break;
        }


        // Clear the render
        renderer.clearViewer();

        while (!lidar_cloud.new_measurement)
            ; // wait for new data (we will execute the following code each 100ms)

        // fetch data
        lidar_cloud.mtxData.lock();
        auto cloud = lidar_cloud.getCloud();
        auto color = lidar_cloud.getColor();
        auto boxes = lidar_cloud.getBoxes();
        auto centroids_x = lidar_cloud.getCentroidsX();
        auto centroids_y = lidar_cloud.getCentroidsY();
        lidar_cloud.new_measurement = false;
        lidar_cloud.mtxData.unlock();

        // render pointcloud and boxes
        renderer.renderPointCloud(cloud, "pointCloud", color);
        for (size_t i = 0; i < boxes.size(); ++i)
            renderer.renderBox(boxes[i], i);

        // Call the tracker on the detected clusters
        tracker.track(centroids_x, centroids_y, renderer.getLidarStatus());

        // retrieve tracklets and render the trackers
        auto tracks = tracker.getTracks();
        for (size_t i = 0; i < tracks.size(); ++i)
        {
            renderer.addCircle(tracks[i].getX(), tracks[i].getY(), tracks[i].getId());
            renderer.addText(tracks[i].getX() + 0.01, tracks[i].getY() + 0.01, tracks[i].getId());
        }

        renderer.spinViewerOnce();    
      
    }

    t.join();

    cout<< "ANALISI 'OFFLINE', Lancia lo script python ottenere gli output\n";

    // Raggruppa tutte le track
    auto total_tracks = post_analysis::collectAllTracks(tracker);

    // Stampa su console i passi di tutte le track
    if (Config::debug_storia)
        post_analysis::printTracksSummary(total_tracks);

    post_analysis::exportTracksToCSV(total_tracks, "../build/track_history.csv");

    //Esporta distanze percorse da ogni track 
    post_analysis::computeTrackDistances(total_tracks);

    
    post_analysis::computeTracksInArea(total_tracks,
                                            Config::x_min,Config::x_max,
                                            Config::y_min,Config::y_max);
    




    return 0;
}
