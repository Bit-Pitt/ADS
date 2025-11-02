#ifndef TRACKER_H_
#define TRACKER_H_

#include "tracker/Tracklet.h"
#include <limits>

class Tracker
{
public:
  Tracker();
  ~Tracker();

  // handle tracklets
  void removeTracks();
  void addTracks(const std::vector<bool> &associated_detections,
                 const std::vector<double> &centroids_x,
                 const std::vector<double> &centroids_y);

  // associate tracklets and detections
  void dataAssociation(std::vector<bool> &associated_detections,
                       const std::vector<double> &centroids_x,
                       const std::vector<double> &centroids_y);

  // track objects
  void track(const std::vector<double> &centroids_x,
             const std::vector<double> &centroids_y,
             bool lidarStatus);

  // getters
  const std::vector<Tracklet> &getTracks() const { return tracks_; }
  const std::vector<Tracklet>& getLostTracks() const { return lost_tracks_; }
  const std::vector<Tracklet>& getArchivedTracks() const { return archived_tracks_; }


  

private:
  // tracklets attivi
  std::vector<Tracklet> tracks_;

  // trackelets persi recentemente
  std::vector<Tracklet> lost_tracks_;

  // archiviate definitivamente (per analisi)
  std::vector<Tracklet> archived_tracks_; 


  int cur_id_;

  // association
  std::vector<std::pair<int, int>> associated_track_det_ids_;

  // thresholds
  double distance_threshold_;
  double covariance_threshold;
  int loss_threshold;
  int resurrection_window_;
};

#endif // TRACKER_H_
