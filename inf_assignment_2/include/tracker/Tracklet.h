#ifndef TRACKLET_H_
#define TRACKLET_H_

#include <vector>
#include <cmath>

#include "KalmanFilter.h"

struct TrackPoint //saranno i "passi" salvati di un oggetto tracckato
{
    double x;
    double y;
};

class Tracklet
{
public:
  Tracklet(int idTrack, double x, double y);
  ~Tracklet();

  void predict();
  void update(double x, double y, bool lidarStatus);

  // getters
  double getX() { return kf_.getX(); }
  double getY() { return kf_.getY(); }
  double getVX() { return kf_.getVX(); }
  double getVY() { return kf_.getVY(); }
  double getXCovariance() { return kf_.getXCovariance(); }
  double getYCovariance() { return kf_.getYCovariance(); }
  int getLossCount() { return loss_count_; }
  const int getId() { return id_; }
  KalmanFilter& getFilter() { return kf_; }
  void incrementLossCount(){ loss_count_++; }
  void resetLossCount() { loss_count_ = 0;}
  std::vector<TrackPoint> getHistory() const { return history_; }


private:
  // filter
  KalmanFilter kf_;

  // tracklet id
  int id_;

  // number of loss since last update
  int loss_count_;

  //"passi" compiuti
  std::vector<TrackPoint> history_;
};

#endif // TRACKLET_H_
