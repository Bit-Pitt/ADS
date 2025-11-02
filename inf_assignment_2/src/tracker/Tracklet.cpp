#include "tracker/Tracklet.h"

Tracklet::Tracklet(int idTrack, double x, double y)
{
  // set id
  id_ = idTrack;

  // initialize filter
  kf_.init(0.1);    //10 Hz (freq aggiorniamento che simuliamo)
  kf_.setState(x, y);
  history_.push_back({x, y});   //primo passo

  // set loss count to 0
  loss_count_ = 0;
}

Tracklet::~Tracklet()
{
}

// Predict a single measurement
void Tracklet::predict()
{
  kf_.predict();
  loss_count_++;
}

// Update with a real measurement
void Tracklet::update(double x, double y, bool lidarStatus)
{
  Eigen::VectorXd raw_measurements_ = Eigen::VectorXd(2);

  // measurement update
  if (lidarStatus)
  {
    raw_measurements_ << x, y;
    kf_.update(raw_measurements_);
    history_.push_back({kf_.getX(), kf_.getY()});   //aggiorno con la pos stimata dal KF
    loss_count_ = 0;
  }
}
