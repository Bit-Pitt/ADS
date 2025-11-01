#include "tracker/KalmanFilter.h"

KalmanFilter::KalmanFilter() {}
KalmanFilter::~KalmanFilter() {}

void KalmanFilter::init(double dt)
{
  dt_ = dt;

  // Stato [x, y, vx, vy]
  x_ = Eigen::VectorXd(4);
  x_ << 0., 0., 0., 0.;


  // Covarianza iniziale del nostro modello ==>  valori grandi xchè non conosciamo bene lo stato iniziale
  P_ = Eigen::MatrixXd(4, 4);
  P_ << 9999., 0., 0., 0.,
        0., 9999., 0., 0.,
        0., 0., 9999., 0.,
        0., 0., 0., 9999.;

  //  MATRICE DI MISURA (solo x,y osservabili) 
  H_ = Eigen::MatrixXd(2, 4);
  H_ << 1., 0., 0., 0.,
        0., 1., 0., 0.;

  // COVARIANZA DEL RUMORE DI MISURA 
  // ±15 cm di errore medio nel cluster centroid
  R_ = Eigen::MatrixXd(2, 2);
  R_ << 0.0225, 0.,
        0., 0.0225;

  //  MATRICE DI TRANSIZIONE
  F_ = Eigen::MatrixXd(4, 4);
  F_ << 1., 0., dt_, 0.,
        0., 1., 0., dt_,
        0., 0., 1., 0.,
        0., 0., 0., 1.;

  // --- MATRICE DI COVARIANZA DEL PROCESSO ---
  double noise_ax_ = 2.0;
  double noise_ay_ = 2.0;
  double dt_2 = dt_ * dt_;
  double dt_3 = dt_2 * dt_;
  double dt_4 = dt_3 * dt_;

  Q_ = Eigen::MatrixXd(4, 4);   //quella delle slide
  Q_ << dt_4 / 4. * noise_ax_, 0., dt_3 / 2. * noise_ax_, 0.,
        0., dt_4 / 4. * noise_ay_, 0., dt_3 / 2. * noise_ay_,
        dt_3 / 2. * noise_ax_, 0., dt_2 * noise_ax_, 0.,
        0., dt_3 / 2. * noise_ay_, 0., dt_2 * noise_ay_;
}

// Fase di Predizione   (nuova x, nuova P)
void KalmanFilter::predict()
{
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

// Fase di Update
void KalmanFilter::update(const Eigen::VectorXd &z)
{
  
  Eigen::VectorXd y = z - H_ * x_;
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();    //Kalman gain (corda che tira in una delle due direzioni)
  x_ = x_ + (K * y);
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}

// per impostare stato iniziale
void KalmanFilter::setState(double x, double y)
{
  x_ << x, y, 0., 0.;
}
