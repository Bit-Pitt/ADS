#include "tracker/Tracker.h"
#include <iostream>

Tracker::Tracker()
{
    cur_id_ = 0;
    distance_threshold_ = 1.0;       // 1 m: per associare detections vicine
    covariance_threshold = 50.0;     // media delle varianze x,y (unità: m²)  [quindi +-max 7m] 
    loss_threshold = 20;         //max num frame che non vedo la track
    resurrection_window_ = 20; // frame in cui una vecchia track può tornare

}
Tracker::~Tracker()
{
}


/* 
    Active --> lost_tracks secondo una delle  2 condizioni
*   - 1) troppi frame persi dell'oggetto (Loss_count > soglia)
*   - 2) covarianza di P (incertezza modello) troppo alta
    --> Dopo che passa la "resurrection window" : lost definitivo della track
*/  

void Tracker::removeTracks()
{
    std::vector<Tracklet> tracks_to_keep;

    for (auto &t : tracks_)
    {
        bool lost_too_long = t.getLossCount() > loss_threshold;
        double pos_uncertainty = (t.getXCovariance() + t.getYCovariance()) / 2.0;
        bool too_uncertain = pos_uncertainty > covariance_threshold;

        if (!(lost_too_long || too_uncertain))
        {
            tracks_to_keep.push_back(t);
        }
        else
        {
            std::cout << "Track ID " << t.getId() << " lost (loss_count="
                      << t.getLossCount() << ", uncertainty=" << pos_uncertainty << ")\n";

            // Salva come "temporaneamente perso"
            t.resetLossCount();  // opzionale, resetta per contare da zero
            lost_tracks_.push_back(t);
        }
    }

    tracks_.swap(tracks_to_keep);

    // Aggiorna i lost tracks: aumenta il contatore interno e rimuovi quelli troppo vecchi
    std::vector<Tracklet> still_lost;
    for (auto &lt : lost_tracks_)
    {
        lt.incrementLossCount();
        if (lt.getLossCount() < resurrection_window_)
            still_lost.push_back(lt);
        else
        {
            std::cout << "Rimozione def. old lost track ID " << lt.getId() << "\n";
            archived_tracks_.push_back(lt); // per analisi offline
        }
    }
    lost_tracks_.swap(still_lost);
}

/*
    Una track non associata sarà o:
    - associata ad una track persa recentemente che si trova in quell'area (distance_threashould)
    - altrimenti sarà una nuova track
*/
void Tracker::addTracks(const std::vector<bool> &associated_detections,
                        const std::vector<double> &centroids_x,
                        const std::vector<double> &centroids_y)
{
    for (size_t i = 0; i < associated_detections.size(); ++i)
    {
        if (associated_detections[i])
            continue; // detection già usata

        double x = centroids_x[i];
        double y = centroids_y[i];

        // Controlla se c’è un lost track vicino
        int resurrect_id = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t j = 0; j < lost_tracks_.size(); ++j)
        {
            double dx = lost_tracks_[j].getX() - x;
            double dy = lost_tracks_[j].getY() - y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < distance_threshold_ && dist < min_dist)
            {
                resurrect_id = j;
                min_dist = dist;
            }
        }

        if (resurrect_id != -1)     //id ==> posizione nel vettore lost track non track_id
        {
            // Riattiva la track persa
            Tracklet resurrected = lost_tracks_[resurrect_id];
            resurrected.update(x, y, true);
            tracks_.push_back(resurrected);

            std::cout << "Resurrected track ID " << resurrected.getId()
                      << " at (" << x << ", " << y << ")\n";

            lost_tracks_.erase(lost_tracks_.begin() + resurrect_id);
        }
        else
        {
            // Crea nuova track se non associata ad una vecchia
            tracks_.push_back(Tracklet(cur_id_++, x, y));
            std::cout << "New track ID " << cur_id_ - 1
                      << " created at (" << x << ", " << y << ")\n";
        }
    }
}


#include <Eigen/Dense>
#include <limits>
#include <cmath>


// DATA ASSOCIATION con Mahalanobis Distance
void Tracker::dataAssociation(std::vector<bool> &associated_detections,     // sono vettori paralleli
                              const std::vector<double> &centroids_x,
                              const std::vector<double> &centroids_y)
{
    associated_track_det_ids_.clear();

    // Soglia di gating (chi-quadrato per 2 DOF: 95% → 5.99, 99% → 9.21)
    const double mahalanobis_threshold = 9.21;

    // Scorri tutti i track attivi
    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        int closest_point_id = -1;
        double min_maha = std::numeric_limits<double>::max();

        // Predizione attuale del track (x, y)
        double track_x = tracks_[i].getX();
        double track_y = tracks_[i].getY();

        // Recupera la covarianza del Kalman Filter
        // Nota: devi avere un getter nel KalmanFilter (Eigen::MatrixXd getP() const)
        Eigen::MatrixXd P = tracks_[i].getFilter().getP(); // oppure getCovarianceMatrix()
        
        // Estrai il blocco 2x2 in alto a sinistra di P (posizione x,y)
        Eigen::Matrix2d P_pos = P.block<2, 2>(0, 0);

        // Matrice di rumore di misura R (stessa definita nel KalmanFilter)
        Eigen::Matrix2d R;
        R << 0.0225, 0.0,
             0.0, 0.0225;

        // Covarianza innovazione: S = HPH^T + R (H = [I 0])
        Eigen::Matrix2d S = P_pos + R;

        // Fattorizzazione numericamente stabile per S^{-1}
        Eigen::LDLT<Eigen::Matrix2d> ldlt(S);
        if (ldlt.info() != Eigen::Success)
        {
            std::cerr << "[WARN] Singular innovation covariance for track " << i << std::endl;
            continue;
        }

        // Calcola la distanza Mahalanobis tra il track e tutte le detection
        for (size_t j = 0; j < centroids_x.size(); ++j)
        {
            if (associated_detections[j]) 
                continue;  // già assegnata

            Eigen::Vector2d z;  // misura (detection)
            z << centroids_x[j], centroids_y[j];

            Eigen::Vector2d z_pred;  // predizione
            z_pred << track_x, track_y;

            // innovazione (residuo)
            Eigen::Vector2d y = z - z_pred;

            // distanza di Mahalanobis
            double maha2 = y.transpose() * ldlt.solve(y);

            // Mantieni la detection con la minore distanza Mahalanobis
            if (maha2 < min_maha && maha2 < mahalanobis_threshold)
            {
                min_maha = maha2;
                closest_point_id = j;
            }
        }

        // Se la migliore detection è entro la soglia, associamo
        if (closest_point_id != -1)
        {
            associated_track_det_ids_.push_back(std::make_pair(closest_point_id, i));
            associated_detections[closest_point_id] = true; // detection usata
        }
    }
}




/*  
    - detection dei cluster a tempo t
    - a tempo t predico dove si trovano i track a tempo t+1
    - associo track-detections
    - update del track con la nuova detection associata(eventuale rimozioni e aggiunte)
*/
void Tracker::track(const std::vector<double> &centroids_x,
                    const std::vector<double> &centroids_y,
                    bool lidarStatus)
{

    std::vector<bool> associated_detections(centroids_x.size(), false);         //alla fine avremo associato quel cluster (con centroide x) ad un track con la ::dataAssociation

    //TODO: predict delle track
    for (auto &t : tracks_)
        t.predict();            //causerà kf_.predit()
    

    // TODO: Associate the predictions with the detections
    dataAssociation(associated_detections, centroids_x, centroids_y);

    // Update tracklets with the new detections
    for (int i = 0; i < associated_track_det_ids_.size(); ++i)
    {
        auto det_id = associated_track_det_ids_[i].first;
        auto track_id = associated_track_det_ids_[i].second;    //id del track

        //prendo il determinato track  "tracks_[track_id]" e lo aggiorno con la nuova misura sua "centroids_x[det_id], centroids_y[det_id]"
        tracks_[track_id].update(centroids_x[det_id], centroids_y[det_id], lidarStatus);        //se statoLidar == false (perchè premuto "v" nel render) allora non verrà effettuata la misura
    }

    // 5. Rimuovi track obsoleti
    removeTracks();

    // 6. Aggiungi nuovi tracklet per le detection non associate
    addTracks(associated_detections, centroids_x, centroids_y);
}















// Questa era la versione mia che si basava su una pesatura della distanza in base all'angolo tra la direzione della track e il vettore
// track --> detection    (maggiore l'angolo (quindi cos(teta)) ==> aumento della distanza)
/*
void Tracker::dataAssociation(std::vector<bool> &associated_detections,
                              const std::vector<double> &centroids_x,
                              const std::vector<double> &centroids_y)
{
    associated_track_det_ids_.clear();

    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        int closest_point_id = -1;
        double min_weighted_dist = std::numeric_limits<double>::max();

        // Predizione del tracklet i (posizione e velocità)
        double track_x = tracks_[i].getX();
        double track_y = tracks_[i].getY();
        double track_vx = tracks_[i].getVX();
        double track_vy = tracks_[i].getVY();

        // Direzione del moto del track
        Eigen::Vector2d dir_track(track_vx, track_vy);
        double speed = dir_track.norm();

        // Se il track è quasi fermo → ignora la direzione
        bool has_valid_direction = speed > 1e-3;
        if (has_valid_direction)
            dir_track.normalize();

        // Cerca detection più "coerente"
        for (size_t j = 0; j < centroids_x.size(); ++j)
        {
            if (associated_detections[j]) continue;  // detection già assegnata

            double det_x = centroids_x[j];
            double det_y = centroids_y[j];

            // Vettore posizione relativa detection-track
            Eigen::Vector2d dir_det(det_x - track_x, det_y - track_y);
            double dist = dir_det.norm();
            if (dist < 1e-6) dist = 1e-6;  // evita divisioni per zero

            // Calcolo del coseno dell'angolo
            double angle_penalty = 1.0;
            if (has_valid_direction)
            {
                dir_det.normalize();
                double cos_theta = dir_track.dot(dir_det);
                // clamp tra -1 e 1
                cos_theta = std::max(-1.0, std::min(1.0, cos_theta));

                // penalità crescente con l’angolo
                angle_penalty = 1.0 + (1.0 - cos_theta);
            }

            // distanza pesata dalla direzione
            double weighted_dist = dist * angle_penalty;

            // Mantieni la detection più coerente
            if (weighted_dist < min_weighted_dist)
            {
                min_weighted_dist = weighted_dist;
                closest_point_id = j;
            }
        }

        // Se la detection trovata è sotto soglia → associa
        if (closest_point_id != -1 && min_weighted_dist < distance_threshold_)
        {
            associated_track_det_ids_.push_back(std::make_pair(closest_point_id, i));
            associated_detections[closest_point_id] = true;
        }
    }
}

*/