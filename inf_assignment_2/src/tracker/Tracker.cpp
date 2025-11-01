#include "tracker/Tracker.h"
#include <iostream>

Tracker::Tracker()
{
    cur_id_ = 0;
    distance_threshold_ = 1.0;       // 1 m: per associare detections vicine
    covariance_threshold = 50.0;     // media delle varianze x,y (unità: m²)  [quindi +-max 7m]
    loss_threshold = 15;         //max num frame che non vedo la track

}
Tracker::~Tracker()
{
}


/* 
    Rimozioni secondo una delle  2 condizioni
*   - 1) troppi frame persi dell'oggetto (Loss_count > soglia)
*   - 2) covarianza di P (incertezza modello) troppo alta
*/  
void Tracker::removeTracks()
{
    std::vector<Tracklet> tracks_to_keep;
    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        Tracklet &t = tracks_[i];

        // 1) cond
        bool lost_too_long = t.getLossCount() > loss_threshold;

        // 2) cond
        double pos_uncertainty = (t.getXCovariance() + t.getYCovariance()) / 2.0;
        bool too_uncertain = pos_uncertainty > covariance_threshold;

        if (!(lost_too_long || too_uncertain))
            tracks_to_keep.push_back(t);
        else        //per debug
            std::cout << "Removed Track ID: " << t.getId()
                      << " (loss_count=" << t.getLossCount()
                      << ", uncertainty=" << pos_uncertainty << ")\n";
    }
    tracks_.swap(tracks_to_keep);
}


/*
    This function add new tracks to the set of tracks ("tracks_" is the object that contains this)
*/
void Tracker::addTracks(const std::vector<bool> &associated_detections, const std::vector<double> &centroids_x, const std::vector<double> &centroids_y)
{
    // Adding not associated detections
    for (size_t i = 0; i < associated_detections.size(); ++i)
        if (!associated_detections[i])
            tracks_.push_back(Tracklet(cur_id_++, centroids_x[i], centroids_y[i]));
}

/*
    This function associates detections (centroids_x,centroids_y) with the tracks (tracks_)
    Input:
        associated_detection an empty vector to host the associated detection
        centroids_x & centroids_y measurements representing the detected objects
*/
void Tracker::dataAssociation(std::vector<bool> &associated_detections,     //sono vettori "paralleli"
                              const std::vector<double> &centroids_x,
                              const std::vector<double> &centroids_y)
{
    associated_track_det_ids_.clear();

    // Per ogni track cerco il cluster più vicino (centroidi x,y sono i "centri" del cluster)
    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        int closest_point_id = -1;
        double min_dist = std::numeric_limits<double>::max();

        // Posizione predetta del tracklet i  (la predict è stata fatta prima in ::track())
        double track_x = tracks_[i].getX();
        double track_y = tracks_[i].getY();
        double track_vx = tracks_[i].getVX();
        double track_vy = tracks_[i].getVY();

        //Calcola qui la direzione della track:
        // basta ottenere il vettore somma di vx e vy

        // Confronta con tutte le detection (cluster) del frame corrente per trovare la più vicina e salvarla in "closest_point_id"
        for (size_t j = 0; j < centroids_x.size(); ++j)
        {
            if (associated_detections[j]) 
                continue;  //skip se già assegnata

            double det_x = centroids_x[j];
            double det_y = centroids_y[j];

            double dx = track_x - det_x;
            double dy = track_y - det_y;
            double dist = std::sqrt(dx * dx + dy * dy);

            //Adesso qui la direzione è il vettore da track_(x,y) e det_(x,y) e maggiore è l'angolo che ha con la direzione
            // della track maggiore sarà aumentata la distanza! (magari normalizziamo uno 0,1 a cui poi aggiungiamo 1)
            //Ovvero stessa direzione ==> angolo 0 aggiungi 1 e quindi dist = dist*1 (invariata), angolo opposto verrà dist*2.

            // Mantieni la detection più vicina
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_point_id = j;
            }
        }

        // Se il più vicino è sotto la soglia di distanza minima, associamo
        if (closest_point_id != -1 && min_dist < distance_threshold_)
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
    - update del track con la nuova detection associata (eventuale rimozioni e aggiunte)
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
