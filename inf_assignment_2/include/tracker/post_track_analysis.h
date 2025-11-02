
#ifndef POST_TRACK_ANALYSIS_H_
#define POST_TRACK_ANALYSIS_H_


#include <vector>
#include <map>
#include <string>
#include <iostream>
#include "tracker/Tracklet.h"
#include "tracker/Tracker.h"


namespace post_analysis
{

//Per ottenere tutte le track insieme
std::vector<Tracklet> collectAllTracks(const Tracker &tracker);

// Stampa di debug di tutte le track
void printTracksSummary(std::vector<Tracklet> &tracks);


// Per esportare in CSV
void exportTracksToCSV(std::vector<Tracklet> &tracks,const std::string &filename);

// Computa il percorso totale di ogni track
std::map<int, double> computeTrackDistances(std::vector<Tracklet> &tracks);

// Conta quanti step di ciascuna traccia cadono in una determinata area (x_min, x_max, y_min, y_max)
void computeTracksInArea(std::vector<Tracklet> &tracks,
                         double x_min, double x_max,
                         double y_min, double y_max);


} 

#endif // POST_TRACK_ANALYSIS_H_
