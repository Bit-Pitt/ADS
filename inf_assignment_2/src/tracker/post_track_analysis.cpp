#include "tracker/post_track_analysis.h"
#include <fstream>
#include <cmath>
#include <iomanip>

namespace post_analysis
{

    //Per ottenere tutte le track insieme
    std::vector<Tracklet> collectAllTracks(const Tracker &tracker)
    {
        std::vector<Tracklet> total_tracks;

        auto active = tracker.getTracks();
        total_tracks.insert(total_tracks.end(), active.begin(), active.end());

        auto lost = tracker.getLostTracks();
        total_tracks.insert(total_tracks.end(), lost.begin(), lost.end());

        auto archived = tracker.getArchivedTracks();
        total_tracks.insert(total_tracks.end(), archived.begin(), archived.end());

        return total_tracks;
    }

    // Stampa di debug di tutte le track
    void printTracksSummary(std::vector<Tracklet> &tracks)
    {
        std::cout << "\n========== TRACK HISTORY SUMMARY ==========\n";

        for (auto &t : tracks)
        {
            std::cout << "Track ID " << t.getId()
                    << " (" << t.getHistory().size() << " points)"
                    << " | LossCount=" << t.getLossCount() << "\n";

            const auto &history = t.getHistory();
            for (size_t i = 0; i < history.size(); ++i)
            {
                std::cout << std::fixed << std::setprecision(3)
                        << "  [" << i << "] X=" << history[i].x
                        << ", Y=" << history[i].y << "\n";
            }

            std::cout << "------------------------------------------\n";
        }

        std::cout << "==========================================\n";
    }


    // Per esportare in CSV da usare in python
    void exportTracksToCSV(std::vector<Tracklet> &tracks,
                        const std::string &filename)
    {
        std::ofstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "[ERROR] Impossibile aprire il file " << filename << "\n";
            return;
        }

        file << "track_id,step,x,y,loss_count\n";

        for (auto &t : tracks)
        {
            const auto &history = t.getHistory();
            for (size_t i = 0; i < history.size(); ++i)
            {
                file << t.getId() << "," << i << ","
                    << history[i].x << "," << history[i].y << ","
                    << t.getLossCount() << "\n";
            }
        }

        file.close();
        std::cout << "[INFO] Track export completata su " << filename << "\n";
    }


    //Calcola lunghezza percorso e salva su file
    std::map<int, double> computeTrackDistances(std::vector<Tracklet> &tracks)
    {
        std::map<int, double> distances;

        for (auto &t : tracks)
        {
            const auto &history = t.getHistory();
            double total_dist = 0.0;

            for (size_t i = 1; i < history.size(); ++i)
            {
                double dx = history[i].x - history[i - 1].x;
                double dy = history[i].y - history[i - 1].y;
                total_dist += std::sqrt(dx * dx + dy * dy);
            }

            distances[t.getId()] = total_dist;
        }

        // === Esporta anche su CSV ===
        const std::string filename = "track_distances.csv";
        std::ofstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "[ERROR] Impossibile aprire il file " << filename << " per scrittura.\n";
            return distances;
        }

        file << "track_id,distance\n";
        for (const auto &p : distances)
        {
            file << p.first << "," << std::fixed << std::setprecision(3) << p.second << "\n";
        }

        file.close();
        std::cout << "[INFO] File '" << filename << "' generato con "
                << distances.size() << " track.\n";

        return distances;
    }


    //Decisa un area [xmin,xmax] [ymin,ymax] computo quali track ci sono state e per quanto
    void computeTracksInArea(std::vector<Tracklet> &tracks,
                         double x_min, double x_max,
                         double y_min, double y_max)
    {
        std::map<int, int> steps_in_area; // track_id -> numero di step dentro l’area


        for (auto &t : tracks)
        {
            const auto &history = t.getHistory();
            int count = 0;

            for (const auto &p : history)
            {
                if (p.x >= x_min && p.x <= x_max && p.y >= y_min && p.y <= y_max)
                    count++;
            }

            if (count > 0)
                steps_in_area[t.getId()] = count;
        }

        std::string output_csv = "../build/steps_in_area.csv";

        std::ofstream out(output_csv);
        if (!out.is_open())
        {
            std::cerr << "[ERROR] Impossibile creare il file di output: " << output_csv << std::endl;
            return;
        }

        out << "track_id,steps_in_area\n";
        for (std::map<int, int>::iterator it = steps_in_area.begin(); it != steps_in_area.end(); ++it)
        {
            out << it->first << "," << it->second << "\n";
        }
        out.close();

        std::cout << "[INFO] Analisi area completata.\n";
        std::cout << "[INFO] Risultati salvati in: " << output_csv << std::endl;
    }

    
}


