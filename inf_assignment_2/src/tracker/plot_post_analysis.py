#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import matplotlib.pyplot as plt
import sys
import os


#Istogramma con lunghezza percorso di ogni track
def plot_track_distances(csv_path: str):
    if not os.path.exists(csv_path):
        print(f"[ERRORE] File non trovato: {csv_path}")
        sys.exit(1)

    df = pd.read_csv(csv_path)

    # Controllo colonne
    if "track_id" not in df.columns or "distance" not in df.columns:
        print("[ERRORE] Il file deve contenere le colonne 'track_id' e 'distance'")
        sys.exit(1)

    plt.figure(figsize=(8, 5))  
    #Qui create le colonne
    plt.bar(df["track_id"].astype(str), df["distance"], color="skyblue", edgecolor="black")

    plt.title("Distanza percorsa da ciascuna persona", fontsize=14)
    plt.xlabel("ID della traccia (persona)", fontsize=12)
    plt.ylabel("Distanza percorsa [m]", fontsize=12)

    for i, val in enumerate(df["distance"]):
        plt.text(i, val + 0.02, f"{val:.2f}", ha="center", fontsize=10)

    plt.grid(axis="y", linestyle="--", alpha=0.7)
    plt.tight_layout()

    build_dir = os.path.join(os.path.dirname(csv_path), "")
    output_path = os.path.join(build_dir, "track_distances_plot.png")
    plt.savefig(output_path, dpi=300)
    plt.close()  

    print(f"[INFO] Grafico salvato in: {output_path}")



#Grafico dei percorsi
def plot_track_paths(csv_path: str):
    if not os.path.exists(csv_path):
        print(f"[ERRORE] File non trovato: {csv_path}")
        sys.exit(1)

    df = pd.read_csv(csv_path)

    if not {"track_id", "x", "y"}.issubset(df.columns):
        print("[ERRORE] Il file deve contenere le colonne 'track_id', 'x', 'y'")
        sys.exit(1)

    plt.figure(figsize=(8, 8))
    plt.title("Percorsi tracciati delle persone", fontsize=14)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")

    # ottieni tutti gli ID unici e una colormap
    track_ids = sorted(df["track_id"].unique())
    cmap = plt.colormaps.get_cmap("tab20")  # 20 colori distinti

    # disegna ogni percorso con un colore diverso (riciclando se > 20)
    for i, tid in enumerate(track_ids):
        track_data = df[df["track_id"] == tid]
        color = cmap(i % cmap.N)
        plt.plot(
            track_data["x"],
            track_data["y"],
            marker="o",
            markersize=2,
            linewidth=1.5,
            color=color,
            label=f"Track {tid}"
        )

    plt.legend(title="ID tracce", fontsize=8, loc="upper right", ncol=2)
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.axis("equal")

    output_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../build/track_paths_plot.png"))
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"[INFO] Grafico percorsi salvato in: {output_path}")

#Funzionameno analogo "plot_track_distances"
def plot_steps_in_area(csv_path: str):
    if not os.path.exists(csv_path):
        print(f"[ERRORE] File non trovato: {csv_path}")
        sys.exit(1)

    df = pd.read_csv(csv_path)

    if "track_id" not in df.columns or "steps_in_area" not in df.columns:
        print("[ERRORE] Il file deve contenere le colonne 'track_id' e 'steps_in_area'")
        sys.exit(1)

    plt.figure(figsize=(8, 5))
    # x=traccia, y=step 
    plt.bar(df["track_id"].astype(str), df["steps_in_area"],
            color="lightgreen", edgecolor="black")

    plt.title("Numero di step nell'area per ciascuna traccia", fontsize=14)
    plt.xlabel("ID della traccia (persona)", fontsize=12)
    plt.ylabel("Step nell'area", fontsize=12)

    for i, val in enumerate(df["steps_in_area"]):
        plt.text(i, val + 1, str(val), ha="center", fontsize=10)

    plt.grid(axis="y", linestyle="--", alpha=0.7)
    plt.tight_layout()

    build_dir = os.path.join(os.path.dirname(csv_path), "")
    output_path = os.path.join(build_dir, "steps_in_area_plot.png")
    plt.savefig(output_path, dpi=300)
    plt.close()

    print(f"[INFO] Grafico salvato in: {output_path}")



if __name__ == "__main__":
    script_dir = os.path.dirname(__file__)
    csv_file_distanze = os.path.join(script_dir, "../../build/track_distances.csv")
    csv_file_percorso = os.path.join(script_dir, "../../build/track_history.csv")
    csv_file_area = os.path.join(script_dir, "../../build/steps_in_area.csv")
    plot_track_distances(csv_file_distanze)
    plot_track_paths(csv_file_percorso)
    plot_steps_in_area(csv_file_area)
