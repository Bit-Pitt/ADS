                        SECONDO ASSIGNMENT

Di seguito sono riportate le modifiche fatte

1)
- Tracker con KF:

    - data association  implementata con Mahalanobis Distance  (molto stabile e affermata in letteratura)
        - in aggiunta ho ideato e implementato una variante che sostanzialmente calcola l'angolo tra la direzione della traccia e il vettore [track_predetta -> detections], l'euristica è che se l'angolo è piccolo allora è simile alla direzione della traccia nell'istante precedente per cui è viene premiata e al contrario se l'angolo aumenta viene penalizzata (il codice si trova commentato in fondo in tracker.cpp tuttavia è meno robusta per cui ho tenuto la Mahalanobis)

- Aggiunta e rimozione delle track
    idea di base: 
        -  una detection non associata --> nuova track
        - una track non associata per un certi numero di frame o alta covarianza --> track persa
    Estensione:
    L'idea è provare a riconoscere in certi casi delle nuove track come in realtà il ritrovamento di una persa, per questo le track recentemente perse vengono temporaneamente tenute e se si presenta una nuova detection in un punto sufficientemente vicino (distance_threashold) ad una track recente persa allora viene "riesumata" la precedente
    Si può consultare l'implementazione in addTracks() e removeTrack()

2) Commento di scenari dovuti alla modifica dei parametri del KF
Variazione di R:
    - Aumentando R(0,0) e R(1,1), e quindi il rumore che si associa alle misure si riscontra una maggiore instabilità del filtro 
    che aumenta molto nel momento in cui le tracks si muovono e sono vicine tra loro, portando molto più di prima le track a scambiarsi tra loro e/o a perdersi. In conclusione essendo le misure del LiDAR precise è logico che trattandole come più rumorose l'efficiacia del filtro deteriorasse.


Variazione di Q:
    - Aumentando la covarianza di Q e quindi l'incertezza del nostro modello di poco (2/3x) non porta ad un sostanziale cambiamento ma un aumento più alto porta anche in questo caso ad una alta instabilità del modello nei momenti in cui c'è movimento delle track andando a scambiarsi frequentamente.
    Mentre con un Q basso la traiettoria del filtro segue fedelmente quella della persone quando si perdono frame (es con il tasto "v"), aumentando il Q spesso il filtro crea una traiettoria che diverge con quella effettiva della persona portando quindi a perdere o scambiare le  track. In definita quindi modellare il rumore del modello come eccessivamente incerto porta giustamente ad avere comportamenti poco stabili quando le track sono vicine tra loro.

Variazione di P:
    Sia un  grande aumento che diminuzioni di P (min provato: 1, max provato: 99999) non hanno portato a delle differenze sostanziali, facendo intuire che grazie all'altra freq di aggiornamento del lidar e della sua precisione i valori di P iniziali tendono a non avere un effettivo impatto, andando a convergere velocemente in valori bassi.


FUNZIONALITA' IMPLEMENTATE    con annessi plot creati in python salvando l'output su file
- Distanza percorsa da ogni track ==> (barplot)
- Determinata un area (nel main) ==> grafico (track_id, step) effettuati nell'area   (barplot)
- Mappa del percorso di ogni track ==>   (grafico 2D)

*Le funzionalità sono state implementate in un file "post_track_analysis.cpp" con proprio header per pulizia del codice, per cui è stato modificato il  CMakeLists.
Queste analisi vengono effettuate "offline" ovvero a fine ciclo di lettura delle PCL, per implementarlo ho impostato un numero di frame nel main (nella parte in alto per la configurazione) che fa terminare il processo dopo MAX_FRAMES frame.  (per cui cambiato anche il cloud manager che anche lui dovrà stopparsi insieme al main)

Il codice è commentato e ho mantenuto le stampe su terminale perchè documentano come si comporta il codice a fronte delle funzionalità aggiunte.

A seguito di una run dovrebbe creare 3 file CSV, successivamente runnare lo script in python (che ha path dinamici) per ottenere i plot.




