
                    PARTICLE FILTER ASSIGNMENT

- come gli altri progetti nel main è presente un namespace "config" da cui si può velocemente modificare i parametri del particle filter

- REPORT:  scritto su un file a parte mandato per e-mail


- Implementazione del metodo di resampling:

L'implementazione scelta (a cui ho aggiunto feature aggiuntive che tratto dopo) consiste nel sistematic resampling "estremo". L'idea è nata perchè ho pensato che la versione vista a lezione della resampling wheel per un n-->inf consistesse nel prendere uniformemente nuove particelle in relazioni al loro peso, allora ho pensato di eseguire direttamente un resampling sistematico con N=num_particelle andando ad approssimare il comportamente della resampling wheel ma a costo molto minore (O(n) invece di O(n^2)). 
Un ulteriore idea, sempre nata dal laboratorio visto in classe è stata quella di aggiungere del rumore per una maggiore esplorazione delle particelle cercando di diminuire la convergenza prematura del filtro.


- Funzionalità aggiuntive "on top of the pf".

Ottimizzazioni:
    - Associazione landmark <-> obs.id
Mentre nella versione base questa associazione avveniva a costo O(n_landmark) cercando l'id corrispondente, adesso l'associazione avviene a tempo costante sfruttando l'ordinamento del vettore "landmark_mappa".

    - Come anticipato il resampling avviene a costo lineare anzichè quadratico.

    - Adaptive particle filter
Per implementare un filtro adattivo (diminuendo e aumentando particelle) ho consultato "https://arxiv.org/pdf/1602.03572" che parla dell' ESS, una metrica che permette di capire l'attuale varianza delle particelle e scegliere quindi se aumentare o diminuirne il numero.
E' stato quindi aggiunta questa funzionalità prima del resampling, e sempre a seconda dell'ESS viene anche eventualmente skippata la fase.
In sostanza un ESS basso comporta che il peso di poche particelle domina la popolazione  ==> rischio degenerazione e quindi resampling
Ess alto al contrario la popolazione ha una buona varianza ==> potremmo diminuire le particelle o skippare il resampling


*** NOTA:
- All'inizio ho avuto un po di problemi a far girare il codice (l'installazione di ros come nel readme non funzionava ...) alla fine ho cambiato la disposizione delle cartelle per farlo funzionare, il progetto ora:

build/
data/
install/
log/
src/
  pf/
    include/
      particle/
        CircleFit.h
        helper_cloud.h
        helper_functions.h
        map.h
        particle_filter.h
      Box.hpp
      Renderer.hpp
    src/
      main.cpp
      particle_filter.cpp
      Renderer.cpp
    CMakeLists.txt
    package.xml
    cmakepatch.txt
    pf_slam.txt


