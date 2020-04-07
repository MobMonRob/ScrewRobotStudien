# ply-classifier

Dieses Werkzeug dient zur manuellen Klassifikation von Punktwolken im Kontext des Erkennens einer Schraube. Die Punktwolken müssen unter `YOUR_INPUT_PATH` als *.ply-Dateien vorliegen. Die Punktwolken werden während der Klassifikation entsprechend in die Ordner `YOUR_SCREW_PATH` und `YOUR_NOSCREW_PATH` gespeichert.

## Nutzung unter Ubuntu

- Installieren der PointCloudLibrary
  
        sudo apt install libpcl-dev

- Kompilieren
    
        mkdir build && cd build
        cmake ..
        make

- Ausführen

        ./ply-classifier YOUR_INPUT_PATH YOUR_SCREW_PATH YOUR_NOSCREW_PATH
