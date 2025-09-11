#  Izmjene
##  ORB-SLAM3 wrapper
Paket za pokretanje traži vocabulary i config datoteke. Dodane su `camera.yaml` i `camera_real.yaml` datoteke koje definiraju parametre kalibracije i distorzije korištene monokularne kamere.<br><br>
`Camera.yaml` se odnosi na kameru i parametre koji se izvorno nalaze u [tello_ros](https://github.com/clydemcqueen/tello_ros), a korišteni tijekom simulacije, dok `camera_real.yaml` sadrži podatke koji su rezultat kalibracije kamere na stvarnom dronu te su oni kasnije korišteni u radu na letjelici.<br><br>
`orbslam3_tello_launch.py` je .launch datoteka koja se koristi za lakše pokretanje alata. Unutar datoteke se definira putanja do vocabulary i config datoteke, način rada i korišteni camera topic. Camera topic se razlikuje za rad u simulaciji i na stvarnoj letjelici, gdje je `/drone1/image_raw` za simulaciju, a `/image_raw` za rad na stvarnoj letjelici.

##  tello_ros
`camera_info.yaml` sadrži podatke o kameri koji su rezultat kalibracije.<br><br>
`tello.xml` je URDF(Unified Robot Description Format) koji služi za opis geometrije i vizualizaciju robota. Dodan je LiDAR senzor koji je kasnije iskorišten za simulaciju senzora blizine u svrhu detekcije dinamičkih prepreka.<br><br>
`tello_joy_node.cpp` i `tello_joy_node.hpp` su dio joystick drivera koji omogućava korištenje kontrolera unutar ROS 2 sustava, a u ovom slučaju se koriste za upravljanje letjelicom uz pomoć Xbox kontrolera.<br>
Unutar `tello_joy_node.cpp` je dodana funkcionalnost da se pritiskom na tipku `A` onemogući slanje brzine letjelici, čime se uklanja problem stick drifta i omogućuje potpuna autonomija letjelice. Druga funkcionalnost je da se pritiskom tipke `X` objavljuje tema /stop_tracker i šalje signal za zaustavljanje letjelice.
