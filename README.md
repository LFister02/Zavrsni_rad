# Razvoj sustava za planiranje putanje bespilotnih letjelica

Repozitorij sadrži razvojni ROS 2 paket `a_star_test` unutar kojeg se razvijao A* algoritam za planiranje putanje na DJI Tello dronu. Unutar paketa se nalaze svi potrebni čvorovi i `.launch` datoteke za testiranje i vizualizaciju rada algoritma u RViz-u, simulaciju rada u Gazebo simulatoru i rad na stvarnoj letjelici.

Direktorij `calibration` sadrži podatke potrebne za proces kalibracije i dobivene rezultate kalibracije kamere koji su se iskorišteni u `tello_ros` driveru i `ORB-SLAM3` wrapper-u za definiranje konfiguracijske datoteke kamere. Za proces kalibracije se koristio [Kalibr](https://github.com/ethz-asl/kalibr) alat za kalibraciju monokularne kamere.

`Gazebo` direktorij sadrži modele koji su korišteni za izradu simulacijskog okruženja unutar kojeg je testiran rad algoritma i gdje je pripremljena podloga za rad na stvarnoj letjelici. Modeli su izrađeni u CAD alatima, a definirani su `model.config` i `model.sdf` datotekama uz koje dolazi STL datoteka za njihov geometrijski opis i vizualizaciju.

Sve izmjene koje su napravljene na preuzetim paketima nalaze se u direktoriju `izmjene`.

## Preuzeti paketi
### [tello_ros](https://github.com/clydemcqueen/tello_ros?tab=readme-ov-file#tello_ros)
`Tello_ros` je driver za rad na DJI Tello i Tello EDU dronovima. Paket je korišten za izradu simulacije i testiranje sustava unutar iste. Također, paket nudi mogućnost povezivanja sa stvarnom letjelicom te njeno upravljanje.

### [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
ORB-SLAM3 je sustav za vizualni SLAM (Simultana lokalizacija i mapiranje) koji radi u realnom vremenu, a nudi mogućnost korištenja različitih senzora. U ovom slučaju se koristio uz monokularno kameru na Tello dronu za mapiranje prostora i lokalizaciju unutar istog.

<pre> @article{ORBSLAM3_TRO,
  title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
           and Multi-Map {SLAM}},
  author={Campos, Carlos AND Elvira, Richard AND G\´omez, Juan J. AND Montiel, 
          Jos\'e M. M. AND Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics}, 
  volume={37},
  number={6},
  pages={1874-1890},
  year={2021}
 } </pre>

### [ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2)
Ovaj paket je ROS 2 wrapper za ORB-SLAM3, a njegova funkcija je da omogućiti korištenje ORB-SLAM3 sustava u ROS 2 okruženju.
