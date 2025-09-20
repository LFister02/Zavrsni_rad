# Razvoj sustava za planiranje putanje bespilotnih letjelica

Repozitorij sadrži razvojni ROS 2 paket `a_star_test` unutar kojeg se razvijao A* algoritam za planiranje putanje na DJI Tello dronu. Unutar paketa se nalaze svi potrebni čvorovi i `launch` datoteke za testiranje i vizualizaciju rada algoritma u RViz-u, simulaciju rada u Gazebo simulatoru i rad na stvarnoj letjelici.

Direktorij `calibration` sadrži podatke potrebne za proces kalibracije i dobivene rezultate kalibracije kamere koji su iskorišteni u `tello_ros` driveru i `ORB-SLAM3` wrapper-u za definiranje konfiguracijskih datoteka kamere. Za proces kalibracije se koristio [Kalibr](https://github.com/ethz-asl/kalibr) alat za kalibraciju monokularne kamere.

`Gazebo` direktorij sadrži modele koji su korišteni za izradu simulacijskog okruženja unutar kojeg je testiran rad algoritma i gdje je pripremljena podloga za rad na stvarnoj letjelici. Modeli su izrađeni u CAD alatima, a definirani su `model.config` i `model.sdf` datotekama uz koje dolazi STL datoteka za njihov geometrijski opis i vizualizaciju.

Sve izmjene koje su napravljene na preuzetim paketima nalaze se u direktoriju `izmjene`.

## Preuzeti paketi
### [tello_ros](https://github.com/clydemcqueen/tello_ros?tab=readme-ov-file#tello_ros)
`Tello_ros` je driver za rad na DJI Tello i Tello EDU dronovima. Paket je korišten za izradu simulacije i testiranje sustava unutar iste. Također, paket nudi mogućnost povezivanja sa stvarnom letjelicom te njeno upravljanje.

### [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
ORB-SLAM3 je sustav za vizualni SLAM (Simultana lokalizacija i mapiranje) koji radi u realnom vremenu, a nudi mogućnost korištenja različitih senzora. U ovom slučaju se koristio uz monokularnu kameru na Tello dronu za mapiranje prostora i lokalizaciju unutar istog.

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

### [Kalibr](https://github.com/ethz-asl/kalibr)
Kalibr alat za kalibraciju monokularne kamere omogućava određivanje unutarnjih i vanjskih parametara kamere pomoću kalibracijskih uzoraka poput šahovnice, aprilgrid-a ili ArUco markera. Dobiveni parametri se koriste za uklanjanje distorzija i precizno mapiranje 3D točaka svijeta na 2D slike. Proces kalibracije uključuje snimanje uzoraka iz različitih kutova i udaljenosti, nakon čega Kalibr daje vrijednosti kalibracijskih parametara kamere, procjenu poza tijekom procesa te iznose pogrešaka.

#### Literatura
1. Joern Rehder, Janosch Nikolic, Thomas Schneider, Timo Hinzmann, Roland Siegwart (2016). Extending kalibr: Calibrating the extrinsics of multiple IMUs and of individual axes. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 4304-4311, Stockholm, Sweden.
2. Paul Furgale, Joern Rehder, Roland Siegwart (2013). Unified Temporal and Spatial Calibration for Multi-Sensor Systems. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Tokyo, Japan.
3. Paul Furgale, T D Barfoot, G Sibley (2012). Continuous-Time Batch Estimation Using Temporal Basis Functions. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 2088–2095, St. Paul, MN.
4. J. Maye, P. Furgale, R. Siegwart (2013). Self-supervised Calibration for Robotic Systems, In Proc. of the IEEE Intelligent Vehicles Symposium (IVS)
5. L. Oth, P. Furgale, L. Kneip, R. Siegwart (2013). Rolling Shutter Camera Calibration, In Proc. of the IEEE Computer Vision and Pattern Recognition (CVPR)
