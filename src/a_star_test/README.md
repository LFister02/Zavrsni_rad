#  a_star_test

`a_star_test` je glavni razvojni paket unutar ovog repozirotija, a sadrži sve potrebne ROS 2 čvorove koji obrađuju podatke s ostalih paketa te tako omogućavaju rad sustava.<br><br>

##  Popis ROS 2 čvorova
1. `a_star.py` - čvor korišten za testiranje i vizualizaciju A* algoritma unutar RViz-a. Unutar čvora je moguće mijenjati broj prepreka, veličinu 3D prostora, tip heuristike, način kretanja itd. što je korisno prilikom testiranja.<br><br>
Postoje 4 načina rada:
- `non_random` - pozicije starta i cilja se zadaju unutar čvora
- `random` - nasumične pozicije starta i cilja
- `point_click` - korisnik može odabrati poziciju starta i cilja preko RViz brzih funkcija `2D Pose Estimate` i `2D Goal Pose`
- `penalty` - korišteno za testiranje utjecaja kazne za penjanje, rezlika je u generiranju prepreka različitih visina, dok su u ostalim načinima rada visine prepreka jednake<br><br>
2. `gazebo_a_star.py` - koristi se uz Gazebo simulaciju. Učitava mapu snimljenu pomoću `save_pointcloud.py` te računa optimalnu putanju između trenutačne pozicije drona (dobivena iz SLAM-a) i odabrane točke unutar RViz-a.<br><br>
3. `gazebo_follower.py` - pretplačuje se na temu /astar_path koju objavljuje gazebo_a_star.py i prihvaća listu točaka koje čine putanju do cilja. Pomoću P-regulatora se računa brzina koja se objavljuje na temu `/drone1/cmd_vel` nakon čega dron započinje s kretanjem. P-regulator osigurava precizno praćenje putanje do cilja.<br>
Pritiskom tipke `X` na Xbox kontroleru aktivira se sigurnosno zaustavljanje misije što je korisno u slučaju da dođe do greške.<br><br>
4. `follower_dyn_obs` - funkcija slična `gazebo_follower` uz dodatak reakcije na dinamičku prepreku. Čvor sluša temu koju objavljuje `scan_to_range` i u slučaju detekcije prepreke unutar definirane udaljenosti letjelica se zaustavlja.<br><br>
5. `save_pointcloud` - sprema pointcloud podatke koje objavljuje `octomap_server`<br><br>
6. `scan_to_range` - pretplaćuje se na temu `/scan` koja objavljuje podatke s LiDAR senzora na letjelici i pretvara ih u podatke tipa Range te ih objavljuje na temu `/range_sensor`. Ovaj čvor se koristi s follower_dyn_obs.py za detekciju dinamičkih prepreka<br>


##  Popis launch datoteka
Launch datoteke služe za istovremeno pokretanje većeg broja čvorova.
1. `astar_test_launch.py` - testiranje A* algoritma i vizualizacija u RViz-u. Nudi mogućnost promjene određenih parametara unutar `a_star.py` čvora.<br><br>
2. `gazebo_path_planning_launch.py` - planiranje putanje i navigacija drona unutar Gazebo simulatora<br><br>
3. `gazebo_dynamic_obs_launch.py` - planiranje putanje uz mogućnost reakcije na dinamičku prepreku<br><br>
4. `CRTA_path_planning_launch.py` - planiranje putanje i navigacija na stvarnoj letjelici
