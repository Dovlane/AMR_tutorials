# ROS 2 Humble Tutorijali

Ovaj repozitorijum pruža kratak pregled osnovnih funkcionalnosti ROS 2 sistema koje će studenti koristiti kasnije tokom izrade domaćih i projektnih zadataka na kursu.

Cilj repozitorijuma nije da bude kompletan kurs iz ROS 2 niti produkcioni robotski softverski sistem. Njegova svrha je da pruži sažet i praktičan uvod u glavne ROS 2 koncepte kroz male primere koji mogu lokalno da se build-uju i pokrenu.

## O Čemu Je Ovaj Repozitorijum

Primeri u ovom repozitorijumu namenjeni su da pomognu studentima da se upoznaju sa:

- strukturom ROS 2 paketa
- `colcon` build postupkom
- Python ROS 2 nodovima zasnovanim na `rclpy`
- komunikacijom preko topika
- publisher i subscriber obrascima
- definisanjem i pozivanjem servisa
- jednostavnim upravljanjem robotom preko ROS 2 interfejsa
- osnovnom obradom senzorskih podataka

Ovi primeri treba da posluže kao početna osnova za kasniji rad na kursu, gde će studenti samostalno razvijati veće ROS 2 aplikacije.

## Podešavanje Okruženja

Nemoj koristiti ovaj `README` kao vodič za podešavanje razvojnog okruženja.

Za pripremu virtuelne mašine, instalaciju Ubuntu sistema, instalaciju ROS 2 Humble verzije, potrebne zavisnosti i proveru instalacije, koristi [VM_SETUP.md](VM_SETUP.md).

## Struktura Repozitorijuma

Repozitorijum trenutno sadrži više ROS 2 tutorijal paketa unutar direktorijuma `Kodovi/`:

- `Kodovi/hello_world`
- `Kodovi/move_robot`
- `Kodovi/homework_3/line_fitting`
- `Kodovi/homework_4/ekf_line_localization`

## Paketi

### `hello_world`

Ovaj paket prikazuje najosnovnije obrasce komunikacije u ROS 2 sistemu:

- publisher
- subscriber
- korisnički definisan servis

Njegova namena je da bude prvi kontakt studenata sa ROS 2 nodovima, topicima i servisima.

### `move_robot`

Ovaj paket prikazuje jednostavan primer upravljanja robotom pomoću servisa.

On pokazuje kako ROS 2 node može da:

- prima odometriju
- objavljuje komande brzine
- izloži korisnički definisan servis
- koordinira kretanje na osnovu ulaza dobijenog kroz servis

Ovaj paket predstavlja vezu između apstraktnih primera ROS 2 komunikacije i jednostavnijih robotskih aplikacija.

### `line_fitting`

Ovaj paket prikazuje jednostavan primer obrade senzorskih podataka zasnovan na `LaserScan` porukama.

On pokazuje kako ROS 2 node može da:

- se pretplati na senzorske podatke
- obradi numerička merenja
- izračuna izvedene veličine
- prikaže rezultate kroz logovanje noda

Ovaj paket je namenjen kao lagan uvod u obradu podataka i percepcione zadatke u ROS 2 sistemu.

## Build

Nakon završetka koraka iz [VM_SETUP.md](VM_SETUP.md), repozitorijum se build-uje iz korenskog direktorijuma:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths Kodovi/hello_world Kodovi/move_robot Kodovi/homework_3/line_fitting
source install/setup.bash
```

Za četvrti domaći, build-uj i paket za linijsku EKF lokalizaciju:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths Kodovi/homework_3/line_fitting Kodovi/homework_4/ekf_line_localization
source install/setup.bash
```

## Zašto Su Ovi Primeri Važni

Repozitorijum je osmišljen tako da studentima omogući da brzo razumeju osnovni ROS 2 radni tok pre nego što pređu na zahtevnije zadatke na kursu.

Posebno, ovi primeri treba da pomognu studentima da se osećaju sigurnije u radu sa:

- kreiranjem i pokretanjem nodova
- ispitivanjem ROS 2 interfejsa
- razumevanjem toka poruka i servisa
- organizacijom paketa
- povezivanjem softverske logike sa ponašanjem robota

Kada ove osnove budu jasne, studenti će biti spremniji za rad na većim zadacima kasnije tokom kursa.
