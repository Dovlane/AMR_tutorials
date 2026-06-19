# VM Setup for This Repo

Ovaj vodič opisuje kako da podesiš virtuelnu mašinu tako da možeš da build-uješ i pokrećeš ovaj repozitorijum.

Ciljna konfiguracija:

- host: bilo koji OS koji može da pokrene VirtualBox ili VMware
- guest: `Ubuntu 22.04`
- ROS: `ROS 2 Humble`
- Python: `3`
- build alat: `colcon`

## 1. Kreiranje virtuelne mašine

Preporučene specifikacije:

- RAM: `6 GB` minimum, `8 GB` preporučeno
- CPU: `2` jezgra minimum, `4` preporučeno
- disk: `35 GB` minimum, `50 GB` preporučeno
- video memorija: `128 MB` ako hypervisor to podržava
- mreža: `NAT` je dovoljna

Ako koristiš VirtualBox:

1. instaliraj VirtualBox
2. preuzmi `Ubuntu 22.04 LTS` ISO
3. kreiraj novu virtuelnu mašinu
4. poveži ISO kao boot disk
5. instaliraj Ubuntu standardnim putem

## 2. Osnovno podešavanje Ubuntu sistema

Posle prvog podizanja sistema pokreni:

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y build-essential curl git wget vim
```

Ako koristiš VirtualBox, instaliraj i guest pakete:

```bash
sudo apt install -y virtualbox-guest-utils virtualbox-guest-x11 virtualbox-guest-dkms
```

Zatim restartuj VM:

```bash
sudo reboot
```

## 3. Instalacija ROS 2 Humble

Podesi locale:

```bash
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Omogući `universe` repozitorijum:

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
```

Dodaj ROS 2 repozitorijum:

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Instaliraj ROS 2 Humble:

```bash
sudo apt update
sudo apt install -y ros-humble-desktop
```

Dodaj ROS okruženje u shell:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 4. Instalacija razvojnih alata

```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip
```

Inicijalizuj `rosdep`:

```bash
sudo rosdep init
rosdep update
```

## 5. Zavisnosti za ovaj repozitorijum

Instaliraj potrebne pakete:

```bash
sudo apt install -y \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-msgs \
  ros-humble-turtlebot3-simulations \
  python3-numpy
```

Po želji trajno podesi model:

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

## 6. Kloniranje repozitorijuma

```bash
mkdir -p ~/workspace
cd ~/workspace
git clone <repo-url> AMR_tutorials
cd AMR_tutorials
```

Zameni `<repo-url>` stvarnim URL-om repozitorijuma.

## 7. Build repozitorijuma

Iz root direktorijuma repozitorijuma:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths Kodovi/hello_world Kodovi/move_robot Kodovi/homework_3/line_fitting
```

Posle uspešnog build-a:

```bash
source install/setup.bash
```

Ako želiš da se workspace automatski učitava:

```bash
echo "source ~/workspace/AMR_tutorials/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 8. Provera ROS 2 instalacije

Pre nego što pokreneš kod iz ovog repozitorijuma, proveri da ROS 2 radi pomoću standardnih alata `turtlesim` i `teleop`.

Instaliraj test pakete ako već nisu prisutni:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-turtlesim \
  ros-humble-teleop-twist-keyboard
```

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Ako želiš da `teleop_twist_keyboard` upravlja `turtlesim` čvorom, potrebno je remapirati temu ka `turtle1/cmd_vel`:

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/turtle1/cmd_vel
```

Ako je instalacija ispravna:

- otvoriće se `turtlesim` prozor
- unos sa tastature će pomerati kornjaču
- `ros2` CLI komande će raditi bez greške

Možeš dodatno proveriti aktivne nodove:

```bash
source /opt/ros/humble/setup.bash
ros2 node list
```

Tek nakon toga pređi na build i pokretanje ovog repozitorijuma.

## 9. Česti problemi

Ako je Gazebo spor:

- povećaj RAM na `8 GB`
- povećaj broj CPU jezgara na `4`
- uključi 3D akceleraciju ako hypervisor to podržava
- zatvori teške aplikacije na host mašini

Ako ROS komande nisu dostupne:

```bash
source /opt/ros/humble/setup.bash
```

Ako paketi iz repozitorijuma nisu dostupni:

```bash
cd ~/workspace/AMR_tutorials
source install/setup.bash
```

Ako build posle izmena krene da puca:

```bash
cd ~/workspace/AMR_tutorials
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --base-paths Kodovi/hello_world Kodovi/move_robot Kodovi/homework_3/line_fitting
```

## 10. Pokretanje ovog repozitorijuma

Kada je ROS 2 instalacija proverena, možeš pokretati pakete iz ovog repozitorijuma prema uputstvima iz [README.md](README.md).

## 11. Preporučeni raspored

Preporučena lokacija repozitorijuma:

```text
~/workspace/AMR_tutorials
```

Korisno je da u `~/.bashrc` ostane:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
```
