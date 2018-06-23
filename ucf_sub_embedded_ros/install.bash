cd ~
mkdir RoboSub_depends
cd RoboSub_depends
git clone https://github.com/RoboticsClubatUCF/ucf_utilities.git
cd ucf_utilities
mkdir build
cd build
cmake ..
make
sudo make install
cd ../..
git clone https://github.com/open-source-parsers/jsoncpp.git
cd jsoncpp
mkdir build
cd build
cmake ..
make
sudo make install
cd ../..
