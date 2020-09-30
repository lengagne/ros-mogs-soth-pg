# ros-mogs-soth-pg

This package use th stack of stack located in : https://github.com/stack-of-tasks/soth


After cloning this repository do : 
git submodule init
git submodule update

go in ThirdParty/soth and do
git submodule init
git submodule update

To compile the soth
cd ThirdParty/soth
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Realease ..
make
(sudo) make install

To compile go in ThirdParty
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make 
(sudo) make install
