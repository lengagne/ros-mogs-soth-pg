#!/bin/bash
# 
# 
# mkdir auto_install
# cd auto_install
# 
# # a telecharger depuis la version github
# # installation of the soth (used in the MoGS_Simulator)
# echo "StrictHostKeyChecking no" >> ~/.ssh/config
# git clone --recursive https://github.com/laas/soth.git
# cd soth
# git checkout topic/eigen_3_2
# git submodule init
# git submodule update
# mkdir build
# cd build/
# cmake -DCMAKE_BUILD_TYPE=Release ..
# make
# sudo make install
# cd ../..
