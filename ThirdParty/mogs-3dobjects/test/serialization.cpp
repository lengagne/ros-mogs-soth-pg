//      serialization.cpp
//      Copyright (C) 2014 lengagne (lengagne@gmail.com)
// 
//      This program is free software: you can redistribute it and/or modify
//      it under the terms of the GNU General Public License as published by
//      the Free Software Foundation, either version 3 of the License, or
//      (at your option) any later version.
// 
//      This program is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//      GNU General Public License for more details.
// 
//      You should have received a copy of the GNU General Public License
//      along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//      This program was developped in the following labs:
//	from 2013 : Université Blaise Pascal / axis : ISPR / theme MACCS

#include <fstream>
#include <iostream>

#include "Mesh.h"

#ifdef Boost_SERIALIZATION_FOUND
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#endif


int main (int argc, char *argv[])
{
#ifdef Boost_SERIALIZATION_FOUND
	Mesh Init;
	Mesh File;
	std::cout << " Test of the file: argv[1] = " << argv[1] << std::endl;
	Init.parse(argv[1]);

	// *************************************
	// ****
	// **** Test before saving
	// ****
	// *************************************

	{
		int nb_geom  = Init.getNumSubGeometries();
		std::cout<<" There is/are "<< nb_geom<<" geometries."<<std::endl;
		for(int i=0;i<nb_geom;i++)
		{
			std::cout<<" The geom("<<i<<") has "<< Init.subGeometries_[i]->points_.size()<<" points."<<std::endl;
			std::cout<<" The first one is : "<< Init.subGeometries_[i]->points_[0].transpose()<<std::endl;			
			std::cout<<" The geom("<<i<<") has "<< Init.subGeometries_[i]->faces_.size()<<" faces_."<<std::endl;
			std::cout<<" The geom("<<i<<") has "<< Init.subGeometries_[i]->texPoints_.size()<<" texPoints_."<<std::endl;
			std::cout<<" The geom("<<i<<") has "<< Init.subGeometries_[i]->texFaces_.size()<<" texFaces_."<<std::endl;
			std::cout<<" The geom("<<i<<") has "<< Init.subGeometries_[i]->normals_.size()<<" normals_."<<std::endl;	
			std::cout<<" The geom("<<i<<") has the ambientIntensity_ "<< Init.subGeometries_[i]->ambientIntensity_ <<std::endl;
		}
	}

	// *************************************
	// ****
	// **** Save the Mesh in file
	// ****
	// *************************************

	{
		std::cout << " *** Saving " << std::endl;

		std::ofstream ofs ("/tmp/Mesh.txt");
		boost::archive::text_oarchive oa (ofs);

		oa << Init;
	}

	// *************************************
	// ****
	// **** Restore the Mesh from file
	// ****
	// *************************************


	{
		std::cout << " *** Reloading " << std::endl;

		std::ifstream ifs ("/tmp/Mesh.txt");
		boost::archive::text_iarchive ia (ifs);

		ia >> File;
		std::cout << " *** Releoad Done " << std::endl;
	}


	// *************************************
	// ****
	// **** Test after loading
	// ****
	// *************************************

	{
		int nb_geom  = File.getNumSubGeometries();
		std::cout<<" There is/are "<< nb_geom<<" geometries."<<std::endl;
		for(int i=0;i<nb_geom;i++)
		{
			std::cout<<" The geom("<<i<<") has "<< File.subGeometries_[i]->points_.size()<<" points."<<std::endl;
			std::cout<<" The first one is : "<< File.subGeometries_[i]->points_[0].transpose()<<std::endl;
			std::cout<<" The geom("<<i<<") has "<< File.subGeometries_[i]->faces_.size()<<" faces_."<<std::endl;
			std::cout<<" The geom("<<i<<") has "<< File.subGeometries_[i]->texPoints_.size()<<" texPoints_."<<std::endl;
			std::cout<<" The geom("<<i<<") has "<< File.subGeometries_[i]->texFaces_.size()<<" texFaces_."<<std::endl;
			std::cout<<" The geom("<<i<<") has "<< File.subGeometries_[i]->normals_.size()<<" normals_."<<std::endl;			
			std::cout<<" The geom("<<i<<") has the ambientIntensity_ "<< File.subGeometries_[i]->ambientIntensity_ <<std::endl;
		}
		
	}	

#else
	std::cout<<" The serialization test is not compiled"<<std::endl;
#endif
	return 0;
}
