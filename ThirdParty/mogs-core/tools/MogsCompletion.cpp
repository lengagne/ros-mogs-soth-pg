//      MoGS_completion.cpp
//      Copyright (C) 2012 lengagne (lengagne@gmail.com)
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

#include <iostream>
#include <string>
#include "MogsProject.h"
#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
	if(mogs_string(argv[argc-1]) != "args")
	{
		std::cerr<<"Error in the completion file"<<std::endl;
		exit(0);
	}
	if (argc == 2)
// 		std::cout<<"new robot environment problem copy export_all plugins create_result_file"<<std::endl;
		std::cout<<"new copy open help plugins "<<std::endl;
	else
	{
		mogs_string command(argv[1]);
		if( command == "copy")
		{
			if (argc == 3)
				std::cout<<"file: xml"<<std::endl;
			if (argc == 4)
				std::cout<<" project.xml"<<std::endl;
		}else if (command == "open")
		{
			if (argc == 3 || argc == 7)
				std::cout<<"file: xml"<<std::endl;
			if (argc == 4)
				std::cout<<"add ls rm solve "<<std::endl;
			if (argc == 5)
			{
				mogs_string cmd = argv[3];
				if (cmd == "add" || cmd =="rm")
					std::cout<<"scalable_robot robot problem object"<<std::endl;
				else if (cmd == "ls")
					std::cout<<"problem object robot"<<std::endl;
				else if (cmd == "solve");
					std::cout<<"problem"<<std::endl;
			}
			if (argc == 6)
			{
				mogs_string cmd = argv[3];
				mogs_string type = argv[4];
// 				qDebug()<<cmd;
				if (cmd == "add")
					std::cout<<"name?"<<std::endl;
				else if (cmd == "rm" && type =="object")
				{
					MogsProject project;
					project.read_project(argv[2]);
					project.print_objects();
				}else if (cmd == "rm" && type =="robot")
				{
					MogsProject project;
					project.read_project(argv[2]);
					project.print_robots();
				}else if (cmd == "rm" && type =="problem")
				{
					MogsProject project;
					project.read_project(argv[2]);
					project.print_problems();
				}else if (cmd == "solve" && type =="problem")
				{
					MogsProject project;
					project.read_project(argv[2]);
					project.print_problems();
				}
			}

			if (argc ==7)
            {
				mogs_string cmd = argv[3];
				mogs_string type = argv[4];
				if (cmd == "add" && (type =="robot" || type =="scalable_robot"))
                    std::cout<<"file: [xml,urdf]"<<std::endl;
				if (cmd == "add" && type =="object")
                    std::cout<<"file: [xml,stl,dae,wrl]"<<std::endl;
            }

			if (argc ==8)
            {
				mogs_string cmd = argv[3];
				mogs_string type = argv[4];
				if (cmd == "add" && type =="scalable_robot")
                    std::cout<<"taille?"<<std::endl;
            }
			if (argc ==9)
            {
				mogs_string cmd = argv[3];
				mogs_string type = argv[4];
				if (cmd == "add" && type =="scalable_robot")
                    std::cout<<"poids?"<<std::endl;
            }
			if (argc ==10)
            {
				mogs_string cmd = argv[3];
				mogs_string type = argv[4];
				if (cmd == "add" && type =="scalable_robot")
                    std::cout<<"puissance?"<<std::endl;
            }

		}else if (command == "plugins" )
		{
		    if (argc == 3)
            {
                std::cout<<"add show"<<std::endl;
            }else if (argc ==4)
            {
                mogs_string cmd = argv[2];
                if (cmd =="add")
                    std::cout<<"problem robot NewType?"<<std::endl;
            }else if (argc == 5){
                std::cout<<"Name?"<<std::endl;
            }else if (argc == 6)
                std::cout<<"lib?"<<std::endl;
		}
	}

	return 1;
}
