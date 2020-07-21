//      main.cpp
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
//      2009-2011:  Joint robotics Laboratory - CNRS/AIST,Tsukuba, Japan.
//      2011-2012:  Karlsruhe Institute fur Technologie, Karlsruhe, Germany
//      2012-2013: IUT de Beziers/ LIRMM, Beziers, France
//      2012-2013:  IUT de Beziers/ LIRMM, Beziers, France
//	from 2013:  Université Blaise Pascal / axis : ISPR / theme MACCS

#include <iostream>
#include "MogsProject.h"
#include "config_mogs_core.h"

//
#include <QApplication>

void print_help()
{
	std::cout<<"You can call the following routines" <<std::endl;
	std::cout<<"\t mogs2 help : Print this help"<<std::endl;
	std::cout<<"\t mogs2 new project_file : Create a new project file"<<std::endl;
	std::cout<<"\t mogs2 copy file1.xml file2.xml : Copy a project file"<<std::endl;
	std::cout<<"\t mogs2 open project_file.xml : Open a project file and do some tests"<<std::endl;
	std::cout<<"\t mogs2 open project_file.xml add robot robot_name robot_xml: add a robot to the project"<<std::endl;
	std::cout<<"\t mogs2 open project_file.xml rm robot robot_name : remove a robot to the project"<<std::endl;
	std::cout<<"\t mogs2 open project_file.xml add problem problem_name problem_xml: add a problem to the project"<<std::endl;
	std::cout<<"\t mogs2 open project_file.xml rm problem problem_name : remove a problem to the project"<<std::endl;
	std::cout<<"\t mogs2 open project_file.xml ls problem|robot|object"<<std::endl;
	std::cout<<"\t mogs2 plugins show : Show the plugins and the libraries."<<std::endl;
}

int main (int argc, char *argv[])
{
	std::cout<<" **********************************************************"<<std::endl;
	std::cout<<" **********************************************************"<<std::endl;
	std::cout<<" ******* Welcome in Motion Generation Software (MoGS) *****"<<std::endl;
	std::cout<<" ****************** Version "<<VERSION_MAJOR<<"."<<VERSION_MINOR<<" ***************************"<<std::endl;
	std::cout<<" **********************************************************"<<std::endl;
	std::cout<<" **********************************************************"<<std::endl;
//	qDebug()<<"argc = "<< argc ;

	// this application is required due to QXmlValidator
    QApplication a(argc, argv);

	if (argc == 3)
	{
		mogs_string arg1 = argv[1];
		mogs_string arg2 = argv[2];
		qDebug()<<"arg1 = "<< arg1 ;
		if (arg1 == "new")
		{
			// FIXME check the extension of the file.
			std::cout<<"Asking for new project"<<std::endl;
			mogs_string cmd = "cp " + mogs_string(TEMPLATE_REPOSITORY) + "/mogs_project.xml " + arg2;
			qDebug()<<cmd;
			int dummy = system(cmd.toAscii());
			return 1;
		}
	}
//
// 	// init
	if (argc > 1)
	{
		mogs_string arg1 = argv[1];
		if (arg1 == "help")
		{
			print_help();
			return 1;
		}

		if (arg1 == "copy" && argc == 4)
		{
			mogs_string project1 = argv[2];
			mogs_string project2 = argv[3];
			mogs_string cmd = "cp " + project1+" " + project2;
			qDebug()<<cmd;
			int dummy = system(cmd.toAscii());
			return 1;

		}

		if (arg1 == "open" && argc >= 3)
		{
			mogs_string project_name = argv[2];
			// qDebug()<<"Opening "<< project_name;
			MogsProject project(project_name);
			mogs_string arg3 = argv[3];
			if (arg3 == "add")
			{
				mogs_string type = argv[4];
				if(type == "robot")
				{
					qDebug()<<"Adding robot to project "<< project_name<<" argc = "<< argc;
					mogs_string robot_name = argv[5];
					QFileInfo tool (argv[6]);
					mogs_string robot_url = tool.absoluteFilePath();
					if(argc == 7)
					{
						project.add_robot(robot_name,robot_url);
					}else{
						print_help();
					}

					return 1;
				}else if(type == "scalable_robot")
				{
					qDebug()<<"Adding robot to project "<< project_name<<" argc = "<< argc;
					mogs_string robot_name = argv[5];
					QFileInfo tool (argv[6]);
					mogs_string robot_url = tool.absoluteFilePath();

                    std::cout<<"size = "<< argv[7]<<std::endl;
				    double size = atof(argv[7]);
				    std::cout<<"size = "<< size<<std::endl;
				    double weight = atof(argv[8]);
				    double power = atof(argv[9]);
					project.add_robot(robot_name,robot_url,size,weight,power);
					return 1;
				}else if (type == "problem")
				{
					qDebug()<<"Adding problem to project "<< project_name<<" argc = "<< argc;
					mogs_string problem_name = argv[5];
					QFileInfo tool (argv[6]);
					mogs_string problem_url = tool.absoluteFilePath();
					if(argc == 7)
					{
						project.add_problem(problem_name,problem_url);
					}else{
						print_help();
					}
					return 1;
				}else if (type == "object")
				{
					qDebug()<<"Adding object to project "<< project_name<<" argc = "<< argc;
					mogs_string object_name = argv[5];
					QFileInfo tool (argv[6]);
					mogs_string object_url = tool.absoluteFilePath();
					if(argc == 7)
					{
						project.add_object(object_name,object_url);
					}else{
						print_help();
					}
					return 1;
				}
			}
			if (arg3 == "ls")
			{
				mogs_string type = argv[4];
				if(type == "robot")
				{
                    project.print_robots();
					return 1;
				}else if(type == "problem")
				{
					project.print_problems();
					return 1;
				}else if(type == "object")
				{
					project.print_objects();
					return 1;
				}
			}

			if (arg3 == "rm")
			{
				mogs_string type = argv[4];
				if(type == "robot")
				{
					qDebug()<<"Removing robot to project "<< project_name<<" argc = "<< argc;
					mogs_string robot_name = argv[5];
					if(argc == 6)
					{
						project.delete_robot(robot_name);
					}else{
						print_help();
					}

					return 1;
				}else if(type == "problem")
				{
					qDebug()<<"Removing problem to project "<< project_name<<" argc = "<< argc;
					mogs_string problem_name = argv[5];
					if(argc == 6)
					{
						project.delete_problem(problem_name);
					}else{
						print_help();
					}

					return 1;
				}else if(type == "object")
				{
					qDebug()<<"Removing object to project "<< project_name<<" argc = "<< argc;
					mogs_string object_name = argv[5];
					if(argc == 6)
					{
						project.delete_problem(object_name);
					}else{
						print_help();
					}

					return 1;
				}
			}
			if (arg3 == "solve")
			{
				mogs_string type = argv[4];
				if(type == "problem")
				{
					qDebug()<<"Solving problem "<< project_name;
					mogs_string problem_name = argv[5];
					if(argc == 6)
					{
						project.solve(problem_name);
					}else{
						print_help();
					}
					return 1;
				}
			}
		}

		if (arg1 == "plugins" && argc >= 3)	// no check of show
		{
			mogs_string command = argv[2];
			if (command == "show")
			{
				MogsProblemClassifier pbcl;
				pbcl.print_plugins_with_libs();
				pbcl.print_plugins_problem_with_libs();
				pbcl.print_plugins_robot_with_libs();
				return 1;
			}

			if (command == "add")
			{
				mogs_string type = argv[3];
				mogs_string name = argv[4];
				mogs_string lib = argv[5];
				MogsProblemClassifier pbcl;
				if(type == "problem")
				{
					pbcl.add_library_problem(name,lib);
					return 1;
				}
				if(type == "robot")
				{
					pbcl.add_library_robot(name,lib);
					return 1;
				}
				pbcl.add_library_plugins(type,name,lib);
				return 1;
			}
		}
	}
	print_help();

	return a.exec();
}
