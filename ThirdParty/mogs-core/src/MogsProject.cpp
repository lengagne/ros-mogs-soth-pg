//      MogsProject.cpp
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
//      2012-2013:  IUT de Beziers/ LIRMM, Beziers, France
//	from 2013:  Université Blaise Pascal / axis : ISPR / theme MACCS

#include <iomanip>      // std::setprecision
#include "MogsProject.h"
#include <cstdlib>	// system()

MogsProject::MogsProject ()
{
	project_is_read_ = false;
}

MogsProject::~MogsProject ()
{

}

MogsProject::MogsProject(const mogs_string & project_name)
{
	if (!read_project(project_name))
	{
		std::cerr<<"Error when trying to read the project"<<std::endl;
		exit(0);
	}
}

void MogsProject::add_object (  const mogs_string & name,
                                const mogs_string & url)
{
	if (!project_is_read_)
	{
		std::cerr << " You have to read the project first !!" <<std::endl;
		exit (0);
	}
	if (!object_defined (name))
	{
		QDomElement object = doc_->createElement ("object");
		ElObjects_.appendChild (object);

        QDomElement ELname = doc_->createElement ("name");
        QDomText text = doc_->createTextNode (name);
        ELname.appendChild (text);
        object.appendChild (ELname);

        QDomElement ELxml = doc_->createElement ("file");
        text = doc_->createTextNode (url);
        ELxml.appendChild (text);
        object.appendChild (ELxml);

        save();
	}
	else
	{
		std::cerr <<"This object already exists in the project file" <<std::endl;
		exit (0);
	}
	std::cout << " object added" << std::endl;
}

void MogsProject::add_problem (	const mogs_string & name,
                                const mogs_string & url)
{
	if (!project_is_read_)
	{
		std::cerr << " You have to read the project first !!" <<std::endl;
		exit (0);
	}
	if (!problem_defined (name))
	{
		MogsProblemClassifier pcl;
		if (pcl.set_problem_type (url, &problem_))
		{
			QDomElement problem = doc_->createElement ("problem");
			ElProblems_.appendChild (problem);

			QDomElement ELname = doc_->createElement ("name");
			QDomText text = doc_->createTextNode (name);
			ELname.appendChild (text);
			problem.appendChild (ELname);

			QDomElement ELtype = doc_->createElement ("type");
			text = doc_->createTextNode (problem_->get_type());
			ELtype.appendChild (text);
			problem.appendChild (ELtype);

			QDomElement ELxml = doc_->createElement ("xml_file");
			text = doc_->createTextNode (url);
			ELxml.appendChild (text);
			problem.appendChild (ELxml);

			save();
		}
		else
		{
			std::cerr <<"Error in "<< __FILE__<<" at line:"<< __LINE__<<"."<< std::endl;
			std::cerr <<"There is an error when loading the problem file "<< url.toStdString() << std::endl;
			exit(0);
		}
	}
	else
	{
		std::cerr <<"This problem already exists in the project file" <<std::endl;
		exit (0);
	}
	std::cout << " problem added" << std::endl;
}

void MogsProject::add_robot (	const mogs_string & name,
                                const mogs_string & url,
                                const double size,
                                const double weight,
                                const double power)
{
	if (!project_is_read_)
	{
		std::cerr << " You have to read the project first !!" <<std::endl;
		exit (0);
	}
 	if (!robot_defined (name))
 	{
		MogsAbstractRobot* robot;
		MogsProblemClassifier pcl;
		pcl.set_scalable_robot_type(url,&robot,size,weight,power);
#ifdef DEBUG
		qDebug()<<"Robot type = "<< robot->get_type();
#endif
		robot->set_name(name);
		if (robot->is_ready ())
		{
			QDomElement Elrobot = doc_->createElement ("robot");
			Elrobot.setAttribute ("scalable", "yes");

			QDomElement ELname = doc_->createElement ("name");
			QDomText text = doc_->createTextNode (name);
			ELname.appendChild (text);
			Elrobot.appendChild (ELname);

			QDomElement ELtype = doc_->createElement ("robot_type");
			text = doc_->createTextNode (robot->get_type());
			ELtype.appendChild (text);
			Elrobot.appendChild (ELtype);

			QDomElement ELxml = doc_->createElement ("xml_file");
			text = doc_->createTextNode (url);
			ELxml.appendChild (text);
			Elrobot.appendChild (ELxml);

            std::cout<<"size = "<< size <<std::endl;
			QDomElement ELsize = doc_->createElement ("size");
			text = doc_->createTextNode (QString::number(size, 'f', 4));
			ELsize.appendChild (text);
			Elrobot.appendChild (ELsize);

            QDomElement ELweight = doc_->createElement ("weight");
			text = doc_->createTextNode (QString::number(weight, 'f', 4));
			ELweight.appendChild (text);
			Elrobot.appendChild (ELweight);

            QDomElement ELpower = doc_->createElement ("power");
			text = doc_->createTextNode (QString::number(power, 'f', 4));
			ELpower.appendChild (text);
			Elrobot.appendChild (ELpower);

			ElRobots_.appendChild (Elrobot);
			save();
		}else{
			std::cerr<<"The provided xml is not suitable"<<std::endl;
		}
	}
	else
	{
		std::cerr << "This robot already exists in the project file"<< std::endl;
		exit (0);
	}
}


void MogsProject::add_robot (	const mogs_string & name,
                                const mogs_string & url)
{
	if (!project_is_read_)
	{
		std::cerr << " You have to read the project first !!" <<std::endl;
		exit (0);
	}
 	if (!robot_defined (name))
 	{
		MogsAbstractRobot* robot;
		MogsProblemClassifier pcl;
		pcl.set_robot_type(url,&robot);
#ifdef DEBUG
		qDebug()<<"Robot type = "<< robot->get_type();
#endif
		robot->set_name(name);
		if (robot->is_ready ())
		{
			QDomElement Elrobot = doc_->createElement ("robot");
			Elrobot.setAttribute ("scalable", "no");

			QDomElement ELname = doc_->createElement ("name");
			QDomText text = doc_->createTextNode (name);
			ELname.appendChild (text);
			Elrobot.appendChild (ELname);

			QDomElement ELtype = doc_->createElement ("robot_type");
			text = doc_->createTextNode (robot->get_type());
			ELtype.appendChild (text);
			Elrobot.appendChild (ELtype);

			QDomElement ELxml = doc_->createElement ("xml_file");
			text = doc_->createTextNode (url);
			ELxml.appendChild (text);
			Elrobot.appendChild (ELxml);

			ElRobots_.appendChild (Elrobot);
			save();
		}else{
			std::cerr<<"The provided xml is not suitable"<<std::endl;
		}
	}
	else
	{
		std::cerr << "This robot already exists in the project file"<< std::endl;
		exit (0);
	}
}

void MogsProject::delete_problem (const mogs_string & name)
{
	if (!project_is_read_)
	{
		std::cerr << " You have to read the project first !!" << std::endl;
		exit (0);
	}
	if (problem_defined (name))
	{
		  ElProblems_.removeChild (ElProblem_);
		  save();
	}
	else
	{
		std::cerr <<"This problem does not exist and you want to delete it (strange isn't it ?)"<< std::endl;
		exit (0);
	}
}

void MogsProject::delete_robot (const mogs_string &name)
{
	if (!project_is_read_)
	{
		std::cerr << " You have to read the project first !!" << std::endl;
		exit (0);
	}
	if (robot_defined (name))
	{
		ElRobots_.removeChild (ElRobot_);
		save();
	}
	else
	{
		std::cerr <<"This robot does not exist and you want to delete it (strange isn't it ?)"<< std::endl;
		exit (0);
	}
}

mogs_string MogsProject::get_problem_url(const mogs_string & name)
{
	if (!project_is_read_)
	{
		std::cerr << " You have to read the project first !!" << std::endl;
		exit (0);
	}
	if (problem_defined (name))
	{
		QDomElement Elname = ElProblem_.firstChildElement ("xml_file");
		return Elname.text ().trimmed();
	}
	else
	{
		std::cerr <<"This problem ("<<name.toStdString()<<") not exist "<< std::endl;
		exit (0);
	}
}

mogs_string MogsProject::get_robot_url( const mogs_string & name)
{
	if (!project_is_read_)
	{
		std::cerr << " You have to read the project first !!" << std::endl;
		exit (0);
	}
	if (robot_defined (name))
	{
		QDomElement Elname = ElRobot_.firstChildElement ("xml_file");
		return Elname.text ();
	}
	else
	{
		std::cerr <<"This robot ("<<name.toStdString()<<") not exist "<< std::endl;
		exit (0);
	}
}

void MogsProject::init_problem()
{
	std::vector<mogs_string> robot_names = problem_->get_robots_name();
// 	std::vector<mogs_string> robot_url;
// 	for (int i=0;i<robot_names.size();i++)
// 		qDebug()<<robot_names[i];
// 		robot_url.push_back(get_robot_url(robot_names[i]));
// 	problem_->set_robot_url(robot_url);


	std::vector<MogsRobotProperties*> robots;
	for (int i=0;i<robot_names.size();i++)
	{
		bool robot_not_found = true;
		for (ElRobot_= ElRobots_.firstChildElement ("robot"); !ElRobot_.isNull(); ElRobot_ = ElRobot_.nextSiblingElement("robot"))
		{
			QDomElement Elname = ElRobot_.firstChildElement ("name");
			if (Elname.text() == robot_names[i])
			{
				// read the robot
				MogsRobotProperties* tmp_robot = new MogsRobotProperties();
				mogs_string  scalable = ElRobot_.attribute ("scalable");

				mogs_string type = "undefined_robot_type";
				mogs_string xml = "undefined_xml";

				QDomElement ELtype = ElRobot_.firstChildElement ("robot_type");
				if(!ELtype.isNull())	type = ELtype.text ().trimmed();
				else{	std::cerr<<"robot_type not defined for "<< robot_names[i].toStdString()<<std::endl;	exit(0);}

				QDomElement ELxml = ElRobot_.firstChildElement ("xml_file");
				if(!ELxml.isNull())	xml = ELxml.text ();
				else{	std::cerr<<"xml_file not defined for "<< robot_names[i].toStdString()<<std::endl;	exit(0);}

				if(convert_to_bool(scalable))
				{

					double size = 1.0;
					double weight = 50.0;
					double power = 1.0;

					// read scalable robot


					QDomElement ELsize = ElRobot_.firstChildElement ("size");
					if(!ELsize.isNull())	size = ELsize.text ().toDouble();
					else{	std::cerr<<"size not defined for "<< robot_names[i].toStdString()<<std::endl;	exit(0);}

					QDomElement ELweight = ElRobot_.firstChildElement ("weight");
					if(!ELweight.isNull())	weight = ELweight.text ().toDouble();
					else{	std::cerr<<"weight not defined for "<< robot_names[i].toStdString()<<std::endl;	exit(0);}

					QDomElement ELpower = ElRobot_.firstChildElement ("power");
					if(!ELpower.isNull())	power = ELpower.text ().toDouble();
					else{	std::cerr<<"power not defined for "<< robot_names[i].toStdString()<<std::endl;	exit(0);}

					tmp_robot->set_scale (size, weight, power);

				}else
				{
						// read non scalable robot

				}
// 				qDebug()<<" xml = "<<xml;
				tmp_robot->SetRobotFile(xml);
				tmp_robot->setRobotName(robot_names[i]);
				robots.push_back(tmp_robot);
				robot_not_found = false;
			}
		}
		if(robot_not_found)
		{
			std::cerr<<"Error the the Robot " << robot_names[i].toStdString() <<" is unknown for this project."<<std::endl;
			exit(0);
		}
	}
	problem_->set_robots(robots);

	// load the objects
	std::vector<mogs_string> object_names = problem_->get_objects_name();
	std::vector< Eigen::Matrix < double, 3, 1 >> pos = problem_->get_objects_position();
	std::vector< Eigen::Matrix < double, 3, 1 >> rot = problem_->get_objects_rotation();
	std::vector<MogsGeometry*> objects;
	for (int i=0;i<object_names.size();i++)
	{
		bool object_not_found = true;
		for (ElObject_= ElObjects_.firstChildElement ("object"); !ElObject_.isNull(); ElObject_ = ElObject_.nextSiblingElement("object"))
		{
			QDomElement Elname = ElObject_.firstChildElement ("name");
// 			qDebug()<<"Elname.text().simplified()  = " << Elname.text().simplified() ;
// 			qDebug()<<"object_names["<<i<<"] = "<< object_names[i];
			if (Elname.text().simplified() == object_names[i])
			{
				// read the object
				mogs_string xml = "undefined_xml";

				QDomElement ELxml = ElObject_.firstChildElement ("file");
				if(!ELxml.isNull())	xml = ELxml.text ();
				else{	std::cerr<<"file not defined for "<< object_names[i].toStdString()<<std::endl;	exit(0);}

				MogsGeometry* obj = new MogsGeometry(xml);
				obj->move(pos[i],rot[i]);
                                objects.push_back(obj);
                                object_not_found = false;
			}
		}
		if(object_not_found)
		{
		    std::cerr<<"Error in "<< __FILE__<<" at line "<< __LINE__<<"."<<std::endl;
			std::cerr<<"Error the object " << object_names[i].toStdString() <<" is unknown for this project."<<std::endl;
			exit(0);
		}
	}
	problem_->set_objects(objects);
        
        
        std::vector<additionnal_robot_body_mesh> add_meshes = problem_->get_additionnal_meshes();
        unsigned int nb = add_meshes.size();
        for (int i =0;i<nb;i++)
        {
  		bool object_not_found = true;
		for (ElObject_= ElObjects_.firstChildElement ("object"); !ElObject_.isNull(); ElObject_ = ElObject_.nextSiblingElement("object"))
		{
			QDomElement Elname = ElObject_.firstChildElement ("name");
			if (Elname.text().simplified() == add_meshes[i].mesh_name__)
			{
				// read the object
				mogs_string xml = "undefined_xml";

				QDomElement ELxml = ElObject_.firstChildElement ("file");
				if(!ELxml.isNull())	xml = ELxml.text ();
				else{	std::cerr<<"file not defined for "<< object_names[i].toStdString()<<std::endl;	exit(0);}

				MogsGeometry* obj = new MogsGeometry(xml);
				obj->move(add_meshes[i].position__,add_meshes[i].rotation__);
                                add_meshes[i].mesh__ = obj;
                                object_not_found = false;
			}
		}
		if(object_not_found)
		{
                        std::cerr<<"Error in "<< __FILE__<<" at line "<< __LINE__<<"."<<std::endl;
			std::cerr<<"Error the object " << object_names[i].toStdString() <<" is unknown for this project."<<std::endl;
			exit(0);
		}          
        }
        problem_->set_additionnal_meshes(add_meshes);
}

bool MogsProject::object_defined (const mogs_string &name)
{
	for (ElObject_ = ElObjects_.firstChildElement ("object"); !ElObject_.isNull(); ElObject_ = ElObject_.nextSiblingElement("object"))
	  {
		  QDomElement Elname = ElObject_.firstChildElement ("name");
		  mogs_string name_xml = Elname.text ();
		  if (name_xml == name)
			  return true;
	  }
	return false;
}

void MogsProject::print_objects()
{
	std::vector < mogs_string > v;
	for (ElObject_ = ElObjects_.firstChildElement ("object"); !ElObject_.isNull(); ElObject_ = ElObject_.nextSiblingElement("object"))
	  {
		  QDomElement Elname = ElObject_.firstChildElement ("name");
		  v.push_back (Elname.text ());
	  }
	std::sort (v.begin (), v.end ());
	for (int i = 0; i < v.size (); i++)
		std::cout << v[i].toStdString() << std::endl;
}


void MogsProject::print_problems ()
{
	std::vector < mogs_string > v;
	for (ElProblem_ = ElProblems_.firstChildElement ("problem"); !ElProblem_.isNull(); ElProblem_ = ElProblem_.nextSiblingElement("problem"))
	  {
		  QDomElement Elname = ElProblem_.firstChildElement ("name");
		  v.push_back (Elname.text ());
	  }
	std::sort (v.begin (), v.end ());
	for (int i = 0; i < v.size (); i++)
		std::cout << v[i].toStdString() << std::endl;
}

void MogsProject::print_robots ()
{
	std::vector < mogs_string > v;
	for (ElRobot_= ElRobots_.firstChildElement ("robot"); !ElRobot_.isNull(); ElRobot_ = ElRobot_.nextSiblingElement("robot"))
	  {
		  QDomElement Elname = ElRobot_.firstChildElement ("name");
		  v.push_back (Elname.text());
	  }
	std::sort (v.begin (), v.end ());
	for (int i = 0; i < v.size (); i++)
		std::cout << v[i].toStdString() << std::endl;
}

bool MogsProject::problem_defined (const mogs_string &name)
{
	for (ElProblem_ = ElProblems_.firstChildElement ("problem"); !ElProblem_.isNull(); ElProblem_ = ElProblem_.nextSiblingElement("problem"))
	  {
		  QDomElement Elname = ElProblem_.firstChildElement ("name");
		  mogs_string name_xml = Elname.text ();
		  if (name_xml == name)
			  return true;
	  }
	return false;
}

bool MogsProject::read_project (const mogs_string & filename)
{
    project_file_ = mogs_get_absolute_link(filename);
    // test the if the xml fit the xsd
    QUrl schemaUrl(XSD_PROJECT_FILE);
    QXmlSchema schema;
    schema.load(schemaUrl);
    if (schema.isValid()) {
        QFile file(project_file_);
        file.open(QIODevice::ReadOnly);
        QXmlSchemaValidator validator(schema);
#ifdef DEBUG
        qDebug()<<" xsd is valid";
#endif
        if (! validator.validate(QUrl(QString("file://")+project_file_)))
        {
            qDebug() << "instance document "<< QString("file://")+project_file_ << " is invalid";
            return false;
        }
#ifdef DEBUG
        else
            qDebug() << "instance document "<< QString("file://")+path << " is valid";
#endif
    }else
    {
        qDebug()<<" Error the xsd "<< XSD_PROJECT_FILE<< " is not valid";
        exit(0);
    }
    doc_ = new QDomDocument("Project");
    QFile file(project_file_);
    if (!file.open(QIODevice::ReadWrite))
    {
        return false;
    }
    if (!doc_->setContent(&file)) {
        file.close();
        return false;
    }
    file.close();
 	project_is_read_ = true;
//
	root_ = doc_->documentElement ();
	if (root_.isNull())
	{
		std::cerr << " Error cannot find the root" << std::endl;
		exit (0);
	}

	ElObjects_ = root_.firstChildElement ("objects");
	if (ElObjects_.isNull())
	{
		std::cerr << " Error cannot find the objects" << std::endl;
		exit (0);
	}

	ElRobots_ = root_.firstChildElement ("robots");
	if (ElRobots_.isNull())
	{
		std::cerr << " Error cannot find the Robots" << std::endl;
		exit (0);
	}

	ElProblems_ = root_.firstChildElement ("problems");
	if (ElProblems_.isNull())
	{
		std::cerr << " Error cannot find the problems" << std::endl;
		exit (0);
	}
	return true;
}
//
bool MogsProject::robot_defined (const mogs_string &name)
{
	for (ElRobot_ = ElRobots_.firstChildElement ("robot"); !ElRobot_.isNull(); ElRobot_ = ElRobot_.nextSiblingElement("robot"))
	  {
		  QDomElement Elname = ElRobot_.firstChildElement ("name");
		  mogs_string name_xml = Elname.text ();
		  if (name_xml == name)
			  return true;
	  }
	return false;
}

void MogsProject::save ()
{
	if (project_is_read_)
    {
        QFile file(project_file_);
        file.open(QIODevice::ReadWrite);
       	QByteArray xml = doc_->toByteArray();
        file.write(xml);
        file.close();
    }
}

void MogsProject::solve(const mogs_string& name)
{
	if (!project_is_read_)
	{
		std::cerr << " You have to read the project first !!" <<std::endl;
		exit (0);
	}
	if (problem_defined (name))
	{
		MogsProblemClassifier pcl;
		if (pcl.set_problem_type (get_problem_url(name), &problem_))
		{
			init_problem();
			problem_->solve();
		}
		else
		{
			std::cerr <<"Error in "<< __FILE__<<" at line:"<< __LINE__<<"."<< std::endl;
			std::cerr <<"There is an error when loading the problem file "<< get_problem_url(name).toStdString() << std::endl;
		}
	}
	else
	{
		std::cerr <<"This problem "<< name.toStdString()<<" is not defined" <<std::endl;
		exit (0);
	}
	std::cout << " problem solved" << std::endl;
}
