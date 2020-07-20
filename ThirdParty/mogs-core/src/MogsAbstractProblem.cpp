//      MogsAbstractProblem.cpp
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
//      from 2013:  Université Blaise Pascal / axis : ISPR / theme MACCS
#include "MogsAbstractProblem.h"
#include "MogsProblemClassifier.h"
MogsAbstractProblem::MogsAbstractProblem(): collision_(NULL)
{
}
MogsAbstractProblem::~MogsAbstractProblem()
{
        if(collision_)
        {
                destroy_collision_(collision_);
                dlclose(collision_construct_);
        }
}

void MogsAbstractProblem::set_additionnal_meshes(const std::vector<additionnal_robot_body_mesh>& in)
{
    add_meshes_ = in;
    // add meshes to body of robots
    unsigned int nb_add = add_meshes_.size();
    for (int i=0;i<nb_add;i++) 
    {
        bool robot_found = false;
        for (int j=0;j<nb_robots_;j++)  if (add_meshes_[i].robot_name__ == robots_name_[j])
        {
            robot_found = true;
            robots_[j]->add_mesh_to_body(add_meshes_[i].body_name__, add_meshes_[i].mesh__);
        }
        
        if (!robot_found)
        {
            std::cerr<<"Error in "<<__FILE__<<" at line "<< __LINE__<<std::endl;
            std::cerr<<"Cannot find the robot "<< add_meshes_[i].robot_name__.toStdString()<<std::endl;
            exit(23);            
        }
    }
}

void MogsAbstractProblem::read_problem (const mogs_string & filename)
{
    QUrl schemaUrl(QString("file://") + mogs_get_absolute_link(xsd_file_));
    QFileInfo fxml(filename);
    QXmlSchema schema;
    schema.load(schemaUrl);
    if (schema.isValid()) {
        QFile file(filename);
        file.open(QIODevice::ReadWrite);
        QXmlSchemaValidator validator(schema);
        if (!validator.validate(QUrl(QString("file://")+fxml.absoluteFilePath())))
            qDebug() << "instance document "<< filename << " is invalid regarding "<< xsd_file_;
    }else
    {
        qDebug()<<" Error the xsd "<< xsd_file_<< " is not valid"<<endl;
    }
        xml_file_ = filename;
    file_ = new QFile(xml_file_);
    if (!file_->open(QIODevice::ReadOnly)) {
        std::cerr<<"Error in "<<__FILE__<<" at line : "<< __LINE__<<std::endl;
        std::cerr<<"Cannot open the file "<< xml_file_.toStdString() <<std::endl;
        exit(0);
    }
    // Parse file
    if (!doc_.setContent(file_)) {
        std::cerr<<"Error in "<<__FILE__<<" at line : "<< __LINE__<<std::endl;
       std::cerr<<"Cannot parse the content of "<< xml_file_.toStdString()<<std::endl;
       file_->close();
       exit(0);
    }
    file_->close();
        root_ = doc_.documentElement();
        assert (!root_.isNull());
        problem_type_ = root_.attribute ("type");
        for (QDomElement Elrobot = root_.firstChildElement ("robot"); !Elrobot.isNull(); Elrobot = Elrobot.nextSiblingElement("robot"))
        {
                el_robots_.push_back(Elrobot);
                QDomElement Elrobot_name = Elrobot.firstChildElement ("name");
                robots_name_.push_back (Elrobot_name.text());
//              qDebug()<<"Finding robot "<< Elrobot_name.text();
//              Robot_xml_.push_back ("no_xml_for_the_moment");
                Eigen::Matrix < double, 3, 1 > tmp;
                tmp(0) = tmp(1) = tmp(2) = 0.;
                QDomElement ElPosition = Elrobot.firstChildElement ("position");
                if (!ElPosition.isNull())
                {
                        std::istringstream spos (ElPosition.text ().toStdString(), std::ios_base::in);
                        for (int i = 0; i < 3; i++)
                                spos >> tmp(i);
                }
                robot_position_.push_back(tmp);
                tmp(0) = tmp(1) =tmp(2) = 0.;
                QDomElement ElRotation = Elrobot.firstChildElement ("rotation");
                if (!ElRotation.isNull())
                {
                        std::istringstream srot (ElRotation.text ().toStdString(), std::ios_base::in);
                        for (int i = 0; i < 3; i++)
                                srot >> tmp(i);
                }
                robot_orientation_.push_back(tmp);
                std::map<mogs_string,mogs_string> config;
                for (QDomElement Elconfig_file = Elrobot.firstChildElement ("config_file"); !Elconfig_file.isNull(); Elconfig_file = Elconfig_file.nextSiblingElement("config_file"))
            {
                config[Elconfig_file.attribute("type").simplified()] = Elconfig_file.text().simplified();
            }
            robot_config_files_.push_back(config);
        }
        nb_robots_ = robots_name_.size ();
        std::cout<<"There are "<< nb_robots_<<" robots."<<std::endl;
        for (QDomElement ElObject = root_.firstChildElement ("object"); !ElObject.isNull(); ElObject = ElObject.nextSiblingElement("object"))
        {
                QDomElement ElObject_name = ElObject.firstChildElement ("name");
                objects_name_.push_back (ElObject_name.text().simplified());
                qDebug()<<"Finding object "<< ElObject_name.text().simplified();
                Eigen::Matrix < double, 3, 1 > tmp;
                tmp(0) = tmp(1) = tmp(2) = 0.;
                QDomElement ElPosition = ElObject.firstChildElement ("position");
                if (!ElPosition.isNull())
                {
                        std::istringstream spos (ElPosition.text ().toStdString(), std::ios_base::in);
                        for (int i = 0; i < 3; i++)
                                spos >> tmp(i);
                }
                objects_position_.push_back(tmp);
                tmp(0) = tmp(1) =tmp(2) = 0.;
                QDomElement ElRotation = ElObject.firstChildElement ("rotation");
                if (!ElRotation.isNull())
                {
                        std::istringstream srot (ElRotation.text ().toStdString(), std::ios_base::in);
                        for (int i = 0; i < 3; i++)
                                srot >> tmp(i);
                }
                objects_orientation_.push_back(tmp);
        }
        
        for (QDomElement ElAddMesh = root_.firstChildElement ("add_mesh"); !ElAddMesh.isNull(); ElAddMesh = ElAddMesh.nextSiblingElement("add_mesh"))
        {
                additionnal_robot_body_mesh tmp;
                QDomElement ElAddMesh_name = ElAddMesh.firstChildElement ("name");
                if (!ElAddMesh_name.isNull())
                    tmp.mesh_name__ = ElAddMesh_name.text().simplified();
                else
                {
                        std::cerr<<"Error in "<< __FILE__<<" at line "<< __LINE__ <<std::endl;
                        std::cerr<<"add_mesh must have a a balise name "<< std::endl;
                        exit(2);
                }
                
                QDomElement ElRobot_name = ElAddMesh.firstChildElement ("ref_robot");
                if (!ElRobot_name.isNull())
                    tmp.robot_name__ = ElRobot_name.text().simplified();
                else
                {
                        std::cerr<<"Error in "<< __FILE__<<" at line "<< __LINE__ <<std::endl;
                        std::cerr<<"add_mesh must have a a balise ref_robot "<< std::endl;
                        exit(2);
                }                

                QDomElement ElBody_name = ElAddMesh.firstChildElement ("ref_body");
                if (!ElBody_name.isNull())
                    tmp.body_name__ = ElBody_name.text().simplified();
                else
                {
                        std::cerr<<"Error in "<< __FILE__<<" at line "<< __LINE__ <<std::endl;
                        std::cerr<<"add_mesh must have a a balise ref_body "<< std::endl;
                        exit(2);
                }    
                
                QDomElement ElPosition = ElAddMesh.firstChildElement ("position");
                if (!ElPosition.isNull())
                    tmp.position__ = convert_to_vec3(ElPosition);
                else
                {
                    tmp.position__ = Eigen::Matrix<double,3,1>::Zero();
                }                 
                
                QDomElement ElRotation = ElAddMesh.firstChildElement ("orientation");
                if (!ElRotation.isNull())
                    tmp.rotation__ = convert_to_vec3(ElRotation);
                else
                {
                    tmp.rotation__ = Eigen::Matrix<double,3,1>::Zero();
                }                  
                tmp.mesh__ = NULL;
                add_meshes_.push_back(tmp);
        }        
        
        QDomElement Elcoll = root_.firstChildElement ("collision_detector");
        if (!Elcoll.isNull())
        {
                mogs_string coll_type = Elcoll.attribute ("type").simplified();
                MogsProblemClassifier classifier;
                mogs_string library_so_output;
                if ( classifier.get_library_plugin("collision_detector",coll_type,library_so_output))
                {
                        // load the library
                        collision_construct_ = dlopen(library_so_output.toAscii(), RTLD_LAZY);
                        if (!collision_construct_) {
                                std::cerr << "Cannot load library: " << dlerror() << '\n';
                                exit(0);
                        }
                        // load the symbols
                        create_collision_ = (create_collision*) dlsym(collision_construct_, "create");
                        destroy_collision_ = (destroy_collision*) dlsym(collision_construct_, "destroy");
                        if (!create_collision_ || !destroy_collision_)
                        {
                                std::cerr << "Cannot load symbols: " << dlerror() << '\n';
                                exit(0);
                        }
                        // create an instance of the class
                        collision_ = create_collision_(Elcoll);
                }else
                {
                        std::cerr<<"Error in "<<__FILE__<<" at line "<< __LINE__<<std::endl;
                        qDebug()<<"The collision_detection "<< coll_type<<" is not defined";
                        exit(0);
                }
        }
}
void MogsAbstractProblem::set_robots(std::vector<MogsRobotProperties*> robots)
{
    robots_ = robots;
    for (int i=0;i<nb_robots_;i++)
    {
        qDebug()<<"Finding robot "<< robots_[i]->getRobotName()<<" that has "<< robots_[i]->getNDof()<<" dof.";
        robots_[i]->set_root_transformation(robot_position_[i],robot_orientation_[i]);
        robots_[i]->set_config_files(robot_config_files_[i]);
    }
}
