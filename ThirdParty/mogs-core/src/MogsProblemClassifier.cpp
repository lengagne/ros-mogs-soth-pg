//      Problem_holder.cpp
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

#include "MogsProblemClassifier.h"
#include <dlfcn.h>

MogsProblemClassifier::MogsProblemClassifier ()
{
	if(!read_xml())
	{
		create_xml();
	}
}

MogsProblemClassifier::~MogsProblemClassifier ()
{
    save_xml();
}

void MogsProblemClassifier::add_library_plugins(	const mogs_string & plugin_type,
							const mogs_string & control_name,
							const mogs_string & library_so)
{
	qDebug()<<"Add the plugin "<<control_name<<" of type "<< plugin_type<<" with the library "<< library_so;
	int user = getuid();
	if (user != 0)
	{
		std::cerr<<"Error you must be root to add library to the plugins !! "<<std::endl;
		exit(0);
	}

	if(!read_xml())
	{
		std::cerr<<"Cannot find the config file"<<std::endl;
		create_xml();
	}

	// check if the problem_name is already defined
	for (int i=0;i<libs_plugins_.size();i++)
	{
		if (libs_plugins_[i].library_name == control_name && libs_plugins_[i].library_type == plugin_type)
		{
			std::cerr<<"The plugin "<< control_name.toStdString()<<" for the type "<< plugin_type.toStdString() <<" is already defined.";
			std::cerr<<" We will replace it."<<std::endl;
			rm_library_plugins(plugin_type,control_name);
		}
	}

	QDomElement ElPlugin = doc_->createElement("plugin");

	QDomElement ElType = doc_->createElement ("type");
	QDomText ElText3 = doc_->createTextNode ( plugin_type);
	ElType.appendChild(ElText3);
	ElPlugin.appendChild(ElType);

	QDomElement ElName = doc_->createElement ("name");
	QDomText ElText = doc_->createTextNode ( control_name);
	ElName.appendChild(ElText);
	ElPlugin.appendChild(ElName);

	QDomElement ElLib = doc_->createElement ("library");
	QDomText ElText2 = doc_->createTextNode ( library_so);
	ElLib.appendChild(ElText2);
	ElPlugin.appendChild(ElLib);

	ElPlugins_.appendChild(ElPlugin);
	save_xml();
}

void MogsProblemClassifier::add_library_problem(const mogs_string & problem_name,
                                                const mogs_string & library_so)
{
	std::cout<<"Add the plugin "<<problem_name.toStdString()<<" with the library "<< library_so.toStdString()<<std::endl;
	int user = getuid();
	if (user != 0)
	{
		std::cerr<<"Error you must be root to add library to the plugins !! "<<std::endl;
		exit(0);
	}

	while(!read_xml())
	{
		std::cerr<<"Cannot find the config file"<<std::endl;
		create_xml();
	}

	// check if the problem_name is already defined
	for (int i=0;i<libs_problem_.size();i++)
	{
		if (libs_problem_[i].library_name == problem_name)
		{
			std::cerr<<"The plugin for the problem "<< problem_name.toStdString()<<" is already defined.";
			std::cerr<<" We will replace it."<<std::endl;
			rm_library_problem(problem_name);
		}
	}

	// add the library in the xml
	QDomElement ElProblem = doc_->createElement ("problem");
	QDomElement ElName = doc_->createElement ("name");
	QDomText ElText = doc_->createTextNode ( problem_name);
	ElName.appendChild(ElText);
	ElProblem.appendChild(ElName);
	QDomElement ElLib = doc_->createElement ("library");
	QDomText ElText2 = doc_->createTextNode ( library_so);
	ElLib.appendChild(ElText2);
	ElProblem.appendChild(ElLib);

	ElProblems_.appendChild(ElProblem);
	save_xml();
}

void MogsProblemClassifier::add_library_robot(	const mogs_string & robot_name,
						const mogs_string & library_so)
{
	std::cout<<"Add the plugin "<<robot_name.toStdString()<<" with the library "<< library_so.toStdString()<<std::endl;
	int user = getuid();
	if (user != 0)
	{
		std::cerr<<"Error you must be root to add library to the plugins !! "<<std::endl;
		exit(0);
	}

	if(!read_xml())
	{
		std::cerr<<"Cannot find the config file"<<std::endl;
		create_xml();
	}

	// check if the problem_name is already defined
	for (int i=0;i<libs_robot_.size();i++)
	{
		if (libs_robot_[i].library_name == robot_name)
		{
			std::cerr<<"The plugin for the problem "<< robot_name.toStdString()<<" is already defined.";
			std::cerr<<" We will replace it."<<std::endl;
			rm_library_problem(robot_name);
		}
	}

	// add the library in the xml
	QDomElement ElRobot = doc_->createElement ("robot");

	QDomElement ElName = doc_->createElement ("name");
	QDomText ElText = doc_->createTextNode ( robot_name);
	ElName.appendChild(ElText);
	ElRobot.appendChild(ElName);

	QDomElement ElLib = doc_->createElement ("library");
	QDomText ElText2 = doc_->createTextNode ( library_so);
	ElLib.appendChild(ElText2);
	ElRobot.appendChild(ElLib);

	ElRobots_.appendChild(ElRobot);
	save_xml();
}


void MogsProblemClassifier::create_xml()
{
    std::string cmd = "cp " + std::string(TEMPLATE_REPOSITORY) +"/plugins.xml " +  std::string(PLUGIN_CONFIG_FILE )+ " -v";
    int dummy=system(cmd.c_str());
    open_xml();
}

bool MogsProblemClassifier::get_library_plugin(	const mogs_string & plugin_type,
						const mogs_string & plugin_name,
						mogs_string & library_so_output)
{
	if(!read_xml())
	{
		std::cerr<<"Error in "<<__FILE__<<" at line "<< __LINE__<<std::endl;
		std::cerr<<"Cannot read the config file"<<std::endl;
		exit(0);
	}

	for (int i=0;i<libs_plugins_.size();i++)
	{
		if (libs_plugins_[i].library_type == plugin_type &&
			libs_plugins_[i].library_name == plugin_name)
		{
			library_so_output = libs_plugins_[i].library_so;
			return true;
		}
	}
	return false;
}

bool MogsProblemClassifier::open_xml()
{
    if(!doc_)
    {
        doc_ = new QDomDocument();
        QFile *file_ = new QFile(PLUGIN_CONFIG_FILE);

        QFile::Permissions p = file_->permissions();
        QIODevice::OpenModeFlag type = QIODevice::ReadOnly;

        if (!file_->open(type)) {
            std::cerr<<"Error in "<<__FILE__<<" at line : "<< __LINE__<<std::endl;
            std::cerr<<"Cannot open the file "<< PLUGIN_CONFIG_FILE <<std::endl;
            return false;
        }

        // Parse file
        if (!doc_->setContent(file_)) {
            std::cerr<<"Error in "<<__FILE__<<" at line : "<< __LINE__<<std::endl;
           std::cerr<<"Cannot parse the content of "<< PLUGIN_CONFIG_FILE<<std::endl;
           file_->close();
           return false;
        }
        file_->close();
        return true;
    }
    return true;
}


void MogsProblemClassifier::print_plugins()
{
	if(!read_xml())
		create_xml();
	std::cout<<"known plugins "<<std::endl;
	for(int i=0;i<libs_plugins_.size();i++)
		std::cout<<"type:"<< libs_plugins_[i].library_type.toStdString() <<":\t"<< libs_plugins_[i].library_name.toStdString()<<std::endl;
}

void MogsProblemClassifier::print_plugins_with_libs()
{
	if(!read_xml())
		create_xml();
	std::cout<<"known plugins "<<std::endl;
	for(int i=0;i<libs_plugins_.size();i++)
		std::cout<<"type:"<< libs_plugins_[i].library_type.toStdString() <<":\t"<< libs_plugins_[i].library_name.toStdString()<<"  \t"<<libs_plugins_[i].library_so.toStdString()<<std::endl;
}

void MogsProblemClassifier::print_plugins_problem()
{
	if(!read_xml())
		create_xml();
	std::cout<<"known problem plugins "<<std::endl;
	for(int i=0;i<libs_problem_.size();i++)
		std::cout<< libs_problem_[i].library_name.toStdString()<<std::endl;
}

void MogsProblemClassifier::print_plugins_problem_with_libs()
{
	if(!read_xml())
		create_xml();
	std::cout<<"known problem plugins "<<std::endl;
	for(int i=0;i<libs_problem_.size();i++)
		std::cout<< libs_problem_[i].library_name.toStdString()<<"  \t"<<libs_problem_[i].library_so.toStdString()<<std::endl;
}

void MogsProblemClassifier::print_plugins_robot()
{
	if(!read_xml())
		create_xml();
	std::cout<<"known robot plugins "<<std::endl;
	for(int i=0;i<libs_robot_.size();i++)
		std::cout<< libs_robot_[i].library_name.toStdString()<<std::endl;
}

void MogsProblemClassifier::print_plugins_robot_with_libs()
{
	if(!read_xml())
		create_xml();
	std::cout<<"known robot plugins "<<std::endl;
	for(int i=0;i<libs_robot_.size();i++)
		std::cout<< libs_robot_[i].library_name.toStdString()<<"  \t"<<libs_robot_[i].library_so.toStdString()<<std::endl;
}

bool MogsProblemClassifier::read_xml()
{
	libs_plugins_.clear();
	libs_problem_.clear();
	libs_robot_.clear();

	QUrl schemaUrl(XSD_PLUGIN_CONFIG_FILE);
    QXmlSchema schema;
    schema.load(schemaUrl);
    if (schema.isValid()) {
        QXmlSchemaValidator validator(schema);
        if (! validator.validate(QUrl(QString("file://")+ QString(PLUGIN_CONFIG_FILE))))
        {
            qDebug() << "instance document "<< QString("file://")+ QString(PLUGIN_CONFIG_FILE)<< " is invalid";
            qDebug() << "We try to create one";
            create_xml();
        }

    }else
    {
        qDebug()<<" Error the xsd "<< XSD_PLUGIN_CONFIG_FILE<< " is not valid"<<endl;
    }

    open_xml();

	root_ = doc_->documentElement();
	assert(!root_.isNull());
	ElProblems_ = root_.firstChildElement ("problems");
	assert(!ElProblems_.isNull());
	for (QDomElement ElProblem = ElProblems_.firstChildElement ("problem");!ElProblem.isNull();ElProblem = ElProblem.nextSiblingElement("problem"))
	{
		libname tmp;
		tmp.library_type = "problem";
		QDomElement ElName = ElProblem.firstChildElement ("name");
		tmp.library_name = ElName.text ();
		QDomElement ElLib = ElProblem.firstChildElement ("library");
		tmp.library_so = ElLib.text ();
		libs_problem_.push_back(tmp);
	}

	ElPlugins_ = root_.firstChildElement ("plugins");
	assert(!ElPlugins_.isNull());
	for (QDomElement ElPlugin = ElPlugins_.firstChildElement ("plugin"); !ElPlugin.isNull() ; ElPlugin = ElPlugin.nextSiblingElement("plugin"))
	{
		libname tmp;
		QDomElement ElType = ElPlugin.firstChildElement ("type");
		tmp.library_type = ElType.text ();
		QDomElement ElName = ElPlugin.firstChildElement ("name");
		tmp.library_name = ElName.text ();
		QDomElement ElLib = ElPlugin.firstChildElement ("library");
		tmp.library_so = ElLib.text ();
		libs_plugins_.push_back(tmp);
	}

	ElRobots_ = root_.firstChildElement ("robots");
	assert(!ElRobots_.isNull());
	for (QDomElement ElRobot = ElRobots_.firstChildElement ("robot"); !ElRobot.isNull() ; ElRobot = ElRobot.nextSiblingElement("robot"))
	{
		libname tmp;
		tmp.library_type = "robot";
		QDomElement ElName = ElRobot.firstChildElement ("name");
		tmp.library_name = ElName.text ();
		QDomElement ElLib = ElRobot.firstChildElement ("library");
		tmp.library_so = ElLib.text ();
		libs_robot_.push_back(tmp);
	}


//    QByteArray xml = doc_->toByteArray();
//    qDebug()<<"Reading\n"<< xml;

	return true;
}

void MogsProblemClassifier::rm_library_plugins(const mogs_string & plugin_type, const mogs_string & plugin_name)
{
	int user = getuid();
	if (user != 0)
	{
		std::cerr<<"Error you must be root to remove library to the plugins !! "<<std::endl;
		exit(0);
	}

	if(!read_xml())
	{
		std::cerr<<"Cannot find the config file so cannot remove the plugins"<<std::endl;
	}else
	{
		for (QDomElement ElPlugin = ElPlugins_.firstChildElement ("plugin"); !ElPlugin.isNull(); ElPlugin = ElPlugin.nextSiblingElement("plugin"))
		{
			QDomElement ElType = ElPlugin.firstChildElement ("type");
			mogs_string type = ElType.text ();
			QDomElement ElName = ElPlugin.firstChildElement ("name");
			mogs_string name = ElName.text ();
			if (type == plugin_type && name == plugin_name)
			{
				ElPlugins_.removeChild (ElPlugin);
				return;
			}
		}
		std::cerr<<"Error in MogsProblemClassifier::rm_library the plugin "<< plugin_name.toStdString() <<" of type "<< plugin_type.toStdString()<<" is not defined"<<std::endl;
	}
}

void MogsProblemClassifier::rm_library_problem(const mogs_string & problem_name)
{
	int user = getuid();
	if (user != 0)
	{
		std::cerr<<"Error you must be root to remove library to the plugins !! "<<std::endl;
		exit(0);
	}

	if(!read_xml())
	{
		std::cerr<<"Cannot find the config file so cannot remove the plugins"<<std::endl;
	}else
	{
		for (QDomElement ElProblem = ElProblems_.firstChildElement ("problem"); !ElProblem.isNull() ; ElProblem = ElProblem.nextSiblingElement("problem"))
		{
			QDomElement ElName = ElProblem.firstChildElement ("name");
			mogs_string tmp = ElName.text ();
			if (tmp == problem_name)
			{
				ElProblems_.removeChild (ElProblem);
				return;
			}
		}

		std::cerr<<"Error in MogsProblemClassifier::rm_library_problem the plugin \""<< problem_name.toStdString() <<"\" is not defined"<<std::endl;
	}
}

void MogsProblemClassifier::rm_library_robot(const mogs_string & robot_name)
{
	int user = getuid();
	if (user != 0)
	{
		std::cerr<<"Error you must be root to remove library to the plugins !! "<<std::endl;
		exit(0);
	}

	if(!read_xml())
	{
		std::cerr<<"Cannot find the config file so cannot remove the plugins"<<std::endl;
	}else
	{
		for (QDomElement ElRobot = ElRobots_.firstChildElement ("robot");!ElRobot.isNull(); ElRobot = ElRobot.nextSiblingElement("robot"))
		{
			QDomElement ElName = ElRobot.firstChildElement ("name");
			mogs_string tmp = ElName.text ();
			if (tmp.compare(robot_name)==0)
			{
				ElRobots_.removeChild (ElRobot);
				return;
			}
		}
		std::cerr<<"Error in MogsProblemClassifier::rm_library the plugin "<< robot_name.toStdString() <<" is not defined"<<std::endl;
	}
}

void MogsProblemClassifier::delete_problem (MogsAbstractProblem ** pb)
{
	// destroy the class
	destroy_problem_((*pb));
	// unload the triangle library
	dlclose(library_);
}

void MogsProblemClassifier::save_xml()
{
    QFile *file = new QFile(PLUGIN_CONFIG_FILE);
    QFile::Permissions p = file->permissions();
    QIODevice::OpenMode type = QIODevice::ReadOnly;
    if ( p & QFile::WriteUser)
    {
        type = QIODevice::ReadWrite | QIODevice::Unbuffered;
    }
//    else{
//            qDebug()<<"Open "<< PLUGIN_CONFIG_FILE <<" in ReadOnly mode";
//            qDebug()<<"Cannot save the file!!";
//    }

    if (!file->open(type))
    {
        std::cerr<<"Error in "<<__FILE__<<" at line : "<< __LINE__<<std::endl;
        std::cerr<<"Cannot open the file "<< PLUGIN_CONFIG_FILE <<std::endl;
    }
    if ( p & QFile::WriteUser)
    {
        QByteArray xml = doc_->toByteArray();
        file->write(xml);
    }
    file->close();
    delete file;
}

bool MogsProblemClassifier::set_problem_type (	const mogs_string & filename,
						MogsAbstractProblem ** pb)
{
    // FIX ME dO QT Validation/home/lengagne/Bureau/redaction/Tactile_Recognition/code
	// get the type of the problem
//	mogs_string cmd = "xmllint --noout " + filename;
//	int retCode = system (cmd.toAscii());
//	if (retCode != 0)
//	{
//		std::cerr << "Error when execute " << cmd.toStdString() << "  (to check the XML file)" << std::endl;
//		return false;
//	}

    QDomDocument doc("Problem");
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly))
    {
        std::cerr << "Error in " << __FILE__ << " at line " << __LINE__ << "\n Cannot open the file : " << filename.toStdString() << std::endl;
        return false;
    }

    if (!doc.setContent(&file)) {
        file.close();
        std::cerr << "Error in " << __FILE__ << " at line " << __LINE__ << "\n Cannot set content from the file : " << filename.toStdString() << std::endl;
        return false;
    }
    file.close();

	QDomElement problem = doc.documentElement();
	assert (!problem.isNull());
	mogs_string type = problem.attribute ("type");

	if(! read_xml())
	{
		std::cerr<<"Cannot find the config file : "<< PLUGIN_CONFIG_FILE  <<std::endl;
		return false;
	}
	bool not_found = true;
	for (int i =0;i<libs_problem_.size();i++)
	{
		if (type == libs_problem_[i].library_name)
		{
			// load the library
			library_ = dlopen(libs_problem_[i].library_so.toAscii(), RTLD_LAZY);
			if (!library_) {
				std::cerr <<"Error in "<<__FILE__<<" at line "<<__LINE__<< " : Cannot load library ("<< libs_problem_[i].library_so.toStdString()<<"), with the error : " << dlerror() << '\n';
				exit(0);
			}

			// load the symbols
			create_problem_ = (create_problem*) dlsym(library_, "create");
			destroy_problem_ = (destroy_problem*) dlsym(library_, "destroy");
			if (!create_problem_ || !destroy_problem_)
			{
				std::cerr <<"Error in "<<__FILE__<<" at line "<<__LINE__<< " : Cannot load symbols of ("<< libs_problem_[i].library_so.toStdString()<<"), with the error : " << dlerror() << '\n';
				exit(0);
			}

			// create an instance of the class
			(*pb) = create_problem_();
			(*pb)->read_problem(filename);
			return true;
		}
	}
// 	if(not_found)
// 	{
		  std::cerr << "Error in " << __FILE__ << " at line " << __LINE__ << "\nWhen reading: " << filename.toStdString();
		  std::cerr<<" The type of problem "<< type.toStdString() << " is not known !! Please refer to the documentation." << std::endl;
		  return false;
// 	}
}

void MogsProblemClassifier::set_robot_type (const mogs_string & fileurl,
                                            MogsAbstractRobot ** robot)
{
	// get the type of the robot
	QUrl schemaUrl(XSD_ROBOT_FILE);
    QXmlSchema schema;
    schema.load(schemaUrl);
    if (schema.isValid()) {
        QFile file("robot.xml");
        file.open(QIODevice::ReadOnly);

        QXmlSchemaValidator validator(schema);
        if (! validator.validate(QUrl(fileurl)))
		{
            qDebug() << "instance document "<< fileurl << " is invalid";
			getchar();
		}
    }else
    {
        qDebug()<<" Error the xsd "<< XSD_PLUGIN_CONFIG_FILE<< " is not valid"<<endl;
        exit(0);
    }

    QDomDocument doc("Robot");
    QFile file(fileurl);
    if (!file.open(QIODevice::ReadOnly))
    {
        std::cerr<<"Error cannot detect the type of the robot."<<std::endl;
        exit(0);
    }
    if (!doc.setContent(&file)) {
        file.close();
        std::cerr<<"Error cannot detect the type of the robot."<<std::endl;
        exit(0);
    }
    file.close();

	QDomElement root = doc.documentElement();
	assert(!root.isNull());
	QDomElement ElRobot_type = root.firstChildElement ("Robot_type");
	mogs_string type = ElRobot_type.text();

	// check if the robot is already defined in a library
	if(! read_xml())
	{
		std::cerr<<"Cannot find the config file : "<< PLUGIN_CONFIG_FILE <<std::endl;
		exit(0);
	}
	bool not_found = true;
	for (int i =0;i<libs_robot_.size();i++)
	{
		if (type == libs_robot_[i].library_name)
		{
// 			FIXME
// 			// load the library
// 			library_ = dlopen(mogs_string_to_char(libs_robot_[i].library_so), RTLD_LAZY);
// 			if (!library_) {
// 				std::cerr << "Cannot load library: " << dlerror() << '\n';
// 				exit(0);
// 			}
//
// 			// load the symbols
// 			create_problem_ = (create_t*) dlsym(library_, "create");
// 			destroy_problem_ = (destroy_t*) dlsym(library_, "destroy");
// 			if (!create_problem_ || !destroy_problem_)
// 			{
// 				std::cerr << "Cannot load symbols: " << dlerror() << '\n';
// 				exit(0);
// 			}
//
// 			// create an instance of the class
// 			(*robot) = create_problem_();
// 			not_found = false;
		}
	}
	// if the library is not defined use the MogsDefaultRobot
	if(not_found)
	{
		(*robot) = new MogsDefaultRobot(fileurl);
	}
}

 void MogsProblemClassifier::set_scalable_robot_type (const mogs_string & fileurl, MogsAbstractRobot ** robot,
                                                        const double size, const double weight, const double power)
{
    (*robot) = new MogsDefaultRobot(fileurl,size,weight,power);
}
