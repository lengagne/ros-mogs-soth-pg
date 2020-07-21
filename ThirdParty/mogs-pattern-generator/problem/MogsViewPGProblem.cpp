//      MogsViewPGProblem.cpp
//      Copyright (C) 2015 lengagne (lengagne@gmail.com)
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
//      2012-2013: IUT de Beziers/ LIRMM, Beziers, France
//	from 2013 : Université Blaise Pascal / axis : ISPR / theme MACCS

// #define PROBLEM_VISU_DEBUG

#include "MogsViewPGProblem.h"
#include "MogsKinematics.h"
#include "MogsPatternGeneratorClassifier.h"


#include <sys/time.h>

#define VISU_TIME 0.04
#define DT_PATTERN 0.02

MogsViewPGProblem::MogsViewPGProblem ()
{
	xsd_file_ = MOGS_PROBLEM_PG_VIEWER_XSD_FILE;
}

MogsViewPGProblem::~MogsViewPGProblem ()
{

}

void MogsViewPGProblem::solve()
{
	std::cout<<"Solving MogsViewPGProblem"<<std::endl;
	for (int i=0;i<robots_name_.size();i++)
	{
		std::cout<<"We are considering the robot "<<robots_name_[i].toStdString()<<std::endl;
		std::cout<<" The robot has "<< robots_[i]->getNDof()<<" degrees of freedom"<<std::endl;
	}


	if(!collision_)
	{
		std::cerr<<" Error in "<<__FILE__<<" at line "<< __LINE__<<"."<<std::endl;
		std::cerr<<" The MogsViewPGProblem requires a collision detector balise"<<std::endl;
		exit(0);
	}

#ifdef MogsVisu_FOUND
    qDebug()<<"MogsVisu found";
	visu_window_ = new VisuHolder();
// 	visu_window_->show_frame(false);
#else
    qDebug()<<"MogsVisu not found";

#endif
	std::vector<MogsKinematics<double> * > kin(nb_robots_);
	std::vector<Eigen::Matrix <  double, Eigen::Dynamic, 1 > > q(nb_robots_);
	std::vector<Eigen::Matrix <  double, Eigen::Dynamic, 1 > > qpattern(nb_robots_),dqpattern(nb_robots_);
	pg_.resize(nb_robots_);
	MogsPatternGeneratorClassifier pg_classifier;

	// init the robots
	for (int i =0;i<nb_robots_;i++)
	{
		if (! robots_[i]->is_robot_floating_base())
			robots_[i]->set_root_transformation(robot_position_[i],robot_orientation_[i]);
		// initialize collision properties
		collision_->init_collision_robot(robots_[i]);
		kin[i] = new MogsKinematics<double>();
		kin[i]->SetRobot(robots_[i]);
	}
	#ifdef MogsVisu_FOUND
	visu_window_->init_from_problem(this);
    #endif
	collision_->init(kin);

	// init the pattern generators
	for (int i =0;i<nb_robots_;i++)
	{
		std::cout<<"Reading the robot informations generator info"<<std::endl;
		// Reading the pattern generator
		QDomNode elrobot = el_robots_[i];

		#ifdef MogsVisu_FOUND
		visu_window_->add(robots_name_[i],robots_[i]);
		#endif

		int n;
		std::cout<<" Sending robot to viewer"<<std::endl;
		n = robots_[i]->getNDof();
		q[i].resize(n);
		qpattern[i].resize(n);
		dqpattern[i].resize(n);

		std::vector<double> qmin,qmax;
		robots_[i]->getPositionLimit(qmin,qmax);

		std::vector<std::string> joint_names;
		joint_names = robots_[i]->GetJointsName();
		for(int j=0;j<n;j++)
		{
		  double delta = (qmax[j]-qmin[j])*0.01;
			q[i](j) = 0.01;
			if(q[i](j)< qmin[j]+delta) q[i](j) = qmin[j]+delta;
			if(q[i](j)> qmax[j]-delta) q[i](j) = qmax[j]-delta;

// 			std::cout<<joint_names[j]<<"\tq["<<j<<"] = "<< qmin[j]<<" " << qmax[j]<<std::endl;
			dqpattern[i](j) = 0;
		}



		if (robots_[i]->is_robot_floating_base())	for (int j=0;j<3;j++)
		{
			q[i](j) = robot_position_[i](j);
			q[i](j+3) = robot_orientation_[i](j);
		}

		kin[i]->UpdateKinematicsCustom(&q[i]);

        // Reading the pattern generator
        QDomElement Elpg = elrobot.firstChildElement ("pattern_generator");
        if (!Elpg.isNull())
        {
            mogs_string pg_name = Elpg.attribute("type").simplified();
            qDebug()<<"name = "<< pg_name;
            if( ! pg_classifier.set_pattern_generator(pg_name, &pg_[i]))
            {
                qDebug()<<"Error when try to set the PG";
                exit(0);
            }
            qDebug()<<"pg_[i] = "<<pg_[i];
            pg_[i]->set_robot(robots_[i]);
            pg_[i]->read_xml(Elpg, &kin);
        }else
        {
            qDebug()<<"No PG found for the robot "<<robots_[i]->getRobotName();
            pg_[i] = NULL;
        }
	}



	double time = 0.0;
	bool test = true;
	#ifdef MogsVisu_FOUND
	struct timeval  tv;
	gettimeofday(&tv, NULL);
	double actual_time = tv.tv_sec + 1e-6*tv.tv_usec;
	double actual_start_time = actual_time;
	double last_visu = 0.0;
	for (int i=0;i<nb_robots_;i++)
		visu_window_->apply_q(robots_name_[i],&q[i]);
	#endif
//	for (int i=0;i<nb_robots_;i++)
//	std::cout<<" qqq["<<i<<"] = "<< q[i].transpose()<<std::endl;

	do
	{
		// update the kinematics
		for (int i=0;i<nb_robots_;i++)
			kin[i]->UpdateKinematicsCustom(&q[i]);
		
		// ask for the new torques
		for (int i=0;i<nb_robots_;i++)
		{
			// compute the PG
			if (pg_[i] )
			{
				qpattern[i] = q[i];
				test = test & pg_[i]->compute(time,&qpattern[i],&kin, &dqpattern[i]);
				q[i] = qpattern[i];
			}
		}

		time += DT_PATTERN;
		#ifdef MogsVisu_FOUND
		if (time - last_visu > VISU_TIME)
		{
			printf(" time = %.2f\r",time);
			for (int i=0;i<nb_robots_;i++)
				visu_window_->apply_q(robots_name_[i],&q[i]);
			do{
				gettimeofday(&tv, NULL);
				actual_time = tv.tv_sec + 1e-6*tv.tv_usec;
			}
			while(actual_time - actual_start_time <=  VISU_TIME);
			actual_start_time = actual_time;
// 			usleep(VISU_TIME*1e6);
			last_visu = time;
// 			getchar();
		}
		#endif

		if (time> 2000.)
			test = false;

	}while( test);

#ifdef MoGSVisu_FOUND
    visu_window_->wait_close();
	delete visu_window_;
#endif
}

extern "C" MogsViewPGProblem* create()
{
    return new MogsViewPGProblem();
}

extern "C" void destroy(MogsViewPGProblem* p)
{
    delete p;
}
