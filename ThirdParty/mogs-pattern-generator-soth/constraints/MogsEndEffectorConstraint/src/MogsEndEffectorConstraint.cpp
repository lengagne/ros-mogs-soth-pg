//      MogsEndEffectorConstraint.cpp
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
//      2009-2011:  Joint robotics Laboratory - CNRS/AISdouble,Tsukuba, Japan.
//      2011-2012:  Karlsruhe Institute fur Technologie, Karlsruhe, Germany
//      2012-2013:  IUT de Beziers/ LIRMM, Beziers, France
//	from 2013:  Université Blaise Pascal / axis : ISPR / theme MACCS

#include "MogsEndEffectorConstraint.h"

MogsEndEffectorConstraint::MogsEndEffectorConstraint():
#ifdef MogsJoystick_FOUND
pad_(NULL),
#endif 
count_position_(0),body_ref_(0),threshold_(1.5e-2)
{
	desired_mode_ = DESIRED;
	positions_.push_back(Eigen::Matrix<double,3,1>(0,0,0));
	positions_.clear();
	effector_position_ = Eigen::Matrix<double,3,1>(0,0,0);
        body_ref_ = 0;
        nb_compute_without_modif_ = 1000;
}

MogsEndEffectorConstraint::MogsEndEffectorConstraint(	const std::string & in,
							int body_id,
							const Eigen::Matrix<double,3,1> position,
							const Eigen::Matrix<double,3,1> initial_desired_position,
                                                        unsigned int ref,
                                                        double threshold):threshold_(threshold)
{
        nb_compute_without_modif_ = 1000;
    #ifdef MogsJoystick_FOUND
	if( in != "joystick" && in != "JOYSTICK")
	{
		std::cerr<<"ERROR in "<< __FILE__<<" at line "<< __LINE__<<" string in must be \"joysitck\" "<<std::endl;
		exit(0);
	}
	desired_mode_ = JOYSTICK;
	pad_ = new MogsJoystick();
	if (!pad_->ready())
	{
		std::cerr<<"Error !! not pad connected" <<std::endl;
		exit(0);
	}
	body_id_ = body_id;
	effector_position_ = position;
	target_robot_id_ = -1;
	target_body_id_ = -1;
	desired_position_ = initial_desired_position;
        body_ref_ = ref;
    #else 
        std::cerr<<"ERROR in "<< __FILE__<<" at line "<< __LINE__<<" Cannot acces to this functionnality since MogsJoystick is not installed"<<std::endl;
        exit(0);

    #endif
}


MogsEndEffectorConstraint::MogsEndEffectorConstraint(	int body_id,
							int target_robot_id,
							int target_body_id,
							const Eigen::Matrix<double,3,1> position,
                                                        unsigned int ref,
                                                        double threshold):threshold_(threshold)
{
	desired_mode_ = BODY;
	effector_position_ = position;
	target_robot_id_ = target_robot_id;
	target_body_id_ = target_body_id;
	body_id_ = body_id;
        body_ref_= ref;
        nb_compute_without_modif_ = 1000;
}

MogsEndEffectorConstraint::MogsEndEffectorConstraint(	int body_id,
                                                        const Eigen::Matrix<double,3,1> position,
                                                        const Eigen::Matrix<double,3,1> target,
                                                        unsigned int ref,
                                                        double threshold):threshold_(threshold)
{
	desired_mode_ = BODY;
	body_id_ = body_id;
	target_robot_id_ = -1;
	target_body_id_ = -1;
	effector_position_ = position;
	desired_position_ = target;
        body_ref_= ref;
        nb_compute_without_modif_ = 1000;
}

MogsEndEffectorConstraint::MogsEndEffectorConstraint(	QDomElement pg_root,
                                                        RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                        std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots ):
#ifdef MogsJoystick_FOUND
pad_(NULL),
#endif 
count_position_(0)
{
	positions_.clear();
	effector_position_ = Eigen::Matrix<double,3,1>(0,0,0);
        threshold_ = 1.5e-2;

	QDomElement ElBody = pg_root.firstChildElement("body");
	if (ElBody.isNull())
	{
		std::cerr<<"Error : you must defined a body in the constraint position_effector"<<std::endl;
		exit(0);
	}
	mogs_string body_name = ElBody.text().simplified();
	body_id_ = kin->model->GetBodyId(body_name);
	QDomElement ElPosition = pg_root.firstChildElement("position");
	if (!ElPosition.isNull())
	{
		mogs_string position_name = ElPosition.text();
		std::istringstream smallData (position_name.toStdString(), std::ios_base::in);
		double tval;
		for (int i = 0; i < 3; i++)
		{
			smallData >> tval;
			effector_position_ (i) = tval;
		}
	}else
	{
		std::cout<<"you  can specify an end effector position but you don't"<<std::endl;
	}

        QDomElement Elframe = pg_root.firstChildElement("ref_body");
        if(Elframe.isNull())
        {
            body_ref_ = 0;
        }else
        {
            qDebug()<<"ref_body = "<< Elframe.text().simplified()<<".";
            body_ref_ = kin->model->GetBodyId( Elframe.text().simplified());                          	
        }

        QDomElement Elth = pg_root.firstChildElement("threshold");
        if(!Elframe.isNull())
        {
            threshold_ =  Elth.text().toDouble();
        }
        
        
        
	QDomElement Eldesired = pg_root.firstChildElement("desired");
	if (Eldesired.isNull())
	{
		std::cerr<<"Error : you must defined a desired in the constraint position_effector"<<std::endl;
		exit(0);
	}
	mogs_string type = Eldesired.attribute("type");
	if (type == "joystick")
	{
                #ifdef MogsJoystick_FOUND
		desired_mode_ = JOYSTICK;
		pad_ = new MogsJoystick();
		if (!pad_->ready())
		{
			std::cerr<<"Error !! not pad connected" <<std::endl;
			exit(0);
		}
		QDomElement ElValue = Eldesired.firstChildElement("value");
		if (!ElValue.isNull())
		{
			mogs_string text = ElValue.text();
			std::istringstream smallData (text.toStdString(), std::ios_base::in);
			double tval;
			for (int i = 0; i < 3; i++)
			{
				smallData >> tval;
				desired_position_ (i) = tval;
			}
			return;
		}
                #else
                std::cerr<<"ERROR in "<< __FILE__<<" at line "<< __LINE__<<" Cannot acces to this functionnality since MogsJoystick is not installed"<<std::endl;
                exit(0);
                #endif
	}else 	if (type == "body")
	{
		if (!all_the_robots)
		{
			std::cerr<<" ERROR in "<< __FILE__<<" at line "<< __LINE__<<std::endl;
			std::cerr<<" You cannot call body type without specifying robots"<<std::endl;
			exit(0);
		}


		desired_mode_ = BODY;
		QDomElement Elrobot = Eldesired.firstChildElement("target_robot");
		if(Elrobot.isNull())
		{
			std::cerr<<"Error you must define the target_robot balise"<<std::endl;
			exit(0);
		}
		mogs_string robot =  Elrobot.text();
		target_robot_id_ = -1;
		for (int i=0;i< all_the_robots->size();i++)
		{
			if (robot == (*all_the_robots)[i]->getRobotName())
			{
				target_robot_id_ = i;
				break;
			}
		}
		if( target_robot_id_ == -1)
		{
			std::cerr<<"Robot "<< robot.toStdString()<<" not defined"<<std::endl;
			exit(0);
		}
		
		QDomElement Elbody = Eldesired.firstChildElement("target_body");
		if(Elbody.isNull())
		{
			std::cerr<<"Error you must define the target_body balise"<<std::endl;
			exit(0);
		}
		target_body_id_ = (*all_the_robots)[target_robot_id_]->model->GetBodyId( Elbody.text().simplified());
                return;
	}else
	{
		desired_mode_ = DESIRED;
		QDomNode ElValue = Eldesired.firstChildElement("value");
		if (ElValue.isNull())
		{
			std::cerr<<"Error you must specify at least one value (or use the joystick type) in the constraint position_effector."<<std::endl;
		}
		for (ElValue; !ElValue.isNull() ; ElValue = ElValue.nextSiblingElement ("value"))
		{
			mogs_string text = ElValue.toElement().text();
			std::istringstream smallData (text.toStdString(), std::ios_base::in);
			Eigen::Matrix<double,3,1> tmp;
			double tval;
			for (int i = 0; i < 3; i++)
			{
				smallData >> tval;
				tmp (i) = tval;
			}
			positions_.push_back(tmp);
			std::cout<<" will go to "<< tmp.transpose()<<std::endl;
		}
//		return;
	}
	
        if (desired_mode_ == DESIRED)
        {
            for(int i=0;i<3;i++)
                desired_position_(i) = positions_[0](i);
        }	
        nb_compute_without_modif_ = 100;
}

MogsEndEffectorConstraint::~MogsEndEffectorConstraint()
{

}

void MogsEndEffectorConstraint::compute(	Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,
						RigidBodyDynamics::MogsKinematics<F<double> > * kin,
						HQPSolver::Priority* task_info,
						std::vector< RigidBodyDynamics::MogsKinematics< double > * > * all_the_robots)
{
	// to do before
	switch(desired_mode_){
		case DESIRED:

				break;
                #ifdef MogsJoystick_FOUND
		case JOYSTICK:
				desired_position_(0) += pad_->get_side_velocity()  * 0.005;
				desired_position_(1) += pad_->get_forward_velocity() * 0.005;
				desired_position_(2) += pad_->get_up_velocity() * 0.005;
// 				std::cout<<" desired_position_ = "<< desired_position_.transpose()<<std::endl;
				break;
                #endif
		case BODY:
				if(target_robot_id_ != -1)
					desired_position_ = (*all_the_robots)[target_robot_id_]->getBodyToBaseCoordinates(target_body_id_);
				break;

		default:	break;
	}

	//	compute the actual effector position
	Ftmp_ = kin->getPosition(body_id_, effector_position_); 
        
	current_effector_position_ = Eigen::Matrix<double,3,1> (Ftmp_(0).x(),Ftmp_(1).x(),Ftmp_(2).x());
        Eigen::Matrix<double,3,1> desired;

	if ( (current_effector_position_ ).norm() > 0.5)
	{
		Eigen::Matrix<double,3,1> normal = - current_effector_position_ ;
		normal.normalize();
		desired = current_effector_position_ + (0.5 * normal);
	}else
	{
		desired = desired_position_;
	}

        Ftmp_ = Ftmp_ - kin->getPosition(body_ref_, desired);
	for (int j=0;j<3;j++)
	{
		for (int l=0;l<kin->getNDof();l++)
			task_info->Jacobian(j,l) =  Ftmp_(j).d(l);
		task_info->error(j) = (- Ftmp_(j).x());
	}

	distance_ = task_info->error.norm();
	switch(desired_mode_){
		case DESIRED:
				if (task_info->error.norm() < threshold_)
				{
					count_position_ ++;
					if (count_position_ >= positions_.size())
						count_position_ = 0;
					desired_position_ = positions_[count_position_];
				}
				break;

		case JOYSTICK:

				break;

		case BODY:
				break;
		default:	break;
	}
//         std::cout<<std::endl;
}

extern "C" MogsEndEffectorConstraint* create(   QDomElement pg_root,
                                                RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
    return new MogsEndEffectorConstraint(pg_root, kin, all_the_robots);
}

extern "C" void destroy(MogsEndEffectorConstraint* p)
{
    delete p;
}
