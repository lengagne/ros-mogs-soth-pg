//      MogsOrientationConstraint.cpp
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

#include "MogsOrientationConstraint.h"


MogsOrientationConstraint::MogsOrientationConstraint(	int body_id, bool x, bool y, bool z, double vx, double vy, double vz,
							Eigen::Matrix <double,3, 1 > offset)
{
	body_id_ = body_id;
	value_.resize(3);
	ctr_.resize(3);
	value_[0] = vx;	ctr_[0] = x;
	value_[1] = vy;	ctr_[1] = y;
	value_[2] = vz;	ctr_[2] = z;
	offset_ = RPY_mat(offset);
}

MogsOrientationConstraint::MogsOrientationConstraint( QDomElement pg_root,
                                                    RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                    std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
	QDomElement ElBody = pg_root.firstChildElement("body");
	if (ElBody.isNull())
	{
		std::cerr<<"Error : you must defined a body in the constraint position_effector"<<std::endl;
		exit(0);
	}
	mogs_string body_name = ElBody.text().simplified();
	body_id_ = kin->model->GetBodyId(body_name);
        std::cout<<"body_id_ = "<< body_id_ <<std::endl;
    
	QDomElement ElRotation = pg_root.firstChildElement("rotation");
	if (!ElRotation.isNull())
	{
		mogs_string rotation_name = ElRotation.text();
		std::istringstream smallData (rotation_name.toStdString(), std::ios_base::in);
		double tval;
                Eigen::Matrix <double,3, 1 > offset;
		for (int i = 0; i < 3; i++)
		{
			smallData >> tval;
			offset (i) = tval;
		}
		offset_ = RPY_mat(offset);
	}else
	{
		std::cout<<"you  can specify an end effector rotation but you don't"<<std::endl;
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
    
	QDomElement Eldesired = pg_root.firstChildElement("desired");
	if (Eldesired.isNull())
	{
		std::cerr<<"Error : you must defined a desired in the constraint position_effector"<<std::endl;
		exit(0);
	}
	mogs_string type = Eldesired.attribute("type");
        if (type == "body")
	{
                ctr_.resize(3);
                ctr_[0] = true;
                ctr_[1] = true;
                ctr_[2] = true;
                
                value_.resize(3);
                value_[0] = 0.0;
                value_[1] = 0.0;
                value_[2] = 0.0;
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
            qDebug()<<"Error in "<< __FILE__<<" at line "<< __LINE__<<" constructor is not implemented yet";
            exit(0);
        }
}


MogsOrientationConstraint::~MogsOrientationConstraint()
{

}

void MogsOrientationConstraint::compute(	Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,
						RigidBodyDynamics::MogsKinematics<F<double> > * kin,
						HQPSolver::Priority* task_info,
						std::vector< RigidBodyDynamics::MogsKinematics< double > * > * all_the_robots)
{
	//	compute the actual effector position
	 //kin->getFrameOrientation(body_id_, Ftmp_);
	 
	switch(desired_mode_){
		case DESIRED:

				break;
		case BODY:
				if(target_robot_id_ != -1)
					TargetRotation_ = (*all_the_robots)[target_robot_id_]->CalcBodyWorldOrientation(target_body_id_);
				break;

		default:	break;
	}    
    
	kin->getFrameCoordinate(body_id_,FTransform_);
	
	FTransform_tmp_ = (TargetRotation_.transpose() * offset_).template cast<F<double>>() * FTransform_.E;
	convert_mat_to_RPY(FTransform_tmp_,Ftmp_);
	int cpt = 0;
	for (int i =0;i<3;i++)if (ctr_[i])
	{
		for (int l=0;l<kin->getNDof();l++)
			task_info->Jacobian(cpt,l) = Ftmp_(i).d(l);
		task_info->error(cpt) = value_[i] - Ftmp_(i).x();
		cpt ++;
	}
// 	std::cout<<"task_info->error = "<<std::endl<< task_info->error <<std::endl;
}


extern "C" MogsOrientationConstraint* create(   QDomElement pg_root,
                                                RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
    return new MogsOrientationConstraint(pg_root, kin, all_the_robots);
}

extern "C" void destroy(MogsOrientationConstraint* p)
{
    delete p;
}
