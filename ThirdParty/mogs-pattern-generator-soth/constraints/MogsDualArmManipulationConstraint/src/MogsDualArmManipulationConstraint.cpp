//      MogsDualArmManipulationConstraint.cpp
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

#include "MogsDualArmManipulationConstraint.h"

MogsDualArmManipulationConstraint::MogsDualArmManipulationConstraint(	QDomElement pg_root,
								RigidBodyDynamics::MogsKinematics<F<double> >* kin,
								std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots )
{
	QDomElement ElBody1 = pg_root.firstChildElement("body1");
	if (ElBody1.isNull())
	{
		std::cerr<<"Error : you must defined a body in the constraint position_effector"<<std::endl;
		exit(0);
	}
	mogs_string body1_name = ElBody1.text().simplified();
	body1_id_ = kin->model->GetBodyId(body1_name);


	QDomElement ElBody2 = pg_root.firstChildElement("body2");
	if (ElBody2.isNull())
	{
		std::cerr<<"Error : you must defined a body in the constraint position_effector"<<std::endl;
		exit(0);
	}
	mogs_string body2_name = ElBody2.text().simplified();
	body2_id_ = kin->model->GetBodyId(body2_name);

    Eigen::Matrix<double,3,1> pos(0,0,0);
    Eigen::Matrix<double,3,1> rot(0,0,0);
	QDomElement ElPosition = pg_root.firstChildElement("position");
	if (!ElPosition.isNull())
	{
		mogs_string position_name = ElPosition.text();
		std::istringstream smallData (position_name.toStdString(), std::ios_base::in);
		double tval;
		for (int i = 0; i < 3; i++)
		{
			smallData >> tval;
			pos (i) = tval;
		}
	}else
	{
		std::cout<<"you  can specify a position but you don't"<<std::endl;
	}

	QDomElement ElRotation = pg_root.firstChildElement("rotation");
	if (!ElPosition.isNull())
	{
		mogs_string rotation_name = ElRotation.text();
		std::istringstream smallData (rotation_name.toStdString(), std::ios_base::in);
		double tval;
		for (int i = 0; i < 3; i++)
		{
			smallData >> tval;
			rot (i) = tval;
		}
	}else
	{
		std::cout<<"you  can specify a rotation but you don't"<<std::endl;
	}

        trans_ = SpatialTransform<double>(rot,pos);

	qDebug()<<"body1_id_ = "<<body1_id_;
	qDebug()<<"body2_id_ = "<<body2_id_;
//	std::cout<<"trans_ = "<<trans_<<std::endl;
}

MogsDualArmManipulationConstraint::~MogsDualArmManipulationConstraint()
{

}

void MogsDualArmManipulationConstraint::compute(	Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,
						RigidBodyDynamics::MogsKinematics<F<double> > * kin,
						HQPSolver::Priority* task_info,
						std::vector< RigidBodyDynamics::MogsKinematics< double > * > * all_the_robots)
{

	//	compute the actual effector position

	kin->getFrameCoordinate(body1_id_,Ftrans1_);
	kin->getFrameCoordinate(body2_id_,Ftrans2_);

	Ftot_ = Ftrans1_ * ( trans_ * Ftrans2_).transpose();

	Ftot_.get_RPY(Frot_,Fpos_);
	for (int j=0;j<3;j++)
	{
		for (int l=0;l<kin->getNDof();l++)
			task_info->Jacobian(j,l) =  Frot_(j).d(l);
		task_info->error(j) = - Frot_(j).x();

		for (int l=0;l<kin->getNDof();l++)
			task_info->Jacobian(3+j,l) =  Fpos_(j).d(l);
		task_info->error(3+j) = - Fpos_(j).x();
	}
}

extern "C" MogsDualArmManipulationConstraint* create(   QDomElement pg_root,
                                                RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
    return new MogsDualArmManipulationConstraint(pg_root, kin, all_the_robots);
}

extern "C" void destroy(MogsDualArmManipulationConstraint* p)
{
    delete p;
}
