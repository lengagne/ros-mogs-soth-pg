//      MogsSameZConstraint.cpp
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

#include "MogsSameZConstraint.h"




MogsSameZConstraint::MogsSameZConstraint(	int body_id1,
                                                const Eigen::Matrix<double,3,1> position1,
                                                int body_id2,
                                                const Eigen::Matrix<double,3,1> position2)
{
	effector_position1_ = position1;
        effector_position2_ = position2;
	body_id1_ = body_id1;
        body_id2_ = body_id2;
}


MogsSameZConstraint::MogsSameZConstraint(   QDomElement pg_root,
                                            RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                            std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots )
{
	effector_position1_ = Eigen::Matrix<double,3,1>(0,0,0);
        effector_position2_ = Eigen::Matrix<double,3,1>(0,0,0);

	QDomElement ElBody = pg_root.firstChildElement("body1");
	if (ElBody.isNull())
	{
		std::cerr<<"Error : you must defined a body1 in the constraint SameZConstraint"<<std::endl;
		exit(0);
	}
	mogs_string body_name = ElBody.text().simplified();
	body_id1_ = kin->model->GetBodyId(body_name);
        
	QDomElement ElPosition = pg_root.firstChildElement("position1");
	if (!ElPosition.isNull())
	{
		mogs_string position_name = ElPosition.text();
		std::istringstream smallData (position_name.toStdString(), std::ios_base::in);
		double tval;
		for (int i = 0; i < 3; i++)
		{
			smallData >> tval;
			effector_position1_ (i) = tval;
		}
	}else
	{
		std::cout<<"you  can specify an end effector position but you don't"<<std::endl;
	}


	ElBody = pg_root.firstChildElement("body2");
	if (ElBody.isNull())
	{
		std::cerr<<"Error : you must defined a body2 in the constraint SameZConstraint"<<std::endl;
		exit(0);
	}
	body_name = ElBody.text().simplified();
	body_id2_ = kin->model->GetBodyId(body_name);
        
	ElPosition = pg_root.firstChildElement("position2");
	if (!ElPosition.isNull())
	{
		mogs_string position_name = ElPosition.text();
		std::istringstream smallData (position_name.toStdString(), std::ios_base::in);
		double tval;
		for (int i = 0; i < 3; i++)
		{
			smallData >> tval;
			effector_position2_ (i) = tval;
		}
	}else
	{
		std::cout<<"you  can specify an end effector position but you don't"<<std::endl;
	}

	

}

MogsSameZConstraint::~MogsSameZConstraint()
{

}

void MogsSameZConstraint::compute(	Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,
						RigidBodyDynamics::MogsKinematics<F<double> > * kin,
						HQPSolver::Priority* task_info,
						std::vector< RigidBodyDynamics::MogsKinematics< double > * > * all_the_robots)
{

	//	compute the actual effector position
	Ftmp1_ = kin->getPosition(body_id1_, effector_position1_);
        Ftmp2_ = kin->getPosition(body_id2_, effector_position2_);

        F<double> diff = Ftmp1_(2) - Ftmp2_(2);
        
        for (int l=0;l<kin->getNDof();l++)
                task_info->Jacobian(0,l) =  -diff.d(l);
        task_info->error(0) = diff.x();

}

extern "C" MogsSameZConstraint* create(   QDomElement pg_root,
                                                RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
    return new MogsSameZConstraint(pg_root, kin, all_the_robots);
}

extern "C" void destroy(MogsSameZConstraint* p)
{
    delete p;
}
