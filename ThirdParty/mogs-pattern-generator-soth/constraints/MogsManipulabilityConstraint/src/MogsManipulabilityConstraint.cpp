//      MogsManipulabilityConstraint.cpp
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

#include "MogsManipulabilityConstraint.h"

MogsManipulabilityConstraint::MogsManipulabilityConstraint(	int body_id)
{
	body_id_ = body_id;
}

MogsManipulabilityConstraint::MogsManipulabilityConstraint( QDomElement pg_root,
								RigidBodyDynamics::MogsKinematics<F<double> >* kin)
{
	QDomElement ElBody = pg_root.firstChildElement("body");
	if (ElBody.isNull())
	{
		std::cerr<<"Error : you must defined a body in the constraint manipulability"<<std::endl;
		exit(0);
	}
	mogs_string body_name = ElBody.text().simplified();
	body_id_ = kin->model->GetBodyId(body_name);
}

MogsManipulabilityConstraint::~MogsManipulabilityConstraint()
{

}

void MogsManipulabilityConstraint::compute(	Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,
						RigidBodyDynamics::MogsKinematics<F<double> > * kin,
						HQPSolver::Priority* task_info,
						std::vector< RigidBodyDynamics::MogsKinematics< double > * > * all_the_robots)
{
	Jacobian_.resize(3,kin->getNDof());
	//	compute the actual effector position
	 kin->CalcPointJacobian(Q,body_id_, Eigen::Matrix<F<double>,3,1> (0,0,0), Jacobian_);

	determinant_ = (Jacobian_ * Jacobian_.transpose()).determinant();
// 	if (determinant_ > 1000.)
// 		determinant_ = 1000.;

	for (int l=0;l<kin->getNDof();l++)
		task_info->Jacobian(0,l) = - determinant_.d(l);
	task_info->error(0) = determinant_.x();
}

extern "C" MogsManipulabilityConstraint* create(QDomElement pg_root,
                                                RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
    return new MogsManipulabilityConstraint(pg_root, kin);
}

extern "C" void destroy(MogsManipulabilityConstraint* p)
{
    delete p;
}
