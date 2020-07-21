//      MogsHigherBodyConstraint.cpp
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

#include "MogsHigherBodyConstraint.h"


MogsHigherBodyConstraint::MogsHigherBodyConstraint(	unsigned int body_up,
							unsigned int body_down)
{
	body_up_ = body_up;
	body_down_ = body_down;
}

MogsHigherBodyConstraint::MogsHigherBodyConstraint( QDomElement pg_root,
                                                    RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                    std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
    qDebug()<<"Error in "<< __FILE__<<" at line "<< __LINE__<<" constructor is not implemented yet";
    exit(0);
}

MogsHigherBodyConstraint::~MogsHigherBodyConstraint()
{

}

void MogsHigherBodyConstraint::compute(	Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,
						RigidBodyDynamics::MogsKinematics<F<double> > * kin,
						HQPSolver::Priority* task_info,
						std::vector< RigidBodyDynamics::MogsKinematics< double > * > * all_the_robots)
{
	Eigen::Matrix< F<double> , 3,1> tmp1,tmp2;
	tmp1 = kin->getBodyToBaseCoordinates(body_up_);
	tmp2 = kin->getBodyToBaseCoordinates(body_down_);

	F<double> out = tmp1(2) - tmp2(2);
	for (int l=0;l<kin->getNDof();l++)
		task_info->Jacobian(0,l) = - out.d(l);
	task_info->error(0) = out.x();
}

extern "C" MogsHigherBodyConstraint* create(QDomElement pg_root,
                                            RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                            std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
    return new MogsHigherBodyConstraint(pg_root, kin, all_the_robots);
}

extern "C" void destroy(MogsHigherBodyConstraint* p)
{
    delete p;
}
