//      MogsJointEqualityConstraint.cpp
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
//	from 2013:  Université Clermont Auvergne Pascal / axis : ISPR / theme MACCS

#include "MogsJointEqualityConstraint.h"




MogsJointEqualityConstraint::MogsJointEqualityConstraint(   unsigned int joint_id1,
                                                            unsigned int joint_id2,
                                                            double mul, double offset)
{
        joint_id1_ = joint_id1;
        joint_id2_ = joint_id2;
        coeff_mul_ = mul;
        offset_ = offset;
}


MogsJointEqualityConstraint::MogsJointEqualityConstraint(   QDomElement pg_root,
                                            RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                            std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots )
{
	QDomElement ElJoint = pg_root.firstChildElement("joint1");
	if (ElJoint.isNull())
	{
		std::cerr<<"Error : you must defined a joint1 in the constraint JointEqualityConstraint"<<std::endl;
		exit(0);
	}
	mogs_string joint_name = ElJoint.text().simplified();
	joint_id1_ = kin->model->get_joint_id(joint_name);
        
  	ElJoint = pg_root.firstChildElement("joint2");
	if (ElJoint.isNull())
	{
		std::cerr<<"Error : you must defined a joint2 in the constraint JointEqualityConstraint"<<std::endl;
		exit(0);
	}
	joint_name = ElJoint.text().simplified();
	joint_id2_ = kin->model->get_joint_id(joint_name);      
        
        coeff_mul_ = 1.0;
        QDomElement ElMultiplier = pg_root.firstChildElement("mulitplier");
	if (!ElMultiplier.isNull())
                coeff_mul_ = ElMultiplier.text().toDouble();

        offset_ = 0.0;
        QDomElement ElOffset = pg_root.firstChildElement("offset");
	if (!ElOffset.isNull())
                offset_ = ElOffset.text().toDouble();
}

MogsJointEqualityConstraint::~MogsJointEqualityConstraint()
{

}

void MogsJointEqualityConstraint::compute(	Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,
						RigidBodyDynamics::MogsKinematics<F<double> > * kin,
						HQPSolver::Priority* task_info,
						std::vector< RigidBodyDynamics::MogsKinematics< double > * > * all_the_robots)
{
        F<double> diff = Q(joint_id1_) - ( coeff_mul_ * Q(joint_id2_) + offset_) ;
        
        for (int l=0;l<kin->getNDof();l++)
                task_info->Jacobian(0,l) =  -diff.d(l);
        task_info->error(0) = diff.x();

}

extern "C" MogsJointEqualityConstraint* create(   QDomElement pg_root,
                                                RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
    return new MogsJointEqualityConstraint(pg_root, kin, all_the_robots);
}

extern "C" void destroy(MogsJointEqualityConstraint* p)
{
    delete p;
}
