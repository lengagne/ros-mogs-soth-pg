//      MogsSelfCollisionConstraint.cpp
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

#include "MogsSelfCollisionConstraint.h"

MogsSelfCollisionConstraint::MogsSelfCollisionConstraint()
{

}

MogsSelfCollisionConstraint::MogsSelfCollisionConstraint(	QDomElement pg_root,
                                                                RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                                std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
    
    for (QDomElement childJoint = pg_root.firstChildElement ("collision"); ! childJoint.isNull(); childJoint = childJoint.nextSiblingElement("collision") )
    {
        name_body1_.push_back(childJoint.attribute("body1"));
        name_body2_.push_back(childJoint.attribute("body2"));
    }
    
    MogsSelfCollisionConstraint_tolerance = 0.001; 
    init(kin);
}

void MogsSelfCollisionConstraint::init( RigidBodyDynamics::MogsKinematics<F<double> >* kin)
{
    mogs_string cfg_file;
    bool test = kin->model->get_config_file("MogsBoxCollision",cfg_file);
    if (test)   std::cout<<"We found the config file "<< cfg_file.toStdString()<<std::endl;
    else        std::cout<<"Cannot find the config file "<< cfg_file.toStdString()<<std::endl;
    // set the collisions
    for (int i=0;i<name_body1_.size();i++)
    {
        collision tmp;
        tmp.body_id_1 = kin->model->GetBodyId(name_body1_[i]);
        tmp.body_id_2 = kin->model->GetBodyId(name_body2_[i]);
        
        box_def1_.push_back(new MogsBoxCollisionDefinition(mogs_get_absolute_link(cfg_file),name_body1_[i]));
        box_def2_.push_back(new MogsBoxCollisionDefinition(mogs_get_absolute_link(cfg_file),name_body2_[i]));
        
        if (tmp.body_id_1 == std::numeric_limits<unsigned int>::max() || tmp.body_id_2 == std::numeric_limits<unsigned int>::max())
        {
            if (tmp.body_id_1 == std::numeric_limits<unsigned int>::max())
            {
                std::cerr<<"Cannot find the body "<< name_body1_[i].toStdString()<<std::endl;
            }
            if (tmp.body_id_2 == std::numeric_limits<unsigned int>::max())
            {
                std::cerr<<"Cannot find the body "<< name_body2_[i].toStdString()<<std::endl;
            }                
            exit(1);
        }
        collisions_.push_back(tmp);
        
    }
    nb_collisions_ = collisions_.size();
    
    coll_detector_ = new MogsBoxCollision();
}


MogsSelfCollisionConstraint::~MogsSelfCollisionConstraint()
{
    delete coll_detector_;
    for (int i=0;i<box_def1_.size();i++)
        delete box_def1_[i];
    for (int i=0;i<box_def2_.size();i++)
        delete box_def2_[i];
}

void MogsSelfCollisionConstraint::compute(	Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,
							RigidBodyDynamics::MogsKinematics<F<double> > * kin,
							HQPSolver::Priority* task_info,
							std::vector< RigidBodyDynamics::MogsKinematics< double >* > * all_the_robots)
{
	SpatialTransform<F<double>> T1,T2;
	F<double> distance;
	for (int i=0;i<nb_collisions_;i++)
	{
                kin->getFrameCoordinate(collisions_[i].body_id_1,T1);
                kin->getFrameCoordinate(collisions_[i].body_id_2,T2);
                
                distance = coll_detector_->compute_one_distance<F<double>>(T1,T2,box_def1_[i],box_def2_[i]);
                
		for (int l=0;l<kin->getNDof();l++)
			task_info->Jacobian(i,l) = distance.d(l);
		task_info->error(i) =  - distance.x() + MogsSelfCollisionConstraint_tolerance;

// 		std::cout<<" collisions_[i].body_id_1 = "<<  collisions_[i].body_id_1 <<std::endl;
// 		std::cout<<" collisions_[i].body_id_2 = "<<  collisions_[i].body_id_2 <<std::endl;
// 		std::cout<<" distance = "<< distance<<std::endl;
//                 std::cout<<" task_info->Jacobian = "<< task_info->Jacobian<<std::endl;
	}

// 	std::cout<<"task_info->error = "<< task_info->error.transpose()<<std::endl;
}

extern "C" MogsSelfCollisionConstraint* create(  QDomElement pg_root,
                                                        RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                        std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
    return new MogsSelfCollisionConstraint(pg_root, kin, all_the_robots);
}

extern "C" void destroy(MogsSelfCollisionConstraint* p)
{
    delete p;
}
