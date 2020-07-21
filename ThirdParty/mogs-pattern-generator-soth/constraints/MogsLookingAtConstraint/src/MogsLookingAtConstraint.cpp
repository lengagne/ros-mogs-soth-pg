//      MogsLookingAtConstraint.cpp
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

#include "MogsLookingAtConstraint.h"

MogsLookingAtConstraint::MogsLookingAtConstraint():
count_position_(0),body_ref_(0)
{
	desired_positions_.clear();
	body_ref_ = 0;
}


MogsLookingAtConstraint::MogsLookingAtConstraint(   int body_id,
                                                    const Eigen::Matrix<double,3,1> position1,
                                                    const Eigen::Matrix<double,3,1> position2,
                                                    const Eigen::Matrix<double,3,1> target,
                                                    unsigned int ref)
{
    body_id_ = body_id;
    pos1_ = position1;
    pos2_ = position2;
    desired_position_ = target;
    desired_positions_.push_back(target);
    body_ref_ = ref;
    
}

MogsLookingAtConstraint::MogsLookingAtConstraint(	QDomElement pg_root,
								RigidBodyDynamics::MogsKinematics<F<double> >* kin,
								std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots ):
count_position_(0)
{
	desired_positions_.clear();
	pos1_ = Eigen::Matrix<double,3,1>(0,0,0);
        pos2_ = Eigen::Matrix<double,3,1>(0,0,0);

	QDomElement ElBody = pg_root.firstChildElement("body");
	if (ElBody.isNull())
	{
		std::cerr<<"Error : you must defined a body in the constraint position_effector"<<std::endl;
		exit(0);
	}
	mogs_string body_name = ElBody.text().simplified();
	body_id_ = kin->model->GetBodyId(body_name);
        
        
	QDomElement ElPosition1 = pg_root.firstChildElement("position1");
	if (!ElPosition1.isNull())
	{
		mogs_string position_name = ElPosition1.text();
		std::istringstream smallData (position_name.toStdString(), std::ios_base::in);
		double tval;
		for (int i = 0; i < 3; i++)
		{
			smallData >> tval;
			pos1_ (i) = tval;
		}
	}else
	{
		std::cout<<"you  can specify an end effector position1 but you don't"<<std::endl;
	}

	QDomElement ElPosition2 = pg_root.firstChildElement("position2");
	if (!ElPosition2.isNull())
	{
		mogs_string position_name = ElPosition2.text();
		std::istringstream smallData (position_name.toStdString(), std::ios_base::in);
		double tval;
		for (int i = 0; i < 3; i++)
		{
			smallData >> tval;
			pos2_ (i) = tval;
		}
	}else
	{
		std::cout<<"you  can specify an end effector position2 but you don't"<<std::endl;
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


        for (QDomElement ElValue=pg_root.firstChildElement("value"); !ElValue.isNull() ; ElValue = ElValue.nextSiblingElement ("value"))
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
                desired_positions_.push_back(tmp);
                std::cout<<" will look at to "<< tmp.transpose()<<std::endl;
        }
	

        desired_position_ = desired_positions_[0];
}

MogsLookingAtConstraint::~MogsLookingAtConstraint()
{

}

void MogsLookingAtConstraint::compute(	Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,
						RigidBodyDynamics::MogsKinematics<F<double> > * kin,
						HQPSolver::Priority* task_info,
						std::vector< RigidBodyDynamics::MogsKinematics< double > * > * all_the_robots)
{
	// to do before

	//	compute the actual effector position
	Ftmp1_ = kin->getPosition(body_id_, pos1_); 
        Ftmp2_ = kin->getPosition(body_id_, pos2_); 
	Ftmp_target_ = kin->getPosition(body_ref_, desired_position_); 
        
        
        Eigen::Matrix<F<double>,3,1> BC = (Ftmp2_ - Ftmp1_);      
        Eigen::Matrix<F<double>,3,1> BA = Ftmp_target_ - Ftmp1_;
        
        F<double> distance = (BA.cross(BC)).norm()/(BC.norm());
        
        for (int l=0;l<kin->getNDof();l++)
                task_info->Jacobian(0,l) =  -distance.d(l);
        task_info->error(0) = distance.x();

        
//         std::cout<<"error = "<< distance.x()<<std::endl;
        distance_ = distance.x();
        if (distance_ < 1.0e-2)
        {
            count_position_ ++;
            if (count_position_ >= desired_positions_.size())
                count_position_ = 0;
            desired_position_ = desired_positions_[count_position_];
        }
}

extern "C" MogsLookingAtConstraint* create(   QDomElement pg_root,
                                                RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
    return new MogsLookingAtConstraint(pg_root, kin, all_the_robots);
}

extern "C" void destroy(MogsLookingAtConstraint* p)
{
    delete p;
}
