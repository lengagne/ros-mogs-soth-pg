//      MogsCenterOfMassConstraint.cpp
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

#include "MogsCenterOfMassConstraint.h"

MogsCenterOfMassConstraint::MogsCenterOfMassConstraint():
body_ref_(0),xmin_(0.0),xmax_(0.0),ymin_(0.0),ymax_(0.0)
{

}

MogsCenterOfMassConstraint::MogsCenterOfMassConstraint( unsigned int body_ref,
                                                        double xmin, double xmax,
                                                        double ymin, double ymax):
body_ref_(body_ref),xmin_(xmin),xmax_(xmax),ymin_(ymin),ymax_(ymax)
{

}

MogsCenterOfMassConstraint::MogsCenterOfMassConstraint(	QDomElement pg_root,
                                                        RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                        std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots )
{
        QDomElement Elframe = pg_root.firstChildElement("ref_body");
        if(Elframe.isNull())
        {
            body_ref_ = 0;
        }else
        {
            qDebug()<<"ref_body = "<< Elframe.text().simplified()<<".";
            body_ref_ = kin->model->GetBodyId( Elframe.text().simplified());                          	
        }

	QDomElement ElXmin = pg_root.firstChildElement("Xmin");
	if (ElXmin.isNull())
	{
		std::cerr<<"Error : you must defined the balise Xmin in the constraint CenterOfMassConstraint"<<std::endl;
		exit(0);
	}
	xmin_ = ElXmin.text().toDouble();
        
	QDomElement ElXmax = pg_root.firstChildElement("Xmax");
	if (ElXmax.isNull())
	{
		std::cerr<<"Error : you must defined the balise Xmax in the constraint CenterOfMassConstraint"<<std::endl;
		exit(0);
	}
	xmax_ = ElXmax.text().toDouble();
        
	QDomElement ElYmin = pg_root.firstChildElement("Ymin");
	if (ElYmin.isNull())
	{
		std::cerr<<"Error : you must defined the balise Ymin in the constraint CenterOfMassConstraint"<<std::endl;
		exit(0);
	}
	ymin_ = ElYmin.text().toDouble();
        
	QDomElement ElYmax = pg_root.firstChildElement("Ymax");
	if (ElYmax.isNull())
	{
		std::cerr<<"Error : you must defined the balise Ymax in the constraint CenterOfMassConstraint"<<std::endl;
		exit(0);
	}
	ymax_ = ElYmax.text().toDouble();                
}

MogsCenterOfMassConstraint::~MogsCenterOfMassConstraint()
{

}

void MogsCenterOfMassConstraint::compute(	Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,
						RigidBodyDynamics::MogsKinematics<F<double> > * kin,
						HQPSolver::Priority* task_info,
						std::vector< RigidBodyDynamics::MogsKinematics< double > * > * all_the_robots)
{
	Ftmp_ = kin->getCenterOfMAss(); 
        kin->getFrameCoordinate(body_ref_,ref_trans_);
        Ftmp2_ = ref_trans_.Get_Local_Position(Ftmp_);
        
        for (int l=0;l<kin->getNDof();l++)
        {
            task_info->Jacobian(0,l) =  Ftmp2_(0).d(l);
            task_info->Jacobian(1,l) =  Ftmp2_(1).d(l);
            task_info->Jacobian(2,l) =  -Ftmp2_(0).d(l);
            task_info->Jacobian(3,l) =  -Ftmp2_(1).d(l);            
        }
        task_info->error(0) = xmin_ - Ftmp2_(0).x();
        task_info->error(1) = ymin_ - Ftmp2_(1).x();
        task_info->error(2) = Ftmp2_(0).x() - xmax_;
        task_info->error(3) = Ftmp2_(1).x() - ymax_;
        
}

extern "C" MogsCenterOfMassConstraint* create(   QDomElement pg_root,
                                                RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
    return new MogsCenterOfMassConstraint(pg_root, kin, all_the_robots);
}

extern "C" void destroy(MogsCenterOfMassConstraint* p)
{
    delete p;
}
