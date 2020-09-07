//      MogsSothPatternGenerator.cpp
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
//      2009-2011:  Joint robotics Laboratory - CNRS/AIST,Tsukuba, Japan.
//      2011-2012:  Karlsruhe Institute fur Technologie, Karlsruhe, Germany
//      2012-2013:  IUT de Beziers/ LIRMM, Beziers, France
//	from 2013:  Université Blaise Pascal / axis : ISPR / theme MACCS

#include "MogsSothPatternGenerator.h"
#include <Eigen/LU>
#include <Eigen/Core>

//#include <math.h>	// isnan function
#include "MogsProblemClassifier.h"

double frand_a_b(double a, double b){
    return ( rand()/(double)RAND_MAX ) * (b-a) + a;
}

MogsSothPatternGenerator::MogsSothPatternGenerator():t_start_(0.),coeff_(1.0)
{

}

MogsSothPatternGenerator::MogsSothPatternGenerator( RigidBodyDynamics::MogsRobotProperties * this_robot,
                                                    std::vector< MogsAbstractSothConstraint * > constraints,
                                                    double coeff):t_start_(0.),coeff_(coeff)
{
        test_derivative_ = false;
	set_robot(this_robot);
	constraints_ = constraints;
	initialize_solver();
}

MogsSothPatternGenerator::~MogsSothPatternGenerator()
{

}

void MogsSothPatternGenerator::initialize_solver()
{
	// set the hqp solver
	nb_tasks_ = constraints_.size();
	std::vector <int> taskSize;
	for (int i=0;i<nb_tasks_;i++)
	{
		taskSize.push_back(constraints_[i]->get_task_size());
	}

 	solver_->configureHook(taskSize);

	for (int i=0;i<nb_tasks_;i++)
	{
		// we try to stay in the feasible space
		int errorDimension = constraints_[i]->get_task_size();
		solver_->priorities[i]->Jacobian.resize(errorDimension, nb_dof_);
		solver_->priorities[i]->error.resize(errorDimension);
		solver_->priorities[i]->error_max.resize(errorDimension);
		solver_->priorities[i]->inequalities.resize(errorDimension);

		for (int j=0;j<errorDimension;j++)
		{
			solver_->priorities[i]->error_max[j] = constraints_[i]->get_max(j);
			solver_->priorities[i]->inequalities[j] = constraints_[i]->get_inequalities(j);
		}
	}
	robot_->getVelocityLimit(dq_max_);
	robot_->getPositionLimit(q_min_,q_max_);
}

void MogsSothPatternGenerator::read_constraint_xml( const mogs_string & filename)
{
     std::cout<<"MogsSothPatternGenerator::read_constraint_xml"<<std::endl;
    QFile * file = new QFile(filename);
    QDomDocument doc;
    if (!file->open(QIODevice::ReadOnly)) {
        std::cerr<<"Error in "<<__FILE__<<" at line : "<< __LINE__<<std::endl;
        std::cerr<<"Cannot open the file "<< filename.toStdString() <<std::endl;
        exit(0);
    }
    // Parse file
    if (!doc.setContent(file)) {
        std::cerr<<"Error in "<<__FILE__<<" at line : "<< __LINE__<<std::endl;
       std::cerr<<"Cannot parse the content of "<< filename.toStdString()<<std::endl;
       file->close();
       exit(0);
    }
    file->close();
    root_ = doc.documentElement();
    std::vector< RigidBodyDynamics::MogsKinematics<double> *> all_robots;
    all_robots.push_back( new RigidBodyDynamics::MogsKinematics<double>(kin_->model));
    
    read_root_xml(&all_robots);
    initialize_solver();
}

void MogsSothPatternGenerator::read_root_xml(std::vector< RigidBodyDynamics::MogsKinematics<double> *> * all_robots)
{
    std::cout<<"MogsSothPatternGenerator::read_root_xml"<<std::endl;
    // read the speed up coefficient
        QDomElement Elspeedup = root_.firstChildElement("speed_up");
        if(!Elspeedup.isNull())
            coeff_ = Elspeedup.text().toDouble();
        
        test_derivative_ = false;
        QDomElement ElTestDeriv = root_.firstChildElement("test_derivative");
        if(!ElTestDeriv.isNull())
        {
            test_derivative_ = convert_to_bool(ElTestDeriv.text());
            std::cout<<"test_derivative_ = "<< ElTestDeriv.text().toStdString() <<std::endl;    
        }
	// read the constraints
	int cpt = 0;

	MogsProblemClassifier mpc;
	mogs_string library_so;

	QDomElement Elconstraint = root_.firstChildElement("constraint");
	for (; !Elconstraint.isNull() ; Elconstraint = Elconstraint.nextSiblingElement ("constraint"))
	{
		create_soth_constraint* creator;
		destroy_soth_constraint* destructor;
		mogs_string ctr_type  = Elconstraint.toElement().attribute("type");
                qDebug()<<" parsing config ctr_type = "<< ctr_type;
		if ( mpc.get_library_plugin("MogsSothPatternGeneratorConstraint",ctr_type,library_so))
		{
		    // load the library
		    void * library = dlopen(library_so.toAscii(), RTLD_LAZY);
		    if (!library) {
			std::cerr <<"Error in "<<__FILE__<<" at line "<<__LINE__<< " : Cannot load library ("<< library_so.toStdString()<<"), with the error : " << dlerror() << '\n';
			exit(0);
		    }

		    // load the symbols
		    creator = (create_soth_constraint*) dlsym(library, "create");
		    destructor = (destroy_soth_constraint*) dlsym(library, "destroy");
		    if (!creator || !destructor){
			std::cerr <<"Error in "<<__FILE__<<" at line "<<__LINE__<< " : Cannot load symbols of ("<< library_so.toStdString()<<"), with the error : " << dlerror() << '\n';
			exit(0);
		    }

		    constraints_.push_back(creator(Elconstraint.toElement(),kin_,all_robots));
		    cpt++;
		}
		else
		{
		    qDebug()<<"Error cannot load the plugin "<<ctr_type<<" as an MogsSothPatternGeneratorConstraint plugin";
		    exit(0);
		}
	}
	std::cout<<"There are "<< cpt <<" known constraints"<<std::endl;
	if (cpt == 0)   exit(0);
}

void MogsSothPatternGenerator::read_xml(QDomElement PG_root,
					std::vector< RigidBodyDynamics::MogsKinematics<double> *> * all_robots)
{
	MogsAbstractPatternGenerator::read_xml(PG_root);
	read_root_xml(all_robots);
	initialize_solver();
}

bool MogsSothPatternGenerator::compute(		double time,
						Eigen::Matrix <double,Eigen::Dynamic, 1 > *Q,
						std::vector< RigidBodyDynamics::MogsKinematics<double> *> * kin_double,
						Eigen::Matrix <double,Eigen::Dynamic, 1 > *DQ,
						Eigen::Matrix <double,Eigen::Dynamic, 1 > *DDQ,
						Eigen::Matrix <double,Eigen::Dynamic, 1 > *torques)
{
        if (test_derivative_)
        {
            unsigned int nb_error = 0;
            std::cout<<"Performing derivative test "<<std::endl;
            double eps = 1e-3;
            for (int i=0;i<nb_tasks_;i++)
            {            
                for (int k=0;k<solver_->priorities[i]->error.size();k++)
                {
                    for (int j=0;j<nb_dof_;j++)
                    {
                        q_(j) = (q_min_[j] + q_max_[j]) /2.0;
                        q_(j).diff(j,nb_dof_);                        
                    }
                    constraints_[i]->compute(q_,kin_, solver_->priorities[i],kin_double);
                    double value = solver_->priorities[i]->error(k);
                    for (int kk=0;kk<nb_dof_;kk++)
                    {
                        q_(kk) += eps;
                        constraints_[i]->compute(q_,kin_, solver_->priorities[i],kin_double);                        
                        double value_plus = solver_->priorities[i]->error(k);
                        q_(kk) -= 2*eps;
                        constraints_[i]->compute(q_,kin_, solver_->priorities[i],kin_double);                        
                        double value_moins = solver_->priorities[i]->error(k);
                        
                        if ( fabs ( solver_->priorities[i]->Jacobian(k,kk) + (value_plus-value_moins)/(2*eps)) > eps)   
                        {
                            std::cout<<"Error in derivative for task "<<i<<" error number "<< k<<" regarding input "<< kk <<"   "<<solver_->priorities[i]->Jacobian(k,kk)<<" != "<<  (value_plus-value_moins)/(2*eps) <<std::endl;
                            nb_error++;
                        }
                    }
                }
                
                
    // 		std::cout<<"task number "<< i <<std::endl;
    // 		std::cout<<"err = "<< solver_->priorities[i]->error <<std::endl<<std::endl;
    // 		std::cout<<"jac = "<< solver_->priorities[i]->Jacobian <<std::endl<<std::endl;

            }   
            std::cout<<"derivative test done with "<< nb_error <<" error(s)."<<std::endl;
            test_derivative_ = false;
        }
    
// 	std::cout<<"q_ = "<<(*Q).transpose()<<std::endl<<std::endl;
	// update the current state of the robot
	for (int i=0;i<nb_dof_;i++)
	{
		q_(i) = (*Q)(i);
		q_(i).diff(i,nb_dof_);
	}

	kin_->UpdateKinematicsCustom(&q_);
	for (int i=0;i<nb_tasks_;i++)
	{
		constraints_[i]->compute(q_,kin_, solver_->priorities[i],kin_double);
// 		std::cout<<"task number "<< i <<std::endl;
// 		std::cout<<"err = "<< solver_->priorities[i]->error <<std::endl<<std::endl;
// 		std::cout<<"jac = "<< solver_->priorities[i]->Jacobian <<std::endl<<std::endl;

	}
	if ( solver_->solve(dq_))
	{

		for (int i=0;i<nb_dof_;i++)
		{

                        dq_(i) *= coeff_;
			if ( ! std::isnan(dq_(i)))
			{
				// FIXME take into account the actual limits
				if (dq_(i)> dq_max_[i])
					dq_(i) = dq_max_[i];
				if (dq_(i)< -dq_max_[i])
					dq_(i) = -dq_max_[i];
				(*Q)(i) += dq_(i) * (time-t_start_);

				if((*Q)(i)< q_min_[i])
                                {  
                                    std::cerr<<"Joint limit violation on joint "<< i<<"  "<< (*Q)(i)  <<" < "<< q_min_[i]<<std::endl;
                                    (*Q)(i) = q_min_[i];   
                                }
				if((*Q)(i)> q_max_[i])
                                {  
                                    std::cerr<<"Joint limit violation on joint "<< i<<"  "<< (*Q)(i)  <<" > "<< q_max_[i]<<std::endl;
                                    (*Q)(i) = q_max_[i];   
                                }
//				std::cout<<"q_ = "<<(*Q).transpose()<<std::endl<<std::endl;
			}else
			{
			    qDebug()<<"Error get nan is compuation of joint velocity";
				getchar();
			}
		}
	}else
        {
                std::cout<<"on ajoute du bruit"<<std::endl;
                for (int i=0;i<nb_dof_;i++)
                    (*Q)(i) += frand_a_b(-0.01,0.01);
        }
	if(DQ)
		(*DQ) = dq_;
// 	std::cout<<"time-t_start_ = "<< time-t_start_ <<std::endl;
// 	std::cout<<"dq = "<< dq_.transpose()<<std::endl<<std::endl<<std::endl;
// 	for (int i=0;i<nb_dof_;i++)
// 	    std::cout<<"q_("<<i<<") = "<<(*Q)(i) <<" in ["<< q_min_[i]<<":"<<q_max_[i]<<"]"<<std::endl;
	t_start_ = time;
	return true;
}

void MogsSothPatternGenerator::set_robot(RigidBodyDynamics::MogsRobotProperties * in)
{
	MogsAbstractPatternGenerator::set_robot(in);
	kin_ = new RigidBodyDynamics::MogsKinematics<F<double> >();
	kin_->SetRobot(in);
	q_.resize(nb_dof_);
	dq_.resize(nb_dof_);
	for (int i=0;i<nb_dof_;i++)
		q_(i) = 0.0;
	kin_->UpdateKinematicsCustom(&q_);

	solver_ = new HQPSolver(nb_dof_);
}

extern "C" MogsSothPatternGenerator* create()
{
    return new MogsSothPatternGenerator();
}

extern "C" void destroy(MogsSothPatternGenerator* p)
{
    delete p;
}
