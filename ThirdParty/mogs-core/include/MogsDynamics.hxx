/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 * Code Modified by S. Lengagne during his work in the following labs:
 *      2011-2012:  Karlsruhe Institute fur Technologie, Karlsruhe, Germany
 *      2013-2012: IUT de Beziers/ LIRMM, Beziers, France
 */


#include <iostream>
#include <limits>
#include <assert.h>
#include <string.h>

namespace RigidBodyDynamics
{

	template < typename T >
	void MogsDynamics < T >::ForwardDynamics (    const Eigen::Matrix < T, Eigen::Dynamic, 1 > &Q,
						      const Eigen::Matrix < T, Eigen::Dynamic, 1 > &QDot,
						      const Eigen::Matrix < T, Eigen::Dynamic, 1 > &Tau,
						      Eigen::Matrix < T, Eigen::Dynamic, 1 > &QDDot,
						      std::vector < Eigen::Matrix < T, 6, 1 >, Eigen::aligned_allocator < Eigen::Matrix < T, 6, 1 > > >  *f_ext)
	{
                this->model->SetForwardJointValue(Q,QDot,Tau);
		Eigen::Matrix < double, 6, 1 > spatial_gravity;
		for (int i=0;i<3;i++)
		{
			spatial_gravity(i) = 0;
			spatial_gravity(i+3) = this->model->gravity(i);
		}
		unsigned int i = 0;

		// Reset the velocity of the root body
		this->model->v[0].setZero ();

		for (i = 1; i < this->model->mBodies.size (); i++)
		{
			SpatialTransform < T > X_J;
			Eigen::Matrix < T, 6, 1 > v_J;
			Eigen::Matrix < T, 6, 1 > c_J;
			unsigned int lambda = this->model->lambda[i];
			MogsRobotJoint & joint = this->model->mJoints[i];

			this->jcalc (i, X_J, this->model->S[i], v_J, c_J, joint.ratio_ * this->model->q_[i - 1], joint.ratio_ *  this->model->dq_[i - 1]);

			this->model->X_lambda[i] = X_J * this->model->X_T[i];

// 			if (lambda != 0)
				this->model->X_base[i] = this->model->X_lambda[i] * this->model->X_base.at (lambda);
// 			else
// 				this->model->X_base[i] = this->model->X_lambda[i];

			this->model->v[i] = this->model->X_lambda[i].apply (this-> model->v.at (lambda)) + v_J;

			this->model->c[i]  = c_J + crossm (this->model->v[i], v_J);
			this->model->IA[i] = this->model->mBodies[i].mSpatialInertia.template cast <T> ();

			this->model->pA[i] = crossf < T > (this->model->v[i], this->model->IA[i]* this->model->v[i]);

			if (f_ext != NULL ) //&& (*f_ext)[i] != SpatialVectorZero)
			{
				this->model->pA[i] -= this->model->X_base[i].toMatrixAdjoint () * (*f_ext)[i];
			}
// 			else
// 				std::cout<<" no external forces for body "<<i<<std::endl;
		}

		for (i = this->model->mBodies.size () - 1; i > 0; i--)
		{
			this->model->U[i] = this->model->IA[i] * this->model->S[i].template cast<T>();
			this->model->d[i] = this->model->S[i].template cast<T>().dot (this->model->U[i]);
			this->model->u[i] = this->model->tau_[i - 1] - this->model->S[i].template cast<T>().dot (this->model->pA[i]);
			unsigned int lambda = this->model->lambda[i];
			if (lambda != 0 && this->model->d[i] != 0.0)
			{
				Eigen::Matrix < T, 6, 6 > Ia = this->model->IA[i]- this->model->U[i] * (this->model->U[i] / this->model->d[i]).transpose ();
				Eigen::Matrix < T, 6, 1 > pa = this->model->pA[i] + Ia * this->model->c[i] + this->model->U[i] * this->model->u[i] / this->model->d[i];
#ifdef EIGEN_CORE_H
				this->model->IA[lambda].noalias () +=  this->model->X_lambda[i].toMatrixTranspose () * Ia * this->model->X_lambda[i].toMatrix ();
				this->model->pA[lambda].noalias () += this->model->X_lambda[i].applyTranspose (pa);
#else
				this->model->IA[lambda] += this->model->X_lambda[i].toMatrixTranspose () * Ia * this->model->X_lambda[i].toMatrix ();
				this->model->pA[lambda] += this->model->X_lambda[i].applyTranspose (pa);
#endif
			}
		}

// 		this->model->a[0] = - spatial_gravity.template cast <T> ();
		this->model->a[0] =  this->model->X_base[0].apply(- spatial_gravity.template cast <T> ());
		for (i = 1; i < this->model->mBodies.size (); i++)
		{
			MogsRobotJoint & joint = this->model->mJoints[i];
			unsigned int lambda = this->model->lambda[i];
			SpatialTransform < T > X_lambda = this->model->X_lambda[i];

			this->model->a[i] = X_lambda.apply (this->model->a[lambda]) + this->model->c[i];

			this->model->ddq_[i - 1] = (1. / this->model->d[i]) * (this->model->u[i] - this->model->U[i].dot (this->model->a[i]));
			this->model->a[i] = this->model->a[i] + this->model->S[i].template cast<T>() * this->model->ddq_[i - 1];
			this->model->ddq_[i-1] /= joint.ratio_;
		}
		
		this->model->GetDDQ(QDDot);
	}

	template < typename T >
		void MogsDynamics < T >::ForwardDynamicsLagrangian (const Eigen::Matrix < T, Eigen::Dynamic, 1 > &Q,
								const Eigen::Matrix < T, Eigen::Dynamic, 1 > &QDot,
								const Eigen::Matrix < T, Eigen::Dynamic, 1 > &Tau,
								Eigen::Matrix < T, Eigen::Dynamic, 1 > &QDDot,
								LinearSolver linear_solver,
								std::vector < Eigen::Matrix < T, 6, 1 > >*f_ext)
	{
//                 this->model->SetForwardJointValue(Q,QDot,Tau);
		Eigen::MatrixXd H = Eigen::MatrixXd::Zero (this->model->dof_count, this->model->dof_count);
		Eigen::Matrix < T, Eigen::Dynamic, 1 > C = Eigen::Matrix < T, Eigen::Dynamic, 1 >::Zero (this->model->dof_count);

		// we set QDDot to zero to compute C properly with the InverseDynamics
		// method.
		QDDot.setZero ();

		InverseDynamics (this->model, Q, QDot, QDDot, C, f_ext);
		CompositeRigidBodyAlgorithm (this->model, Q, H, false);

#ifndef RBDL_USE_SIMPLE_MATH
		switch (linear_solver)
		  {
		  case (LinearSolverPartialPivLU):
			  QDDot = H.partialPivLu ().solve (C * -1. + Tau);
			  break;
		  case (LinearSolverColPivHouseholderQR):
			  QDDot = H.colPivHouseholderQr ().solve (C * -1. + Tau);
			  break;
		  default:
			  assert (0);
			  break;
		  }
#else
		bool solve_successful = LinSolveGaussElimPivot (H, C * -1. + Tau, QDDot);
		assert (solve_successful);
#endif
                this->model->GetDDQ(QDDot);
	}

	template < typename T >
		void MogsDynamics < T >::InverseDynamics (const Eigen::Matrix < T, Eigen::Dynamic, 1 > &Q,
						      const Eigen::Matrix < T, Eigen::Dynamic, 1 > &QDot,
						      const Eigen::Matrix < T, Eigen::Dynamic, 1 > &QDDot,
						      Eigen::Matrix < T, Eigen::Dynamic, 1 > &Tau,
						      std::vector < Eigen::Matrix < T, 6, 1 >, Eigen::aligned_allocator < Eigen::Matrix < T, 6, 1 > > > *f_ext)
	{
                this->model->SetJointValue(Q,QDot,QDDot);
//		Eigen::Matrix < T, 6, 1 > spatial_gravity (0., 0., 0., this->model->gravity[0], this->model->gravity[1], this->model->gravity[2]);
        Eigen::Matrix < T, 6, 1 > spatial_gravity = (Eigen::Matrix < T, 6, 1 > () <<0., 0., 0., this->model->gravity[0], this->model->gravity[1], this->model->gravity[2]).finished();
		unsigned int i;
		// Reset the velocity of the root body
		this->model->v[0].setZero ();
// 		this->model->a[0] = - spatial_gravity;
		this->model->a[0] =  this->model->X_base[0].apply(- spatial_gravity.template cast <T> ());

		for (i = 1; i < this->model->mBodies.size (); i++)
		  {
			  SpatialTransform < T > X_J;
			  Eigen::Matrix < T, 6, 1 > v_J;
			  Eigen::Matrix < T, 6, 1 > c_J;
			  unsigned int lambda = this->model->lambda[i];
			  MogsRobotJoint & joint = this->model->mJoints[i];

			  this->jcalc (i, X_J, this->model->S[i], v_J, c_J, joint.ratio_ *  this->model->q_[i - 1], joint.ratio_ *  this->model->dq_[i - 1]);

			  this->model->X_lambda[i] = X_J * this->model->X_T[i];

// 			  if (lambda == 0)
// 			    {
// 				    this->model->X_base[i] = this->model->X_lambda[i];
// 				    this->model->v[i] = v_J;
// 				    // FIXME
// // 				    this->model->a[i] = this->model->X_base[i].apply (spatial_gravity * -1.) + this->model->S[i] * QDDot[i - 1];
// 				    for(int j=0;j<6;j++)
// 					    this->model->a[i](j) = this->model->S[i](j) * QDDot[i - 1];
// 				    this->model->a[i] += this->model->X_base[i].apply (- spatial_gravity);
// 			    }
// 			  else
// 			    {
				    this->model->X_base[i] = this->model->X_lambda[i] * this->model->X_base.at (lambda);
				    this->model->v[i] = this->model->X_lambda[i].apply (this->model->v[lambda]) + v_J;
				    this->model->c[i] = c_J + crossm (this->model->v[i], v_J);
				    // FIXME
// 				    this->model->a[i] = this->model->X_lambda[i].apply (this->model->a[lambda]) + this->model->S[i] * QDDot[i - 1] + this->model->c[i];
				    for(int j=0;j<6;j++)
					    this->model->a[i](j) = this->model->S[i](j) * joint.ratio_ * this->model->ddq_[i - 1];
				    this->model->a[i] +=  this->model->X_lambda[i].apply (this->model->a[lambda]) + this->model->c[i];

// 			    }

// 			  this->model->f[i] = this->model->mBodies[i].mSpatialInertia * this->model->a[i] + crossf < T > (this->model->v[i], this->model->mBodies[i].mSpatialInertia * this->model->v[i]);
			this->model->f[i] = this->model->mBodies[i].mSpatialInertia.template cast < T > () * this->model->a[i]  + crossf < T > (this->model->v[i], (this->model->mBodies[i].mSpatialInertia.template cast < T > ()) * this->model->v[i]);
                        std::cout<<"F["<<i<<"] " << this->model->X_base[i].applyTranspose (this->model->f[i]).transpose()<<std::endl;
                        
			// FIXME for the moment no external forces
 			  if (f_ext != NULL /*&& (*f_ext)[i] != SpatialVectorZero*/)
 				  this->model->f[i] -= this->model->X_base[i].toMatrixAdjoint () * (*f_ext)[i];
		  }

		for (i = this->model->mBodies.size () - 1; i > 0; i--)
		  {
			  MogsRobotJoint & joint = this->model->mJoints[i];
			  Tau[i - 1] = this->model->S[i].template cast<T>().dot (this->model->f[i]) / joint.ratio_;

			  unsigned int lambda = this->model->lambda[i];

			  if (lambda != 0)
                {
                    this->model->f[lambda] = this->model->f[lambda] + this->model->X_lambda[i].applyTranspose (this->model->f[i]);
                }
		  }
	}

		template < typename T >
    void MogsDynamics < T >::InverseDynamicsStatic (  const Eigen::Matrix < T, Eigen::Dynamic, 1 > &Q,
                                                      Eigen::Matrix < T, Eigen::Dynamic, 1 > &Tau,
                                                      std::vector < Eigen::Matrix < T, 6, 1 >, Eigen::aligned_allocator < Eigen::Matrix < T, 6, 1 > > > *f_ext)
	{
		InverseDynamics(Q,
                        Eigen::Matrix < T, Eigen::Dynamic, 1 >::Zero(this->getNDof()),
                        Eigen::Matrix < T, Eigen::Dynamic, 1 >::Zero(this->getNDof()),
                        Tau,f_ext);
	}

	template < typename T >
		void MogsDynamics < T >::CompositeRigidBodyAlgorithm (	const Eigen::Matrix < T, Eigen::Dynamic, 1 > &Q,
									Eigen::Matrix < T,Eigen::Dynamic, Eigen::Dynamic > & H,
									bool update_kinematics)
	{
		assert (H.rows () == this->model->dof_count && H.cols () == this->model->dof_count);

		if (update_kinematics)
			this->UpdateKinematicsCustom (&Q, NULL, NULL);

		H.setZero ();

		unsigned int i;
		unsigned int dof_i = this->model->dof_count;

		for (i = 1; i < this->model->mBodies.size (); i++)
		  {
			  this->model->Ic[i].createFromMatrix (this->model-> mBodies[i].mSpatialInertia);
		  }

		for (i = this->model->mBodies.size () - 1; i > 0; i--)
		  {
			  unsigned int lambda = this->model->lambda[i];

			  if (lambda != 0)
			    {
				    this->model->Ic[lambda] = this->model->Ic[lambda] + this->model->X_lambda[i].apply (this->model->Ic[i]);
			    }

			  dof_i = i - 1;

			  // Modif here
			  SpatialRigidBodyInertia tmp1;   // double
			  Eigen::Matrix < T, 6, 1 > tmp2; // might be double or F<double>
			  tmp1 = this->model->Ic[i];
			  tmp2 = this->model->S[i];

			  T t_m = (T) tmp1.m;
			  Eigen::Matrix < T, 3, 1 > t_h ((T)tmp1.h[0],(T)tmp1.h[1],(T)tmp1.h[2]);
			  Eigen::Matrix < T, 3, 3 > t_I ((T)tmp1.I(0,0),(T)tmp1.I(0,1),(T)tmp1.I(0,2),
							 (T)tmp1.I(1,0),(T)tmp1.I(1,1),(T)tmp1.I(1,2),
							 (T)tmp1.I(2,0),(T)tmp1.I(2,1),(T)tmp1.I(2,2));

			  Eigen::Matrix < T, 3, 1 > tmp2_upper (tmp2[0], tmp2[1], tmp2[2]);
			  Eigen::Matrix < T, 3, 1 > tmp2_lower (tmp2[3], tmp2[4], tmp2[5]);

			  Eigen::Matrix < T, 3, 1 > res_upper = t_I * tmp2_upper + t_h.cross (tmp2_lower);
			  Eigen::Matrix < T, 3, 1 > res_lower = t_m * tmp2_lower - t_h.cross (tmp2_upper);

			  Eigen::Matrix < T, 6, 1 > F (res_upper[0], res_upper[1], res_upper[2],
								res_lower[0], res_lower[1], res_lower[2]);
			  // end modif here

// 			  Eigen::Matrix < T, 6, 1 > F = this->model->Ic[i] * this->model->S[i];
			  H (dof_i, dof_i) = this->model->S[i].dot (F);

			  unsigned int j = i;
			  unsigned int dof_j = dof_i;

			  while (this->model->lambda[j] != 0)
			    {
				    F = this->model->X_lambda[j].applyTranspose (F);
				    j = this->model->lambda[j];

				    dof_j = j - 1;

				    H (dof_i, dof_j) = F.dot (this->model->S[j]);
				    H (dof_j, dof_i) = H (dof_i, dof_j);
			    }
		  }
	}

}				/* namespace RigidBodyDynamics */
