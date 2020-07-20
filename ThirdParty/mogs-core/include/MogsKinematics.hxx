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
#include <cstring>
#include <assert.h>

#include "MogsKinematics.h"

#include <Eigen/src/Core/MatrixBase.h>

namespace RigidBodyDynamics
{

	template < typename T >
		void MogsKinematics < T >::UpdateKinematics (	const Eigen::Matrix <T, Eigen::Dynamic,1 > &Q,
								const Eigen::Matrix <T, Eigen::Dynamic,1 > &QDot,
								const Eigen::Matrix <T, Eigen::Dynamic,1 > &QDDot)
	{
                model->SetJointValue(Q,QDot,QDDot);
		unsigned int i;
// 		Eigen::Matrix < T, 6, 1 > spatial_gravity (0., 0., 0.,model->gravity[0],model->gravity[1],model->gravity[2]);
                Eigen::Matrix < T, 6, 1 > spatial_gravity = (Eigen::Matrix < T, 6, 1 > () <<0., 0., 0., this->model->gravity[0], this->model->gravity[1], this->model->gravity[2]).finished();
		model->a[0] = spatial_gravity;

		for (i = 1; i < model->mBodies.size (); i++)
                {
                    SpatialTransform < T > X_J;
                    Eigen::Matrix < T, 6, 1 > v_J;
                    Eigen::Matrix < T, 6, 1 > c_J;
                    MogsRobotJoint & joint = model->mJoints[i];
                    unsigned int lambda = model->lambda[i];

                    jcalc (i, X_J, model->S[i], v_J, c_J, joint.ratio_ * model->q_[i - 1], joint.ratio_ * model->dq_[i - 1]);

                    model->X_lambda[i] = X_J * model->X_T[i];

                    model->X_base[i] = model->X_lambda[i] *model->X_base.at (lambda);
                    model->v[i] = model->X_lambda[i].apply (model->v[lambda]) + v_J;
                    model->c[i] = c_J + crossm (model->v[i], v_J);

                    model->a[i] = model->X_lambda[i].apply (model->a[lambda]) + model->c[i];

                    Eigen::Matrix < T, 6, 1 > tmp;
                    for(int k=0;k<6;k++)
                        tmp(k) = model->S[i](k) * joint.ratio_ * model->ddq_[i - 1];
                    model->a[i] = model->a[i] + tmp;
                }
	}

	template < typename T >
		void MogsKinematics < T >::UpdateKinematicsCustom ( 	const Eigen::Matrix < T,Eigen::Dynamic, 1 > *Q,
									const Eigen::Matrix < T,Eigen::Dynamic, 1 > *QDot,
									const Eigen::Matrix < T,Eigen::Dynamic, 1 > *QDDot)
	{
                model->SetCustomJointValue(Q,QDot,QDDot);
		unsigned int i;
//          std::cout<<"UpdateKinematicsCustom"<<std::endl;
		if (Q)
		{
			for (i = 1; i < model->mBodies.size (); i++)
			{
//			    std::cout<<"i = "<< i <<" / "<< model->mBodies.size ()<<std::endl;
				Eigen::Matrix < T, 6, 1 > v_J;
				Eigen::Matrix < T, 6, 1 > c_J;
				SpatialTransform < T > X_J;
				MogsRobotJoint & joint = model->mJoints[i];
				unsigned int lambda = model->lambda[i];
				jcalc (i, X_J, model->S[i], v_J, c_J,joint.ratio_  * model->q_[i - 1], 0.);
				model->X_lambda[i] = X_J * model->X_T[i];
				model->X_base[i] = model->X_lambda[i] * model->X_base.at (lambda);
			}
		}

		if (QDot)
		  {
			  for (i = 1; i < model->mBodies.size (); i++)
			    {
				    Eigen::Matrix < T, 6, 1 > v_J;
				    Eigen::Matrix < T, 6, 1 > c_J;
				    SpatialTransform < T > X_J;
				    MogsRobotJoint & joint = model->mJoints[i];
				    unsigned int lambda = model->lambda[i];

				    jcalc (i, X_J, model->S[i], v_J, c_J,joint.ratio_ * model->q_[i - 1], joint.ratio_ * model->dq_[i - 1]);

				model->v[i] = model->X_lambda[i].apply(model->v[lambda]) + v_J;
				model->c[i] = c_J + crossm (model->v[i],v_J);
			    }
		  }

		if (QDDot)
                {
//                         Eigen::Matrix < T, 6, 1 > spatial_gravity (0., 0., 0.,model->gravity[0],model->gravity[1],model->gravity[2]);
                        Eigen::Matrix < T, 6, 1 > spatial_gravity = (Eigen::Matrix < T, 6, 1 > () <<0., 0., 0., this->model->gravity[0], this->model->gravity[1], this->model->gravity[2]).finished();
                        model->a[0] = spatial_gravity;                      
                        for (i = 1; i < model->mBodies.size (); i++)
                        {
                                MogsRobotJoint & joint = model->mJoints[i];
                                unsigned int lambda = model->lambda[i];

                                model->a[i] = model->X_lambda[i].apply (model->a[lambda]) + model->c[i];

//				    model->a[i] = model->a[i] + model->S[i].template cast<T>() * joint.ratio_ * (*QDDot)[i - 1];
                                Eigen::Matrix < T, 6, 1 > tmp;
                                for(int k=0;k<6;k++)
                                    tmp(k) = model->S[i](k) * joint.ratio_ * model->ddq_[i - 1];
                                model->a[i] = model->a[i] + tmp;
                        }
		  }
	}

	template < typename T >
	Eigen::Matrix <T,3,1 > MogsKinematics <T>::CalcBodyToBaseCoordinates (	const Eigen::Matrix<T,Eigen::Dynamic,1> &Q,
										unsigned int body_id,
										const Eigen::Matrix<T,3,1> &point_body_coordinates,
										bool update_kinematics)
	{
		if (update_kinematics)
		  {
			  UpdateKinematicsCustom (&Q, NULL, NULL);
		  }
		  return getBodyToBaseCoordinates(body_id,point_body_coordinates);
	}


	template < typename T >
	Eigen::Matrix <T,3,1 > MogsKinematics <T>::getBodyToBaseCoordinates (
		unsigned int body_id,
		const Eigen::Matrix<T,3,1> &point_body_coordinates) const
	{
		if (body_id >= model->fixed_body_discriminator)
		  {
			  unsigned int fbody_id = body_id - model->fixed_body_discriminator;
			  unsigned int parent_id = model->mFixedBodies[fbody_id].mMovableParent;

			  Eigen::Matrix < T, 3, 3 > fixed_rotation = model->mFixedBodies[fbody_id].mParentTransform.E.transpose ().template cast < T > ();
			  Eigen::Matrix < T, 3, 1 > fixed_position =model->mFixedBodies[fbody_id].mParentTransform.r.template cast < T > ();

			  Eigen::Matrix < T, 3, 3 > parent_body_rotation = model->X_base[parent_id].E.transpose ();
			  Eigen::Matrix < T, 3, 1 > parent_body_position = model->X_base[parent_id].r;
			  return parent_body_position + parent_body_rotation * (fixed_position + fixed_rotation * (point_body_coordinates));
		  }

		Eigen::Matrix < T, 3, 3 > body_rotation = model->X_base[body_id].E.transpose ();
		Eigen::Matrix < T, 3, 1 > body_position = model->X_base[body_id].r;

		return body_position + body_rotation * point_body_coordinates;
	}

	template < typename T >
	Eigen::Matrix <T,3,1 > MogsKinematics <T>::getBodyToBaseCoordinates (
		unsigned int body_id) const
	{
		if (body_id >= model->fixed_body_discriminator)
		  {
			  unsigned int fbody_id = body_id - model->fixed_body_discriminator;
			  unsigned int parent_id = model->mFixedBodies[fbody_id].mMovableParent;
			  // parent_body_position
			  return model->X_base[parent_id].r;
		  }
		// body_position
		return model->X_base[body_id].r;
	}
	
	
	template < typename T >
        Eigen::Matrix<T,3,1> MogsKinematics <T>::getBaseToBodyCoordinates(    unsigned int body_id,
                                                                        const Eigen::Matrix<T,3,1> & world_point_position)
        {
                SpatialTransform<T> trans;
                getFrameCoordinate(body_id,trans);
                return trans.Get_Local_Position(world_point_position);
        }
        
	template < typename T >
        Eigen::Matrix<T,3,1> MogsKinematics <T>::getBaseToBodyCoordinates(    unsigned int source_body_id,
                                                                        unsigned int target_body_id,
                                                                        const Eigen::Matrix<T,3,1> & source_point_position)
        {
                Eigen::Matrix<T,3,1> tmp = getPosition(source_body_id,source_point_position);
                SpatialTransform<T> trans;
                getFrameCoordinate(target_body_id,trans);
                return trans.Get_Local_Position(tmp);
        }        

	template < typename T >
	Eigen::Matrix <T,3,1 > MogsKinematics <T>::getCenterOfMAss(	const Eigen::Matrix<T,Eigen::Dynamic,1> &Q,
								bool update_kinematics)
	{
		if (update_kinematics)
			UpdateKinematicsCustom(&Q,NULL,NULL);

		return getCenterOfMAss();
	}

	template < typename T >
	Eigen::Matrix <T,3,1 > MogsKinematics <T>::getCenterOfMAss( )
	{
		Eigen::Matrix <T,3,1 > com(0.,0.,0.);
		double mass = 0;
		int nbbodies = getNBodies();
		for (int i=0;i<nbbodies;i++)
		{
			com = com + getPosition(i, model->mBodies[i].mCenterOfMass) * model->mBodies[i].mMass;
			mass += model->mBodies[i].mMass;
		}
		com(0) /= mass;
		com(1) /= mass;
		com(2) /= mass;
		return com;
	}

	template < typename T >
	void MogsKinematics <T>::getFrameCoordinate(unsigned int body_id,
						Eigen::Matrix<T,3,1> & rot_RPY,
						Eigen::Matrix<T,3,1> & pos) const
	{
		if (body_id >= model->fixed_body_discriminator)
		  {
			  unsigned int fbody_id = body_id - model->fixed_body_discriminator;
			  unsigned int parent_id = model->mFixedBodies[fbody_id].mMovableParent;
			  model->X_base[parent_id].get_RPY(rot_RPY,pos) ;
			  return;
		  }
		model->X_base[body_id].get_RPY(rot_RPY,pos) ;
	}

	template < typename T >
	void MogsKinematics <T>::getFrameOrientation(unsigned int body_id,
						Eigen::Matrix<T,3,1> & rot_RPY) const
	{
		if (body_id >= model->fixed_body_discriminator)
		  {
			  unsigned int fbody_id = body_id - model->fixed_body_discriminator;
			  unsigned int parent_id = model->mFixedBodies[fbody_id].mMovableParent;
			  model->X_base[parent_id].get_RPY(rot_RPY) ;
			  return;
		  }
		model->X_base[body_id].get_RPY(rot_RPY) ;
	}

	template < typename T >
	void MogsKinematics <T>::getFrameCoordinate(unsigned int body_id,
						Eigen::Matrix<T,4,4> & transform) const
	{
		if (body_id >= model->fixed_body_discriminator)
		{
			  unsigned int fbody_id = body_id - model->fixed_body_discriminator;
			  unsigned int parent_id = model->mFixedBodies[fbody_id].mMovableParent;
			  model->X_base[parent_id].get_4x4(transform) ;
			  return;
		}
		model->X_base[body_id].get_4x4(transform) ;
	}

	template < typename T >
	void MogsKinematics <T>::getFrameCoordinate(unsigned int body_id,
						SpatialTransform<T> & transform) const
	{
		if (body_id >= model->fixed_body_discriminator)
		{
			  unsigned int fbody_id = body_id - model->fixed_body_discriminator;
			  unsigned int parent_id = model->mFixedBodies[fbody_id].mMovableParent;
			  transform = model->X_base[parent_id];
			  return;
		}
		transform = model->X_base[body_id];
	}


	template < typename T >
	Eigen::Matrix < T, 3, 1 > MogsKinematics <T >::CalcBaseToBodyCoordinates (	const Eigen::Matrix < T,
										Eigen::Dynamic, 1 > &Q,
										unsigned int body_id,
										const Eigen::Matrix < T, 3,
										1 > &point_base_coordinates,
										bool update_kinematics)
	{
		if (update_kinematics)
		  {
			  UpdateKinematicsCustom (&Q, NULL, NULL);
		  }

		if (body_id >= model->fixed_body_discriminator)
		  {
			  unsigned int fbody_id =
				  body_id - model->fixed_body_discriminator;
			  unsigned int parent_id = model->mFixedBodies[fbody_id].mMovableParent;

			  Eigen::Matrix < T, 3, 3 > fixed_rotation = model->mFixedBodies[fbody_id].mParentTransform.E.template cast < T > ();
			  Eigen::Matrix < T, 3, 1 > fixed_position = model->mFixedBodies[fbody_id].mParentTransform.r.template cast < T > ();

			  Eigen::Matrix < T, 3, 3 > parent_body_rotation = model->X_base[parent_id].E;
			  Eigen::Matrix < T, 3, 1 > parent_body_position = model->X_base[parent_id].r;

			  return fixed_rotation * (-fixed_position - parent_body_rotation * (parent_body_position - point_base_coordinates));
		  }

		Eigen::Matrix < T, 3, 3 > body_rotation = model->X_base[body_id].E;
		Eigen::Matrix < T, 3, 1 > body_position = model->X_base[body_id].r;

		return body_rotation * (point_base_coordinates - body_position);
	}

	template < typename T >
	Eigen::Matrix < T, 3, 3 > MogsKinematics < T >::CalcBodyWorldOrientation (	const Eigen::Matrix < T,
										Eigen::Dynamic, 1 > &Q,
										const unsigned int body_id,
										bool update_kinematics)
	{
		// update the MogsKinematics if necessary
		if (update_kinematics)
		  {
			  UpdateKinematicsCustom (&Q, NULL, NULL);
		  }

		if (body_id >= model->fixed_body_discriminator)
		  {
			  // fixed body orientation
			  unsigned int fbody_id =
				  body_id - model->fixed_body_discriminator;
// 			  model->mFixedBodies[fbody_id].mBaseTransform = model->X_base[model->mFixedBodies[fbody_id].mMovableParent] * model->mFixedBodies[fbody_id].mParentTransform;
// 			  return model->mFixedBodies[fbody_id].mBaseTransform.E;
			return (model->X_base[model->mFixedBodies[fbody_id].mMovableParent] * model->mFixedBodies[fbody_id].mParentTransform).E;
		  }

		return model->X_base[body_id].E;
	}

	template < typename T >
	Eigen::Matrix < T, 3, 3 > MogsKinematics < T >::CalcBodyWorldOrientation (	const unsigned int body_id) const
	{

		if (body_id >= model->fixed_body_discriminator)
		  {
			  // fixed body orientation
			  unsigned int fbody_id =  body_id - model->fixed_body_discriminator;
// 			  model->mFixedBodies[fbody_id].mBaseTransform = model->X_base[model->mFixedBodies[fbody_id].mMovableParent] * model->mFixedBodies[fbody_id].mParentTransform;
// 			  return model->mFixedBodies[fbody_id].mBaseTransform.E.template cast<T> ();
			  return (model->X_base[model->mFixedBodies[fbody_id].mMovableParent] * model->mFixedBodies[fbody_id].mParentTransform).E;
		  }

		return model->X_base[body_id].E;
	}

	template < typename T >
	Eigen::Matrix<T,3,1>  MogsKinematics < T >::get_RPY_orientation(const unsigned int body_id,
									const Eigen::Matrix<T,3,3> &rot)	const
	{
		Eigen::Matrix < T, 3, 1 > out;
		Eigen::Matrix < T, 3, 3 > tmp = rot * CalcBodyWorldOrientation(body_id);
		convert_mat_to_RPY( tmp , out);
		return out;
	}

	template < typename T >
	void MogsKinematics < T >::CalcPointJacobian (	const Eigen::Matrix < T, Eigen::Dynamic, 1 > &Q,
							unsigned int body_id,
							const Eigen::Matrix < T, 3, 1 > &point_position,
							Eigen::Matrix < T, 3, Eigen::Dynamic > &G,
							bool update_kinematics)
	{
		// update the MogsKinematics if necessary
		if (update_kinematics)
		  {
			  UpdateKinematicsCustom (&Q, NULL, NULL);
		  }

		Eigen::Matrix < T, 3, 1 > point_base_pos = CalcBodyToBaseCoordinates (Q, body_id, point_position,false);
		Eigen::Matrix < T, 6, 6 > point_trans = Xtrans_mat (point_base_pos);
		assert (G.rows () == 3 && G.cols () == model->getNDof());
		G.setZero ();

		// we have to make sure that only the joints that contribute to the
		// bodies motion also get non-zero columns in the jacobian.
		// Eigen::Matrix<T,Eigen::Dynamic,1> e = Eigen::Matrix<T,Eigen::Dynamic,1>::Zero(Q.size() + 1);
		char *e = new char[Q.size () + 1];
		if (e == NULL)
		{
			  std::cerr << "Error: allocating memory." << std::endl;
			  abort ();
		}
		memset (&e[0], 0, Q.size () + 1);

		unsigned int reference_body_id = body_id;

		if (model->IsFixedBodyId (body_id))
		{
			  unsigned int fbody_id = body_id - model->fixed_body_discriminator;
			  reference_body_id = model->mFixedBodies[fbody_id].mMovableParent;
		}

		unsigned int j = reference_body_id;

		// e[j] is set to 1 if joint j contributes to the jacobian that we are
		// computing. For all other joints the column will be zero.
		while (j != 0)
		{
			e[j] = 1;
			j = model->lambda[j];
		}

		G = Eigen::Matrix < T, 3, Eigen::Dynamic >::Zero(3,model->getNDof());
		for (j = 1; j < model->mBodies.size (); j++)
		{
//                         std::cout<<"model->info_mimic_["<<j-1<<"].source_id = "<< model->info_mimic_[j-1].source_id <<std::endl;
			if (e[j] == 1)
			{
				MogsRobotJoint & joint = model->mJoints[j];
				Eigen::Matrix < T, 6, 1 > S_base;
				S_base = point_trans * spatial_inverse (model->X_base[j].toMatrix ()) * model->S[j].template cast<T>();
				G (0, model->info_mimic_[j-1].source_id) += S_base[3] * joint.ratio_;
				G (1, model->info_mimic_[j-1].source_id) += S_base[4] * joint.ratio_;
				G (2, model->info_mimic_[j-1].source_id) += S_base[5] * joint.ratio_;
			}
		}
		delete[]e;
	}

	template < typename T >
	void MogsKinematics < T >::Calc6DJacobian (	const Eigen::Matrix < T, Eigen::Dynamic, 1 > &Q,
							unsigned int body_id,
							Eigen::Matrix < T, 6, Eigen::Dynamic > &G,
							bool update_kinematics)
	{
		// update the MogsKinematics if necessary
		if (update_kinematics)
		{
			  UpdateKinematicsCustom (&Q, NULL, NULL);
		}

		Eigen::Matrix < T, 3, 1 > point_base_pos = getBodyToBaseCoordinates (body_id);
		Eigen::Matrix < T, 6, 6 > point_trans = Xtrans_mat (point_base_pos);
		assert (G.rows () == 3 && G.cols () == model->dof_count);
		G.setZero ();

		// we have to make sure that only the joints that contribute to the
		// bodies motion also get non-zero columns in the jacobian.
		// Eigen::Matrix<T,Eigen::Dynamic,1> e = Eigen::Matrix<T,Eigen::Dynamic,1>::Zero(Q.size() + 1);
		char *e = new char[Q.size () + 1];
		if (e == NULL)
		{
			  std::cerr << "Error: allocating memory." << std::endl;
			  abort ();
		}
		memset (&e[0], 0, Q.size () + 1);

		unsigned int reference_body_id = body_id;

		if (model->IsFixedBodyId (body_id))
		{
			  unsigned int fbody_id = body_id - model->fixed_body_discriminator;
			  reference_body_id = model->mFixedBodies[fbody_id].mMovableParent;
		}

		unsigned int j = reference_body_id;

		// e[j] is set to 1 if joint j contributes to the jacobian that we are
		// computing. For all other joints the column will be zero.
		while (j != 0)
		{
			  e[j] = 1;
			  j = model->lambda[j];
		}

		for (j = 1; j < model->mBodies.size (); j++)
		{
			if (e[j] == 1)
			{
				MogsRobotJoint & joint = model->mJoints[j];
				Eigen::Matrix < T, 6, 1 > S_base;
// 				S_base = point_trans * model->X_base[j].spatial_inverse () * model->S[j].template cast<T>();
				S_base = point_trans * spatial_inverse (model->X_base[j].toMatrix ())  * model->S[j].template cast<T>();


				for(int k=0;k<3;k++)
					G (k+3, j - 1) = S_base[k] * joint.ratio_;
				for(int k=0;k<3;k++)
					G (k, j - 1) = S_base[k+3] * joint.ratio_;
			}
		}
		delete[]e;
	}

	template < typename T >
	Eigen::Matrix < T, 3, 1 > MogsKinematics < T >::CalcPointVelocity (const Eigen::Matrix <T, Eigen::Dynamic,1 > &Q,
									const Eigen::Matrix <T, Eigen::Dynamic,1 > &QDot,
									unsigned int body_id,
									const Eigen::Matrix <T, 3, 1 > &point_position,
									bool update_kinematics)
	{
		assert (model->IsBodyId (body_id));
		assert (model->mBodies.size () == Q.size () + 1);
		assert (model->mBodies.size () == QDot.size () + 1);

		// Reset the velocity of the root body
		model->v[0].setZero ();

		// update the MogsKinematics with zero acceleration
		if (update_kinematics)
		{
			  UpdateKinematicsCustom (&Q, &QDot, NULL);
		}

		Eigen::Matrix < T, 3, 1 > point_abs_pos = CalcBodyToBaseCoordinates (Q, body_id, point_position,false);

		unsigned int reference_body_id = body_id;

		if (model->IsFixedBodyId (body_id))
		{
			  unsigned int fbody_id = body_id - model->fixed_body_discriminator;
			  reference_body_id = model->mFixedBodies[fbody_id].mMovableParent;
		}

		// Now we can compute the spatial velocity at the given point
//      Eigen::Matrix< T, 6, 1> body_global_velocity (global_velocities.at(body_id));
		Eigen::Matrix < T, 6, 1 > point_spatial_velocity = Xtrans_mat (point_abs_pos) * spatial_inverse (model->X_base[reference_body_id].toMatrix ()) * model->v[reference_body_id];

		return Eigen::Matrix < T, 3, 1 > (point_spatial_velocity[3],point_spatial_velocity[4],point_spatial_velocity[5]);
	}

	template < typename T >
	Eigen::Matrix < T, 3, 1 > MogsKinematics < T >::CalcPointVelocity ( unsigned int body_id,
									const Eigen::Matrix <T, 3, 1 > &point_position)
	{
		assert (model->IsBodyId (body_id));

		Eigen::Matrix < T, 3, 1 > point_abs_pos = getBodyToBaseCoordinates (body_id, point_position);

		unsigned int reference_body_id = body_id;

		if (model->IsFixedBodyId (body_id))
		{
			  unsigned int fbody_id = body_id - model->fixed_body_discriminator;
			  reference_body_id = model->mFixedBodies[fbody_id].mMovableParent;
		}

		// Now we can compute the spatial velocity at the given point
//      Eigen::Matrix< T, 6, 1> body_global_velocity (global_velocities.at(body_id));
		Eigen::Matrix < T, 6, 1 > point_spatial_velocity = Xtrans_mat (point_abs_pos) * spatial_inverse (model->X_base[reference_body_id].toMatrix ()) * model->v[reference_body_id];

		return Eigen::Matrix < T, 3, 1 > (point_spatial_velocity[3],point_spatial_velocity[4],point_spatial_velocity[5]);
	}

	template < typename T >
	Eigen::Matrix < T, 3, 1 > MogsKinematics < T >::CalcFrameVelocity ( unsigned int body_id) const
	{
		unsigned int reference_body_id = body_id;

		if (model->IsFixedBodyId (body_id))
		{
			  unsigned int fbody_id = body_id - model->fixed_body_discriminator;
			  reference_body_id = model->mFixedBodies[fbody_id].mMovableParent;
		}
		Eigen::Matrix < T, 6, 1 > point_spatial_velocity =spatial_inverse (model->X_base[reference_body_id].toMatrix ()) * model->v[reference_body_id];

		return Eigen::Matrix < T, 3, 1 > (point_spatial_velocity[3],point_spatial_velocity[4],point_spatial_velocity[5]);
	}

	template < typename T >
	Eigen::Matrix < T, 6, 1 > MogsKinematics < T >::get6DVelocity ( unsigned int body_id) const
	{
		unsigned int reference_body_id = body_id;

		if (model->IsFixedBodyId (body_id))
		{
			  unsigned int fbody_id = body_id - model->fixed_body_discriminator;
			  reference_body_id = model->mFixedBodies[fbody_id].mMovableParent;
		}
		return spatial_inverse (model->X_base[reference_body_id].toMatrix ()) * model->v[reference_body_id];
	}

	template < typename T >
	Eigen::Matrix < T, 3, 1 > MogsKinematics < T >::CalcPointAcceleration (	const Eigen::Matrix < T, Eigen::Dynamic, 1 > &Q,
										const Eigen::Matrix < T, Eigen::Dynamic, 1 > &QDot,
										const Eigen::Matrix < T, Eigen::Dynamic, 1 > &QDDot,
										unsigned int body_id,
										const Eigen::Matrix < T, 3, 1 > &point_position,
										bool update_kinematics)
	{
		// Reset the velocity of the root body
		model->v[0].setZero ();
		model->a[0].setZero ();

		if (update_kinematics)
			UpdateKinematics (Q, QDot, QDDot);

		unsigned int reference_body_id = body_id;
		Eigen::Matrix < T, 3, 1 > reference_point = point_position;

		if (model->IsFixedBodyId (body_id))
		  {
			  unsigned int fbody_id = body_id - model->fixed_body_discriminator;
			  reference_body_id = model->mFixedBodies[fbody_id].mMovableParent;
			  Eigen::Matrix < T, 3, 1 > base_coords = CalcBodyToBaseCoordinates (Q, body_id,point_position,false);
			  reference_point = CalcBaseToBodyCoordinates (Q,reference_body_id,base_coords,false);
		  }

		Eigen::Matrix < T, 6, 1 > body_global_velocity (spatial_inverse(model->X_base[reference_body_id].toMatrix ()) * model->v[reference_body_id]);
		Eigen::Matrix < T, 3, 3 > global_body_orientation_inv = CalcBodyWorldOrientation (Q, reference_body_id,false).transpose ();
		SpatialTransform < T > p_X_i (global_body_orientation_inv, reference_point);
		Eigen::Matrix < T, 6, 1 > p_v_i = p_X_i.apply (model->v[reference_body_id]);
		Eigen::Matrix < T, 6, 1 > p_a_i = p_X_i.apply (model->a[reference_body_id]);
		Eigen::Matrix < T, 6, 1 > frame_acceleration = crossm (Eigen::Matrix < T, 6, 1 > (0., 0., 0., p_v_i[3], p_v_i[4],p_v_i[5]), (body_global_velocity));
		Eigen::Matrix < T, 6, 1 > p_a_i_dash = p_a_i - frame_acceleration;
		return Eigen::Matrix < T, 3, 1 > (p_a_i_dash[3],p_a_i_dash[4],p_a_i_dash[5]);
	}

	template < typename T >
	Eigen::Matrix < T, 3, 1 > MogsKinematics < T >::CalcPointAcceleration (	unsigned int body_id,
										const Eigen::Matrix < T, 3, 1 > &point_position)
	{
		unsigned int reference_body_id = body_id;
		Eigen::Matrix < T, 3, 1 > reference_point = point_position;

		if (model->IsFixedBodyId (body_id))
		  {
			  unsigned int fbody_id = body_id - model->fixed_body_discriminator;
			  reference_body_id = model->mFixedBodies[fbody_id].mMovableParent;
			  Eigen::Matrix < T, 3, 1 > base_coords = getBodyToBaseCoordinates (body_id,point_position);
			  reference_point = getBodyToBaseCoordinates (reference_body_id,base_coords);
		  }

		Eigen::Matrix < T, 6, 1 > body_global_velocity (spatial_inverse(model->X_base[reference_body_id].toMatrix ()) * model->v[reference_body_id]);
		Eigen::Matrix < T, 3, 3 > global_body_orientation_inv = CalcBodyWorldOrientation (reference_body_id).transpose ();
		SpatialTransform < T > p_X_i (global_body_orientation_inv, reference_point);
		Eigen::Matrix < T, 6, 1 > p_v_i = p_X_i.apply (model->v[reference_body_id]);
		Eigen::Matrix < T, 6, 1 > p_a_i = p_X_i.apply (model->a[reference_body_id]);
		Eigen::Matrix < T, 6, 1 > frame_acceleration = crossm (Eigen::Matrix < T, 6, 1 > (0., 0., 0., p_v_i[3], p_v_i[4],p_v_i[5]), (body_global_velocity));
		Eigen::Matrix < T, 6, 1 > p_a_i_dash = p_a_i - frame_acceleration;
		return Eigen::Matrix < T, 3, 1 > (p_a_i_dash[3],p_a_i_dash[4],p_a_i_dash[5]);
	}

	template < typename T > Eigen::Matrix < T, 6, 1 > MogsKinematics < T >::get6DAcceleration(	unsigned int body_id)
	{
		unsigned int reference_body_id = body_id;
		if (model->IsFixedBodyId (body_id))
		{
			  unsigned int fbody_id = body_id - model->fixed_body_discriminator;
			  reference_body_id = model->mFixedBodies[fbody_id].mMovableParent;
		}
		Eigen::Matrix < T, 6, 1 > body_global_velocity (spatial_inverse(model->X_base[reference_body_id].toMatrix ()) * model->v[reference_body_id]);
		Eigen::Matrix < T, 3, 3 > global_body_orientation_inv = CalcBodyWorldOrientation (reference_body_id).transpose ();
		SpatialTransform < T > p_X_i (global_body_orientation_inv, Eigen::Matrix < T, 3, 1 >::Zero(3,1));
		Eigen::Matrix < T, 6, 1 > p_v_i = p_X_i.apply (model->v[reference_body_id]);
		Eigen::Matrix < T, 6, 1 > p_a_i = p_X_i.apply (model->a[reference_body_id]);
		Eigen::Matrix < T, 6, 1 > frame_acceleration = crossm (Eigen::Matrix < T, 6, 1 > (0., 0., 0., p_v_i[3], p_v_i[4],p_v_i[5]), (body_global_velocity));
		return p_a_i - frame_acceleration;
	}

	template < typename T > bool MogsKinematics < T >::InverseKinematics (	const Eigen::Matrix < T, Eigen::Dynamic, 1 > &Qinit,
										const std::vector < unsigned int >&body_id,
										const std::vector < Eigen::Matrix < T, 3, 1 > >&body_point,
										const std::vector < Eigen::Matrix < T, 3, 1 > >&target_pos,
										Eigen::Matrix < T, Eigen::Dynamic, 1 > &Qres,
										double step_tol,
										double lambda,
										unsigned int max_iter)
	{

		assert (Qinit.size () == model->dof_count);
		assert (body_id.size () == body_point.size ());
		assert (body_id.size () == target_pos.size ());

		Eigen::MatrixXd J = Eigen::MatrixXd::Zero (3 * body_id.size (), model->dof_count);
		Eigen::Matrix < T, Eigen::Dynamic, 1 > e = Eigen::Matrix < T, Eigen::Dynamic, 1 >::Zero (3 * body_id.size ());

		Qres = Qinit;

		for (unsigned int ik_iter = 0; ik_iter < max_iter; ik_iter++)
		  {
			  UpdateKinematicsCustom (&Qres, NULL, NULL);

			  for (unsigned int k = 0; k < body_id.size (); k++)
			    {
				    Eigen::Matrix < T, 3, Eigen::Dynamic > G (3, model->dof_count);
				    CalcPointJacobian (Qres,body_id[k],body_point[k], G,false);
				    Eigen::Matrix < T, 3, 1 > point_base = CalcBodyToBaseCoordinates (Qres,body_id[k],body_point[k],false);

				    for (unsigned int i = 0; i < 3; i++)
				      {
					      for (unsigned int j = 0; j < model->dof_count; j++)
						{
							unsigned int row = k * 3 + i;
							J (row, j) = G (i, j);
						}

					      e[k * 3 + i] = target_pos[k][i] - point_base[i];
				      }
				    // abort if we are getting "close"
				    if (e.norm () < step_tol)
				      {
					      return true;
				      }
			    }

			  Eigen::Matrix < T, Eigen::Dynamic, Eigen::Dynamic > JJTe_lambda2_I = J * J.transpose () + lambda * lambda * Eigen::Matrix < T, Eigen::Dynamic, Eigen::Dynamic >::Identity (e.size (),e.size ());

			  Eigen::Matrix < T, Eigen::Dynamic, 1 > z (body_id.size () * 3);
#ifndef RBDL_USE_SIMPLE_MATH
			  z = JJTe_lambda2_I.colPivHouseholderQr ().solve (e);
#else
			  bool solve_successful =
				  LinSolveGaussElimPivot (JJTe_lambda2_I, e,z);
			  assert (solve_successful);
#endif
			  Eigen::Matrix < T, Eigen::Dynamic, 1 > delta_theta = J.transpose () * z;
			  Qres = Qres + delta_theta;
			  if (delta_theta.norm () < step_tol)
			    {
				    return true;
			    }

			  Eigen::Matrix < T, Eigen::Dynamic, 1 > test_1 (z.size ());
			  Eigen::Matrix < T, Eigen::Dynamic, 1 > test_res (z.size ());

			  test_1.setZero ();

			  for (unsigned int i = 0; i < z.size (); i++)
			    {
				    test_1[i] = 1.;
				    Eigen::Matrix < T, Eigen::Dynamic, 1 > test_delta = J.transpose () * test_1;
				    test_res[i] = test_delta.squaredNorm ();
				    test_1[i] = 0.;
			    }
		  }

		return false;
	}

	template < typename T >
		void MogsKinematics < T >::jcalc (const unsigned int &joint_id,
					      SpatialTransform < T > &XJ,
					      Eigen::Matrix < double, 6, 1 > &S,
					      Eigen::Matrix < T, 6, 1 > &v_J,
					      Eigen::Matrix < T, 6, 1 > &c_J,
					      const T & q, const T & qdot)
	{
		// exception if we calculate it for the root body
		assert (joint_id > 0);

		// Calculate the spatial joint velocity
		// FIXME why is not in RBDL from Martin Felis ?
// 		v_J = model->S.at (joint_id);

		// Set the joint axis
		S = model->mJoints[joint_id].mJointAxes[0];

		// the velocity dependent spatial acceleration is != 0 only for rhenomic
		// constraints (see RBDA, p. 55)
		c_J.setZero ();
		if (model->mJoints[joint_id].mJointType == JointTypeRevolute)
		{
			XJ = Xrot (q, Eigen::Matrix < double, 3, 1 >(	model->mJoints[joint_id].mJointAxes[0][0],
                                                            model->mJoints[joint_id].mJointAxes[0][1],
                                                            model->mJoints[joint_id].mJointAxes[0][2]));
		}
		else if (model->mJoints[joint_id].mJointType == JointTypePrismatic)
		{
			XJ = Xtrans (Eigen::Matrix < T, 3, 1 >
				(model->mJoints[joint_id].mJointAxes[0][3] * q,
				model->mJoints[joint_id].mJointAxes[0][4] * q,
				model->mJoints[joint_id].mJointAxes[0][5] * q));
		}
		else
		{
			// Only revolute joints supported so far
			assert (0);
		}
// 		v_J = S * qdot;
		for (int i=0;i<6;i++)
			v_J(i) = S(i) * qdot;
	}

	template < typename T >
		bool MogsKinematics < T >::SetRobotFile (const mogs_string name,
							bool forced_fixed)
	{
	    if(!model)
            model = new MogsTemplateRobotProperties < T > ();
		return model->SetRobotFile (name, forced_fixed);
	}

	template < typename T >
	void MogsKinematics < T >::SetRobot(  MogsTemplateRobotProperties < double >* Robot_in)
	{
		model = new MogsTemplateRobotProperties <T> (Robot_in);
	}

	template < typename T >
	void MogsKinematics < T >::SetRobot(  MogsRobotProperties * Robot_in)
	{
		model = new MogsTemplateRobotProperties <T> (Robot_in);
	}

	template < typename T >
	void MogsKinematics < T >::SetRobotSphere(double radius, double mass)
	{
		model = new MogsTemplateRobotProperties<T>();
		model->SetRobotSphere(radius,mass);
	}

}
