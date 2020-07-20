/*
 * RBDL - Rigid MogsRobotBody Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 * Code Modified by S. Lengagne during his work in the following labs:
 *      2011-2012:  Karlsruhe Institute fur Technologie, Karlsruhe, Germany
 *      2013-2012: IUT de Beziers/ LIRMM, Beziers, France
 */


#ifndef __MOGSROBOTBODY_H__
#define __MOGSROBOTBODY_H__

// to restore
#include <assert.h>
#include <string>

#include "MogsRBDLMathUtils.h"
#include "MogsSpatialAlgebraOperators.h"
#include "MogsSerialization.h"
#include "MogsGeometry.h"
#include "MogsAbstractCollisionDefinition.h"

namespace RigidBodyDynamics
{

/** \brief Describes all properties of a single body
 *
 * A MogsRobotBody contains information about mass, the location of its center of
 * mass, and the ineria tensor in the center of mass. This class is
 * designed to use the given information and transform it such that it can
 * directly be used by the spatial algebra.
 *
 * The spatial inertia of the member variable MogsRobotBody::mSpatialInertia is
 * expressed at the origin of the coordinate frame.
 *
 */
	class MogsRobotBody
	{
	      public:

		MogsRobotBody ():
			name_ ("unnamed"),
			mMass (0.),
			mCenterOfMass (0., 0., 0.),
			mInertia (Eigen::Matrix < double, 3, 3 >::Zero (3, 3)),
			mSpatialInertia (Eigen::Matrix < double, 6, 6 >::Zero (6, 6)),
			collision_definition_(NULL)
		{
			mGeom = NULL;
		};

		MogsRobotBody (const MogsRobotBody & body):
			name_ (body.name_),
			mMass (body.mMass),
			mCenterOfMass (body.mCenterOfMass),
			mInertia (body.mInertia),
			mSpatialInertia (body.mSpatialInertia)
		{
			length_ = body.length_;
			mGeom = body.mGeom;
			collision_definition_ = body.collision_definition_;
		};

		MogsRobotBody (const mogs_string &  name,
		      MogsGeometry * geom = NULL):
		name_ (name),
		mMass (1.),
		mCenterOfMass (0., 0., 0.),
		mInertia (Eigen::Matrix < double, 3, 3 >::Zero (3, 3)),
		mSpatialInertia (Eigen::Matrix < double, 6, 6 >::Zero (6, 6)),
		collision_definition_(NULL)
		{
			mGeom = geom;
		};

		MogsRobotBody & operator= (const MogsRobotBody & body)
		{
			if (this != &body)
			{
				name_ = body.name_;
				mMass = body.mMass;
				mInertia = body.mInertia;
				mCenterOfMass = body.mCenterOfMass;
				mSpatialInertia = body.mSpatialInertia;
				length_ = body.length_;

				if(body.mGeom)
					mGeom = new MogsGeometry (body.mGeom);
				else
					mGeom = NULL;

                // FIXME
//				if(body.collision_definition_)
//					body.collision_definition_->copy(&collision_definition_);
//				else
					collision_definition_ = NULL;
			}

			return *this;
		}

	/** \brief Constructs a body from mass, center of mass and radii of gyration
	 *
	 * This constructor eases the construction of a new body as all the
	 * required parameters can simply be specified as parameters to the
	 * constructor. These are then used to generate the spatial inertia
	 * matrix which is expressed at the origin.
	 *
	 * \param mass the mass of the body
	 * \param com  the position of the center of mass in the bodies coordinates
	 * \param gyration_radius the radius of gyration at the center of mass of the body
	 * \param geom pointer to the MogsGeometry. Can be set to NULL
	 */
	      MogsRobotBody (	const mogs_string &  name,
				const double &mass,
				const Eigen::Matrix < double, 3, 1 > &com,
				const Eigen::Matrix < double, 3, 1 > &gyration_radius,
				MogsGeometry * geom = NULL):
		name_ (name), mMass (mass), mCenterOfMass (com)
		{

			mGeom = geom;

			Eigen::Matrix < double, 3, 3 > com_cross = (Eigen::Matrix < double, 3, 3 >()<<0., -com[2], com[1], com[2], 0., -com[0], -com[1], com[0], 0.).finished();
			Eigen::Matrix < double, 3, 3 > parallel_axis;
			parallel_axis = mass * com_cross * com_cross.transpose ();

			Eigen::Matrix < double, 3, 1 > gr (gyration_radius);
			Eigen::Matrix < double, 3, 3 > pa (parallel_axis);
			Eigen::Matrix < double, 3, 3 > mcc = mass * com_cross;
			Eigen::Matrix < double, 3, 3 > mccT = mcc.transpose ();

//			mSpatialInertia.set (gr[0] + pa (0, 0), pa (0, 1),pa (0, 2), mcc (0, 0), mcc (0,1), mcc (0, 2), pa (1, 0), gr[1] + pa (1, 1), pa (1, 2), mcc (1, 0), mcc (1, 1), mcc (1,2), pa (2, 0), pa (2, 1), gr[2] + pa (2, 2), mcc (2, 0), mcc (2, 1), mcc (2, 2), mccT (0, 0), mccT (0, 1), mccT (0, 2), mass, 0., 0., mccT (1, 0), mccT (1, 1), mccT (1, 2), 0., mass, 0., mccT (2, 0), mccT (2, 1), mccT (2, 2), 0., 0., mass);
            mSpatialInertia = (Eigen::Matrix < double, 6, 6 >()<< gr[0] + pa (0, 0), pa (0, 1),pa (0, 2), mcc (0, 0), mcc (0,1), mcc (0, 2), pa (1, 0), gr[1] + pa (1, 1), pa (1, 2), mcc (1, 0), mcc (1, 1), mcc (1,2), pa (2, 0), pa (2, 1), gr[2] + pa (2, 2), mcc (2, 0), mcc (2, 1), mcc (2, 2), mccT (0, 0), mccT (0, 1), mccT (0, 2), mass, 0., 0., mccT (1, 0), mccT (1, 1), mccT (1, 2), 0., mass, 0., mccT (2, 0), mccT (2, 1), mccT (2, 2), 0., 0., mass).finished();

			mInertia = mSpatialInertia.block < 3, 3 > (0, 0);
		}

	/** \brief Constructs a body from mass, center of mass, and a 3x3 inertia matrix
	 *
	 * This constructor eases the construction of a new body as all the
	 * required parameters can simply be specified as parameters to the
	 * constructor. These are then used to generate the spatial inertia
	 * matrix which is expressed at the origin.
	 *
	 * \param mass the mass of the body
	 * \param com  the position of the center of mass in the bodies coordinates
	 * \param inertia_C the inertia at the center of mass
	 * \param geom pointer to the MogsGeometry
	 */
		MogsRobotBody (const mogs_string &  name,
			const double &mass,
			const Eigen::Matrix < double, 3, 1 > &com,
			const Eigen::Matrix < double, 3, 3 > &inertia_C,
			double length,
			MogsGeometry * geom = NULL):
			name_ (name),
			mMass (mass),
			mCenterOfMass (com),
			mInertia (inertia_C)
		{

			mGeom = geom;
			length_ = length;

			mSpatialInertia = compute_SpatialInertia(mass, com, inertia_C);
		}


		Eigen::Matrix < double, 6, 6 > compute_SpatialInertia( 	const double &mass,
									const Eigen::Matrix < double, 3, 1 > &com,
									const Eigen::Matrix < double, 3, 3 > &inertia_C)
		{
//			Eigen::Matrix < double, 6, 6 > out;
			Eigen::Matrix < double, 3, 3 > com_cross = (Eigen::Matrix < double, 3, 3 >() <<0., -com[2], com[1], com[2], 0., -com[0], -com[1], com[0], 0.).finished();
			Eigen::Matrix < double, 3, 3 > parallel_axis = mass * com_cross * com_cross.transpose ();

			Eigen::Matrix < double, 3, 3 > pa (parallel_axis);
			Eigen::Matrix < double, 3, 3 > mcc = mass * com_cross;
			Eigen::Matrix < double, 3, 3 > mccT = mcc.transpose ();

//			out.set (inertia_C (0, 0) + pa (0, 0),
//					inertia_C (0, 1) + pa (0, 1),
//					inertia_C (0, 2) + pa (0, 2),
//					mcc (0, 0), mcc (0, 1), mcc (0,
//									2),
//					inertia_C (1, 0) + pa (1, 0),
//					inertia_C (1, 1) + pa (1, 1),
//					inertia_C (1, 2) + pa (1, 2),
//					mcc (1, 0), mcc (1, 1), mcc (1,
//									2),
//					inertia_C (2, 0) + pa (2, 0),
//					inertia_C (2, 1) + pa (2, 1), inertia_C (2, 2) + pa (2, 2), mcc (2, 0), mcc (2, 1), mcc (2, 2), mccT (0, 0), mccT (0, 1), mccT (0, 2), mass, 0., 0., mccT (1, 0), mccT (1, 1), mccT (1, 2), 0., mass, 0., mccT (2, 0), mccT (2, 1), mccT (2, 2), 0., 0., mass);
//
//			return out;

			return (Eigen::Matrix < double, 6, 6 > () <<inertia_C (0, 0) + pa (0, 0),
					inertia_C (0, 1) + pa (0, 1),
					inertia_C (0, 2) + pa (0, 2),
					mcc (0, 0), mcc (0, 1), mcc (0,2),
					inertia_C (1, 0) + pa (1, 0),
					inertia_C (1, 1) + pa (1, 1),
					inertia_C (1, 2) + pa (1, 2),
					mcc (1, 0), mcc (1, 1), mcc (1,2),
					inertia_C (2, 0) + pa (2, 0),
					inertia_C (2, 1) + pa (2, 1), inertia_C (2, 2) + pa (2, 2), mcc (2, 0), mcc (2, 1), mcc (2, 2), mccT (0, 0), mccT (0, 1), mccT (0, 2), mass, 0., 0., mccT (1, 0), mccT (1, 1), mccT (1, 2), 0., mass, 0., mccT (2, 0), mccT (2, 1), mccT (2, 2), 0., 0., mass).finished();
		}

	/** \brief Constructs a body out of the given parameters
	 *
	 * This constructor eases the construction of a new body as all the
	 * required parameters can simply be specified as parameters to the
	 * constructor. These are then used to generate the spatial inertia
	 * matrix which is expressed at the origin.
	 *
	 * \param mass the mass of the body
	 * \param com  the position of the center of mass in the bodies coordinates
	 * \param length the length of the segment (needed to compute the inertia at the CoM
	 * \param gyration_radius the radii of gyration at the center of mass of the body in percentage of the segment length
	 * \param geom pointer to the MogsGeometry. Can be set to NULL
	 */
		MogsRobotBody (	const mogs_string &  name,
			const double &mass,
			const Eigen::Matrix < double, 3, 1 > &com,
			double length,
			const Eigen::Matrix < double, 3, 1 > &gyration_radius,
			MogsGeometry * geom = NULL):
			name_ (name),
			mMass (mass),
			mCenterOfMass (com)
		{
			length_ = length;
			mGeom = geom;

			Eigen::Matrix < double, 3, 3 > com_cross = (Eigen::Matrix < double, 3, 3 >() << 0., -com[2], com[1], com[2], 0., -com[0], -com[1], com[0], 0.).finished();
			Eigen::Matrix < double, 3, 3 > parallel_axis;
			parallel_axis = mass * com_cross * com_cross.transpose ();

			Eigen::Matrix < double, 3, 1 > gr = mass * Eigen::Matrix < double, 3, 1 > (gyration_radius[0] * gyration_radius[0] * length * length, gyration_radius[1] * gyration_radius[1] * length * length, gyration_radius[2] * gyration_radius[2] * length * length);
			Eigen::Matrix < double, 3, 3 > pa (parallel_axis);
			Eigen::Matrix < double, 3, 3 > mcc = mass * com_cross;
			Eigen::Matrix < double, 3, 3 > mccT = mcc.transpose ();

//			mInertia.set (gr[0], 0., 0., 0., gr[1], 0., 0., 0., gr[2]);
			mInertia = (Eigen::Matrix<double,3,3>() <<gr[0], 0., 0., 0., gr[1], 0., 0., 0., gr[2]).finished();

//			mSpatialInertia.set (gr[0] + pa (0, 0), pa (0, 1),
//					     pa (0, 2), mcc (0, 0), mcc (0,1),
//					     mcc (0, 2), pa (1, 0),
//					     gr[1] + pa (1, 1), pa (1, 2),
//					     mcc (1, 0), mcc (1, 1), mcc (1,2),
//					     pa (2, 0), pa (2, 1), gr[2] + pa (2, 2), mcc (2, 0), mcc (2, 1), mcc (2, 2), mccT (0, 0), mccT (0, 1), mccT (0, 2), mass, 0., 0., mccT (1, 0), mccT (1, 1), mccT (1, 2), 0., mass, 0., mccT (2, 0), mccT (2, 1), mccT (2, 2), 0., 0., mass);
			mSpatialInertia = (Eigen::Matrix<double,6,6>() <<gr[0] + pa (0, 0), pa (0, 1),
					     pa (0, 2), mcc (0, 0), mcc (0,1),
					     mcc (0, 2), pa (1, 0),
					     gr[1] + pa (1, 1), pa (1, 2),
					     mcc (1, 0), mcc (1, 1), mcc (1,2),
					     pa (2, 0), pa (2, 1), gr[2] + pa (2, 2), mcc (2, 0), mcc (2, 1), mcc (2, 2), mccT (0, 0), mccT (0, 1), mccT (0, 2), mass, 0., 0., mccT (1, 0), mccT (1, 1), mccT (1, 2), 0., mass, 0., mccT (2, 0), mccT (2, 1), mccT (2, 2), 0., 0., mass).finished();

		}

	/** \brief Joins inertial parameters of two bodies to create a composite
	 * body.
	 *
	 * This function can be used to joint inertial parameters of two bodies
	 * to create a composite body that has the inertial properties as if the
	 * two bodies were joined by a fixed joint.
	 *
	 * \note Both bodies have to have their inertial parameters expressed in
	 * the same orientation.
	 *
	 * \param transform The frame transformation from the origin of the
	 * original body to the origin of the added body
	 */
		void Join (const SpatialTransform < double >&transform, const MogsRobotBody & other_body)
		{
			// nothing to do if we join a massles body to the current.
			if (other_body.mGeom && other_body.mMass == 0. && other_body.mInertia == Eigen::Matrix < double, 3, 3 >::Zero ())
			  {
				  std::cout<<"Considering no mass body, so we ignore it (be carefull, there is no visualziation for this object"<<std::endl;
				  return;
			  }

			double other_mass = other_body.mMass;
			double new_mass = mMass + other_mass;
            Eigen::Matrix < double, 3, 1 > other_com, new_com;
			if (new_mass != 0.)
          {
                other_com = transform.E.transpose () * other_body.mCenterOfMass + transform.r;
                new_com = (1 / new_mass) * (mMass * mCenterOfMass + other_mass * other_com);
          }

			// We have to transform the inertia of other_body to the new COM. This
			// is done in 4 steps:
			//
			// 1. Transform the inertia from other origin to other COM
			// 2. Rotate the inertia that it is aligned to the frame of this body
			// 3. Transform inertia of other_body to the origin of the frame of
			// this body
			// 4. Sum the two inertias
			// 5. Transform the summed inertia to the new COM

			Eigen::Matrix < double, 3, 3 > inertia_other = other_body.mSpatialInertia.block < 3, 3 > (0, 0);

			// 1. Transform the inertia from other origin to other COM
			Eigen::Matrix < double, 3, 3 > other_com_cross = VectorCrossMatrix (other_body.mCenterOfMass);
			Eigen::Matrix < double, 3, 3 > inertia_other_com = inertia_other - other_mass * other_com_cross * other_com_cross.transpose ();

			// 2. Rotate the inertia that it is aligned to the frame of this body
			Eigen::Matrix < double, 3, 3 > inertia_other_com_rotated = transform.E.transpose () * inertia_other_com * transform.E;

			// 3. Transform inertia of other_body to the origin of the frame of this body
			Eigen::Matrix < double, 3, 3 > inertia_other_com_rotated_this_origin = parallel_axis (inertia_other_com_rotated,
													      other_mass, other_com);

			// 4. Sum the two inertias
			Eigen::Matrix < double, 3, 3 > inertia_summed = Eigen::Matrix < double, 3, 3 > (mSpatialInertia.block < 3, 3 > (0, 0)) + inertia_other_com_rotated_this_origin;

			// 5. Transform the summed inertia to the new COM
			Eigen::Matrix < double, 3, 3 > new_inertia = inertia_summed - new_mass * VectorCrossMatrix (new_com) * VectorCrossMatrix (new_com).transpose ();

			// do not change the name
			mMass = new_mass;
			mCenterOfMass = new_com,
			mInertia = new_inertia;
			// FIXME do not change length
			mSpatialInertia = compute_SpatialInertia(mMass, mCenterOfMass, mInertia);
			if (!mGeom)
            {
                mGeom = new MogsGeometry();
//                qDebug()<<"new geom";
            }
            if(other_body.mGeom)
            {
                mGeom->fusion( transform,other_body.mGeom);
            }

		}

		~MogsRobotBody ()
		{
		};

		void add_geom( MogsGeometry* in)
		{
			if (mGeom)
				mGeom->fusion(in);
			else
				set_geom(in);
		}

		double getLenght () const
		{
			return length_;
		}

		void set_geom( MogsGeometry * in)
		{
			mGeom = in;
		}

		/// \brief The name of the body
		QString name_;
		/// \brief The mass of the body
		double mMass;
		/// \brief The position of the center of mass in body coordinates
		Eigen::Matrix < double, 3, 1 > mCenterOfMass;
		/// \brief Inertia matrix at the center of mass
		Eigen::Matrix < double, 3, 3 > mInertia;
		/// \brief The spatial inertia that contains both mass and inertia information
		Eigen::Matrix < double, 6, 6 > mSpatialInertia;
		/// \brief Reference to the geometry information
		MogsGeometry *mGeom;
		/// \brief length of the body (used for scalable bodies)
		double length_;

		/// \brief pointer of the class used to compute collision and distances
		MogsAbstractCollisionDefinition * collision_definition_;

                
                friend std::ostream & operator<< (std::ostream & output,
                                                    const MogsRobotBody & in)
                {
                        output << "name = " << in.name_.toStdString() << std::endl;
                        output << "Mass = " << in.mMass << std::endl;
                        output << "mCenterOfMass = " << in.mCenterOfMass.transpose() << std::endl;
                        output << "mInertia = " << in.mInertia << std::endl;
                        output << "mSpatialInertia = " << in.mSpatialInertia << std::endl;
                        output << "mGeom = " << in.mGeom << std::endl;
                        return output;
                }                
                
	      private:

		friend class boost::serialization::access;

		template < class Archive > void serialize (Archive & ar, const unsigned int version)
		{

			ar & name_;
			ar & mMass;
			ar & mCenterOfMass;
			ar & mInertia;
			ar & mSpatialInertia;
			ar & length_;
			ar & mGeom;
			// ar & collision_definition_; // not serialized since not usefull in the visualization ?

		}




	};

/** \brief Keeps the information of a body and how it is attached to another body.
 *
 * When using fixed bodies, i.e. a body that is attached to anothe via a
 * fixed joint, the attached body is merged onto its parent. By doing so
 * adding fixed joints do not have an impact on runtime.
 */
	struct MogsRobotFixedBody
	{
		/// \brief The mass of the body
		double mMass;
		/// \brief The position of the center of mass in body coordinates
		  Eigen::Matrix < double, 3, 1 > mCenterOfMass;
		/// \brief The spatial inertia that contains both mass and inertia information
		  Eigen::Matrix < double, 6, 6 > mSpatialInertia;

		/// \brief Id of the movable body that this fixed body is attached to.
		unsigned int mMovableParent;
		/// \brief Transforms spatial quantities expressed for the parent to the
		// fixed body.
		  SpatialTransform < double >mParentTransform;
		  SpatialTransform < double >mBaseTransform;

		static MogsRobotFixedBody CreateFromBody (const MogsRobotBody & body)
		{
			MogsRobotFixedBody fbody;

			  fbody.mMass = body.mMass;
			  fbody.mCenterOfMass = body.mCenterOfMass;
			  fbody.mSpatialInertia = body.mSpatialInertia;

			  return fbody;
		}

		template < typename Archive > void serialize (Archive & ar, const unsigned int version)
		{

			ar & mMass;
			ar & mCenterOfMass;
			ar & mSpatialInertia;
			ar & mMovableParent;
			ar & mParentTransform;
			ar & mBaseTransform;
		}

	};

}

#endif /* __MOGSROBOTBODY_H__ */
