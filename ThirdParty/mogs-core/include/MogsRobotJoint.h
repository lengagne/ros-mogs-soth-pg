/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 * Code Modified by S. Lengagne during his work in the following labs:
 *      2011-2012:  Karlsruhe Institute fur Technologie, Karlsruhe, Germany
 *      2013-2012: IUT de Beziers/ LIRMM, Beziers, France
 * 	from 2013 : Université Blaise Pascal / axis : ISPR / theme MACCS
 */

#ifndef _JOINT_H
#define _JOINT_H

#include <assert.h>
#include <iostream>

#include "MogsRBDLMathUtils.h"
#include "MogsSpatialAlgebraOperators.h"
#include "MogsSerialization.h"

namespace RigidBodyDynamics
{

/** \brief General types of joints
 */
	enum JointType
	{
		JointTypeUndefined = 0,
		JointTypeRevolute,
		JointTypePrismatic,
		JointTypeFixed,

		JointType1DoF,
		JointType2DoF,
		JointType3DoF,
		JointType4DoF,
		JointType5DoF,
		JointType6DoF
	};

/** \brief Store additionnal data
 */
	struct additionnal_data
	{

		/// \breif default constructor ( necessary for bools serialization )

		additionnal_data ()
		{
		}

		/// \brief name of the link
		mogs_string name;

		/// \brief name of the dad body
		  mogs_string dad_body_name;
		/// \brief name of the child body
		  mogs_string child_body_name;

		/// \brief Static parameter
		  Eigen::Matrix < double, 6, 1 > sparam;

 		/// \brief Joint position limits
		std::vector<double> qmin;
		std::vector<double> qmax;

		/// \brief Joint velocity limit
		std::vector<double> dqmax;

		/// \brief Joint torque limit
		std::vector<double> torquemax;
                
                bool is_mimic;
                mogs_string name_mimic;
                double multiplier_mimic;
                double offset_mimic;


		  template < typename Archive >
			void serialize (Archive & ar,
					const unsigned int version)
		{
			ar & name;
			ar & dad_body_name;
			ar & child_body_name;
			ar & sparam;
			ar & qmin;
			ar & qmax;
			ar & dqmax;
			ar & torquemax;
		}
	};

/** \brief Describes a joint relative to the predecessor body.
 *
 * This class contains all information required for one single joint. This
 * contains the joint type and the
 * axis of the joint.
 */

	class MogsRobotJoint
	{
	      public:
		MogsRobotJoint ():mJointType (JointTypeUndefined), mDoFCount (0)
		{
			ratio_ = 1.0;
		};

	      MogsRobotJoint (JointType type, double r=1.0):
		mJointType (type),ratio_(r)
		{
			if (type != JointTypeFixed)
			  {
				  std::cerr <<
					  "Error: Invalid use of MogsRobotJoint constructor MogsRobotJoint(JointType type). Only allowed when type == JointTypeFixed."
					  << std::endl;
				  assert (0);
				  abort ();
			  }
			mDoFCount = 0;
		};

		MogsRobotJoint (const MogsRobotJoint & joint):
			mJointType (joint.mJointType),
			mDoFCount (joint.mDoFCount),
			ratio_(joint.ratio_)
		{

			mJointAxes.resize (mDoFCount);
			for (unsigned int i = 0; i < mDoFCount; i++)
				mJointAxes[i] = joint.mJointAxes[i];
			set_additionnal_data (joint.data_);
		};

		~MogsRobotJoint ()
		{
			mDoFCount = 0;
		}

	/** \brief Constructs a joint from the given cartesian parameters.
	 *
	 * This constructor creates all the required spatial values for the given
	 * cartesian parameters.
	 *
	 * \param joint_type whether the joint is revolute or prismatic
	 * \param joint_axis the axis of rotation or translation
	 */
		MogsRobotJoint (const JointType joint_type,
				const Eigen::Matrix < double, 3, 1 > &joint_axis,
			       double r=1.0)
		:ratio_(r)
		{
			mDoFCount = 1;
			mJointAxes.resize (mDoFCount);
			// Some assertions, as we concentrate on simple cases

			// Only rotation around the Z-axis
			assert (joint_type == JointTypeRevolute
				|| joint_type == JointTypePrismatic);

			mJointType = joint_type;

			if (joint_type == JointTypeRevolute)
			  {
				  // make sure we have a unit axis
//				  mJointAxes[0].set (joint_axis[0],
//						     joint_axis[1],
//						     joint_axis[2],
//						     0., 0., 0.);

				  mJointAxes[0] = (Eigen::Matrix<double,6,1>()<< joint_axis[0],joint_axis[1],joint_axis[2],0., 0., 0.).finished();


			  }
			else if (joint_type == JointTypePrismatic)
			  {
				  // make sure we have a unit axis
				  assert (joint_axis.squaredNorm () == 1.);

//				  mJointAxes[0].set (0., 0., 0.,
//						     joint_axis[0],
//						     joint_axis[1],
//						     joint_axis[2]);
                    mJointAxes[0] = (Eigen::Matrix<double,6,1>()<< 0., 0., 0.,joint_axis[0],joint_axis[1],joint_axis[2]).finished();
			  }
		}

	/** \brief Constructs a 1 DoF joint with the given motion subspaces.
	 *
	 * The motion subspaces are of the format:
	 * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
	 *
	 * \note So far only pure rotations or pure translations are supported.
	 *
	 * \param axis_0 Motion subspace for axis 0
	 */
		MogsRobotJoint (const Eigen::Matrix < double, 6, 1 > &axis_0,
				double r=1.0)
		:ratio_(r)
		{
			ratio_ = 1.0;
			mJointType = JointType1DoF;
			mDoFCount = 1;
			mJointAxes.resize (mDoFCount);
			mJointAxes[0] = axis_0;

			validate_spatial_axis (mJointAxes[0]);
		}

	/** \brief Constructs a 2 DoF joint with the given motion subspaces.
	 *
	 * The motion subspaces are of the format:
	 * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
	 *
	 * \note So far only pure rotations or pure translations are supported.
	 *
	 * \param axis_0 Motion subspace for axis 0
	 * \param axis_1 Motion subspace for axis 1
	 */
		MogsRobotJoint (const Eigen::Matrix < double, 6, 1 > &axis_0,
				const Eigen::Matrix < double, 6, 1 > &axis_1,
				double r=1.0)
		:ratio_(r)
		{
			ratio_ = 1.0;
			mJointType = JointType2DoF;
			mDoFCount = 2;

			mJointAxes.resize (mDoFCount);
			mJointAxes[0] = axis_0;
			mJointAxes[1] = axis_1;

			validate_spatial_axis (mJointAxes[0]);
			validate_spatial_axis (mJointAxes[1]);
		}

	/** \brief Constructs a 3 DoF joint with the given motion subspaces.
	 *
	 * The motion subspaces are of the format:
	 * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
	 *
	 * \note So far only pure rotations or pure translations are supported.
	 *
	 * \param axis_0 Motion subspace for axis 0
	 * \param axis_1 Motion subspace for axis 1
	 * \param axis_2 Motion subspace for axis 2
	 */
		MogsRobotJoint (const Eigen::Matrix < double, 6, 1 > &axis_0,
				const Eigen::Matrix < double, 6, 1 > &axis_1,
				const Eigen::Matrix < double, 6, 1 > &axis_2,
				double r=1.0)
			:ratio_(r)
		{
			ratio_ = 1.0;
			mJointType = JointType3DoF;
			mDoFCount = 3;

			mJointAxes.resize (mDoFCount);

			mJointAxes[0] = axis_0;
			mJointAxes[1] = axis_1;
			mJointAxes[2] = axis_2;

			validate_spatial_axis (mJointAxes[0]);
			validate_spatial_axis (mJointAxes[1]);
			validate_spatial_axis (mJointAxes[2]);
		}

	/** \brief Constructs a 4 DoF joint with the given motion subspaces.
	 *
	 * The motion subspaces are of the format:
	 * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
	 *
	 * \note So far only pure rotations or pure translations are supported.
	 *
	 * \param axis_0 Motion subspace for axis 0
	 * \param axis_1 Motion subspace for axis 1
	 * \param axis_2 Motion subspace for axis 2
	 * \param axis_3 Motion subspace for axis 3
	 */
		MogsRobotJoint (const Eigen::Matrix < double, 6, 1 > &axis_0,
				const Eigen::Matrix < double, 6, 1 > &axis_1,
				const Eigen::Matrix < double, 6, 1 > &axis_2,
				const Eigen::Matrix < double, 6, 1 > &axis_3,
				double r=1.0)
		:ratio_(r)
		{
			ratio_ = 1.0;
			mJointType = JointType4DoF;
			mDoFCount = 4;

			mJointAxes.resize (mDoFCount);

			mJointAxes[0] = axis_0;
			mJointAxes[1] = axis_1;
			mJointAxes[2] = axis_2;
			mJointAxes[3] = axis_3;

			validate_spatial_axis (mJointAxes[0]);
			validate_spatial_axis (mJointAxes[1]);
			validate_spatial_axis (mJointAxes[2]);
			validate_spatial_axis (mJointAxes[3]);
		}

	/** \brief Constructs a 5 DoF joint with the given motion subspaces.
	 *
	 * The motion subspaces are of the format:
	 * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
	 *
	 * \note So far only pure rotations or pure translations are supported.
	 *
	 * \param axis_0 Motion subspace for axis 0
	 * \param axis_1 Motion subspace for axis 1
	 * \param axis_2 Motion subspace for axis 2
	 * \param axis_3 Motion subspace for axis 3
	 * \param axis_4 Motion subspace for axis 4
	 */
		MogsRobotJoint (const Eigen::Matrix < double, 6, 1 > &axis_0,
				const Eigen::Matrix < double, 6, 1 > &axis_1,
				const Eigen::Matrix < double, 6, 1 > &axis_2,
				const Eigen::Matrix < double, 6, 1 > &axis_3,
				const Eigen::Matrix < double, 6, 1 > &axis_4,
				double r=1.0)
		:ratio_(r)
		{
			ratio_ = 1.0;
			mJointType = JointType5DoF;
			mDoFCount = 5;

			mJointAxes.resize (mDoFCount);
			mJointAxes[0] = axis_0;
			mJointAxes[1] = axis_1;
			mJointAxes[2] = axis_2;
			mJointAxes[3] = axis_3;
			mJointAxes[4] = axis_4;

			validate_spatial_axis (mJointAxes[0]);
			validate_spatial_axis (mJointAxes[1]);
			validate_spatial_axis (mJointAxes[2]);
			validate_spatial_axis (mJointAxes[3]);
			validate_spatial_axis (mJointAxes[4]);
		}

	/** \brief Constructs a 6 DoF joint with the given motion subspaces.
	 *
	 * The motion subspaces are of the format:
	 * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
	 *
	 * \note So far only pure rotations or pure translations are supported.
	 *
	 * \param axis_0 Motion subspace for axis 0
	 * \param axis_1 Motion subspace for axis 1
	 * \param axis_2 Motion subspace for axis 2
	 * \param axis_3 Motion subspace for axis 3
	 * \param axis_4 Motion subspace for axis 4
	 * \param axis_5 Motion subspace for axis 5
	 */
		MogsRobotJoint (const Eigen::Matrix < double, 6, 1 > &axis_0,
				const Eigen::Matrix < double, 6, 1 > &axis_1,
				const Eigen::Matrix < double, 6, 1 > &axis_2,
				const Eigen::Matrix < double, 6, 1 > &axis_3,
				const Eigen::Matrix < double, 6, 1 > &axis_4,
				const Eigen::Matrix < double, 6, 1 > &axis_5,
				double r=1.0)
		:ratio_(r)
		{
			ratio_ = 1.0;
			mJointType = JointType6DoF;
			mDoFCount = 6;

			mJointAxes.resize (mDoFCount);
			mJointAxes[0] = axis_0;
			mJointAxes[1] = axis_1;
			mJointAxes[2] = axis_2;
			mJointAxes[3] = axis_3;
			mJointAxes[4] = axis_4;
			mJointAxes[5] = axis_5;

			validate_spatial_axis (mJointAxes[0]);
			validate_spatial_axis (mJointAxes[1]);
			validate_spatial_axis (mJointAxes[2]);
			validate_spatial_axis (mJointAxes[3]);
			validate_spatial_axis (mJointAxes[4]);
			validate_spatial_axis (mJointAxes[5]);
		}

		mogs_string GetName() const
		{
			return data_.name;
		}

	/** \brief Checks whether we have pure rotational or translational axis.
	 *
	 * This function is mainly used to print out warnings when specifying an
	 * axis that might not be intended.
	 */
		bool validate_spatial_axis (Eigen::Matrix < double, 6,
					    1 > &axis)
		{
			if (fabs (axis.norm () - 1.0) > 1.0e-8)
			  {
				  std::cerr <<
					  "Warning: joint axis is not unit!"
					  << std::endl;
			  }

			bool axis_rotational = false;
			bool axis_translational = false;

			Eigen::Matrix < double, 3, 1 > rotation (axis[0], axis[1], axis[2]);
			Eigen::Matrix < double, 3, 1 > translation (axis[3], axis[4], axis[5]);

			if (fabs (translation.norm ()) < 1.0e-8)
				axis_rotational = true;

			if (fabs (rotation.norm ()) < 1.0e-8)
				axis_translational = true;

			return axis_rotational || axis_translational;
		}

		/// \brief The spatial axis of the joint
		std::vector < Eigen::Matrix < double, 6, 1 >, Eigen::aligned_allocator < Eigen::Matrix < double, 6, 1 > > >mJointAxes;
		/// \brief Type of joint (rotational or prismatic)
		JointType mJointType;
		unsigned int mDoFCount;

		/// \brief param to include ratio or sign
		double ratio_;

		void set_additionnal_data (const additionnal_data & in);

//      dummy to solve cause 1 of http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW additionnal_data data_;

	      private:

		friend class boost::serialization::access;

		template < class Archive >
			void serialize (Archive & ar,
					const unsigned int version)
		{

			ar & mJointAxes;
			ar & mJointType;
			ar & mDoFCount;
			ar & data_;
			ar & ratio_;

		}

	};

}				/*RigidBodyDynamics */

#endif /* _JOINT_H */
