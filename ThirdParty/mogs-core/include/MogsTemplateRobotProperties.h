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

#ifndef _MOGSTEMPLATEROBOTPROPERTIES_H
#define _MOGSTEMPLATEROBOTPROPERTIES_H

#include "MogsRobotProperties.h"

namespace RigidBodyDynamics
{
	template < typename T >  class MogsTemplateRobotProperties : public MogsRobotProperties
	{
	      public:

		MogsTemplateRobotProperties ();
		
		MogsTemplateRobotProperties (MogsTemplateRobotProperties* MogsRobotProperties_in);
		
		MogsTemplateRobotProperties (MogsRobotProperties* MogsRobotProperties_in);
		

		~MogsTemplateRobotProperties ()
		{

		};
		
		
		SpatialTransform < T > get_root_transformation();
		
		// State information
		/// \brief The spatial velocity of the bodies
		std::vector < Eigen::Matrix < T, 6, 1 >, Eigen::aligned_allocator < Eigen::Matrix < T, 6, 1 > > >v;

		/// \brief The spatial acceleration of the bodies
		std::vector < Eigen::Matrix < T, 6, 1 >, Eigen::aligned_allocator < Eigen::Matrix < T, 6, 1 > > >a;

		////////////////////////////////////
		// Dynamics variables

		/// \brief The velocity dependent spatial acceleration
		std::vector < Eigen::Matrix < T, 6, 1 >, Eigen::aligned_allocator < Eigen::Matrix < T, 6, 1 > > >c;
		/// \brief The spatial inertia of the bodies 
		std::vector < Eigen::Matrix < T, 6, 6 >, Eigen::aligned_allocator < Eigen::Matrix < T, 6, 6 > > >IA;

		/// \brief The spatial bias force
		std::vector < Eigen::Matrix < T, 6, 1 >, Eigen::aligned_allocator < Eigen::Matrix < T, 6, 1 > > >pA;
			
		/// \brief Temporary variable U_i (RBDA p. 130)
		std::vector < Eigen::Matrix < T, 6, 1 >, Eigen::aligned_allocator < Eigen::Matrix < T, 6, 1 > > >U;
		/// \brief Temporary variable D_i (RBDA p. 130)
		Eigen::Matrix < T, Eigen::Dynamic, 1 > d;
		/// \brief Temporary variable u (RBDA p. 130)
		Eigen::Matrix < T, Eigen::Dynamic, 1 > u;
		/// \brief Internal forces on the body (used only InverseDynamics())
		std::vector < Eigen::Matrix < T, 6, 1 >, Eigen::aligned_allocator < Eigen::Matrix < T, 6, 1 > > >f;
		/// \brief The spatial inertia of body i (used only in CompositeRigidBodyAlgorithm())
		std::vector < SpatialRigidBodyInertia, Eigen::aligned_allocator < SpatialRigidBodyInertia > >Ic;
                
                /// \bried Joint position value
                Eigen::Matrix < T,Eigen::Dynamic, 1 > q_;
                /// \bried Joint velocity value
                Eigen::Matrix < T,Eigen::Dynamic, 1 > dq_;
                /// \bried Joint acceleration value
                Eigen::Matrix < T,Eigen::Dynamic, 1 > ddq_;
                /// \bried Joint torque value
                Eigen::Matrix < T,Eigen::Dynamic, 1 > tau_;

		////////////////////////////////////
		// Bodies
		/// \brief Transformation from the parent body to the current body
		std::vector < SpatialTransform < T >, Eigen::aligned_allocator < SpatialTransform < T > > >X_lambda;
		
		std::vector < SpatialTransform < T >, Eigen::aligned_allocator < SpatialTransform < T > > >X_base;

		unsigned int AddBody (const unsigned int parent_id,
				      const SpatialTransform < double >&joint_frame,
				      const MogsRobotJoint & joint, const MogsRobotBody & body,
				      const mogs_string & body_name = "");

		/// \brief Initializes the helper values for the dynamics algorithm
		void Init ();
		
		void set_root_transformation(	const Eigen::Matrix < double, 3, 1 > position, 
						const Eigen::Matrix < double, 3, 1 > rotation);
				
		bool SetRobotXml (const mogs_string & name, bool force_fixed = false);
                
                void SetJointValue( const Eigen::Matrix < T,Eigen::Dynamic, 1 > &Q,
                                    const Eigen::Matrix < T,Eigen::Dynamic, 1 > &QDot,
                                    const Eigen::Matrix < T,Eigen::Dynamic, 1 > &QDDot);
                
                void SetCustomJointValue(   const Eigen::Matrix < T,Eigen::Dynamic, 1 > *Q,
                                            const Eigen::Matrix < T,Eigen::Dynamic, 1 > *QDot=NULL,
                                            const Eigen::Matrix < T,Eigen::Dynamic, 1 > *QDDot=NULL);     
                
                void SetForwardJointValue(  const Eigen::Matrix < T,Eigen::Dynamic, 1 > &Q,
                                            const Eigen::Matrix < T,Eigen::Dynamic, 1 > &QDot,
                                            const Eigen::Matrix < T,Eigen::Dynamic, 1 > &Tau);
                
                void GetTorque( Eigen::Matrix < T,Eigen::Dynamic, 1 > &Tau);
                
                void GetDDQ( Eigen::Matrix < T,Eigen::Dynamic, 1 > &ddq);
                
	      private:

		friend class boost::serialization::access;

		template < class Archive >
			void serialize (Archive & ar, const unsigned int version)
		{
			MogsRobotProperties::serialize(ar,version);
			ar & v;
			ar & a;
			ar & c;
			ar & IA;
			ar & pA;
			ar & U;
			ar & d;
			ar & u;
			ar & f;
			ar & Ic;			
			ar & X_base;
			ar & X_lambda;
		}

	};

/** @} */
}

#include "MogsTemplateRobotProperties.hxx"

#endif /* _MODEL_H */
