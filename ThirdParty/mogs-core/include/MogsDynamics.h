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

#ifndef _DYNAMICS_H
#define _DYNAMICS_H

#include <assert.h>
#include <iostream>

#include "MogsKinematics.h"
#include "serialization.h"

namespace RigidBodyDynamics
{

	template < typename T > class MogsDynamics: public MogsKinematics < T >
	{
	      public:
		MogsDynamics ()
		{

		};

		MogsDynamics (MogsRobotProperties* Robot_in)
		{
			MogsKinematics < T >::SetRobot(Robot_in);
		};

		~MogsDynamics ()
		{
		};

/** \defgroup dynamics_group MogsDynamics
 * @{
 */

/** \brief Computes forward dynamics with the Articulated Body Algorithm
 *
 * This function computes the generalized accelerations from given
 * generalized states, velocities and forces:
 *   \f$ \ddot{q} = M(q)^{-1} ( -N(q, dot{q}) + \tau)\f$
 * It does this by using the recursive Articulated Body Algorithm that runs
 * in \f$O(n_{dof})\f$ with \f$n_{dof}\f$ being the number of joints.
 *
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param QDDot accelerations of the internal joints (output)
 * \param f_ext External forces acting on the body in base coordinates (optional, defaults to NULL)
 */
		void ForwardDynamics (	const Eigen::Matrix < T, Eigen::Dynamic, 1 > &Q,
					const Eigen::Matrix < T, Eigen::Dynamic, 1 > &QDot,
					const Eigen::Matrix < T, Eigen::Dynamic, 1 > &Tau,
					Eigen::Matrix < T, Eigen::Dynamic, 1 > &QDDot,
					std::vector < Eigen::Matrix < T, 6, 1 >, Eigen::aligned_allocator < Eigen::Matrix < T, 6, 1 > > > *f_ext = NULL);

/** \brief Computes forward dynamics by building and solving the full Lagrangian equation
 *
 * This method builds and solves the linear system
 * \f[ 	H \ddot{q} = -C + \tau	\f]
 * for \f$\ddot{q}\f$ where \f$H\f$ is the joint space inertia matrix
 * computed with the CompositeRigidBodyAlgorithm(), \f$C\f$ the bias
 * force (sometimes called "non-linear effects").
 *
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param QDDot accelerations of the internal joints (output)
 * \param linear_solver specification which method should be used for solving the linear system
 * \param f_ext External forces acting on the body in base coordinates (optional, defaults to NULL)
 */
		void ForwardDynamicsLagrangian (const Eigen::Matrix < T, Eigen::Dynamic, 1 > &Q,
						const Eigen::Matrix < T, Eigen::Dynamic, 1 > &QDot,
						const Eigen::Matrix < T, Eigen::Dynamic, 1 > &Tau,
						Eigen::Matrix < T, Eigen::Dynamic, 1 > &QDDot,
						LinearSolver linear_solver = LinearSolverColPivHouseholderQR,
						std::vector < Eigen::Matrix < T, 6, 1 > >*f_ext = NULL);

/** \brief Computes inverse dynamics with the Newton-Euler Algorithm
 *
 * This function computes the generalized forces from given generalized
 * states, velocities, and accelerations:
 *   \f$ \tau = M(q) \ddot{q} + N(q, dot{q}) \f$
 *
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param QDDot accelerations of the internals joints
 * \param Tau   actuations of the internal joints (output)
 * \param f_ext External forces acting on the body in base coordinates (optional, defaults to NULL)
 */
		void InverseDynamics (	const Eigen::Matrix < T, Eigen::Dynamic, 1 > &Q,
					const Eigen::Matrix < T, Eigen::Dynamic, 1 > &QDot,
					const Eigen::Matrix < T, Eigen::Dynamic, 1 > &QDDot,
					Eigen::Matrix < T, Eigen::Dynamic, 1 > &Tau,
					std::vector < Eigen::Matrix < T, 6, 1 >, Eigen::aligned_allocator < Eigen::Matrix < T, 6, 1 > > >  *f_ext = NULL);

		void InverseDynamicsStatic (	const Eigen::Matrix < T, Eigen::Dynamic, 1 > &Q,
                                        Eigen::Matrix < T, Eigen::Dynamic, 1 > &Tau,
                                        std::vector < Eigen::Matrix < T, 6, 1 >, Eigen::aligned_allocator < Eigen::Matrix < T, 6, 1 > > >  *f_ext = NULL);

/** \brief Computes the joint space inertia matrix by using the Composite Rigid Body Algorithm
 *
 * This function computes the joint space inertia matrix from a given model and
 * the generalized state vector:
 *   \f$ M(q) \f$
 *
 * \param Q     state vector of the model
 * \param H     a matrix where the result will be stored in
 * \param update_kinematics  whether the kinematics should be updated
 * (safer, but at a higher computational cost!)
 */
		void CompositeRigidBodyAlgorithm (const Eigen::Matrix < T,Eigen::Dynamic, 1 > &Q,
						  Eigen::Matrix < T,Eigen::Dynamic, Eigen::Dynamic > & H,
						  bool update_kinematics = true);

	      private:

		friend class boost::serialization::access;

		template < class Archive >
			void serialize (Archive & ar,
					const unsigned int version)
		{

			ar & boost::serialization::base_object < MogsKinematics < T > >(*this);


		}

	};
/** @} */

}

#include "MogsDynamics.hxx"

#endif /* _DYNAMICS_H */
