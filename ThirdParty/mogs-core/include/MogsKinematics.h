/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 * Code Modified by S. Lengagne during his work in the following labs:
 *      2011-2012:  Karlsruhe Institute fur Technologie, Karlsruhe, Germany
 *      2013-2012: IUT de Beziers/ LIRMM, Beziers, France
 */

#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include <assert.h>
#include <iostream>

#include "MogsTemplateRobotProperties.h"
#include "serialization.h"

namespace RigidBodyDynamics
{

	template < typename T > class MogsKinematics
	{

	      public:


		MogsKinematics ()
		{
		};

		MogsKinematics (MogsRobotProperties* Robot_in)
		{
			SetRobot(Robot_in);
		};

		~MogsKinematics ()
		{
		};

		inline QString GetBodyName(unsigned int id)const
		{
            return model->GetBodyName(id);
		}

		inline unsigned int getNDof () const
		{
			return model->getNDof ();
		}

		inline unsigned int getNBodies () const
		{
			return model->getNBodies ();
		}

		inline mogs_string getRobotName() const
		{
			return model->getRobotName();
		}

		inline void setRobotName(mogs_string name)
		{
			return model->setRobotName(name);
		}

		bool SetRobotFile (const mogs_string xml, bool forced_fixed = false);

		inline bool SetRobotFile(const std::string filename, bool forced_fixed=false)
		{
		  std::cout<<"SetRobotFile string"<<std::endl;
		    return SetRobotFile(QString::fromStdString(filename), forced_fixed);
		}

		virtual void SetRobot(MogsTemplateRobotProperties < double >* Robot_in);

		virtual void SetRobot(MogsRobotProperties* Robot_in);

		void SetRobotSphere(double radius, double mass=0);

		mogs_string get_robot_type() const
		{
			return model->get_robot_type();
		}

/** \defgroup kinematics_group MogsKinematics
 * @{
 *
 * \note Please note that in the Rigid %Body Dynamics Library all angles
 * are specified in radians.
 *
 */

/** \brief Updates and computes velocities and accelerations of the bodies
 *
 * This function updates the kinematic variables such as body velocities
 * and accelerations in the model to reflect the variables passed to this function.
 *
 * \param Q     the positional variables of the model
 * \param QDot  the generalized velocities of the joints
 * \param QDDot the generalized accelerations of the joints
 */
		void UpdateKinematics (const Eigen::Matrix < T,Eigen::Dynamic, 1 > &Q,
				       const Eigen::Matrix < T,Eigen::Dynamic, 1 > &QDot,
				       const Eigen::Matrix < T,Eigen::Dynamic, 1 > &QDDot);

/** \brief Selectively updates model internal states of body positions, velocities and/or accelerations.
 *
 * This function updates the kinematic variables such as body velocities and
 * accelerations in the model to reflect the variables passed to this function.
 *
 * In contrast to UpdateKinematics() this function allows to update the model
 * state with values one is interested and thus reduce computations (e.g. only
 * positions, only positions + accelerations, only velocities, etc.).

 * \param Q     the positional variables of the model
 * \param QDot  the generalized velocities of the joints
 * \param QDDot the generalized accelerations of the joints
 */
		void UpdateKinematicsCustom (const Eigen::Matrix < T,Eigen::Dynamic, 1 > *Q,
					     const Eigen::Matrix < T,Eigen::Dynamic, 1 > *QDot = NULL,
					     const Eigen::Matrix < T,Eigen::Dynamic, 1 > *QDDot = NULL);

/** \brief Returns the world coordinates of a point given in body coordinates.
 * \param Q the curent genereralized positions
 * \param body_id id of the body for which the point coordinates are expressed
 * \param point_body_coordinates coordinates of the point in body coordinates
 * \param update_kinematics whether UpdateKinematics() should be called
 * or not (default: true)
 *
 * \returns a 3-D vector with coordinates of the point in base coordinates
 */
		Eigen::Matrix<T,3,1>CalcBodyToBaseCoordinates (const Eigen::Matrix<T,Eigen::Dynamic,1> &Q,
							       unsigned int body_id,
							       const Eigen::Matrix<T,3,1> &body_point_position,
							       bool update_kinematics=true);

		Eigen::Matrix<T,3,1> getBodyToBaseCoordinates(   unsigned int body_id,
                                                        const Eigen::Matrix<T,3,1> &body_point_position) const;

		inline Eigen::Matrix<T,3,1>getPosition(	unsigned int body_id,
                                                        const Eigen::Matrix<double,3,1> &body_point_position=Eigen::Matrix<double,3,1>::Zero()) const
		{
			return getBodyToBaseCoordinates(body_id, body_point_position.template cast<T> ());
		}

		Eigen::Matrix<T,3,1>getBodyToBaseCoordinates(unsigned int body_id) const;

		void getFrameCoordinate(unsigned int body_id,
					Eigen::Matrix<T,3,1> & rot_PRY,
					Eigen::Matrix<T,3,1> & pos) const;

		void getFrameOrientation(unsigned int body_id,
					Eigen::Matrix<T,3,1> & rot_PRY) const;

		void getFrameCoordinate(unsigned int body_id,
					Eigen::Matrix<T,4,4> & transform) const;

		void getFrameCoordinate(unsigned int body_id,
					SpatialTransform<T> & transform) const;
                                     
                /// return the position of a global point in the body_frame
                Eigen::Matrix<T,3,1> getBaseToBodyCoordinates(    unsigned int body_id,
                                                            const Eigen::Matrix<T,3,1> & world_point_position);
                
                Eigen::Matrix<T,3,1> Get_Local_Position(    unsigned int body_id,
                                                            const Eigen::Matrix<double,3,1> & world_point_position)
                {
                        return getBaseToBodyCoordinates(body_id, world_point_position.template cast<T> ());
                }
                
                /// return the position of a point in frame A in the body_frame
                Eigen::Matrix<T,3,1> getBaseToBodyCoordinates(    unsigned int source_body_id,
                                                            unsigned int target_body_id,
                                                            const Eigen::Matrix<T,3,1> & source_point_position);
                
                Eigen::Matrix<T,3,1> Get_Local_Position(    unsigned int source_body_id,
                                                            unsigned int target_body_id,
                                                            const Eigen::Matrix<double,3,1> & source_point_position)
                {
                        return getBaseToBodyCoordinates(source_body_id,target_body_id, source_point_position.template cast<T> ());
                }                

		inline SpatialTransform<T> getFrameCoordinate(unsigned int body_id) const
		{
			SpatialTransform<T> tmp;
			getFrameCoordinate(body_id,tmp);
			return tmp;
		}

		inline SpatialTransform<T> getFrameCoordinate(const mogs_string& body_name) const
		{
			return getFrameCoordinate( model->GetBodyId(body_name));
		}

		MogsGeometry* get_geometry(unsigned int body_id)	const
		{
			return model->get_geometry(body_id);
		}


		Eigen::Matrix < T, 3, 1 > getCenterOfMAss(	const Eigen::Matrix<T,Eigen::Dynamic,1> &Q,
								bool update_kinematics);

		Eigen::Matrix < T, 3, 1 > getCenterOfMAss(	);

/** \brief Returns the body coordinates of a point given in base coordinates.
 * \param Q the curent genereralized positions
 * \param body_id id of the body for which the point coordinates are expressed
 * \param point_base_coordinates coordinates of the point in base coordinates
 * \param update_kinematics whether UpdateKinematics() should be called or not
 * (default: true).
 *
 * \returns a 3-D vector with coordinates of the point in body coordinates
 */
		Eigen::Matrix<T,3,1> CalcBaseToBodyCoordinates (const Eigen::Matrix<T,Eigen::Dynamic,1> &Q,
								unsigned int body_id,
								const Eigen::Matrix<T,3,1> &base_point_position,
								bool update_kinematics = true);


/** \brief Returns the orientation of a given body as 3x3 matrix
 *
 * \param Q the curent genereralized positions
 * \param body_id id of the body for which the point coordinates are expressed
 * \param update_kinematics whether UpdateKinematics() should be called or not
 * (default: true).
 *
 * \returns An orthonormal 3x3 matrix that rotates vectors from base coordinates
 * to body coordinates.
 */
		Eigen::Matrix<T,3,3> CalcBodyWorldOrientation(const Eigen::Matrix<T,Eigen::Dynamic,1> &Q,
							      const unsigned int body_id,
							      bool update_kinematics = true);

		Eigen::Matrix<T,3,3> CalcBodyWorldOrientation(const unsigned int body_id)const ;

		Eigen::Matrix<T,3,1> get_RPY_orientation(	const unsigned int body_id,
								const Eigen::Matrix<T,3,3> &rot =  Eigen::Matrix<T,3,3>::Identity()) const;

/** \brief Computes the point jacobian for a point on a body
 *
 * If a position of a point is computed by a function \f$ g(q(t))\f$ for which its
 * time derivative is \f$ \frac{d}{dt} g(q(t)) = G(q) dot{q} \f$  then this
 * function computes the jacobian matrix \f$ G(q) \f$.
 *
 * \param Q       state vector of the internal joints
 * \param body_id the id of the body
 * \param point_position the position of the point in body-local data
 * \param G       a matrix where the result will be stored in
 * \param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * \returns A 3 x \#dof_count matrix of the point jacobian
 */
		void CalcPointJacobian (const Eigen::Matrix<T,Eigen::Dynamic,1> &Q,
					unsigned int body_id,
					const Eigen::Matrix<T,3,1> &point_position,
					Eigen::Matrix<T,3,Eigen::Dynamic > &G,
					bool update_kinematics = true);

		void Calc6DJacobian (	const Eigen::Matrix<T,Eigen::Dynamic,1> &Q,
					unsigned int body_id,
					Eigen::Matrix<T,6,Eigen::Dynamic > &G,
					bool update_kinematics = true);

/** \brief Computes the velocity of a point on a body
 *
 * \param Q       state vector of the internal joints
 * \param QDot    velocity vector of the internal joints
 * \param body_id the id of the body
 * \param point_position the position of the point in body-local data
 * \param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * \returns The cartesian velocity of the point in global frame (output)
 */
		Eigen::Matrix<T,3,1> CalcPointVelocity (const Eigen::Matrix<T,Eigen::Dynamic,1> &Q,
							const Eigen::Matrix<T,Eigen::Dynamic,1> &QDot,
							unsigned int body_id,
							const Eigen::Matrix<T,3,1> &point_position,
							bool update_kinematics = true);

		Eigen::Matrix<T,3,1> CalcPointVelocity (unsigned int body_id,
							const Eigen::Matrix<T,3,1> &point_position);

		Eigen::Matrix<T,3,1> CalcFrameVelocity (unsigned int body_id) const;

		Eigen::Matrix<T,6,1> get6DVelocity(unsigned int body_id) const;

/** \brief Computes the acceleration of a point on a body
 *
 * \param Q       state vector of the internal joints
 * \param QDot    velocity vector of the internal joints
 * \param QDDot    velocity vector of the internal joints
 * \param body_id the id of the body
 * \param point_position the position of the point in body-local data
 * \param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * \returns The cartesian acceleration of the point in global frame (output)
 *
 * The kinematic state of the model has to be updated before valid
 * values can be obtained. This can either be done by calling
 * UpdateKinematics() or setting the last parameter update_kinematics to
 * true (default).
 *
 * \note During the execution of ForwardDynamics() the acceleration
 * is only applied on the root body and propagated form there. Therefore
 * in the internal state the accelerations of the bodies only represent
 * the relative accelerations without any gravitational effects.
 *
 * \warning  If this function is called after ForwardDynamics() without
 * an update of the kinematic state one has to add the gravity
 * acceleration has to be added to the result.
 */

		Eigen::Matrix<T,3,1> CalcPointAcceleration (const Eigen::Matrix<T,Eigen::Dynamic,1> &Q,
							    const Eigen::Matrix<T,Eigen::Dynamic,1> &QDot,
							    const Eigen::Matrix<T,Eigen::Dynamic,1> &QDDot,
							    unsigned int body_id,
							    const Eigen::Matrix<T,3,1> &point_position,
							    bool update_kinematics = true);

		Eigen::Matrix < T, 3, 1 > CalcPointAcceleration(unsigned int body_id,
								const Eigen::Matrix < T, 3, 1 > &point_position=Eigen::Matrix<double,3,1>::Zero());

		Eigen::Matrix < T, 6, 1 > get6DAcceleration(	unsigned int body_id);

/** \brief Computes the inverse kinematics iteratively using a damped Levenberg-Marquardt method
 *
 * \param Qinit initial guess for the state
 * \param body_id a vector of all bodies for which we we have kinematic target positions
 * \param body_point a vector of points in body local coordinates that are
 * to be matched to target positions
 * \param target_pos a vector of target positions
 * \param Qres output of the computed inverse kinematics
 * \param step_tol tolerance used for convergence detection
 * \param lambda damping factor for the least squares function
 * \param max_iter maximum number of steps that should be performed
 * \returns true on success, false otherwise
 *
 * This function repeatedly computes
 *   \f[ Qres = Qres + \Delta \theta\f]
 *   \f[ \Delta \theta = G^T (G^T G + \lambda^2 I)^{-1} e \f]
 * where \f$G = G(q) = \frac{d}{dt} g(q(t))\f$ and \f$e\f$ is the
 * correction of the body points so that they coincide with the target
 * positions. The function returns true when \f$||\Delta \theta||_2 \le\f$
 * step_tol or if the error between body points and target gets smaller
 * than step_tol. Otherwise it returns false.
 *
 * The parameter \f$\lambda\f$ is the damping factor that has to
 * be chosen carefully. In case of unreachable positions higher values (e.g
 * 0.9) can be helpful. Otherwise values of 0.0001, 0.001, 0.01, 0.1 might
 * yield good results. See the literature for best practices.
 *
 * \warning The actual accuracy might be rather low (~1.0e-2)! Use this function with a
 * grain of suspicion.
 */
		bool InverseKinematics (const Eigen::Matrix<T,Eigen::Dynamic,1> &Qinit,
					const std::vector<unsigned int>&body_id,
					const std::vector<Eigen::Matrix<T,3,1> > &body_point,
					const std::vector<Eigen::Matrix<T,3,1> > &target_pos,
					Eigen::Matrix<T,Eigen::Dynamic,1> &Qres,
					double step_tol = 1.0e-12,
					double lambda = 0.01,
					unsigned int max_iter = 50);

/** \brief Computes all variables for a joint model
 *
 *	By appropriate modification of this function all types of joints can be
 *	modeled. See RBDA Section 4.4 for details.
 *
 * \param joint_id the id of the joint we are interested in (output)
 * \param XJ       the joint transformation (output)
 * \param S        motion subspace of the joint (output)
 * \param v_J      joint velocity (output)
 * \param c_J      joint acceleration for rhenomic joints (output)
 * \param q        joint state variable
 * \param qdot     joint velocity variable
 */
		void jcalc (const unsigned int &joint_id,
			    SpatialTransform < T > &XJ,
			    Eigen::Matrix < double, 6, 1 > &S,
			    Eigen::Matrix < T, 6, 1 > &v_J,
			    Eigen::Matrix < T, 6, 1 > &c_J,
			    const T & q, const T & qdot);

		inline bool is_free_floating_base() const
		{
			return model->is_robot_floating_base();
		}

		// return true for non moving objects
		inline bool does_not_move() const
		{
			if (!is_free_floating_base() && getNDof() == 0)
				return true;
			return false;
		}

		void getTorqueLimit(std::vector < double >&QMAX)
		{
			model->getTorqueLimit(QMAX);
		}

		void set_root_transformation(	const Eigen::Matrix < double, 3, 1 > position,
						const Eigen::Matrix < double, 3, 1 > rotation)
		{
			model->set_root_transformation(position, rotation);
		}


// *********************** Members data **************************************
// ***
// *** Any addition in this should have effects on the serialization
// *** function ( see below )
// ***
// ***************************************************************************

		MogsTemplateRobotProperties < T > *model;

// *********************** Serialization *************************************


	      private:

		friend class boost::serialization::access;

		template < class Archive >
			void serialize (Archive & ar,
					const unsigned int version)
		{

			ar & model;

		}




	};

/** @} */

}

#include "MogsKinematics.hxx"

#endif /* _KINEMATICS_H */
