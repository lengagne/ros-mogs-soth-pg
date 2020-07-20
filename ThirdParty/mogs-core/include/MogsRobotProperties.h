
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

#ifndef _MOGSROBOTPROPERTIES_H
#define _MOGSROBOTPROPERTIES_H


#include <map>
#include <list>
#include <assert.h>
#include <iostream>
#include <limits>
#include <cstring>

#include "MogsRobotJoint.h"
#include "MogsRobotBody.h"
#include "MogsSerialization.h"

#include "config_mogs_core.h"

#include <QtXml>
#include <QMessageBox>
/** \brief Namespace for all structures of the RigidBodyDynamics library
 */
namespace RigidBodyDynamics
{

/** \defgroup model_group Modelling
 * @{
 *
 * There are one way of creating MogsRobotPropertiess for RBDL using the C++ interface.
 *
 *
 * \section modeling_cpp Modeling using C++
 *
 * Using the C++ interface is more advanced but gives some overview about the
 * internals of RBDL.
 *
 * \subsection modeling_overview Overview
 *
 * All model related values are stored in the model structure \link
 * RigidBodyDynamics::MogsRobotProperties\endlink. The functions
 * \link RigidBodyDynamics::MogsRobotProperties::Init MogsRobotProperties::Init()\endlink,
 * \link RigidBodyDynamics::MogsRobotProperties::AddBody MogsRobotProperties::AddBody(...)\endlink,
 * \link RigidBodyDynamics::MogsRobotProperties::AppendBody MogsRobotProperties::AppendBody(...)\endlink, and
 * \link RigidBodyDynamics::MogsRobotProperties::GetBodyId MogsRobotProperties::GetBodyId(...)\endlink,
 * are used to initialize and construct the \ref model_structure.
 *
 * \section model_construction MogsRobotProperties Construction
 *
 * The construction of \link RigidBodyDynamics::MogsRobotProperties Models \endlink makes
 * use of carefully designed constructors of the classes \link
 * RigidBodyDynamics::Body Body \endlink and \link RigidBodyDynamics::Joint
 * Joint \endlink to ease the process of articulated models. Adding bodies to
 * the model is done by specifying the parent body by its id, the
 * transformation from the parent origin to the joint origin, the joint
 * specification as an object, and the body itself. These parameters are
 * then fed to the function RigidBodyDynamics::MogsRobotProperties::AddBody().
 *
 * To create a model with a floating base (a.k.a a model with a free-flyer
 * joint) it is recommended to use \link
 * RigidBodyDynamics::MogsRobotProperties::SetFloatingBaseBody
 * MogsRobotProperties::SetFloatingBaseBody(...)\endlink.
 *
 * Once this is done, the model structure can be used with the functions of \ref
 * kinematics_group, \ref dynamics_group, \ref contacts_group, to perform
 * computations.
 *
 * A simple example can be found \ref SimpleExample "here".
 *
 * \subsection model_structure MogsRobotProperties Structure
 *
 * The model structure contains all the parameters of the rigid multi-body
 * model such as joint informations, mass and inertial parameters of the
 * rigid bodies, etc. It also contains storage for the transformations and
 * current state, such as velocity and acceleration of each body.
 *
 * \subsection joint_models Joint Modeling
 *
 * The Rigid Body Dynamics Library supports models with multiple degrees of
 * freedom. When a joint with more than one degrees of freedom is used,
 * additional virtual bodies with zero mass that are connected by 1 degree
 * of freedom joints to simulate the multiple degrees of freedom joint. Even
 * though this adds some overhead in terms of memory usage, it allows to
 * exploit fast computations on fixed size elements of the underlying math
 * library Eigen3.
 *
 * Joints are defined by their motion subspace. For each degree of freedom
 * a one dimensional motion subspace is specified as a Eigen::Matrix< double,6, 1>.
 * This vector follows the following convention:
 *   \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
 *
 * To specify a planar joint with three degrees of freedom for which the
 * first two are translations in \f$x\f$ and \f$y\f$ direction and the last
 * is a rotation around \f$z\f$, the following joint definition can be used:
 *
 * \code
 * Joint planar_joint = Joint (
 *     Eigen::Matrix< double,6, 1> (0., 0., 0., 1., 0., 0.),
 *     Eigen::Matrix< double,6, 1> (0., 0., 0., 0., 1., 0.),
 *     Eigen::Matrix< double,6, 1> (0., 0., 1., 0., 0., 0.)
 *     );
 * \endcode
 *
 * \subsubsection joint_models_fixed Fixed Joints
 *
 * Fixed joints do not add an additional degree of freedom to the model.
 * When adding a body that via a fixed joint (i.e. when the type is
 * JointTypeFixed) then the dynamical parameters mass and inertia are
 * merged onto its moving parent. By doing so fixed bodies do not add
 * computational costs when performing dynamics computations.

 * To ensure a consistent API for the Kinematics such fixed bodies have a
 * different range of ids. Where as the ids start at 1 get incremented for
 * each added body, fixed bodies start at MogsRobotProperties::fixed_body_discriminator
 * which has a default value of std::numeric_limits<unsigned int>::max() /
 * 2. This means theoretical a maximum of each 2147483646 movable and fixed
 * bodies are possible.

 * To check whether a body is connected by a fixed joint you can use the
 * function MogsRobotProperties::IsFixedBodyId().
 *
 * See also: \link RigidBodyDynamics::Joint Joint\endlink.
 *
 * \note Please note that in the Rigid %Body Dynamics Library all angles
 * are specified in radians.
 */

/** \brief Contains all information about the rigid body model
 *
 * This class contains all information required to perform the forward
 * dynamics calculation. The variables in this class are also used for
 * storage of temporary values. It is designed for use of the Articulated
 * Rigid Body Algorithm (which is implemented in ForwardDynamics()) and
 * follows the numbering as described in Featherstones book.
 *
 * An important note is that body 0 is the root body and the moving bodies
 * start at index 1. Additionally the vectors for the states q, qdot, etc.
 * have \#MogsRobotProperties::mBodies + 1 entries where always the first entry (e.g.
 * q[0]) contains the value for the base (or "root" body). Thus the
 * numbering might be confusing as q[1] holds the position variable of the
 * first added joint. This numbering scheme is very beneficial in terms of
 * readability of the code as the resulting code is very similar to the
 * pseudo-code in the RBDA book.
 *
 * \note To query the number of degrees of freedom use MogsRobotProperties::dof_count.
 */

	class MogsRobotProperties
	{
	      public:

		MogsRobotProperties ();

		MogsRobotProperties (   MogsRobotProperties* MogsRobotProperties_in);

		MogsRobotProperties (   const mogs_string & name,
                                MogsGeometry* MogsRobotProperties_in);


		~MogsRobotProperties ()
		{

		};

		void new_fixed_robot( 	const mogs_string & name,
					const Eigen::Matrix < double, 3, 1 > & position,
					const Eigen::Matrix < double, 3, 1 > & rotation,
					MogsGeometry *geom);

		void new_free_floating_robot( 	const mogs_string & name,
						MogsGeometry *geom);

		// Structural information
		/// \brief The id of the parents body
		std::vector < unsigned int >lambda;
		/// \brief Contains the ids of all the children of a given body
		std::vector < std::vector < unsigned int > >mu;

		/** \brief number of degrees of freedoms of the model
		*
		* This value contains the number of entries in the generalized state (q)
		* velocity (qdot), acceleration (qddot), and force (tau) vector.
		*/
		unsigned int dof_count;
		/// \brief Id of the previously added body, required for MogsRobotProperties::AppendBody()
		unsigned int previously_added_body_id;

		/// \brief the cartesian vector of the gravity
		Eigen::Matrix < double, 3, 1 > gravity;

		////////////////////////////////////
		// Joints

		/// \brief All joints
		std::vector < MogsRobotJoint, Eigen::aligned_allocator < MogsRobotJoint > >mJoints;
		/// \brief The joint axis for joint i
		std::vector < Eigen::Matrix < double, 6, 1 >, Eigen::aligned_allocator < Eigen::Matrix < double, 6, 1 > > > S;
		/// \brief Transformations from the parent body to the frame of the joint
		std::vector < SpatialTransform < double >, Eigen::aligned_allocator < SpatialTransform < double > > >X_T;
		/// \brief The number of fixed joints that have been declared before each joint.
		std::vector < unsigned int >mFixedJointCount;

		////////////////////////////////////
		// Bodies

		 SpatialTransform < double > X_base0;

		/// \brief All bodies that are attached to a body via a fixed joint.
		std::vector < MogsRobotFixedBody, Eigen::aligned_allocator < MogsRobotFixedBody > >mFixedBodies;

	/** \brief Value that is used to discriminate between fixed and movable
	 * bodies.
	 *
	 * Bodies with id 1 .. (fixed_body_discriminator - 1) are moving bodies
	 * while bodies with id fixed_body_discriminator .. max (unsigned int)
	 * are fixed to a moving body. The value of max(unsigned int) is
	 * determined via std::numeric_limits<unsigned int>::max() and the
	 * default value of fixed_body_discriminator is max (unsigned int) / 2.
	 *
	 * On normal systems max (unsigned int) is 4294967294 which means there
	 * could be a total of 2147483646 movable and / or fixed bodies.
	 */
		unsigned int fixed_body_discriminator;

	/** \brief All bodies 0 ... N_B, including the base
	 *
	 * mBodies[0] - base body <br>
	 * mBodies[1] - 1st moveable body <br>
	 * ... <br>
	 * mBodies[N_B] - N_Bth moveable body <br>
	 */
		std::vector < MogsRobotBody, Eigen::aligned_allocator < MogsRobotBody > >mBodies;

		/// \brief Human readable names for the bodies
		std::map < mogs_string, unsigned int >mBodyNameMap;

	/** \brief Connects a given body to the model
	 *
	 * When adding a body there are basically informations required:
	 * - what kind of body will be added?
	 * - where is the new body to be added?
	 * - by what kind of joint should the body be added?
	 *
	 * The first information "what kind of body will be added" is contained
	 * in the Body class that is given as a parameter.
	 *
	 * The question "where is the new body to be added?" is split up in two
	 * parts: first the parent (or successor) body to which it is added and
	 * second the transformation to the origin of the joint that connects the
	 * two bodies. With these two informations one specifies the relative
	 * positions of the bodies when the joint is in neutral position.gk
	 *
	 * The last question "by what kind of joint should the body be added?" is
	 * again simply contained in the Joint class.
	 *
	 * \param parent_id   id of the parent body
	 * \param joint_frame the transformation from the parent frame to the origin
	 *                    of the joint frame (represents X_T in RBDA)
	 * \param joint       specification for the joint that describes the connection
	 * \param body        specification of the body itself
	 * \param body_name   human readable name for the body (can be used to retrieve its id
	 *                    with GetBodyId())
	 *
	 * \returns id of the added body
	 */
		virtual unsigned int AddBody (const unsigned int parent_id,
                                      const SpatialTransform < double >&joint_frame,
                                      const MogsRobotJoint & joint, const MogsRobotBody & body,
                                      const mogs_string & body_name = "");

	/** \brief Adds a Body to the model such that the previously added Body is the Parent.
	 *
	 * This function is basically the same as MogsRobotProperties::AddBody() however the
	 * most recently added body (or body 0) is taken as parent.
	 */
		unsigned int AppendBody (const SpatialTransform < double >&joint_frame,
                                 const MogsRobotJoint & joint,
                                 const MogsRobotBody & body,
                                 const mogs_string & body_name = "");


		unsigned int AddBodyFixedJoint (const unsigned int parent_id,
                                        const SpatialTransform < double >&joint_frame,
                                        const MogsRobotJoint & joint,
                                        const MogsRobotBody & body,
                                        const mogs_string& body_name);

		unsigned int AddBodyMultiDofJoint (	const unsigned int parent_id,
                                            const SpatialTransform < double >&joint_frame,
                                            const MogsRobotJoint & joint,
                                            const MogsRobotBody & body,
                                            const mogs_string &  body_name);

	/** \brief Specifies the dynamical parameters of the first body and
	 *  \brief assigns it a 6 DoF joint.
	 *
	 * The 6 DoF joint is simulated by adding 5 massless bodies at the base
	 * which are connected with joints. The body that is specified as a
	 * parameter of this function is then added by a 6th joint to the model.
	 *
	 * The floating base has the following order of degrees of freedom:
	 *
	 * \li translation X
	 * \li translation Y
	 * \li translation Z
	 * \li rotation Z
	 * \li rotation Y
	 * \li rotation X
	 *
	 * To specify a different ordering, it is recommended to create a 6 DoF
	 * joint. See \link RigidBodyDynamics::MogsRobotJoint Joint\endlink for more
	 * information.
	 *
	 * \param body Properties of the floating base body.
	 *
	 *  \returns id of the body with 6 DoF
	 */
		unsigned int SetFloatingBaseBody (  const MogsRobotBody & body,
                                            const mogs_string &body_name = "");

	/** \brief Returns the id of a body that was passed to AddBody()
	 *
	 * Bodies can be given a human readable name. This function allows to
	 * resolve its name to the numeric id.
	 *
	 * \note Instead of querying this function repeatedly, it might be
	 * advisable to query it once and reuse the returned id.
	 *
	 * \returns the id of the body or \c std::numeric_limits<unsigned int>::max() if the id was not found.
	 */
		unsigned int GetBodyId (const mogs_string & body_name) const;

	/** \brief Returns the name of a body for a given body id */
		mogs_string GetBodyName (unsigned int body_id) const
		{
			std::map < mogs_string,unsigned int >::const_iterator iter = mBodyNameMap.begin ();

			while (iter != mBodyNameMap.end ())
			  {
				  if (iter->second == body_id)
					  return iter->first;

				  iter++;
			  }

			return "";
		}

	/** \brief Returns the length of the body for a given body id*/
		double GetBodyLength (unsigned int body_id) const
		{
			return mBodies[body_id].getLenght();
		}

		/**	\brief return the total mass of the robot*/
		double GetRobotMass()const;
		
		/** Get the name of the joints 
		 * Be carefull it is not the size of dof_count in case of multidof joints */
		std::vector<std::string> GetJointsName()const;

	/** \brief Checks whether the body is rigidly attached to another body.
	 */
		bool IsFixedBodyId (unsigned int body_id)
		{
			if (body_id >= fixed_body_discriminator && body_id < std::numeric_limits < unsigned int >::max ()
				&& body_id - fixed_body_discriminator < mFixedBodies.size ())
			  {
				  return true;
			  }
			return false;
		}

		bool IsBodyId (unsigned int id)
		{
			if (id > 0 && id < mBodies.size ())
				return true;
			if (id >= fixed_body_discriminator && id < std::numeric_limits <unsigned int >::max ())
			{
				  if (id - fixed_body_discriminator < mFixedBodies.size ())
                  {
                      std::cout<<"true here"<<std::endl;
                      return true;
                  }

			}
			return false;
		}
		/// \brief Initializes the helper values for the dynamics algorithm
		virtual void Init ();

		/**	Add a sphere to the geometry to set the marker position*/
		void add_sphere_to_body(const mogs_string & body_name,
                                        const Eigen::Matrix < double, 3, 1 > & position);
                
		/**	Add a sphere to the geometry to set the marker position*/
		void add_mesh_to_body(  const mogs_string & body_name,
                                        MogsGeometry* mesh);
                

		// return true if the robot was load successfully
		// name is the name of the XML file
		// force_fixed allows not to consider the 6 dof for free floatting base
		virtual bool SetRobotFile (const mogs_string & name, bool force_fixed = false);

		bool SetRobotSphere(double radius, double mass = 0);

		unsigned int getNDof ();
		
		unsigned int getNDofWithMimic()
                {
                        return dof_count;
                }

		bool scalable_robot ()
		{
			return scalable_robot_;
		}

		void set_scale (double size, double weight, double efficiency)
		{
			size_ = size;
			weight_ = weight;
			power_ = efficiency;
			scalable_robot_ = true;
		}

		unsigned int getNBodies () const
		{
			return mBodies.size ();
		}

		mogs_string getRobotName() const
		{
			return robot_name_;
		}

		void setRobotName(const mogs_string & name)
		{
			robot_name_ = name;
		}

		mogs_string get_robot_type() const
		{
			return robot_type_;
		}

		mogs_string get_joint_name(int i) const
		{
			return mJoints[i].data_.name;
		}
		
		unsigned int get_joint_id( const mogs_string& name) const
		{
                    for (unsigned int i=1;i<mJoints.size();i++)  if( mJoints[i].data_.name == name)
                        return i-1;
                    std::cerr<<"ERROR in "<<__FILE__<<" at line "<< __LINE__<<std::endl;
                    std::cerr<<"Cannot find joint "<< name.toStdString()<<std::endl;
                    exit(1);
                }

		bool scalable_robot_;

		bool xml_read_;

		bool free_base_;	// true if the base is free
		mogs_string robot_type_;	// type of the robot
		mogs_string robot_name_;	// name of the robot

		double GetSize()
		{
			return size_;
		}

		// Used for the scaled robot
		double size_, weight_, power_;

		bool is_robot_floating_base() const
		{
			return floating_base_robot_;
		}

		bool floating_base_robot_;

		virtual void set_root_transformation(	const Eigen::Matrix < double, 3, 1 > &position,
                                                const Eigen::Matrix < double, 3, 1 > &rotation);

		MogsGeometry* get_geometry(unsigned int body_id)	const
		{
			return mBodies[body_id].mGeom;
		}

	      private:

                bool create_structure(  std::vector < MogsRobotBody, Eigen::aligned_allocator < MogsRobotBody > > List_Of_Bodies,
                                        std::vector < MogsRobotJoint, Eigen::aligned_allocator < MogsRobotJoint > >List_Of_Joints);

		MogsRobotBody load_new_body_scaled_xml (QDomNode Element,
							double size, double weight,
							const mogs_string & path="",
							const mogs_string &body_extension = "");

		MogsRobotBody load_new_body_unscaled_xml (	QDomNode Element,
							const mogs_string & path="./",
							const mogs_string &body_extension = "");

		MogsRobotBody load_new_body_unscaled_urdf(	QDomNode Element,
							const mogs_string & path="./",
							const mogs_string &body_extension = "");

		MogsRobotJoint load_new_joint_urdf(	QDomNode Element,
                                            const mogs_string &joint_extension = "");

		MogsRobotJoint load_new_joint_xml (	QDomNode Element,
						bool scalable,
						double size,
						double weight, double efficiency,
						const mogs_string &joint_extension = "");

		void read_bodies_xml(std::vector < MogsRobotBody, Eigen::aligned_allocator < MogsRobotBody > > & List_Of_Bodies,
				 const mogs_string & xml,
				 const mogs_string prefix="");

		void read_joints_xml(std::vector < MogsRobotJoint, Eigen::aligned_allocator < MogsRobotJoint > > &List_Of_Joints,
				 const mogs_string & xml,
				 const mogs_string prefix="");

		void read_bodies_urdf(std::vector < MogsRobotBody, Eigen::aligned_allocator < MogsRobotBody > > & List_Of_Bodies,
				 const mogs_string & filename,
				 const mogs_string prefix="");

		void read_joints_urdf(std::vector < MogsRobotJoint, Eigen::aligned_allocator < MogsRobotJoint > > &List_Of_Joints,
				 const mogs_string & filename,
				 const mogs_string prefix="");

		bool SetRobotUrdf (const mogs_string & name);

		bool SetRobotXml (const mogs_string & name, bool force_fixed = false);

		/// \brief Joint position limits
		std::vector<double> qmin_;
		std::vector<double> qmax_;

		/// \brief Joint velocity limit
		std::vector<double> dqmax_;

		/// \brief Joint torque limit
		std::vector<double> torquemax_;
                

                
	      public:
                  
                /// \brief Number of mimic joint for the robot
                unsigned int number_mimic_joint_;
                  
                struct info_mimic
                {
                        unsigned int source_id;
                        unsigned int target_id;
                        double multiplier;
                        double offset;
                };
                
                std::vector<info_mimic> info_mimic_;
                  
		void getPositionLimit (std::vector < double >&QMIN,
				       std::vector < double >&QMAX);

		void getVelocityLimit(std::vector < double >&MAX);

		void getTorqueLimit(std::vector < double >&MAX);

		void set_config_files(const std::map<mogs_string,mogs_string> & config_files)
		{
		    std::cout<<"Setting config files "<<std::endl;
		    config_files_ = config_files;
		}
		
                /// This function return the config file of type 'type' and return true
                /// if this type is not defined the functions return false
                bool get_config_file(   const mogs_string & type,
                                        mogs_string & filename);
    
		std::map<mogs_string,mogs_string> config_files_;

	      protected:

		friend class boost::serialization::access;

		template < class Archive >
			void serialize (Archive & ar, const unsigned int version)
		{

			ar & lambda;
			ar & mu;
			ar & dof_count;
			ar & previously_added_body_id;
			ar & gravity;
			ar & mJoints;
			ar & S;
			ar & X_T;
			ar & mFixedJointCount;
// 			ar & X_lambda;
			ar & mFixedBodies;
			ar & fixed_body_discriminator;
			ar & mBodies;
			ar & mBodyNameMap;
			ar & scalable_robot_;
			ar & xml_read_;
			ar & robot_type_;
			ar & robot_name_;
			ar & size_;
			ar & weight_;
			ar & power_;
			ar & floating_base_robot_;
			ar & qmin_;
			ar & qmax_;
			ar & dqmax_;
			ar & torquemax_;
			ar & X_base0;
                        ar & number_mimic_joint_;
		}

	};

/** @} */

}

#endif /* _MODEL_H */
