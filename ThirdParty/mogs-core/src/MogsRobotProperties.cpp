//     MogsRobotProperties.cpp
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
//      2012-2013: IUT de Beziers/ LIRMM, Beziers, France
// 	from 2013 : Université Blaise Pascal / axis : ISPR / theme MACCS

#include "MogsRobotProperties.h"

using namespace RigidBodyDynamics;

MogsRobotProperties::MogsRobotProperties ()
{
	size_ = 1.8;
	weight_ = 80.0;
	power_ = 1.0;
	xml_read_ = false;
}

MogsRobotProperties::MogsRobotProperties (MogsRobotProperties *MogsRobotProperties_in)
		:dof_count (MogsRobotProperties_in->dof_count),
		gravity (MogsRobotProperties_in->gravity),
		mJoints (MogsRobotProperties_in->mJoints),
		mFixedBodies (MogsRobotProperties_in->mFixedBodies),
		fixed_body_discriminator (MogsRobotProperties_in->fixed_body_discriminator),
		mBodies (MogsRobotProperties_in->mBodies),
		mBodyNameMap (MogsRobotProperties_in->mBodyNameMap),
		scalable_robot_ (MogsRobotProperties_in->scalable_robot_),
		xml_read_ (MogsRobotProperties_in->xml_read_),
		robot_name_ (MogsRobotProperties_in->robot_name_),
		robot_type_ (MogsRobotProperties_in->robot_type_),
		size_ (MogsRobotProperties_in->size_), weight_ (MogsRobotProperties_in->weight_),
		power_ (MogsRobotProperties_in->power_),
		floating_base_robot_(MogsRobotProperties_in->floating_base_robot_),
		X_base0(MogsRobotProperties_in->X_base0),
		config_files_(MogsRobotProperties_in->config_files_),
                number_mimic_joint_(MogsRobotProperties_in->number_mimic_joint_),
                info_mimic_(MogsRobotProperties_in->info_mimic_)
{
	int nb;
	nb = MogsRobotProperties_in->X_T.size ();
	X_T.resize (nb);
	for (int i = 0; i < nb; i++)
		X_T[i] = MogsRobotProperties_in->X_T[i];

	nb = MogsRobotProperties_in->lambda.size ();
	lambda.resize (nb);
	for (int i = 0; i < nb; i++)
		lambda[i] = MogsRobotProperties_in->lambda[i];

	nb = MogsRobotProperties_in->S.size ();
	S.resize (nb);
	for (int i = 0; i < nb; i++)
		S[i] = MogsRobotProperties_in->S[i];

	MogsRobotProperties_in->getPositionLimit(qmin_,qmax_);
	MogsRobotProperties_in->getVelocityLimit(dqmax_);
	MogsRobotProperties_in->getTorqueLimit(torquemax_);
}

MogsRobotProperties::MogsRobotProperties( const mogs_string & name,
                                          MogsGeometry *geom)
{
	Init();
// 	X_base[0] = SpatialTransform < double > (rotation, position );
	X_base0 =  SpatialTransform < double > ();
	MogsRobotBody body;
	body.set_geom(geom);
	robot_name_ = name;
	SetFloatingBaseBody(body,robot_name_);
}

void MogsRobotProperties::add_sphere_to_body(	const mogs_string& body_name,
						const Eigen::Matrix < double, 3, 1 > & position)
{
	MogsGeometry *geom = new MogsGeometry;
	Mesh *meh = new Mesh();
	sub_Mesh *smeh = new sub_Mesh(meh);

	geom->convert_sphere_to_mesh(smeh,position(0), position(1),position(2), 0.01, 0, 255, 0);

	// add subMesh in Mesh and Mesh in Geometry
	meh->subGeometries_.push_back(smeh);
	geom-> add(*meh);
	mBodies[ mBodyNameMap[body_name]].add_geom(geom);
}

void MogsRobotProperties::add_mesh_to_body( const mogs_string & body_name,
                                            MogsGeometry* mesh)
{
    	mBodies[ mBodyNameMap[body_name]].add_geom(mesh);
}

bool MogsRobotProperties::get_config_file(  const mogs_string & type,
                                            mogs_string & filename)
{
    if ( config_files_.find(type) == config_files_.end() ) {
        // not found
        filename = "not found";
        return false;
    } else {
        // found
        filename = config_files_[type];
        return true;
    }
}

unsigned int MogsRobotProperties::getNDof ()
{
        return dof_count - number_mimic_joint_;
}


void MogsRobotProperties::new_free_floating_robot( 	const mogs_string & name,
							MogsGeometry *geom)
{
	Init();
// 	X_base[0] = SpatialTransform < double > (rotation, position );
	X_base0 =  SpatialTransform < double > ();
	MogsRobotBody body;
	body.set_geom(geom);
	robot_name_ = name;

	SetFloatingBaseBody(body,robot_name_);
}

void MogsRobotProperties::new_fixed_robot( 	const mogs_string & name,
						const Eigen::Matrix < double, 3, 1 > & position,
						const Eigen::Matrix < double, 3, 1 > & rotation,
						MogsGeometry *geom)
{
	Init();
// 	X_base[0] = SpatialTransform < double > (rotation, position );
	X_base0 =  SpatialTransform < double > (rotation, position );

	// update geometries normals_ and normals_faces_ with rotation
	double rot_x = rotation[0], rot_y = rotation[1], rot_z = rotation[2];
	Eigen::Matrix<double,3,3> Rx (1,0,0,0,cos(rot_x),-sin(rot_x),0,sin(rot_x),cos(rot_x));
	Eigen::Matrix<double,3,3> Ry (cos(rot_y),0,sin(rot_y),0,1,0,-sin(rot_y),0,cos(rot_y));
	Eigen::Matrix<double,3,3> Rz (cos(rot_z),-sin(rot_z),0,sin(rot_z),cos(rot_z),0,0,0,1);

	int i,j,k;
	for (i = 0; i<geom->tab_mesh.size() ; ++i)
		for (j = 0; j < geom->tab_mesh[i].subGeometries_.size(); ++j)
		{
			for (k = 0; k < geom->tab_mesh[i].subGeometries_[j]->normals_faces_.size(); ++k)
			{
				// normals_faces_
				geom->tab_mesh[i].subGeometries_[j]->normals_faces_[k] =
				geom->tab_mesh[i].subGeometries_[j]->normals_faces_[k].transpose() * Rx * Ry * Rz;
				geom->tab_mesh[i].subGeometries_[j]->normals_faces_[k].transpose();
			}
			for (k = 0; k < geom->tab_mesh[i].subGeometries_[j]->normals_.size(); ++k)
			{
				// normals_faces_
				geom->tab_mesh[i].subGeometries_[j]->normals_[k] =
				geom->tab_mesh[i].subGeometries_[j]->normals_[k].transpose() * Rx * Ry * Rz;
				geom->tab_mesh[i].subGeometries_[j]->normals_[k].transpose();
			}
		}
	mBodies[0].set_geom(geom);
	robot_name_ = name;
}

void MogsRobotProperties::Init ()
{
	MogsRobotBody root_body;
	MogsRobotJoint root_joint;

	// structural information
	lambda.push_back (0.);
	mu.push_back (std::vector < unsigned int >());
	dof_count = 0;
	previously_added_body_id = 0;

	// Joints
	mJoints.push_back (root_joint);
	S.push_back (SpatialVectorZeroDouble);
	X_T.push_back (SpatialTransform < double > ());

	mBodies.push_back (root_body);
	mBodyNameMap["ROOT"] = 0;
	fixed_body_discriminator = std::numeric_limits < unsigned int >::max () / 2;
	floating_base_robot_ = false;
	scalable_robot_ = false;
	floating_base_robot_ = false;
        
        number_mimic_joint_ = 0;
}

double MogsRobotProperties::GetRobotMass()const
{
	double mass = 0;
	for (int i=0;i<mBodies.size();i++)
		mass += mBodies[i].mMass;
	return mass;
}

void MogsRobotProperties::getPositionLimit (std::vector < double >&QMIN, std::vector < double >&QMAX)
{
	QMIN = qmin_;
	QMAX = qmax_;
}

void MogsRobotProperties::getVelocityLimit (std::vector < double >&DQMAX)
{
	DQMAX = dqmax_;
}

void MogsRobotProperties::getTorqueLimit (std::vector < double >&TMAX)
{
	TMAX = torquemax_;
}

unsigned int MogsRobotProperties::AddBodyFixedJoint (	const unsigned int parent_id,
							const SpatialTransform < double >&joint_frame,
							const MogsRobotJoint & joint,
							const MogsRobotBody & body,
							const mogs_string& body_name)
{
	MogsRobotFixedBody fbody = MogsRobotFixedBody::CreateFromBody (body);
	fbody.mMovableParent = parent_id;
	fbody.mParentTransform = joint_frame;

	if (IsFixedBodyId (parent_id))
	{
		MogsRobotFixedBody fixed_parent = mFixedBodies[parent_id - fixed_body_discriminator];
		fbody.mMovableParent = fixed_parent.mMovableParent;
		fbody.mParentTransform = joint_frame * fixed_parent.mParentTransform;
	}

	// merge the two bodies
	mBodies[fbody.mMovableParent].Join (fbody.mParentTransform, body);
	mFixedBodies.push_back (fbody);

	if (mFixedBodies.size () > std::numeric_limits < unsigned int >::max () - fixed_body_discriminator)
	{
		std::cerr << "Error: cannot add more than " << std::numeric_limits < unsigned int >::max () - mFixedBodies.size () << " fixed bodies. You need to modify MogsRobotProperties::fixed_body_discriminator for this." << std::endl;
		assert (0);
		abort ();
	}

	if (body_name.size () != 0)
	{
		if (mBodyNameMap.find (body_name) != mBodyNameMap.end ())
		{
			std::cerr << "Error: Body with name '" << body_name.toStdString() << "' already exists!" << std::endl;
			assert (0);
			abort ();
		}
		mBodyNameMap[body_name] = mFixedBodies.size () + fixed_body_discriminator - 1;
	}

	return mFixedBodies.size () + fixed_body_discriminator - 1;
}

unsigned int MogsRobotProperties::AddBodyMultiDofJoint (const unsigned int parent_id,
							const SpatialTransform < double >&joint_frame,
							const MogsRobotJoint & joint,
							const MogsRobotBody & body,
							const mogs_string &  body_name)
{
	// Here we emulate multi DoF joints by simply adding nullbodies. This
	// allows us to use fixed size elements for S,v,a, etc. which is very
	// fast in Eigen.
	unsigned int joint_count;
	if (joint.mJointType == JointType1DoF)
		joint_count = 1;
	else if (joint.mJointType == JointType2DoF)
		joint_count = 2;
	else if (joint.mJointType == JointType3DoF)
		joint_count = 3;
	else if (joint.mJointType == JointType4DoF)
		joint_count = 4;
	else if (joint.mJointType == JointType5DoF)
		joint_count = 5;
	else if (joint.mJointType == JointType6DoF)
		joint_count = 6;
	else
	{
		std::cerr << "Error: Invalid joint type: " << joint.mJointType << std::endl;
		assert (0 && !"Invalid joint type!");
	}

	MogsRobotBody null_body ("virtual", 0., Eigen::Matrix < double, 3, 1 > (0., 0., 0.), Eigen::Matrix < double, 3, 1 > (0., 0., 0.), NULL);
	unsigned int null_parent = parent_id;
	SpatialTransform < double >joint_frame_transform;

	MogsRobotJoint single_dof_joint;
	unsigned int j;

	// Here we add multiple virtual bodies that have no mass or inertia for
	// which each is attached to the model with a single degree of freedom
	// joint.
	for (j = 0; j < joint_count; j++)
	{
		Eigen::Matrix < double, 3, 1 > rotation (joint.mJointAxes[j][0], joint.mJointAxes[j][1], joint.mJointAxes[j][2]);
		Eigen::Matrix < double, 3, 1 > translation (joint.mJointAxes[j][3], joint.mJointAxes[j][4], joint.mJointAxes[j][5]);

		if (rotation == Eigen::Matrix < double, 3, 1 > (0., 0., 0.))
		{
			single_dof_joint = MogsRobotJoint (JointTypePrismatic, translation);
		}
		else if (translation == Eigen::Matrix < double, 3, 1 > (0., 0., 0.))
		{
			single_dof_joint = MogsRobotJoint (JointTypeRevolute, rotation);
		}

			// the first joint has to be transformed by joint_frame, all the
			// others must have a null transformation
		if (j == 0)
			joint_frame_transform = joint_frame;
		else
			joint_frame_transform = SpatialTransform < double >();

		if (j == joint_count - 1)
			// if we are at the last we must add the real body
			break;
		else
		{
			// otherwise we just add an intermediate body
			null_parent = AddBody (null_parent, joint_frame_transform, single_dof_joint, null_body);
		}
	}
	return AddBody (null_parent, joint_frame_transform, single_dof_joint, body, body_name);
}

unsigned int MogsRobotProperties::AddBody (	const unsigned int parent_id,
						const SpatialTransform < double >&joint_frame,
						const MogsRobotJoint & joint,
						const MogsRobotBody & body,
						const mogs_string & body_name)
{
	if (joint.mJointType == JointTypeFixed)
	{
		previously_added_body_id = AddBodyFixedJoint (parent_id, joint_frame, joint, body, body_name);
		return previously_added_body_id;
	}
	else if (joint.mJointType != JointTypePrismatic && joint.mJointType != JointTypeRevolute)
	{
		previously_added_body_id = AddBodyMultiDofJoint (parent_id, joint_frame, joint, body, body_name);
		return previously_added_body_id;
	}

	assert (lambda.size () > 0);
	assert (joint.mJointType != JointTypeUndefined);

	// If we add the body to a fixed body we have to make sure that we
	// actually add it to its movable parent.
	unsigned int movable_parent_id = parent_id;
	SpatialTransform < double >movable_parent_transform;

	if (IsFixedBodyId (parent_id))
	{
		unsigned int fbody_id = parent_id - fixed_body_discriminator;
		movable_parent_id = mFixedBodies[fbody_id].mMovableParent;
		movable_parent_transform = mFixedBodies[fbody_id].mParentTransform;
	}

	// structural information
	lambda.push_back (movable_parent_id);
	mu.push_back (std::vector < unsigned int >());
	mu.at (movable_parent_id).push_back (mBodies.size ());

	// Bodies
// 	X_lambda.push_back (SpatialTransform < double > ());
// 	X_base.push_back (SpatialTransform < T > ());
	mBodies.push_back (body);

	if (body_name.size () != 0)
	{
		if (mBodyNameMap.find (body_name) != mBodyNameMap.end ())
		{
			std::cerr << "Error: Body with name '" << body_name.toStdString() << "' already exists!" << std::endl;
			assert (0);
			abort ();
		}
		mBodyNameMap[body_name] = mBodies.size () - 1;
	}
	dof_count++;

// 	// state information
// 	v.push_back (SpatialVectorZero);
// 	a.push_back (SpatialVectorZero);
	// Joints
	mJoints.push_back (joint);
	S.push_back (joint.mJointAxes[0]);

	// we have to invert the transformation as it is later always used from the
	// child bodies perspective.
	X_T.push_back (joint_frame * movable_parent_transform);


// 	// Dynamic variables
// 	c.push_back (SpatialVectorZero);
// 	IA.push_back (body.mSpatialInertia.template cast <T> ());
// 	pA.push_back (SpatialVectorZero);
// 	U.push_back (SpatialVectorZero);
//
// 	d = Eigen::Matrix < T, Eigen::Dynamic, 1 >::Zero (mBodies.size ());
// 	u = Eigen::Matrix < T, Eigen::Dynamic, 1 >::Zero (mBodies.size ());

// 	f.push_back (SpatialVectorZero);
// 	Ic.push_back (SpatialRigidBodyInertia (body.mMass, body.mCenterOfMass, body.mInertia));

	if (mBodies.size () == fixed_body_discriminator)
	{
		std::cerr << "Error: cannot add more than " << fixed_body_discriminator << " movable bodies. You need to modify MogsRobotProperties::fixed_body_discriminator for this." << std::endl;
		assert (0);
		abort ();
	}
	previously_added_body_id = mBodies.size () - 1;

	return previously_added_body_id;
}

unsigned int MogsRobotProperties::AppendBody (	const SpatialTransform < double >&joint_frame,
						const MogsRobotJoint & joint,
						const MogsRobotBody & body,
						const mogs_string & body_name)
{
	return MogsRobotProperties::AddBody (previously_added_body_id, joint_frame, joint, body, body_name);
}

unsigned int MogsRobotProperties::GetBodyId (const mogs_string & body_name) const
{
	std::map < mogs_string,unsigned int >::const_iterator iter = mBodyNameMap.begin ();
	if (mBodyNameMap.count (body_name) == 0)
	{
		return std::numeric_limits <unsigned int >::max ();
	}

	return mBodyNameMap.find (body_name)->second;
}

std::vector<std::string> MogsRobotProperties::GetJointsName()const
{
    std::vector<std::string> out;
    for(int i=1;i<mJoints.size();i++)   if(!mJoints[i].data_.is_mimic)
      out.push_back( mJoints[i].data_.name.toStdString());
    
    return out;
}

void MogsRobotProperties::read_bodies_urdf(	std::vector < MogsRobotBody, Eigen::aligned_allocator < MogsRobotBody > > & List_Of_Bodies,
                                            const mogs_string & urdf,
                                            const mogs_string prefix)
{
    mogs_string path = mogs_get_absolute_link(urdf);
    QDomDocument *dom = new QDomDocument(urdf); // Création de l'objet DOM
    QFile xml_doc(urdf);// On choisit le fichier contenant les informations XML.
    if(!xml_doc.open(QIODevice::ReadOnly))// Si l'on n'arrive pas à ouvrir le fichier XML.
    {
         qDebug() <<"Erreur dans "<< __FILE__<<" à la ligne "<< __LINE__;
         qDebug() <<"Erreur à l'ouverture du document URDF : "<<urdf;
         qDebug()<<"Le document URDF n'a pas pu être ouvert. Vérifiez que le nom est le bon et que le document est bien placé";
         exit(1);
    }
    dom->setContent(&xml_doc);
    QDomElement root = dom->documentElement();
    assert (!root.isNull());

    for (QDomElement childLink = root.firstChildElement("link"); !childLink.isNull(); childLink = childLink.nextSiblingElement("link") )
    {
        List_Of_Bodies.push_back (load_new_body_unscaled_urdf (childLink,mogs_get_absolute_path(path),prefix));
    }
}

void MogsRobotProperties::read_joints_urdf(	std::vector < MogsRobotJoint, Eigen::aligned_allocator < MogsRobotJoint > > &List_Of_Joints,
                                            const mogs_string & urdf_file,
                                            const mogs_string prefix)
{
	mogs_string url = mogs_get_absolute_link(urdf_file);
	QDomDocument *dom = new QDomDocument(url); // Création de l'objet DOM
    QFile xml_doc(urdf_file);// On choisit le fichier contenant les informations XML.
    if(!xml_doc.open(QIODevice::ReadOnly))// Si l'on n'arrive pas à ouvrir le fichier XML.
    {
         qDebug() <<"Erreur à l'ouverture du document URDF : "<<urdf_file;
         qDebug()<<"Le document URDF n'a pas pu être ouvert. Vérifiez que le nom est le bon et que le document est bien place";
         exit(1);
    }
    dom->setContent(&xml_doc);
    QDomElement root = dom->documentElement();
    assert (!root.isNull());

    for (QDomElement childJoint = root.firstChildElement("joint"); !childJoint.isNull(); childJoint = childJoint.nextSiblingElement("joint") )
    {
        List_Of_Joints.push_back (load_new_joint_urdf (childJoint,prefix));
    }
}

void MogsRobotProperties::read_bodies_xml(	std::vector < MogsRobotBody, Eigen::aligned_allocator < MogsRobotBody > > & List_Of_Bodies,
                                            const mogs_string & xml,
                                            const mogs_string extension)
{
	mogs_string url = mogs_get_absolute_link(xml);
	QDomDocument *dom = new QDomDocument(url); // Création de l'objet DOM
	QFile xml_doc(url);// On choisit le fichier contenant les informations XML.
    if(!xml_doc.open(QIODevice::ReadOnly))// Si l'on n'arrive pas à ouvrir le fichier XML.
    {
         qDebug() <<"Erreur à l'ouverture du document XML :"<<url;
         qDebug()<<"Le document XML n'a pas pu être ouvert. Vérifiez que le nom est le bon et que le document est bien placé";
         return;
    }
    dom->setContent(&xml_doc);
    QDomElement root = dom->documentElement();

    QDomNodeList childBodies = root.elementsByTagName("Bodies");
	assert (childBodies.count());
	QDomNode bodieshandle = childBodies.item(0);
	for (QDomElement childBody = bodieshandle.firstChildElement("Body"); ! childBody.isNull() ; childBody = childBody.nextSiblingElement("Body"))
	{
		if (scalable_robot_)
			List_Of_Bodies.push_back (load_new_body_scaled_xml (childBody,size_, weight_,mogs_get_absolute_path(url),extension));
		else
			List_Of_Bodies.push_back (load_new_body_unscaled_xml (childBody,mogs_get_absolute_path(url),extension));
	}
}

void MogsRobotProperties::read_joints_xml(	std::vector < MogsRobotJoint, Eigen::aligned_allocator < MogsRobotJoint > > &List_Of_Joints,
                                            const mogs_string & xml,
                                            const mogs_string extension)
{
	mogs_string url = mogs_get_absolute_link(xml);
	QDomDocument *dom = new QDomDocument(url); // Création de l'objet DOM
	QFile xml_doc(url);// On choisit le fichier contenant les informations XML.
    if(!xml_doc.open(QIODevice::ReadOnly))// Si l'on n'arrive pas à ouvrir le fichier XML.
    {
         qDebug() <<"Erreur à l'ouverture du document XML:"<<url;
         qDebug()<<"Le document XML n'a pas pu être ouvert. Vérifiez que le nom est le bon et que le document est bien placé";
         return;
    }
    dom->setContent(&xml_doc);
    QDomElement root = dom->documentElement();
    QDomNode childJoints = root.firstChildElement ("Joints");
	assert (!childJoints.isNull());
	for (QDomElement childJoint = childJoints.toElement().firstChildElement ("Joint"); ! childJoint.isNull(); childJoint = childJoint.nextSiblingElement("Joint") )
		List_Of_Joints.push_back (load_new_joint_xml (childJoint, scalable_robot_, size_, weight_, power_,extension));

}

unsigned int MogsRobotProperties::SetFloatingBaseBody (const MogsRobotBody & body,
						       const mogs_string & body_name)
{
	assert (lambda.size () >= 0);

	// Add five zero weight bodies and append the given body last. Order of
	// the degrees of freedom is:
	// rx ry rz tx ty tz
	MogsRobotJoint floating_base_joint (Eigen::Matrix < double, 6, 1 > (0., 0., 0., 1., 0., 0.),
					Eigen::Matrix < double, 6, 1 > (0., 0., 0., 0., 1., 0.),
					Eigen::Matrix < double, 6, 1 > (0., 0., 0., 0., 0., 1.),
					Eigen::Matrix < double, 6, 1 > (1., 0., 0., 0., 0., 0.),
					Eigen::Matrix < double, 6, 1 > (0., 1., 0., 0., 0., 0.),
					Eigen::Matrix < double, 6, 1 > (0., 0., 1., 0., 0., 0.));


	unsigned int body_id = this->AddBody (0, Xtrans (Eigen::Matrix < double, 3, 1 > (0., 0., 0.)), floating_base_joint, body, body_name);

	floating_base_robot_ = true;
	return body_id;
}

bool MogsRobotProperties::SetRobotSphere (double radius, double mass)
{
	Init();
	// not usefull
	gravity = Eigen::Matrix < double, 3, 1 > (0., 0, -9.81);
	robot_type_ = "sphere";
	robot_name_ = "sphere";

	MogsRobotBody Bodysphere;
	Bodysphere.mGeom = new MogsGeometry();
	Bodysphere.mMass = mass;
	Bodysphere.mInertia =Eigen::Matrix < double, 3, 3 >::Identity()*2./5.*mass*radius*radius;

	MogsGeometry *geom = new MogsGeometry;
	Mesh *meh = new Mesh();
	sub_Mesh *smeh = new sub_Mesh(meh);

	Bodysphere.mGeom->convert_sphere_to_mesh(smeh,0,0,0,radius, 255, 0, 0);

	// add subMesh in Mesh and Mesh in MogsGeometry
	meh->subGeometries_.push_back(smeh);
	Bodysphere.mGeom-> add(*meh);

	MogsRobotJoint floating_base	  (Eigen::Matrix < double, 6, 1 > (0., 0., 0., 1., 0., 0.),
					Eigen::Matrix < double, 6, 1 > (0., 0., 0., 0., 1., 0.),
					Eigen::Matrix < double, 6, 1 > (0., 0., 0., 0., 0., 1.));


	this->AddBody (0, Xtrans (Eigen::Matrix < double, 3, 1 > (0., 0., 0.)), floating_base, Bodysphere, "body_sphere");

        for(int i=0;i<3;i++)
        {
//             qmin_.insert(qmin_.begin(),-1e20);
//             qmax_.insert(qmax_.begin(),1e20);
//             dqmax_.insert(dqmax_.begin(),1e20);
//             torquemax_.insert(torquemax_.begin(),1e20);
	      floating_base.data_.qmin.push_back(-1e20);
	      floating_base.data_.qmax.push_back(1e20);
	      floating_base.data_.dqmax.push_back(1e20);
	      floating_base.data_.torquemax.push_back(1e20);
        }


	return true;
}

bool MogsRobotProperties::SetRobotFile (const mogs_string & name, bool force_fixed)
{
    std::cout<<"SetRobotFile"<<std::endl;
    number_mimic_joint_ = 0;
    if (!xml_read_)
	{
        if(name.endsWith(".xml"))
        {
            qDebug()<<"Reading xml of the robot";
            return SetRobotXml (name, force_fixed);
        }else if(name.endsWith(".urdf"))
        {
            qDebug()<<"Reading urdf of the robot";
            return SetRobotUrdf(name);
        }
	}
	return false;
}

bool MogsRobotProperties::SetRobotUrdf (	const mogs_string & urdf_file)
{
  std::cout<<"SetRobotUrdf"<<std::endl;
    mogs_string path = mogs_get_absolute_link(urdf_file);
    // test the if the xml fit the xsd
    QUrl schemaUrl(XSD_URDF_ROBOT_FILE);
    QXmlSchema schema;
    schema.load(schemaUrl);
    if (schema.isValid()) {
        QFile file(urdf_file);
        file.open(QIODevice::ReadOnly);
        QXmlSchemaValidator validator(schema);
#ifdef DEBUG
        qDebug()<<" xsd is valid";
#endif
        if (! validator.validate(QUrl(QString("file://")+path)))
        {
            qDebug() << "instance document "<< QString("file://")+path << " is invalid";
            qDebug() << "with scheme xsd : "<< XSD_URDF_ROBOT_FILE;
            qDebug() <<" FIXME !!!!";
        }
#ifdef DEBUG
        else
            qDebug() << "instance document "<< QString("file://")+path << " is valid";
#endif
    }else
    {
        qDebug()<<" Error the xsd "<< XSD_URDF_ROBOT_FILE<< " is not valid";
//         exit(0);
    }
    std::vector < MogsRobotBody, Eigen::aligned_allocator < MogsRobotBody > > List_Of_Bodies;
    std::vector < MogsRobotJoint, Eigen::aligned_allocator < MogsRobotJoint > >List_Of_Joints;

    Init ();

    gravity = Eigen::Matrix < double, 3, 1 > (0., 0, -9.81);

    QDomDocument *dom = new QDomDocument(urdf_file); // Création de l'objet DOM
    QFile xml_doc(urdf_file);// On choisit le fichier contenant les informations XML.
    if(!xml_doc.open(QIODevice::ReadOnly))// Si l'on n'arrive pas à ouvrir le fichier XML.
    {
         qDebug() <<"Erreur à l'ouverture du document URDF : "<<urdf_file;
         qDebug()<<"Le document URDF n'a pas pu être ouvert. Vérifiez que le nom est le bon et que le document est bien placé";
         return 0;
    }
    dom->setContent(&xml_doc);
    QDomElement root = dom->documentElement();
    assert (!root.isNull());

    floating_base_robot_ = false;
    mogs_string  s_free_base = root.attribute ("base");
    if (s_free_base == "free")
        floating_base_robot_ = true;
    else if (s_free_base == "fixed")
        floating_base_robot_ = false;

    read_bodies_urdf(List_Of_Bodies,urdf_file);
    read_joints_urdf(List_Of_Joints,urdf_file);

    return create_structure(List_Of_Bodies,List_Of_Joints);
}

bool MogsRobotProperties::SetRobotXml (	const mogs_string & xml_file,
                                        bool force_fixed)
{
    mogs_string path = mogs_get_absolute_link(xml_file);
    mogs_string dir = mogs_get_absolute_path(xml_file);

    // test the if the xml fit the xsd
    QUrl schemaUrl(XSD_ROBOT_FILE);
    QXmlSchema schema;
    schema.load(schemaUrl);
    if (schema.isValid()) {
        QFile file(xml_file);
        file.open(QIODevice::ReadOnly);
        QXmlSchemaValidator validator(schema);
#ifdef DEBUG
        qDebug()<<" xsd is valid";
#endif
        if (! validator.validate(QUrl(QString("file://")+path)))
            qDebug() << "instance document "<< QString("file://")+path << " is invalid";
#ifdef DEBUG
        else
            qDebug() << "instance document "<< QString("file://")+path << " is valid";
#endif
    }else
    {
        qDebug()<<" Error the xsd "<< XSD_ROBOT_FILE<< " is not valid";
        exit(0);
    }

    Init ();

    MogsRobotBody body_a, body_b, body_c;
    MogsRobotJoint joint_a, joint_b, joint_c;

    gravity = Eigen::Matrix < double, 3, 1 > (0., 0, -9.81);

    QDomDocument *dom = new QDomDocument(xml_file); // Création de l'objet DOM
    QFile xml_doc(xml_file);// On choisit le fichier contenant les informations XML.
    if(!xml_doc.open(QIODevice::ReadOnly))// Si l'on n'arrive pas à ouvrir le fichier XML.
    {
         qDebug() <<"Erreur à l'ouverture du document XML : "<<xml_file;
         qDebug()<<"Le document XML n'a pas pu être ouvert. Vérifiez que le nom est le bon et que le document est bien placé";
         return 0;
    }
    dom->setContent(&xml_doc);
    QDomElement root = dom->documentElement();

    assert (!root.isNull());
    scalable_robot_ = false;
    mogs_string scalable = root.attribute ("scalable");
    if (scalable == "yes")
        scalable_robot_ = true;
    else if (scalable == "YES")
        scalable_robot_ = true;
    else if (scalable == "no")
        scalable_robot_ = false;
    else if (scalable == "NO")
        scalable_robot_ = false;
    else
    {
        std::cerr << " the attribute scalable must be set to yes/no in the XML file:" << xml_file.toStdString() << " for the MultiBody tag" << std::endl;
        return false;
    }
    floating_base_robot_ = false;
    mogs_string  s_free_base = root.attribute ("base");
    if (s_free_base == "free")
        floating_base_robot_ = true;
    else if (s_free_base == "fixed")
        floating_base_robot_ = false;
    else
    {
        std::cerr << " the attribute base must be set to free or fixed in the XML file:" << xml_file.toStdString() << " for the MultiBody tag" << std::endl;
        return false;
    }

    if (force_fixed)
        floating_base_robot_ = false;

    QDomNode robot_type = root.firstChildElement ("Robot_type");
    assert (!robot_type.isNull());
    robot_type_ = robot_type.toElement().text().simplified();
    if (robot_type_ == "")
    {
        std::cerr << " You must specify a Robot_type !!" << std::endl;
        return false;
    }

    std::vector < MogsRobotBody, Eigen::aligned_allocator < MogsRobotBody > > List_Of_Bodies;
    std::vector < MogsRobotJoint, Eigen::aligned_allocator < MogsRobotJoint > >List_Of_Joints;

    read_bodies_xml(List_Of_Bodies,xml_file);

    QDomElement childRobots = root.firstChildElement ("Robots");
    for (QDomElement childRobot = childRobots.firstChildElement("Robot"); !childRobot.isNull(); childRobot = childRobot.nextSiblingElement("Robot") )
    {
        mogs_string prefix = childRobot.toElement().attribute("prefix") + mogs_string("_");
        QDomElement file = childRobot.firstChildElement ("File").toElement ();
        assert(!file.isNull());
        mogs_string filename = file.text().simplified();
        QFileInfo info(filename);

        if(info.completeSuffix() == "xml")
        {
            read_bodies_xml(List_Of_Bodies,dir+"/"+filename,prefix);
            read_joints_xml(List_Of_Joints,dir+"/"+filename,prefix);
        }else if (info.completeSuffix() =="urdf")
        {
            read_bodies_urdf(List_Of_Bodies,dir+"/"+filename,prefix);
            read_joints_urdf(List_Of_Joints,dir+"/"+filename,prefix);
        }
    }
    read_joints_xml(List_Of_Joints,xml_file);

    return create_structure(List_Of_Bodies,List_Of_Joints);
}

bool MogsRobotProperties::create_structure(std::vector < MogsRobotBody, Eigen::aligned_allocator < MogsRobotBody > > List_Of_Bodies,
                                           std::vector < MogsRobotJoint, Eigen::aligned_allocator < MogsRobotJoint > >List_Of_Joints)
{
    int nbjoint = List_Of_Joints.size ();
    unsigned int parent_id;
    if (floating_base_robot_)
    {
        parent_id = SetFloatingBaseBody (List_Of_Bodies[0], List_Of_Bodies[0].name_);
        for(int i=0;i<6;i++)
        {
            qmin_.insert(qmin_.begin(),-1e20);
            qmax_.insert(qmax_.begin(),1e20);
            dqmax_.insert(dqmax_.begin(),1e20);
            torquemax_.insert(torquemax_.begin(),1e20);
        }
    }
    else
    {
        std::cout<<"nb body = "<< List_Of_Bodies.size()<<std::endl;
        std::cout<<"List_Of_Bodies[0].name_ = "<< List_Of_Bodies[0].name_.toStdString() <<std::endl;
        AddBodyFixedJoint (0, SpatialTransform < double > (), MogsRobotJoint(JointTypeFixed), List_Of_Bodies[0],  List_Of_Bodies[0].name_);
    }

    bool still_to_test = true;
    std::vector<bool> to_test(nbjoint);
    for (int i = 0; i < nbjoint; i++)
        to_test[i] = true;
    int count = 0;
    while(still_to_test)
    {
        still_to_test = false;
        for (int i = 0; i < nbjoint; i++)	if(to_test[i])
        {
            additionnal_data data = List_Of_Joints[i].data_;
            // get the transformation
            Eigen::Matrix < double, 3, 1 > translation;
            translation (0) = data.sparam (0);
            translation (1) = data.sparam (1);
            translation (2) = data.sparam (2);
            SpatialTransform < double >joint_frame = Xrotx<double> (data.sparam (3)) * Xroty<double> (data.sparam (4)) * Xrotz<double> (data.sparam (5)) * Xtrans < double >(translation);

            /** Set the parent id	*/
            parent_id = GetBodyId (data.dad_body_name);
            if (parent_id == -1)
            {
                if (count > nbjoint)
                {
                    std::cerr << " Error in " << __FILE__ << " at line " << __LINE__ << " the body \"" << data.dad_body_name.toStdString() << "\" was not defined or is not linked (as a child) to existing body." << std::endl;
                    exit(0);
                }
            }else
            {
                /** Set child id	*/
                int child_id = -1;
                for (unsigned int j = 0; j < List_Of_Bodies.size (); j++)
                    if (List_Of_Bodies[j].name_ == data.child_body_name)
                    {
                        child_id = j;
                        break;
                    }
                if (child_id == -1)
                {
                    std::cerr << "Error in " << __FILE__ << " at line " << __LINE__ << " the name: " << data.child_body_name.toStdString() << " is not known." << std::endl;
                    std::cerr << "Known bodies : " << std::endl;
                    for (unsigned int j = 0; j < List_Of_Bodies.size (); j++)
                        qDebug()<< List_Of_Bodies[j].name_ ;
                    exit (0);
                }
                unsigned int body_id = AddBody (parent_id, joint_frame,
                                List_Of_Joints[i],
                                List_Of_Bodies[child_id],
                                List_Of_Bodies[child_id].name_);
                to_test[i] = false;
                still_to_test = true;
            }
        }
        count++;
    }
    xml_read_ = true;
    
    // update min and max values
    unsigned int nb = mJoints.size();
    for (unsigned int i=1;i<nb;i++)
    {
        info_mimic tmp;
        const additionnal_data& d = mJoints[i].data_;
        // We assume that mimic consider only one dof joint
	if ( d.is_mimic)
        {
            tmp.source_id = get_joint_id(d.name_mimic);
            tmp.target_id = i-1;
            tmp.multiplier = d.multiplier_mimic;
            tmp.offset = d.offset_mimic;
        }else
        {
            tmp.source_id = qmin_.size();
            tmp.target_id = i-1;
            tmp.multiplier = 1.0;
            tmp.offset = 0.0;
            
            unsigned int nbd = d.qmin.size();
            for (int j=0;j<nbd;j++)
            {
                qmin_.push_back(d.qmin[j]);
                qmax_.push_back(d.qmax[j]);
                dqmax_.push_back(d.dqmax[j]);
                torquemax_.push_back(d.torquemax[j]);	    
            }            
        }
	info_mimic_.push_back(tmp);
    }
    
    return true;
}


MogsRobotBody MogsRobotProperties::load_new_body_scaled_xml (	QDomNode n,
                                                                double size, double weight,
                                                                const mogs_string  & path,
                                                                const mogs_string & body_prefix)
{
	assert (!n.isNull());
	Eigen::Matrix < double, 3, 1 > COM, Gyration;

	double Mass = 0.;
	COM (0) = 0.0;
	Gyration (0) = 0.0;
	COM (1) = 0.0;
	Gyration (1) = 0.0;
	COM (2) = 0.0;
	Gyration (2) = 0.0;

	QDomElement El1 = n.firstChildElement ("Label").toElement ();
	assert (!El1.isNull());
	mogs_string name = body_prefix+ El1.text().simplified();
	mogs_string Aunit;

	/**	If the MogsRobotProperties is scaled	*/
	bool length_defined = false;
	double length;
	/**	Get the mass	*/
	El1 = n.firstChildElement ("Mass").toElement ();
	assert (!El1.isNull());
	Aunit = El1.attribute ("unit");
	if (Aunit == "ratio")
		Mass = El1. text().toDouble() * weight;
	else
	{
		std::cerr << " For the Body " << name.toStdString() << ", the attribute \"unit\" must be ratio and is " << Aunit.toStdString() << std::endl;
		exit (0);
	}

	/**	Get the length of the body	*/
	El1 = n.firstChildElement ("Length").toElement ();
	assert (!El1.isNull());
	Aunit = El1.attribute ("unit");
	if (Aunit == "ratio_size")
	{
		length = El1.text().toDouble()* size;
		length_defined = true;
	}
	else
	{
		std::cerr << " For the Body " << name.toStdString() << ", the attribute \"unit\" of the tag \"Length\" must be ratio_size and is " << Aunit.toStdString() << std::endl;
		exit (0);
	}

	/**	Get the Center Of Mass	*/
	El1 = n.firstChildElement ("CoM").toElement ();
	assert (!El1.isNull());
	mogs_string stmp = El1.text().simplified();
	double tval;
	std::istringstream smallData (stmp.toStdString(), std::ios_base::in);
	Aunit = El1.attribute ("unit");
	if (Aunit =="ratio_size")
	{
		for (int i = 0; i < 3; i++)
		{
			smallData >> tval;
			COM (i) = tval * size;
		}
	}
	else if (Aunit == "ratio_length")
	{
		if (!length_defined)
		{
			std::cerr << " For the Body " << name.toStdString() << ", You specified the CoM regarding the length but you did not defined the value <Length>" << std::endl;
			exit (0);
		}
		for (int i = 0; i < 3; i++)
		{
			smallData >> tval;
			COM (i) = tval * length;
		}
	}
	else if (Aunit == "m")
	{
		for (int i = 0; i < 3; i++)
		{
			smallData >> tval;
			COM (i) = tval;
		}
	}
	else if (Aunit == "mm")
	{
		for (int i = 0; i < 3; i++)
		{
			smallData >> tval;
			COM (i) = tval * 1e-3;
		}
	}
	else
	{
		std::cerr << " For the CoM of the  Body " << name.toStdString() << ", the attribute \"unit\" is not known\n Please use m, mm (ratio_size or ratio_length for scalable models), send an issue and wait for an update or modify the code in file " << __FILE__ << " at line " << __LINE__ << std::endl;
		exit (0);
	}

	/**	Get the Inertia through the Gyration	*/
	El1 = n.firstChildElement ("Gyration").toElement ();
	assert (!El1.isNull());
	Aunit = El1.attribute ("unit");
	if (Aunit == "ratio")
	{
		mogs_string stmp = El1.text().simplified();
		double tval;
		std::istringstream smallData (stmp.toStdString(), std::ios_base::in);
		for (int i = 0; i < 3; i++)
		{
			smallData >> tval;
			Gyration (i) = tval;
		}
	}
	else
	{
		std::cerr << " For the gyration of Body " << name.toStdString()<< ", the attribute \"unit\" is not known\n Please use ration, send an issue and wait for an update or modify the code in file " << __FILE__ << " at line " << __LINE__ << std::endl;
		exit (0);
	}

	/**	Get the geometry	*/
	MogsGeometry *geom = new MogsGeometry ();
	El1 = n.firstChildElement ("Geometry").toElement ();
	geom->load (El1,length,path);

	return MogsRobotBody (name, Mass, COM, length, Gyration, geom);
}

MogsRobotBody MogsRobotProperties::load_new_body_unscaled_urdf(	QDomNode n,
                                                                const mogs_string & path,
                                                                const mogs_string & body_prefix)
{
	assert (!n.isNull());
	Eigen::Matrix < double, 3, 1 > COM;
	Eigen::Matrix < double, 3, 3 > Inertia=Eigen::Matrix < double, 3, 3>::Zero();
        double length = 0.;
	double Mass = 0.;
	COM (0) = 0.0;
	COM (1) = 0.0;
	COM (2) = 0.0;

	MogsGeometry *geom = NULL;
        QDomElement El = n.toElement();
	mogs_string name = body_prefix + El.attribute("name");

        QDomElement Elinertia =  El.firstChildElement("inertial");
        if(!Elinertia.isNull())
        {
            /**	Get the mass	*/
            QDomElement Elmass = Elinertia.firstChildElement ("mass");
            assert (!Elmass.isNull());
            Mass = Elmass.attribute("value").toDouble();


            /**	Get the Center Of Mass	*/
            QDomElement ElCOM = Elinertia.firstChildElement ("origin").toElement ();
            if (!ElCOM.isNull())
            {
    //             qDebug()<<"Warning no origin for "<<name<<" considering 0 0 0 ";
                for (int i = 0; i < 3; i++)    COM(i) = 0.;
            }else
            {
                mogs_string stmp = ElCOM.attribute("xyz");
                double tval;
                std::istringstream smallData (stmp.toStdString(), std::ios_base::in);
                for (int i = 0; i < 3; i++)
                {
                    smallData >> tval;
                        COM (i) = tval;
                }

            }
    //        qDebug()<<"Link = "<< name <<"  mass = "<<Mass<<"  COM = "<< COM(0)<<" "<<COM(1)<<" "<<COM(2);

            QString rpy = ElCOM.attribute("rpy","0 0 0");
            if( rpy != "0 0 0" && rpy !="")
            {
                qDebug()<<"Error the case where rpy is not \"0 0 0\" is not planned";
                qDebug()<<"rpy "<<rpy;
                exit(0);
            }

            QDomElement Elin = Elinertia.firstChildElement ("inertia").toElement ();
            Inertia(0,0) = Elin.attribute("ixx").toDouble();
            Inertia(0,1) = Inertia(1,0) = Elin.attribute("ixy").toDouble();
            Inertia(0,2) = Inertia(2,0) = Elin.attribute("ixz").toDouble();
            Inertia(1,1) = Elin.attribute("iyy").toDouble();
            Inertia(1,2) = Inertia(2,1) = Elin.attribute("iyz").toDouble();
            Inertia(2,2) = Elin.attribute("izz").toDouble();
    //        std::cout<<" inertie = "<< Inertia<<std::endl;
        }
        /**     Get the geometry         */
        QDomElement Elvisual =  El.firstChildElement("visual");
        Eigen::Matrix<double,3,1> pos_visual(0,0,0), rot_visual(0,0,0);
        if(!Elvisual.isNull())
        {
            QDomElement Elorigin =  Elvisual.firstChildElement("origin");
            if(!Elorigin.isNull())
            {
                QString rot = Elorigin.attribute("rpy");
                QString trans = Elorigin.attribute("xyz");
                std::istringstream smallData (rot.toStdString(), std::ios_base::in);
                std::istringstream smallData2 (trans.toStdString(), std::ios_base::in);
                for (int i = 0; i < 3; i++)
                {
                    smallData >>rot_visual(i);
                    smallData2 >>pos_visual(i);
                }
            }
            QDomElement Elgeometry =Elvisual.firstChildElement("geometry");
            QDomElement Elcylinder =  Elgeometry.firstChildElement("cylinder");
            if(!Elcylinder.isNull())
            {
                if (!geom)
                    geom = new MogsGeometry ();
                QString sl= Elcylinder.attribute("length","0.1");
                QString sr= Elcylinder.attribute("radius","0.1");
                double l = sl.toDouble();
                double r = sr.toDouble();
                geom->add_cylinder(r,l);
                
            }
            
            QDomElement Elmesh =  Elgeometry.firstChildElement("mesh");
            if(!Elmesh.isNull())
            {
    //            Mesh m(path + Elmesh.attribute("filename"));
                if (!geom)
                    geom = new MogsGeometry ();
                //geom->add(m);
                geom->load(path + Elmesh.attribute("filename"));

                QString s = Elmesh.attribute("scale","1 1 1");
                std::istringstream smallData (s.toStdString(), std::ios_base::in);
                Eigen::Matrix<double,3,1> scale;
                for (int i = 0; i < 3; i++)
                    smallData>>scale(i);
                geom->scale(scale);
            }
            if (geom)
                geom->move(pos_visual,rot_visual);
        }
    #ifdef DEBUG
        else
            qDebug()<<"cannot find visual for "<<name;
    #endif

	return MogsRobotBody (name, Mass, COM, Inertia, length, geom);
}

MogsRobotBody MogsRobotProperties::load_new_body_unscaled_xml (	QDomNode n,
                                                                const mogs_string & path,
                                                                const mogs_string & body_prefix)
{
	assert (!n.isNull());
	Eigen::Matrix < double, 3, 1 > COM;
	Eigen::Matrix < double, 3, 3 > Inertia;

	double Mass = 0.;
	COM (0) = 0.0;
	COM (1) = 0.0;
	COM (2) = 0.0;

	QDomElement El1 = n.firstChildElement ("Label").toElement ();
	assert (!El1.isNull());
	mogs_string name = body_prefix + El1.text().simplified();
	mogs_string Aunit;

	/**	Get the mass	*/
	El1 = n.firstChildElement ("Mass").toElement ();
	assert (!El1.isNull());
	Aunit = El1.attribute ("unit");
	if (Aunit == "kg")
		Mass = El1.text().toDouble();
	else
	{
		std::cerr << " For the Mass of Body " << name.toStdString() << ", the attribute \"unit\" is not known\n Please use kg, send an issue and wait for an update or modify the code in file " << __FILE__ << " at line " << __LINE__ << std::endl;
		exit (0);
	}

	/**	Get the length	*/
	double length;
	El1 = n.firstChildElement ("Length").toElement ();
	if (!El1.isNull())
	{
		Aunit = El1.attribute ("unit");
		if (Aunit == "m")
			length = El1.text().toDouble();
		else if (Aunit == "mm")
			length = El1.text().toDouble()* 1e-3;
		else
		{
			std::cerr << " For the length of Body " << name.toStdString() << ", the attribute \"unit\" is not known\n Please use m or mm, send an issue and wait for an update or modify the code in file " << __FILE__ << " at line " << __LINE__ << std::endl;
			exit (0);
		}
	}else
	{
// 			std::cout<<" The length is not defined. For the moment we do nothing"<<std::endl;
		length = 0.0;
	}

/**	Get the Center Of Mass	*/
	El1 = n.firstChildElement ("CoM").toElement ();
	assert (!El1.isNull());
	mogs_string stmp = El1.text().simplified();
	double tval;
	std::istringstream smallData (stmp.toStdString(), std::ios_base::in);
	Aunit = El1.attribute ("unit");
	if (Aunit ==  "m")
		{
			for (int i = 0; i < 3; i++)
			{
				smallData >> tval;
				COM (i) = tval;
			}
		}
	else if (Aunit == "mm")
		{
			for (int i = 0; i < 3; i++)
			{
				smallData >> tval;
				COM (i) = tval * 1e-3;
			}
		}
	else
		{
			std::cerr << " For the CoM of the  Body " << name.toStdString()
				<< ", the attribute \"unit\" is not known\n Please use m, mm (ratio_size or ratio_length for scalable models), send an issue and wait for an update or modify the code in file " << __FILE__ << " at line " << __LINE__ << std::endl;
			exit (0);
		}

/**	Get the Inertia Matrix	*/
	El1 = n.firstChildElement ("Inertia").toElement ();
	assert (!El1.isNull());

	stmp = El1.text().simplified();
	Aunit = El1.attribute ("unit");
	double mul;
	if (Aunit == "kg/s2")
		mul = 1;
	else
		{
			std::cerr << " For the Inertia of Body " << name.toStdString() << ", the attribute \"unit\" is not known\n Please use kg, send an issue and wait for an update or modify the code in file " << __FILE__ << " at line " << __LINE__ << std::endl;
			exit (0);
		}

	std::istringstream smallData2 (stmp.toStdString(), std::ios_base::in);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			{
				smallData2 >> tval;
				Inertia (i, j) = tval * mul;
			}

/**     Get the geometry         */
	MogsGeometry *geom = new MogsGeometry ();
	El1 = n.firstChildElement ("Geometry").toElement ();
	geom->load (El1,path);
	return MogsRobotBody (name, Mass, COM, Inertia, length, geom);
}

MogsRobotJoint MogsRobotProperties::load_new_joint_urdf(QDomNode n,
                                                        const mogs_string &joint_extension )
{
    QDomElement Element = n.toElement();

    additionnal_data data;
    assert (!Element.isNull());

    data.name = joint_extension + Element.attribute("name");
	mogs_string  type = Element.attribute ("type");

    QDomElement ElParent = Element.firstChildElement("parent");
	data.dad_body_name = joint_extension + ElParent.attribute ("link");

	QDomElement ElChild = Element.firstChildElement("child");
	data.child_body_name = joint_extension + ElChild.attribute ("link");

    QDomElement ElAxis = Element.firstChildElement("axis");
    QString axis = ElAxis.attribute ("xyz");

//	qDebug()<<"Joint "<< data.name<<"  type ="<< type<<" parent = "<< data.dad_body_name<<"  child = "<< data.child_body_name<< "  axis = "<<axis<<endl;

	// type used by the library
	JointType RBDL_type;
	MogsRobotJoint *New_Joint;

	if (type == "fixed" || type == "continuous")
	{
	    // FIXME for the moment continuous is fixed
		RBDL_type = JointTypeFixed;
		New_Joint = new MogsRobotJoint (RBDL_type);
	}
	else if (type == "revolute" || type == "prismatic")
	{
		if (type == "revolute")
			RBDL_type = JointTypeRevolute;
		else
			RBDL_type = JointTypePrismatic;

		double ratio= 1.0;
		Eigen::Matrix < double, 3, 1 > joint_axis;
		joint_axis (0) = 0;
		joint_axis (1) = 0;
		joint_axis (2) = 0;
		if (axis == "1 0 0" || axis == "1.0 0 0")
			joint_axis (0) = 1.0;
		else if (axis == "0 1 0" || axis == "0 1.0 0")
			joint_axis (1) = 1.0;
		else if (axis == "0 0 1" || axis == "0 0 1.0")
			joint_axis (2) = 1.0;
		else if (axis == "-1 0 0" || axis == "-1.0 0 0")
		{
			ratio = -1.0;
			joint_axis (0) = 1.0;
		}
		else if (axis == "0 -1 0" || axis == "0 -1.0 0")
		{
			ratio = -1.0;
			joint_axis (1) = 1.0;
		}
                else if (axis == "0 0 -1" || axis == "0 0 -1.0")
		{
			ratio = -1.0;
			joint_axis (2) = 1.0;
		}
		else
		{
                        std::istringstream smallData (axis.toStdString(), std::ios_base::in);
                        for (int i = 0; i < 3; i++)
                        {
                            double tval;
                            smallData >> tval;
                            joint_axis(i) = tval;
                        }
                            
// 			std::cerr << "Error in " << __FILE__ << " at line " << __LINE__ << " the axis " << axis.toStdString() << " was not planned!" << std::endl;
// 			exit (0);
		}
		New_Joint = new MogsRobotJoint (RBDL_type, joint_axis,ratio);

                QDomElement El1 = n.firstChildElement("limit");
                /**	limits	*/
                if (!El1.isNull())
                {
                    double tau_max = El1.attribute("effort").toDouble();
                    double q_min = El1.attribute("lower").toDouble();
                    double q_max = El1.attribute("upper").toDouble();
                    double dq_max = El1.attribute("velocity").toDouble();

                    // store the qmin
                    data.qmin.push_back(q_min);
                    data.qmax.push_back(q_max);
                    data.dqmax.push_back(dq_max);
                    data.torquemax.push_back(tau_max);
                    
        // 	    qDebug()<<"Joint "<< data.name<<" q_min = "<< q_min <<" q_max =" << q_max ;
                }
                else
                {
                    data.qmin.push_back(-1e20);
                    data.qmax.push_back(1e20);
                    data.dqmax.push_back(1e20);
                    data.torquemax.push_back(1e20);
                }
	}
	else
	{
		std::cerr << " the type of joint is not defined  type = " << type.toStdString() << std::endl;
		exit (0);
	}
	data.is_mimic = false;
	QDomElement Elmimic = Element.firstChildElement("mimic");
        if(!Elmimic.isNull())
        {
                number_mimic_joint_ ++;
                data.is_mimic = true;
                data.name_mimic = Elmimic.attribute("joint").simplified();
                data.multiplier_mimic = Elmimic.attribute("multiplier").toDouble();
                data.offset_mimic = Elmimic.attribute("offset").toDouble();
        }
	

    /**	Get the static parameter	*/
	QDomElement El1 = Element.firstChildElement("origin");
	assert (!El1.isNull());
	QString rot = El1.attribute("rpy");
	QString trans = El1.attribute("xyz");
	double tval;
	std::istringstream smallData (rot.toStdString(), std::ios_base::in);
	for (int i = 0; i < 3; i++)
	{
		smallData >> tval;
		data.sparam (i+3) = tval;
	}
	std::istringstream smallData2 (trans.toStdString(), std::ios_base::in);
	for (int i = 0; i < 3; i++)
	{
		smallData2 >> tval;
		data.sparam (i) = tval;
	}
	New_Joint->set_additionnal_data (data);

    return *New_Joint;
}

MogsRobotJoint MogsRobotProperties::load_new_joint_xml (QDomNode n,
                                                        bool scalable,
                                                        double size,
                                                        double weight,
                                                        double efficiency,
                                                        const mogs_string &joint_extension)
{
	additionnal_data data;
	QDomElement Element = n.toElement();
	assert (!Element.isNull());
	// temporary values
	data.dad_body_name = joint_extension + Element.attribute ("innerId");
	data.child_body_name = joint_extension + Element.attribute ("outerId");
	mogs_string  type = Element.attribute ("type");

	double max_torque = 0.1 * size * weight * efficiency;

	QDomElement El1 = n.firstChildElement ("Label").toElement ();
	assert (!El1.isNull());
	data.name = joint_extension + El1.text().simplified();
	int nbdof;

	// type used by the library
	JointType RBDL_type;
	MogsRobotJoint *New_Joint;

	if (type == "fixed")
	{
		nbdof = 0;
		RBDL_type = JointTypeFixed;
		New_Joint = new MogsRobotJoint (RBDL_type);
	}
	else if (type == "revolute" || type == "prismatic")
	{
		nbdof = 1;
		if (type == "revolute")
			RBDL_type = JointTypeRevolute;
		else
			RBDL_type = JointTypePrismatic;

		double ratio= 1.0;
		Eigen::Matrix < double, 3, 1 > joint_axis;
		joint_axis (0) = 0;
		joint_axis (1) = 0;
		joint_axis (2) = 0;
		mogs_string  axis = Element.attribute ("axis");
		if (axis == "x" || axis == "X")
			joint_axis (0) = 1.0;
		else if (axis == "y" || axis == "Y")
			joint_axis (1) = 1.0;
		else if (axis == "z" || axis == "Z")
			joint_axis (2) = 1.0;
		else if (axis == "-x" || axis == "-X")
		{
			ratio = -1.0;
			joint_axis (0) = 1.0;
		}
		else if (axis == "-y" || axis == "-Y")
		{
			ratio = -1.0;
			joint_axis (1) = 1.0;
		}
		else if (axis == "-z" || axis == "-Z")
		{
			ratio = -1.0;
			joint_axis (2) = 1.0;
		}
		else
		{
			std::cerr << "Error in " << __FILE__ << " at line " << __LINE__ << " the axis " << axis.toStdString() << " was not planned!" << std::endl;
			exit (0);
		}
		New_Joint = new MogsRobotJoint (RBDL_type, joint_axis,ratio);
	}
	else if (type == "spherical")
	{
		nbdof = 3;
		RBDL_type = JointType3DoF;
		New_Joint = new MogsRobotJoint (Eigen::Matrix < double, 6, 1 > (1, 0, 0, 0, 0, 0), Eigen::Matrix < double, 6, 1 > (0, 1, 0, 0, 0, 0), Eigen::Matrix < double, 6, 1 > (0, 0, 1, 0, 0, 0));
	}
	else if (type == "eulerXZY")
	{
		nbdof = 3;
		RBDL_type = JointType3DoF;
		New_Joint = new MogsRobotJoint (Eigen::Matrix < double, 6, 1 > (1, 0, 0, 0, 0, 0), Eigen::Matrix < double, 6, 1 > (0, 0, 1, 0, 0, 0), Eigen::Matrix < double, 6, 1 > (0, 1, 0, 0, 0, 0));
	}
	else
	{
		std::cerr << " the type of joint is not defined  type = " << type.toStdString() << std::endl;
		exit (0);
	}

    /**	Get the static parameter	*/
	El1 = n.firstChildElement ("StaticParameters").toElement ();
	assert (!El1.isNull());
	mogs_string  stmp = El1.text().simplified();
	double tval;
	std::istringstream smallData (stmp.toStdString(), std::ios_base::in);
	for (int i = 0; i < 6; i++)
	{
		smallData >> tval;
		data.sparam (i) = tval;
	}
	// Deal with the unit
	mogs_string  AunitT = El1.attribute ("unit_translation");
	if (AunitT == "m")
	{
		// do nothing
	}
	else if (AunitT == "mm")
	{
		for (int i = 0; i < 3; i++)
			data.sparam (i) *= 1e-3;
	}
	else if (scalable && AunitT == "ratio_size")
	{
		for (int i = 0; i < 3; i++)
			data.sparam (i) *= size;
	}
	else
	{
		std::cerr << " The static parameter must have a unit_translation of type m, mm or ratio_size (for scalable model) and you defined:" << AunitT.toStdString() << std::endl;
		exit (0);
	}
	mogs_string  AunitR = El1.attribute ("unit_rotation");
	if (AunitR == "radian")
	{
		// do nothing
	}
	else if (AunitR == "degree")
	{
		for (int i = 3; i < 6; i++)
			data.sparam (i) *= deg_to_rad;
	}
	else
	{
		std::cerr << " The static parameter must have a unit_rotation of type radian or degree and you defined:" << AunitR.toStdString() << std::endl;;
		exit (0);
	}
/**	Joint limits	*/
	El1 = n.firstChildElement ("PositionMin").toElement ();
	if (!El1.isNull())
	{
		Eigen::Matrix < double, Eigen::Dynamic, 1 > qmin(nbdof);
		stmp = El1.text().simplified();

		std::istringstream smallData1 (stmp.toStdString(), std::ios_base::in);
		for (int i = 0; i < nbdof; i++)
		{
			smallData1 >> tval;
			qmin (i) = tval;
		}
		mogs_string  Aunit = El1.attribute ("unit");
		if (Aunit == "radian")
		{
			// do nothing
		}
		else if (Aunit == "degree")
		{
			for (int i = 0; i < nbdof; i++)
				qmin (i) *= deg_to_rad;
		}
		else
		{
			std::cerr << " The PositionMin must have a unit of type radian or degree and you defined:" << Aunit.toStdString() << std::endl;;
			exit (0);;
		}

		// store the qmin
		for (int i = 0; i < nbdof; i++)
			data.qmin.push_back(qmin(i));
	}
	else
	{
		// store the qmin
		for (int i = 0; i < nbdof; i++)
			data.qmin.push_back(-1e20);
	}

	El1 = n.firstChildElement ("PositionMax").toElement ();
	if (!El1.isNull())
	{
		Eigen::Matrix < double, Eigen::Dynamic, 1 > qmax(nbdof);
		stmp = El1.text().simplified();

		std::istringstream smallData1 (stmp.toStdString(), std::ios_base::in);
		for (int i = 0; i < nbdof; i++)
		{
			smallData1 >> tval;
			qmax (i) = tval;
		}
		mogs_string  Aunit = El1.attribute ("unit");
		if (Aunit == "radian")
		{
			// do nothing
		}
		else if (Aunit == "degree")
		{
			for (int i = 0; i < nbdof; i++)
				qmax (i) *= deg_to_rad;
		}
		else
		{
			std::cerr << " The PositionMin must have a unit of type radian or degree and you defined:" << Aunit.toStdString() << std::endl;;
			exit (0);;
		}
		// store the qmax
		for (int i = 0; i < nbdof; i++)
			data.qmax.push_back(qmax(i));
	}
	else
	{
		for (int i = 0; i < nbdof; i++)
			data.qmax.push_back(1e20);
	}

/**	Joint Speed Limit	*/
	El1 = n.firstChildElement ("SpeedLimit").toElement ();
	if (!El1.isNull())
	{
		Eigen::Matrix < double, Eigen::Dynamic, 1 > dqmax(nbdof);
		stmp = El1.text().simplified();
		std::istringstream smallData4 (stmp.toStdString(), std::ios_base::in);
		for (int i = 0; i < nbdof; i++)
		{
			smallData4 >> tval;
			dqmax (i) = tval;
		}
		mogs_string  Aunit = El1.attribute ("unit");
		if (Aunit == "radian/s")
		{
			// do nothing
		}
		else if (Aunit == "degree/s")
		{
			for (int i = 0; i < nbdof; i++)
				dqmax(i) *= deg_to_rad;
		}
		else
		{
			std::cerr << " The SpeedLimit must have a unit of type radian/s or degree/s and you defined:" << Aunit.toStdString() << std::endl;;
			exit (0);
		}
		// store the qmax
		for (int i = 0; i < nbdof; i++)
			data.dqmax.push_back(dqmax(i));
	}
	else
	{
		for (int i = 0; i < nbdof; i++)
			data.dqmax.push_back(1e20);
	}

	/**	Joint Torque Limit	*/
	El1 = n.firstChildElement ("TorqueLimit").toElement ();
	if (!El1.isNull())
	{
		Eigen::Matrix < double, Eigen::Dynamic, 1 > Tmax(nbdof);
		stmp = El1.text().simplified();

		std::istringstream smallData5 (stmp.toStdString(), std::ios_base::in);
		for (int i = 0; i < nbdof; i++)
		{
			smallData5 >> tval;
			Tmax(i) = tval;
		}
		mogs_string  Aunit = El1.attribute ("unit");
		if (Aunit == "Nm")
		{
			// do nothing
		}
		else if (scalable && Aunit == "scale_max_torque")
		{
			for (int i = 0; i < nbdof; i++)
				Tmax(i) *= max_torque;
		}
		else
		{
			std::cerr << " The TorqueLimit must have a unit of type Nm or scale_max_torque (in case of scalable model) and you defined:" << Aunit.toStdString() << std::endl;
			exit (0);
		}
		// store the torque max
		for (int i = 0; i < nbdof; i++)
			data.torquemax.push_back(Tmax(i));
	}
	else
	{
		for (int i = 0; i < nbdof; i++)
			data.torquemax.push_back(1e20);
	}
        data.is_mimic = false;
	New_Joint->set_additionnal_data (data);
	return (*New_Joint);
}

void MogsRobotProperties::set_root_transformation(	const Eigen::Matrix < double, 3, 1 > &position,
							const Eigen::Matrix < double, 3, 1 > &rotation)
{
	X_base0 = SpatialTransform < double > (rotation, position );
//    std::cout<<"changed "<<position(0)<<" "<<position(1)<<" "<<position(2)<<std::endl;
}
