//     MogsTemplateRobotProperties.hxx
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

using namespace RigidBodyDynamics;
template < typename T >
MogsTemplateRobotProperties<T>::MogsTemplateRobotProperties (): MogsRobotProperties()
{

}

template < typename T >
MogsTemplateRobotProperties<T>::MogsTemplateRobotProperties (MogsTemplateRobotProperties<T> *MogsTemplateRobotProperties_in) :
MogsRobotProperties(MogsTemplateRobotProperties_in)
{
	int nb = MogsTemplateRobotProperties_in->a.size ();
	a.resize (nb);

	nb = MogsTemplateRobotProperties_in->c.size ();
	c.resize (nb);

	nb = MogsTemplateRobotProperties_in->v.size ();
	v.resize (nb);

	nb = MogsTemplateRobotProperties_in->pA.size ();
	pA.resize (nb);

	nb = MogsTemplateRobotProperties_in->U.size ();
	U.resize (nb);

	nb = MogsTemplateRobotProperties_in->d.size ();
	d.resize (nb);

	nb = MogsTemplateRobotProperties_in->u.size ();
	u.resize (nb);

	nb = MogsTemplateRobotProperties_in->f.size ();
	f.resize (nb);

	nb = MogsTemplateRobotProperties_in->Ic.size ();
	Ic.resize (nb);
	for (int i = 0; i < nb; i++)
		Ic[i] = MogsTemplateRobotProperties_in->Ic[i];

	nb = MogsTemplateRobotProperties_in->X_base.size ();
	X_base.resize (nb);
	for (int i = 0; i < nb; i++)
		X_base[i] = MogsTemplateRobotProperties_in->X_base[i];

	nb = MogsTemplateRobotProperties_in->X_lambda.size ();
	X_lambda.resize (nb);
	for (int i = 0; i < nb; i++)
		X_lambda[i] = MogsTemplateRobotProperties_in->X_lambda[i];

	nb = MogsTemplateRobotProperties_in->IA.size ();
	IA.resize (nb);
	for (int i = 0; i < nb; i++)
		IA[i] = MogsTemplateRobotProperties_in->IA[i].template cast < T > ();
        
        nb = getNDofWithMimic();
        q_.resize(nb);
        dq_.resize(nb);
        ddq_.resize(nb);
        tau_.resize(nb);        
}

template < typename T >
MogsTemplateRobotProperties<T>::MogsTemplateRobotProperties (MogsRobotProperties *RobotProperties_in) :
MogsRobotProperties(RobotProperties_in)
{
        unsigned int nb = getNBodies();
	a.resize (nb);
	c.resize (nb);
	v.resize (nb);
	pA.resize(nb);
	U.resize (nb);
	d.resize (nb);
	u.resize (nb);
	f.resize (nb);

	Ic.resize(nb);
	X_base.resize(nb);
	IA.resize (nb);
	X_lambda.resize(nb);
        
        nb = getNDofWithMimic();
        q_.resize(nb);
        dq_.resize(nb);
        ddq_.resize(nb);
        tau_.resize(nb);

	X_base[0] = X_base0;
}


template < typename T >
void MogsTemplateRobotProperties<T>::Init ()
{
	MogsRobotProperties::Init();

	X_lambda.push_back (SpatialTransform < T > ());

	// state information
	v.push_back (SpatialVectorZero);
	a.push_back (SpatialVectorZero);

	// Dynamic variables
	c.push_back (SpatialVectorZero);
	IA.push_back (SpatialMatrixIdentity);
	pA.push_back (SpatialVectorZero);
	U.push_back (SpatialVectorZero);

	u = Eigen::Matrix < T, Eigen::Dynamic, 1 >::Zero (1);
	d = Eigen::Matrix < T, Eigen::Dynamic, 1 >::Zero (1);

	f.push_back (SpatialVectorZero);
	Ic.push_back (SpatialRigidBodyInertia (0., Eigen::Matrix < double, 3, 1 >::Zero(), Eigen::MatrixXd::Zero (3, 3)));

	// Bodies
	X_base.push_back (SpatialTransform < T > ());
}

template < typename T >
unsigned int MogsTemplateRobotProperties<T>::AddBody (	const unsigned int parent_id,
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
	X_base.push_back (SpatialTransform < T > ());

	// state information
	v.push_back (SpatialVectorZero);
	a.push_back (SpatialVectorZero);

	S.push_back (joint.mJointAxes[0]);

	// Dynamic variables
	c.push_back (SpatialVectorZero);
	IA.push_back (body.mSpatialInertia.template cast <T> ());
	pA.push_back (SpatialVectorZero);
	U.push_back (SpatialVectorZero);

	d = Eigen::Matrix < T, Eigen::Dynamic, 1 >::Zero (mBodies.size ());
	u = Eigen::Matrix < T, Eigen::Dynamic, 1 >::Zero (mBodies.size ());

	f.push_back (SpatialVectorZero);
	Ic.push_back (SpatialRigidBodyInertia (body.mMass, body.mCenterOfMass, body.mInertia));

	return MogsRobotProperties::AddBody(parent_id, joint_frame,joint,body,body_name);
}

template < typename T >
SpatialTransform < T > MogsTemplateRobotProperties<T>::get_root_transformation()
{
	return X_base[0];
}

template < typename T >
void MogsTemplateRobotProperties<T>::set_root_transformation(	const Eigen::Matrix < double, 3, 1 > position,
								const Eigen::Matrix < double, 3, 1 > rotation)
{
	X_base0 = SpatialTransform < double > (rotation, position );
	X_base[0] = X_base0;
}

template < typename T >
bool MogsTemplateRobotProperties<T>::SetRobotXml (const mogs_string & name, bool force_fixed)
{
	if (!MogsRobotProperties::SetRobotFile(name,force_fixed))
		return false;
        
        unsigned int nb = getNBodies();
	a.resize (nb);
	c.resize (nb);
	v.resize (nb);
	pA.resize(nb);
	U.resize (nb);
	d.resize (nb);
	u.resize (nb);
	f.resize (nb);

	Ic.resize(nb);
	X_base.resize(nb);
	IA.resize (nb);
	X_lambda.resize(nb);
        
        nb = getNDofWithMimic();
        q_.resize(nb);
        dq_.resize(nb);
        ddq_.resize(nb);
        tau_.resize(nb);

	X_base[0] = X_base0;

	return true;
}

template < typename T >
void MogsTemplateRobotProperties<T>::SetJointValue( const Eigen::Matrix < T,Eigen::Dynamic, 1 > &Q,
                                                    const Eigen::Matrix < T,Eigen::Dynamic, 1 > &QDot,
                                                    const Eigen::Matrix < T,Eigen::Dynamic, 1 > &QDDot)
{
    if (number_mimic_joint_ == 0)
    {
            q_ = Q;
            dq_ = QDot;
            ddq_ = QDDot;
    }else
    {
        unsigned int nb = info_mimic_.size();
        for (int i=0;i<nb;i++)
        {
                const info_mimic& info = info_mimic_[i];
                q_(info.target_id) = Q(info.source_id) * info.multiplier + info.offset;
                dq_(info.target_id) = QDot(info.source_id) * info.multiplier;
                ddq_(info.target_id) = QDDot(info.source_id) * info.multiplier;
        }
    }
}

template < typename T >
void MogsTemplateRobotProperties<T>::SetForwardJointValue(  const Eigen::Matrix < T,Eigen::Dynamic, 1 > &Q,
                                                            const Eigen::Matrix < T,Eigen::Dynamic, 1 > &QDot,
                                                            const Eigen::Matrix < T,Eigen::Dynamic, 1 > &Tau)
{
    if (number_mimic_joint_ == 0)
    {
            q_ = Q;
            dq_ = QDot;
            tau_ = Tau;
    }else
    {    
        unsigned int nb = info_mimic_.size();
        for (int i=0;i<nb;i++)
        {
                const info_mimic& info = info_mimic_[i];
                q_(info.target_id) = Q(info.source_id) * info.multiplier + info.offset;
                dq_(info.target_id) = QDot(info.source_id) * info.multiplier;
                tau_(info.target_id) = Tau(info.source_id) / info.multiplier;
        }    
    }
}
                
template < typename T >
void MogsTemplateRobotProperties<T>::SetCustomJointValue(   const Eigen::Matrix < T,Eigen::Dynamic, 1 > *Q,
                                                            const Eigen::Matrix < T,Eigen::Dynamic, 1 > *QDot,
                                                            const Eigen::Matrix < T,Eigen::Dynamic, 1 > *QDDot)
{
    if (number_mimic_joint_ == 0)
    {
            q_ = *Q;
            if( QDot)
                dq_ = *QDot;
            if(QDDot)
                ddq_ = *QDDot;
    }else
    {        
        unsigned int nb = info_mimic_.size();
        for (int i=0;i<nb;i++)
        {
                const info_mimic& info = info_mimic_[i];
                q_(info.target_id) = (*Q)(info.source_id) * info.multiplier + info.offset;
        }
        
        if(QDot)
        for (int i=0;i<nb;i++)
        {
                const info_mimic& info = info_mimic_[i];
                dq_(info.target_id) = (*QDot)(info.source_id) * info.multiplier;
        }
        
        if(QDDot)
        for (int i=0;i<nb;i++)
        {
                const info_mimic& info = info_mimic_[i];
                ddq_(info.target_id) = (*QDDot)(info.source_id) * info.multiplier;
        }   
    }
}
         
template < typename T >
void MogsTemplateRobotProperties<T>::GetDDQ( Eigen::Matrix < T,Eigen::Dynamic, 1 > &ddq)
{
    if (number_mimic_joint_ == 0)
    {
        ddq = ddq_;
    }else
    {     
        ddq = Eigen::Matrix < T,Eigen::Dynamic, 1 >::Zero(getNDof());
        unsigned int nb = info_mimic_.size();
        for (int i=0;i<nb;i++)
        {
                const info_mimic& info = info_mimic_[i];
                ddq(info.source_id) += ddq_(info.target_id) * info.multiplier;
        }        
    }
}
         
template < typename T >
void MogsTemplateRobotProperties<T>::GetTorque( Eigen::Matrix < T,Eigen::Dynamic, 1 > &Tau)
{
    if (number_mimic_joint_ == 0)
    {
            Tau = tau_;
    }else
    {        
        Tau = Eigen::Matrix < T,Eigen::Dynamic, 1 >::Zero(getNDof());
        unsigned int nb = info_mimic_.size();
        for (int i=0;i<nb;i++)
        {
                const info_mimic& info = info_mimic_[i];
                Tau(info.source_id) += tau_(info.target_id) / info.multiplier;
        }
    }
}