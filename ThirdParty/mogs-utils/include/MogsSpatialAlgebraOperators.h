/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 * Code Modified by S. Lengagne during his work in the following labs:
 *      2011-2012:  Karlsruhe Institute fur Technologie, Karlsruhe, Germany
 *      2013-2012: IUT de Beziers/ LIRMM, Beziers, France
 */

#ifndef _SPATIALALGEBRAOPERATORS_H
#define _SPATIALALGEBRAOPERATORS_H

#include "MogsTypes.h"


#include <iostream>
#include <cmath>

#include <boost/serialization/serialization.hpp>

// #include "fadiff.h"

namespace RigidBodyDynamics
{
	template < typename T >
    inline Eigen::Matrix < T, 3, 3 > VectorCrossMatrix (const Eigen::Matrix < T, 3, 1 > &vector)
    {
        return (Eigen::Matrix < T, 3, 3 >()<<  0., -vector[2], vector[1],vector[2], 0., -vector[0],-vector[1], vector[0], 0.).finished();
    }

	template < typename T >
	void convert_mat_to_RPY( const Eigen::Matrix < T, 3, 3 > &E,
				Eigen::Matrix < T, 3, 1 > & rot)
	{
		rot(2) = atan2(E(1,0),E(0,0));		//atan2(sy,sx)
		const T COS = cos(rot(2));
		const T SIN = sin(rot(2));
		rot(1) = atan2(- E(2,0) , COS * E(0,0) + SIN * E(1,0));
		rot(0) = atan2(SIN * E(0,2) - COS*E(1,2), -SIN*E(0,1) + COS * E(1,1));
	}


	template < typename T >
	Eigen::Matrix < T, 3, 3 >  RPY_mat ( const Eigen::Matrix < T, 3, 1 >& RPY)
	{
		Eigen::Matrix < T, 3, 3 > E;
		const T cr = cos(RPY(0)); // ROLL
		const T sr = sin(RPY(0));
		const T cp = cos(RPY(1)); // PITCH
		const T sp = sin(RPY(1));
		const T cy = cos(RPY(2)); // YAW
		const T sy = sin(RPY(2));

		E(0,0) = cy*cp;
		E(0,1) = cy*sp*sr-sy*cr;
		E(0,2) = cy*sp*cr+sy*sr;

		E(1,0) = sy*cp;
		E(1,1) = sy*sp*sr+cy*cr;
		E(1,2) = sy*sp*cr-cy*sr;

		E(2,0) = -sp;
		E(2,1) = cp*sr;
		E(2,2) = cp*cr;

		return E;
	}

	template < typename T >
	T mixte_product(const Eigen::Matrix<T,3,1> &U,
                    const Eigen::Matrix<T,3,1> &V,
                    const Eigen::Matrix<T,3,1> &W)
    {
        return (U(1)*V(2)-U(2)*V(1))*W(0) + (U(2)*V(1)-U(0)*V(2))*W(1) + (U(0)*V(1)-U(1)*V(0))*W(2);
    }


/** \brief Spatial algebra matrices, vectors, and operators. */

	class SpatialRigidBodyInertia
	{
	      public:
		SpatialRigidBodyInertia ():m (0.),
			h (Eigen::Matrix < double, 3, 1 >::Zero (3, 1)),
			I (Eigen::Matrix < double, 3, 3 >::Zero (3, 3))
		{
		}
		SpatialRigidBodyInertia (double mass,
					 const Eigen::Matrix < double, 3, 1 > &com,
					 const Eigen::Matrix < double, 3, 3 > &inertia):m (mass),
			h (com * mass), I (inertia)
		{

		}

		Eigen::Matrix < double, 6, 1 > operator* (const Eigen::Matrix < double, 6, 1 > &mv)
		{
			Eigen::Matrix < double, 3, 1 > mv_upper (mv[0], mv[1], mv[2]);
			Eigen::Matrix < double, 3, 1 > mv_lower (mv[3], mv[4], mv[5]);

			Eigen::Matrix < double, 3, 1 > res_upper = I * Eigen::Matrix < double, 3, 1 > (mv[0], mv[1], mv[2]) + h.cross (mv_lower);
			Eigen::Matrix < double, 3, 1 > res_lower = m * mv_lower - h.cross (mv_upper);

			return (Eigen::Matrix < double, 6, 1 >()<<res_upper[0], res_upper[1], res_upper[2],
							       res_lower[0], res_lower[1], res_lower[2]).finished();
		}

		SpatialRigidBodyInertia operator+ (const SpatialRigidBodyInertia & rbi)
		{
			return SpatialRigidBodyInertia (m + rbi.m, (h + rbi.h) / (m + rbi.m), I + rbi.I);
		}

		void createFromMatrix (const Eigen::Matrix < double, 6, 6 > &Ic)
		{
			m = Ic (3, 3);
//			h.set (-Ic (1, 5), Ic (0, 5), -Ic (0, 4));
			h = (Eigen::Matrix<double,3,1>()<< -Ic (1, 5), Ic (0, 5), -Ic (0, 4)).finished();
			I = Ic.block < 3, 3 > (0, 0);
		}

		Eigen::Matrix < double, 6, 6 > toMatrix () const
		{
			Eigen::Matrix < double, 6, 6 > result;
			  result.block < 3, 3 > (0, 0) = I;
			  result.block < 3, 3 > (0, 3) = VectorCrossMatrix (h);
			  result.block < 3, 3 > (3, 0) = -VectorCrossMatrix (h);
			  result.block < 3, 3 > (3, 3) = Eigen::Matrix < double, 3, 3 >::Identity (3, 3) * m;

			  return result;
		}

		double m;
		Eigen::Matrix < double, 3, 1 > h;
		Eigen::Matrix < double, 3, 3 > I;

	      private:

		friend class boost::serialization::access;

		template < class Archive >
			void serialize (Archive & ar,
					const unsigned int version)
		{

			ar & m;
			ar & h;
			ar & I;

		}

	};

/** \brief Compact representation of spatial transformations.
 *
 * Instead of using a verbose 6x6 matrix, this structure only stores a 3x3
 * matrix and a 3-d vector to store spatial transformations. It also
 * encapsulates efficient operations such as concatenations and
 * transformation of spatial vectors.
 */

	template < typename T > class SpatialTransform
	{

	      public:
	      SpatialTransform ():
			E (Eigen::Matrix < T, 3, 3 >::Identity (3, 3)),
			r (Eigen::Matrix < T, 3, 1 >::Zero (3, 1))
		{
		}

	      SpatialTransform (const Eigen::Matrix < T, 3, 3 >& rotation,
				const Eigen::Matrix < T, 3, 1 >& translation):
			E (rotation),
			r (translation)
		{
		}

	      SpatialTransform (const Eigen::Matrix < T, 3, 1 >& rotation,
				const Eigen::Matrix < T, 3, 1 >& translation):
			r (translation)
		{
			E = RPY_mat ( rotation);
		}

		template < typename T2 >
			SpatialTransform (const SpatialTransform < T2 > &in)
		{
			E = in.E.template cast < T > ();
			r = in.r.template cast < T > ();
		}

	/** Same as X * v.
	 *
	 * \returns (E * w, - E * rxw + E * v)
	 */
		template <typename T2>
		Eigen::Matrix < T2, 6, 1 > apply (const Eigen::Matrix < T2, 6, 1 > &v_sp)
		{
			Eigen::Matrix < T2, 3, 1 > v_rxw (v_sp[3] - r[1] * v_sp[2] + r[2] * v_sp[1],
                                             v_sp[4] - r[2] * v_sp[0] + r[0] * v_sp[2],
                                             v_sp[5] - r[0] * v_sp[1] + r[1] * v_sp[0]);

			return (Eigen::Matrix < T2, 6, 1 >()<<E (0, 0) * v_sp[0] + E (0, 1) * v_sp[1] + E (0, 2) * v_sp[2],
                                              E (1, 0) * v_sp[0] + E (1, 1) * v_sp[1] + E (1, 2) * v_sp[2],
                                              E (2, 0) * v_sp[0] + E (2, 1) * v_sp[1] + E (2, 2) * v_sp[2],
                                              E (0, 0) * v_rxw[0] + E (0, 1) * v_rxw[1] + E (0, 2) * v_rxw[2],
                                              E (1, 0) * v_rxw[0] + E (1, 1) * v_rxw[1] + E (1, 2) * v_rxw[2],
                                              E (2, 0) * v_rxw[0] + E (2, 1) * v_rxw[1] + E (2, 2) * v_rxw[2]).finished();
		}


		Eigen::Matrix < T, 6, 1 > apply (const Eigen::Matrix < T, 6, 1 > &v_sp)
		{
			Eigen::Matrix < T, 3, 1 > v_rxw (v_sp[3] - r[1] * v_sp[2] + r[2] * v_sp[1],
                                             v_sp[4] - r[2] * v_sp[0] + r[0] * v_sp[2],
                                             v_sp[5] - r[0] * v_sp[1] + r[1] * v_sp[0]);

			return (Eigen::Matrix < T, 6, 1 >()<< E (0, 0) * v_sp[0] + E (0, 1) * v_sp[1] + E (0, 2) * v_sp[2],
                                              E (1, 0) * v_sp[0] + E (1, 1) * v_sp[1] + E (1, 2) * v_sp[2],
                                              E (2, 0) * v_sp[0] + E (2, 1) * v_sp[1] + E (2, 2) * v_sp[2],
                                              E (0, 0) * v_rxw[0] + E (0, 1) * v_rxw[1] + E (0, 2) * v_rxw[2],
                                              E (1, 0) * v_rxw[0] + E (1, 1) * v_rxw[1] + E (1, 2) * v_rxw[2],
                                              E (2, 0) * v_rxw[0] + E (2, 1) * v_rxw[1] + E (2, 2) * v_rxw[2]).finished();
		}


		inline Eigen::Matrix < double, 3, 3 > VectorCrossMatrixD (const Eigen::Matrix < double, 3, 1 > &vector)
		{
			return Eigen::Matrix < double, 3, 3 > (0., -vector[2], vector[1],
							  vector[2], 0., -vector[0],
							  -vector[1], vector[0], 0.);
		}
	/** Same as X^T * f.
	 *
	 * \returns (E^T * n + rx * E^T * f, E^T * f)
	 */
		SpatialRigidBodyInertia apply (const SpatialRigidBodyInertia & rbi)
		{
			Eigen::Matrix < double, 3, 3 > Ep = E;
			Eigen::Matrix < double, 3, 1 > rp = r;


			Eigen::Matrix < double, 3, 3 > inertia =
				Ep.transpose () * rbi.I * Ep -
				VectorCrossMatrixD (rp) * VectorCrossMatrixD (Ep. transpose () * rbi.h) -
				VectorCrossMatrixD (Ep. transpose () * (rbi.h) + rp * rbi.m) * VectorCrossMatrixD (rp);

			return SpatialRigidBodyInertia (rbi.m, Ep.transpose () * (rbi.h / rbi.m) + rp,inertia);
		}

		Eigen::Matrix < T, 6, 1 > applyTranspose (const Eigen::Matrix < T, 6, 1 > &f_sp)
		{
			Eigen::Matrix < T, 3, 1 > E_T_f = (Eigen::Matrix < T, 3, 1 >()<< E (0, 0) * f_sp[3] + E (1, 0) * f_sp[4] + E (2, 0) * f_sp[5],
							 E (0, 1) * f_sp[3] + E (1, 1) * f_sp[4] + E (2, 1) * f_sp[5],
							 E (0, 2) * f_sp[3] + E (1, 2) * f_sp[4] + E (2, 2) * f_sp[5]).finished();

			return (Eigen::Matrix < T, 6, 1 > () << E (0, 0) * f_sp[0] + E (1, 0) * f_sp[1] + E (2, 0) * f_sp[2] - r[2] * E_T_f[1] + r[1] * E_T_f[2],
							  E (0, 1) * f_sp[0] + E (1, 1) * f_sp[1] + E (2, 1) * f_sp[2] + r[2] * E_T_f[0] - r[0] * E_T_f[2],
							  E (0, 2) * f_sp[0] + E (1, 2) * f_sp[1] + E (2, 2) * f_sp[2] - r[1] * E_T_f[0] + r[0] * E_T_f[1],
							  E_T_f[0], E_T_f[1], E_T_f[2]).finished();
		}

		Eigen::Matrix < T, 6, 1 > applyAdjoint (const Eigen::Matrix < T, 6, 1 > &f_sp)
		{
			Eigen::Matrix < T, 3, 1 > En_rxf = E * (Eigen::Matrix < T, 3, 1 > () << f_sp[0],f_sp[1],f_sp[2]).finished() - r.cross (Eigen::Matrix < T, 3, 1 > () << f_sp[3], f_sp[4], f_sp[5]).finished();

			return Eigen::Matrix < T, 6, 1 > (En_rxf[0],
							  En_rxf[1],
							  En_rxf[2],
							  E (0, 0) * f_sp[3] + E (0, 1) * f_sp[4] + E (0, 2) * f_sp[5],
							  E (1, 0) * f_sp[3] + E (1, 1) * f_sp[4] + E (1, 2) * f_sp[5],
							  E (2, 0) * f_sp[3] + E (2, 1) * f_sp[4] + E (2, 2) * f_sp[5]);
		}

		inline void read_string(const mogs_string & in)
		{
			std::istringstream smallData (in.toStdString(), std::ios_base::in);
			double tmp;
			smallData >> tmp; r(0) = tmp;
			smallData >> tmp; r(1) = tmp;
			smallData >> tmp; r(2) = tmp;
			Eigen::Matrix < T, 3, 1 > RPY;
			smallData >> tmp; RPY(0) = tmp;
			smallData >> tmp; RPY(1) = tmp;
			smallData >> tmp; RPY(2) = tmp;
			E = RPY_mat(RPY);
		}

		Eigen::Matrix < T, 6, 6 > toMatrix ()const
		{
			Eigen::Matrix < T, 3, 3 > _Erx = E * VectorCrossMatrix (r); //Eigen::Matrix < T, 3, 3 > (Eigen::Matrix < T, 3, 3 >()<< 0., -r[2], r[1],
// 											r[2], 0., -r[0],
// 											-r[1], r[0], 0.).finished();
			Eigen::Matrix < T, 6, 6 > result;

			result.template block < 3, 3 > (0, 0) = E;
			result.template block < 3, 3 > (0, 3) = Eigen::Matrix < T, 3, 3 >::Zero (3, 3);
			result.template block < 3, 3 > (3, 0) = -_Erx;
			result.template block < 3, 3 > (3, 3) = E;

			return result;
		}

		Eigen::Matrix < T, 6, 6 > toMatrixAdjoint () const
		{
			Eigen::Matrix < T, 3, 3 > _Erx = E * VectorCrossMatrix (r); //(Eigen::Matrix < T, 3, 3 >() << 0., -r[2], r[1], r[2], 0., -r[0], -r[1], r[0], 0.).finished();
			Eigen::Matrix < T, 6, 6 > result;
			result.template block < 3, 3 > (0, 0) = E;
			result.template block < 3, 3 > (0, 3) = -_Erx;
			result.template block < 3, 3 > (3, 0) = Eigen::Matrix < T, 3, 3 >::Zero (3, 3);
			result.template block < 3, 3 > (3, 3) = E;

			return result;
		}

		Eigen::Matrix < T, 6, 6 > toMatrixTranspose () const
		{
			Eigen::Matrix < T, 3, 3 > _Erx = E * VectorCrossMatrix (r); //(Eigen::Matrix < T, 3, 3 >() << 0., -r[2], r[1],r[2], 0., -r[0],-r[1],r[0], 0.).finished();
			Eigen::Matrix < T, 6, 6 > result;
			result.template block < 3, 3 > (0, 0) = E.transpose ();
			result.template block < 3, 3 > (0, 3) = -_Erx.transpose ();
			result.template block < 3, 3 > (3, 0) = Eigen::Matrix < T, 3, 3 >::Zero (3, 3);
			result.template block < 3, 3 > (3, 3) = E.transpose ();

			return result;
		}

		SpatialTransform operator* (const SpatialTransform & XT) const
		{
			return SpatialTransform < T > ( Eigen::Matrix < T, 3, 3 > (E * XT.E), XT.r + XT.E.transpose () * r);
		}

		void operator*= (const SpatialTransform & XT)
		{
			r = XT.r + XT.E.transpose () * r;
			E *= XT.E;
		}

		// multiplication by the transpose
		SpatialTransform transpose() const
		{
			return SpatialTransform < T > ( Eigen::Matrix < T, 3, 3 > (E.transpose()) , - E*r);
		}

		inline void RPY_to_mat ( Eigen::Matrix < T, 3, 1 >& RPY)
		{
			E = RPY_mat(RPY);
		}

		void get_RPY( Eigen::Matrix < T, 3, 1 > & rot,
			      Eigen::Matrix < T, 3, 1 > & pos) const
		{
			pos = r;
			get_RPY(rot);
		}

		void get_RPY( Eigen::Matrix < T, 3, 1 > & rot) const
		{
			convert_mat_to_RPY(E,rot);
		}

		void get_4x4( Eigen::Matrix < T, 4, 4 > & mat) const
		{
			mat(3,0) = mat(3,1) = mat(3,2) =  0.0;
			mat(3,3) = 1;

			for (int i=0;i<3;i++)
			{
				mat(i,3) = r(i);
				for (int j=0;j<3;j++)
					mat(i,j) = E(i,j);
			}
		}

        //translate frame position in world position
        template <typename T2>
		Eigen::Matrix < T2, 3, 1 > get_Position_T2_2_T2( const Eigen::Matrix < T2, 3, 1 > & pos) const
		{
			return r.template cast<T2>()  + E.template cast<T2>() .transpose () * pos;
		}

		template <typename T2>
		Eigen::Matrix < T, 3, 1 > get_Position_T2_2_T( const Eigen::Matrix < T2, 3, 1 > & pos) const
		{
			return r + E.transpose () * pos.template cast<T>() ;
		}

		Eigen::Matrix < T, 3, 1 > get_Position( const Eigen::Matrix < T, 3, 1 > & pos) const
		{
			return r + E.transpose () * pos;
		}

		//translate world position in frame position
		Eigen::Matrix < T, 3, 1 > Get_Local_Position( const Eigen::Matrix < T, 3, 1 > & pos) const
		{
			return E * (pos -r);
		}

		template <typename T2>
		Eigen::Matrix < T2, 3, 1 > Get_Local_Position_T2_2_T2( const Eigen::Matrix < T2, 3, 1 > & pos) const
		{
			return E.template cast<T2>() * (pos -r.template cast<T2>());
		}

		template <typename T2>
		Eigen::Matrix < T, 3, 1 > Get_Local_Position_T2_2_T( const Eigen::Matrix < T2, 3, 1 > & pos) const
		{
			return E * (pos.template cast<T>() -r);
		}

		bool operator == ( const SpatialTransform<T> & in) const
		{
			return (E == in.E && r == in.r);
		}

		inline bool operator != ( const SpatialTransform<T> & in) const
		{
			return !(*this == in);
		}


	// Members
		Eigen::Matrix < T, 3, 3 > E;	// rotation matrix
		Eigen::Matrix < T, 3, 1 > r;	// position vector

	private:

		friend class boost::serialization::access;

		template < class Archive >
			void serialize (Archive & ar,
					const unsigned int version)
		{
			ar & E;
			ar & r;
		}


	};

	inline std::ostream & operator<< (std::ostream & output,
					  const SpatialRigidBodyInertia & rbi)
	{
		output << "rbi.m = " << rbi.m << std::endl;
		output << "rbi.h = " << rbi.h.transpose ();
		output << "rbi.I = " << std::endl << rbi.I << std::endl;
		return output;
	}

	template < typename T >
		inline std::ostream & operator<< (std::ostream & output,
						  const SpatialTransform < T >
						  &X)
	{
		output << "X.E = " << std::endl << X.E << std::endl;
		output << "X.r = " << X.r.transpose ();
		return output;
	}

	template < typename T >
		inline SpatialTransform < T > Xrot (const T &  angle_rad,
                                            const Eigen::Matrix < double, 3, 1 > &axis)
	{
//		T s, c , one_minus_c;
		T s = sin (angle_rad);
		T c = cos (angle_rad);
		T one_minus_c = -(c - 1);

		return SpatialTransform < T > ((Eigen::Matrix < T, 3, 3 > ()<<
                        axis[0] * axis[0] * one_minus_c + c,
						axis[1] * axis[0] * one_minus_c + axis[2] * s,
						axis[0] * axis[2] * one_minus_c - axis[1] * s,
						axis[0] * axis[1] * one_minus_c - axis[2] * s,
						axis[1] * axis[1] * one_minus_c + c,
						axis[1] * axis[2] * one_minus_c + axis[0] * s,
						axis[0] * axis[2] * one_minus_c + axis[1] * s,
						axis[1] * axis[2] * one_minus_c - axis[0] * s,
						axis[2] * axis[2] * one_minus_c + c).finished(),
						Eigen::Matrix < T, 3, 1 > (0., 0., 0.));
	}

	template < typename T >
		inline Eigen::Matrix < T, 3, 3 > rotx (const T & xrot)
	{
		T s, c;
		s = sin (xrot);
		c = cos (xrot);
		return Eigen::Matrix < T, 3, 3 > (1., 0., 0., 0., c, s, 0., -s, c);
	}

	template < typename T >
		inline Eigen::Matrix < T, 3, 3 > roty (const T & yrot)
	{
		T s, c;
		s = sin (yrot);
		c = cos (yrot);
		return Eigen::Matrix < T, 3, 3 > (c, 0., -s, 0., 1., 0., s, 0., c);
	}

	template < typename T >
		inline Eigen::Matrix < T, 3, 3 > rotz (const T & zrot)
	{
		T s, c;
		s = sin (zrot);
		c = cos (zrot);
		return Eigen::Matrix < T, 3, 3 > (c, s, 0., -s, c, 0., 0., 0., 1.);
	}


	template < typename T >
		inline SpatialTransform < T > Xrotx (const T & xrot)
	{
// 		T s, c;
// 		s = sin (xrot);
// 		c = cos (xrot);
		return SpatialTransform < T > (rotx(xrot), Eigen::Matrix < T, 3, 1 > (0., 0., 0.));
	}

	template < typename T >
		inline SpatialTransform < T > Xroty (const T & yrot)
	{
// 		T s, c;
// 		s = sin (yrot);
// 		c = cos (yrot);
		return SpatialTransform < T > (roty(yrot), Eigen::Matrix < T, 3, 1 > (0., 0., 0.));
	}

	template < typename T >
		inline SpatialTransform < T > Xrotz (const T & zrot)
	{
// 		T s, c;
// 		s = sin (zrot);
// 		c = cos (zrot);
		return SpatialTransform < T > (rotz(zrot), Eigen::Matrix < T, 3, 1 > (0., 0., 0.));
	}

	template < typename T >
		inline SpatialTransform < T > Xtrans (const Eigen::Matrix < T, 3, 1 > &r)
	{
		return SpatialTransform < T > (Eigen::Matrix < T, 3, 3 > (Eigen::Matrix < T, 3, 3 >::Identity (3, 3)), r);
	}

	template < typename T >
		inline Eigen::Matrix < T, 6, 6 > crossm (const Eigen::Matrix < T, 6, 1 > &v)
	{
		return Eigen::Matrix < T, 6, 6 > (0, -v[2], v[1], 0, 0, 0,
						  v[2], 0, -v[0], 0, 0, 0,
						  -v[1], v[0], 0, 0, 0, 0,
						  0, -v[5], v[4], 0, -v[2], v[1],
						  v[5], 0, -v[3], v[2], 0, -v[0],
						  -v[4], v[3], 0, -v[1], v[0], 0);
	}

	template < typename T >
		inline Eigen::Matrix < T, 6, 1 > crossm (const Eigen::Matrix < T, 6, 1 > &v1,
							 const Eigen::Matrix < T, 6, 1 > &v2)
	{
		return (Eigen::Matrix < T, 6, 1 > () <<	-v1[2] * v2[1] + v1[1] * v2[2],
							v1[2] * v2[0] - v1[0] * v2[2],
							-v1[1] * v2[0] + v1[0] * v2[1],
							-v1[5] * v2[1] + v1[4] * v2[2] - v1[2] * v2[4] + v1[1] * v2[5],
							v1[5] * v2[0] - v1[3] * v2[2] + v1[2] * v2[3] - v1[0] * v2[5],
							-v1[4] * v2[0] + v1[3] * v2[1] - v1[1] * v2[3] + v1[0] * v2[4]).finished();
	}

	template < typename T >
		inline Eigen::Matrix < T, 6, 6 > crossf (const Eigen::Matrix < T, 6, 1 > &v)
	{
		return (Eigen::Matrix < T, 6, 6 > () <<0, -v[2], v[1], 0, -v[5], v[4],
						  v[2], 0, -v[0], v[5], 0, -v[3],
						  -v[1], v[0], 0, -v[4], v[3], 0,
						   0, 0, 0, 0, -v[2], v[1],
						   0, 0, 0, v[2],  0, -v[0],
						   0, 0, 0, -v[1], v[0], 0).finished();
	}

	template < typename T >
		inline Eigen::Matrix < T, 6, 1 > crossf (const Eigen::Matrix < T, 6, 1 > &v1,
							 const Eigen::Matrix < T, 6, 1 > &v2)
	{
		return (Eigen::Matrix < T, 6, 1 > () <<-v1[2] * v2[1] + v1[1] * v2[2] - v1[5] * v2[4] + v1[4] * v2[5],
						   v1[2] * v2[0] - v1[0] * v2[2] + v1[5] * v2[3] - v1[3] * v2[5],
						  -v1[1] * v2[0] + v1[0] * v2[1] - v1[4] * v2[3] + v1[3] * v2[4],
						  -v1[2] * v2[4] + v1[1] * v2[5],
						   v1[2] * v2[3] - v1[0] * v2[5],
						  -v1[1] * v2[3] + v1[0] * v2[4]).finished();
	}

	template < typename T >
		inline Eigen::Matrix < T, 6, 6 > spatial_adjoint (const Eigen::Matrix < T, 6, 6 > &m)
	{
		Eigen::Matrix < T, 6, 6 > res (m);
		res.template block < 3, 3 > (3, 0) = m.block < 3, 3 > (0, 3);
		res.template block < 3, 3 > (0, 3) = m.block < 3, 3 > (3, 0);
		return res;
	}

	template < typename T >
		inline Eigen::Matrix < T, 6, 6 > spatial_inverse (const Eigen::Matrix < T, 6, 6 > &m)
	{
		Eigen::Matrix < T, 6, 6 > res (m);
		res.template block < 3, 3 > (0, 0) = m.template block < 3, 3 > (0, 0).transpose ();
		res.template block < 3, 3 > (3, 0) = m.template block < 3, 3 > (3, 0).transpose ();
		res.template block < 3, 3 > (0, 3) = m.template block < 3, 3 > (0, 3).transpose ();
		res.template block < 3, 3 > (3, 3) = m.template block < 3, 3 > (3, 3).transpose ();
		return res;
	}

	template < typename T >
		inline Eigen::Matrix < T, 3, 3 > get_rotation (const Eigen::Matrix < T, 6, 6 > &m)
	{
		return m.block < 3, 3 > (0, 0);
	}

	template < typename T >
		inline Eigen::Matrix < T, 3, 1 > get_translation (const Eigen::Matrix < T, 6, 6 > &m)
	{
		return (Eigen::Matrix < T, 3, 1 > () <<-m (4, 2), m (3, 2), -m (3, 1)).finished();
	}
}				/* RigidBodyDynamics */

/* _SPATIALALGEBRAOPERATORS_H*/
#endif
