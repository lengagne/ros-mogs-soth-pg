/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 * Code Modified by S. Lengagne during his work in the following labs:
 *      2011-2012:  Karlsruhe Institute fur Technologie, Karlsruhe, Germany
 *      2013-2012: IUT de Beziers/ LIRMM, Beziers, France
 */

#ifndef _MATHUTILS_H
#define _MATHUTILS_H

#include <assert.h>
#include <cmath>

#include "MogsTypes.h"

#define Vector3dZero 		Eigen::Matrix<T,3,1>::Zero()
#define Matrix3dIdentity 	Eigen::Matrix<T,3,3>::Identity()
#define Matrix3dZero		Eigen::Matrix<T,3,3>::Zero()
#define SpatialVectorZero	Eigen::Matrix<T,6,1>::Zero()
#define SpatialVectorZeroDouble	Eigen::Matrix<double,6,1>::Zero()
#define SpatialMatrixIdentity	Eigen::Matrix<T,6,6>::Identity()
#define SpatialMatrixIdentityDouble	Eigen::Matrix<double,6,6>::Identity()
#define SpatialMatrixZero	Eigen::Matrix<T,6,6>::Zero()

namespace RigidBodyDynamics
{

// namespace Math {

/** \brief Available solver methods for the linear systems.
 *
 * Please note that these methods are only available when Eigen3 is used.
 * When the math library SimpleMath is used it will always use a slow
 * column pivoting gauss elimination.
 */
	enum LinearSolver
	{
		LinearSolverUnknown = 0,
		LinearSolverPartialPivLU,
		LinearSolverColPivHouseholderQR,
		LinearSolverLast,
	};



/// \brief Solves a linear system using gaussian elimination with pivoting
	bool LinSolveGaussElimPivot (Eigen::MatrixXd A, Eigen::VectorXd b,
				     Eigen::VectorXd & x);

/// \brief Creates the skew symmetric matrix of the cross product of a given 3D vector
	inline Eigen::Matrix < double, 3, 3 > VectorCrossMatrix (const Eigen::Matrix < double, 3, 1 > &vector)
	{
//	     (Eigen::Matrix4d() << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16).finished();
		return (Eigen::Matrix < double, 3, 3 >() << 0., -vector[2],vector[1],vector[2],0., -vector[0],-vector[1], vector[0],0.).finished();
	}

// \todo write test
	void SpatialMatrixSetSubmatrix (Eigen::Matrix < double, 6, 6 > &dest,
					unsigned int row, unsigned int col,
					const Eigen::Matrix < double, 3, 3 > &matrix);

	bool SpatialMatrixCompareEpsilon (const Eigen::Matrix < double, 6, 6 > &matrix_a,
					  const Eigen::Matrix < double, 6, 6 > &matrix_b,
						double epsilon);

	bool SpatialVectorCompareEpsilon (const Eigen::Matrix < double, 6, 1 > &vector_a,
					  const Eigen::Matrix < double, 6, 1 > &vector_b, double epsilon);

/** \brief Translates the inertia matrix to a new center. */
	Eigen::Matrix < double, 3, 3 > parallel_axis (const Eigen::Matrix < double, 3, 3 > &inertia, double mass,
							const Eigen::Matrix < double, 3, 1 > &com);

/** \brief Creates a transformation of a linear displacement
 *
 * This can be used to specify the translation to the joint center when
 * adding a body to a model. See also section 2.8 in RBDA.
 *
 * \note The transformation returned is for motions. For a transformation for forces
 * \note one has to conjugate the matrix.
 *
 * \param displacement The displacement as a 3D vector
 */
	template <typename T>
	Eigen::Matrix < T, 6, 6 > Xtrans_mat (const Eigen::Matrix < T, 3, 1 > &r)
	{
		return Eigen::Matrix < T, 6, 6 > (1., 0., 0., 0., 0., 0.,
						       0., 1., 0., 0., 0., 0.,
						       0., 0., 1., 0., 0., 0.,
						       0., r[2], -r[1], 1.,
						       0., 0., -r[2], 0.,
						       r[0], 0., 1., 0., r[1],
						       -r[0], 0., 0., 0., 1.);
	}

/** \brief Creates a rotational transformation around the Z-axis
 *
 * Creates a rotation around the current Z-axis by the given angle
 * (specified in radians).
 *
 * \param zrot Rotation angle in radians.
 */
	Eigen::Matrix < double, 6, 6 > Xrotz_mat (const double &zrot);

/** \brief Creates a rotational transformation around the Y-axis
 *
 * Creates a rotation around the current Y-axis by the given angle
 * (specified in radians).
 *
 * \param yrot Rotation angle in radians.
 */
	Eigen::Matrix < double, 6, 6 > Xroty_mat (const double &yrot);

/** \brief Creates a rotational transformation around the X-axis
 *
 * Creates a rotation around the current X-axis by the given angle
 * (specified in radians).
 *
 * \param xrot Rotation angle in radians.
 */
	Eigen::Matrix < double, 6, 6 > Xrotx_mat (const double &xrot);

/** \brief Creates a spatial transformation for given parameters
 *
 * Creates a transformation to a coordinate system that is first rotated
 * and then translated.
 *
 * \param displacement The displacement to the new origin
 * \param zyx_euler The orientation of the new coordinate system, specifyed
 * by ZYX-Euler angles.
 */
	Eigen::Matrix < double, 6, 6 > XtransRotZYXEuler (const Eigen::Matrix < double, 3, 1 > &displacement,
							  const Eigen::Matrix < double, 3, 1 > &zyx_euler);

// } /* Math */

}				/* RigidBodyDynamics */
#endif /* _MATHUTILS_H */
