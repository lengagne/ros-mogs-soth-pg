//      types.h
//      Copyright (C) 2014 lengagne (lengagne@gmail.com)
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
//	from 2013 : Université Blaise Pascal / axis : ISPR / theme MACCS
//
//	See README

#ifndef _AF_MATHS_TYPES_H_
#define _AF_MATHS_TYPES_H_

#include "boost/numeric/ublas/vector.hpp"
#include <iostream>

#include "MogsTypes.h"

namespace Eigen {
template<typename Derived>
std::istream & operator >>(std::istream & Inp, MatrixBase<Derived> & m)
{
 for (int i = 0; i < m.rows(); ++i)
   for (int j = 0; j < m.cols(); j++)
     Inp >> m(i,j);

 return Inp;
}

} // end of Eigen

inline Eigen::Matrix<double,3,1> elt_product(const Eigen::Matrix<double,3,1> & v1, const Eigen::Matrix<double,3,1>& v2)
{
	return Eigen::Matrix<double,3,1>(v1(0) * v2(0), v1(1) * v2(1), v1(2) * v2(2));
}

inline void normalize(Eigen::Matrix<double,3,1> & v, double tol=1e-24)
{
	double l = v.norm();
	assert((l >= tol) && "can not normalize null vector");
	v /= l;
}

template <class T>
inline std::istream& operator>>(std::istream& is, ::Eigen::Matrix<T,6,1> & v)
{
	for (unsigned i=0; i < 6; ++i)
		is >> v[i];
	return is;
}


#endif
