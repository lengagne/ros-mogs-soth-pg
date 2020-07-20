//      serialization.h
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


#ifndef MOGS_SERIALIZ
#define MOGS_SERIALIZ


#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

#include <Eigen/Core>

// *********************************************************
// ***
// *** boost::serialize function for Eigen matrices
// ***
// *********************************************************

using namespace Eigen;

namespace boost
{

	template < class Archive, typename _Scalar, int _Rows, int _Cols,
		int _Options, int _MaxRows,
		int _MaxCols > inline void serialize (	Archive & ar, 
							Matrix < _Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols > &t,
							const unsigned int file_version)
	{
		size_t rows = t.rows (), cols = t.cols ();
		  ar & rows;
		  ar & cols;
		if (rows * cols != t.size ())
			  t.resize (rows, cols);

		for (size_t i = 0; i < t.size (); i++)
			ar & t.data ()[i];
	}
}



#endif
