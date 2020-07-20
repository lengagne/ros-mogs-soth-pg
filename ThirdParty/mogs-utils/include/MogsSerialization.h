/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 * Code Modified by S. Lengagne during his work in the following labs:
 *      2011-2012:  Karlsruhe Institute fur Technologie, Karlsruhe, Germany
 *      2013-2012: IUT de Beziers/ LIRMM, Beziers, France
 */

#ifndef MOGS_SERIALIZ
#define MOGS_SERIALIZ

#include "MogsTypes.h"
#include <vector>
#include <QtCore/QString>
#include <QtCore/QVector>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/vector.hpp>

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
	
	namespace serialization{
		template<class Archive>
			inline void
		save(Archive& ar, const QString& s, const unsigned int /*version*/)
		{
			std::vector<uint> tmp = s.toUcs4().toStdVector();
			ar << make_nvp("QString2Usc4", tmp);
		}
		
		template<class Archive>
		inline void
		load(Archive& ar, QString& s, const unsigned int /*version*/)
		{
			std::vector<uint> tmp;
			ar >> make_nvp("QString2Usc4", tmp);
			s = QString::fromUcs4(&tmp[0],tmp.size());
		}
		
		template<class Archive>
		inline void
		serialize(Archive& ar, QString& s, const unsigned int file_version)
		{
			boost::serialization::split_free(ar, s, file_version);
		}
	}
}



#endif
