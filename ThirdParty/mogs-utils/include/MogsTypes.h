//      MogsTypes.h
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
//      2012-2013:  IUT de Beziers/ LIRMM, Beziers, France
//	from 2013:  Université Blaise Pascal / axis : ISPR / theme MACCS

#ifndef __MOGSTYPES__
#define __MOGSTYPES__

#define Pi 3.14159265358979323846
#define deg_to_rad Pi/180.0

#include<iostream>
#include<string>
#include<vector>
#include <boost/numeric/ublas/vector.hpp>

#define EIGEN_DEFAULT_TO_ROW_MAJOR
#define EIGEN_MATRIX_PLUGIN "MogsMatrixAddons.h"
#include "Eigen/Dense"
#include "Eigen/StdVector"
#include "Eigen/Geometry"
#include "Eigen/Core"

#include <QDir>
#include <QString>
#include <QDebug>

// Library needed for processing XML documents
#include <QtXml>
// Library needed for processing files
#include <QFile>
// Library needed for checking consistency of the XML
#include <QXmlSchema>
#include <QXmlSchemaValidator>

// #include <QCoreApplication>
#include <QApplication>

typedef QString mogs_string;

/**	Atan2	**/
/**	Atan2	**/
template < typename T > T atan2 (const T & a, const T & b)
{
	T C = a / b;
	T OUT = atan (C);
	if (b < 0)
	  {
		  if (a < 0)
			  OUT = OUT - 3.14159265358979323846;
		  else
			  OUT = OUT + 3.14159265358979323846;
	  }
	return (OUT);
}

mogs_string mogs_get_absolute_path( const mogs_string& in);

mogs_string mogs_get_absolute_link( const mogs_string& in);

bool  convert_to_bool( const QString & in);

Eigen::Matrix<double,3,1> convert_to_vec3(QDomElement& el);

#endif
