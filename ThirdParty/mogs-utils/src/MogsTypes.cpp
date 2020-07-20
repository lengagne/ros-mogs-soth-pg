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

#include "MogsTypes.h"



mogs_string mogs_get_absolute_path( const mogs_string& in)
{
	QFileInfo tmp(in);
	return tmp.absolutePath();
}

mogs_string mogs_get_absolute_link( const mogs_string& in)
{
	QFileInfo tmp(in);
	return tmp.absoluteFilePath();
}


bool convert_to_bool( const QString & in)
{
    QString ins = in.simplified();
    if (ins == "true" || ins == "TRUE" || ins == "yes" || ins =="YES" || ins =="OUI" || ins=="oui")
        return true;
    return false;
}

Eigen::Matrix<double,3,1> convert_to_vec3(QDomElement& el)
{
    Eigen::Matrix<double,3,1> vec=Eigen::Matrix<double,3,1>::Zero();
	if (!el.isNull())
	{
		mogs_string stmp = el.text().simplified();
		double tval;
		std::istringstream smallData1 (stmp.toStdString(), std::ios_base::in);
		for (int i = 0; i < 3; i++)
		{
			smallData1 >> tval;
			vec(i) = tval;
		}
	}
	return vec;
}


