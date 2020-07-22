//      ROSConstraint.h
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
//	from 2020:  Université Clermont Auvergne / Pascal Institute / axis : ISPR / theme MACCS

#ifndef __ROSConstraint__
#define __ROSConstraint__

#include <ros/ros.h>

// Library needed for processing XML documents
#include <QtXml>


class ROSConstraint
{
public:
    ROSConstraint(   QDomElement pg_root);
    
protected:
    std::string topic_name_;
};

#endif
