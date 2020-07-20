//      main.cpp
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

#include <iostream>
#include "MogsGeometry.h"

int main ( int argv, char ** argc)
{
    if (argv == 1 || argv > 3)
    {
        std::cout <<" Error you must specify a file and a precision (optional)"<<std::endl;
        return 1;
    }
	// exe object_file precision
	double precision = 0.1;
	std::cout<<" Object affine.cpp"<<std::endl;
	MogsGeometry* geom_ = new MogsGeometry(argc[1]);
    if (argv == 3)
        precision = atof(argc[2]);
	geom_->affine(precision);


}
