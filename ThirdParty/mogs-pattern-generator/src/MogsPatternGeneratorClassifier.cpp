//      Problem_holder.cpp
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

#include "MogsPatternGeneratorClassifier.h"

MogsPatternGeneratorClassifier::MogsPatternGeneratorClassifier ()
{

}

MogsPatternGeneratorClassifier::~MogsPatternGeneratorClassifier ()
{

}

void MogsPatternGeneratorClassifier::delete_pattern_generator (MogsAbstractPatternGenerator ** pb)
{
	// destroy the class
	destroy_pattern_generator_(*pb);
	// unload the triangle library
	dlclose(pg_);
}

bool MogsPatternGeneratorClassifier::set_pattern_generator (	const mogs_string & pg_type,
								MogsAbstractPatternGenerator ** pg)
{
	MogsProblemClassifier classifier;
	mogs_string library_so_output;

	if ( classifier.get_library_plugin("pattern_generator",pg_type,library_so_output))
	{
		// load the library
		pg_ = dlopen(library_so_output.toAscii(), RTLD_LAZY);
		if (!pg_) {
			std::cerr << "Cannot load library: " << dlerror() << '\n';
			exit(0);
		}

		// load the symbols
		create_pattern_generator_ = (create_pattern_generator*) dlsym(pg_, "create");
		destroy_pattern_generator_ = (destroy_pattern_generator*) dlsym(pg_, "destroy");
		if (!create_pattern_generator_ || !destroy_pattern_generator_)
		{
			std::cerr << "Cannot load symbols: " << dlerror() << '\n';
			exit(0);
		}

		// create an instance of the class
		(*pg) = create_pattern_generator_();
		return true;
	}else
	{
		qDebug()<<"Error in "<<__FILE__<<" at line "<< __LINE__;
		qDebug()<<"The pattern generator "<< pg_type<<" is not defined";
		return false;
	}
}
