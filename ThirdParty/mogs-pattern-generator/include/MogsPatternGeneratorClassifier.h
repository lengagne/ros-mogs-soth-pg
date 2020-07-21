//      MogsPatternGeneratorClassifier.h
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

/**	This class classify the type of problem
 * 	and returns the adeaquate integer
 */

#ifndef	__MOGSPATTERNGENERATORCLASSIFIER_H__
#define __MOGSPATTERNGENERATORCLASSIFIER_H__

#include <string>
#include "MogsAbstractPatternGenerator.h"
#include "MogsProblemClassifier.h"


class MogsPatternGeneratorClassifier
{
public:
      /** Constructor*/
	MogsPatternGeneratorClassifier ();

      /** Destructor*/
	~MogsPatternGeneratorClassifier ();
	
	void delete_pattern_generator (MogsAbstractPatternGenerator ** pg);
	
	bool set_pattern_generator (const mogs_string & pg_type, MogsAbstractPatternGenerator ** pb);
	
protected:

	void* pg_;
	create_pattern_generator* create_pattern_generator_;
	destroy_pattern_generator* destroy_pattern_generator_;
};

#endif
