//      MogsViewPGProblem.h
//      Copyright (C) 2015 lengagne (lengagne@gmail.com)
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
//      2012-2013: IUT de Beziers/ LIRMM, Beziers, France 
//	from 2013 : Université Blaise Pascal / axis : ISPR / theme MACCS

#ifndef __PROBLEM_VISU__
#define __PROBLEM_VISU__

#include "MogsAbstractProblem.h"
#include "config_MogsPatternGenerator.h"
#include "MogsAbstractPatternGenerator.h"

#ifdef MogsVisu_FOUND
	#include "VisuHolder.h"
#endif

class MogsViewPGProblem:public MogsAbstractProblem
{
      public:
	MogsViewPGProblem ();

	~MogsViewPGProblem ();
	
	/** Solve the problem	 */
	void solve();

		
      private:
#ifdef MogsVisu_FOUND
	VisuHolder * visu_window_;
#endif
	
	std::vector<MogsAbstractPatternGenerator *> pg_;
	
	  
};

#endif //__PROBLEM_VISU__
