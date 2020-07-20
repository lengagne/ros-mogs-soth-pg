//      MogsProblemClassifier.h
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

#ifndef	__MOGSPROBLEMCLASSIFIER_H__
#define __MOGSPROBLEMCLASSIFIER_H__

#include <string>
#include "MogsAbstractProblem.h"
#include "MogsAbstractRobot.h"
#include "MogsDefaultRobot.h"
#include "config_mogs_core.h"
#include <dlfcn.h>

struct libname
{
	mogs_string library_type;
	mogs_string library_name;
	mogs_string library_so;
};

class MogsProblemClassifier
{
public:
      /** Constructor*/
	MogsProblemClassifier ();

      /** Destructor*/
	~MogsProblemClassifier ();

	void add_library_plugins(	const mogs_string & plugin_type,
					const mogs_string & plugin_name,
					const mogs_string & library_so);

	void add_library_problem(	const mogs_string & problem_name,
					const mogs_string & library_so);

	void add_library_robot	(	const mogs_string & robot_name,
					const mogs_string & library_so);

	void create_xml();

	void delete_problem (MogsAbstractProblem ** pb);

// 	FIXME void delete_robot (MogsAbstractRobot ** robot);

	/** Return false if the plugin is not defined
	 *  Return the url of the library
	 */
	bool get_library_plugin(const mogs_string & plugin_type,
                            const mogs_string & plugin_name,
                            mogs_string & library_so_output);

	void print_plugins();

	void print_plugins_with_libs();

	void print_plugins_problem();

	void print_plugins_problem_with_libs();

	void print_plugins_robot();

	void print_plugins_robot_with_libs();

	void rm_library_plugins( const mogs_string & plugin_type, const mogs_string & control_name);

	void rm_library_problem	(const mogs_string & problem_name);

	void rm_library_robot	(const mogs_string & robot_name);


	/**	Initialize the pb object
	 * 	Return false in case of trouble*/
	bool set_problem_type (const mogs_string & filename, MogsAbstractProblem ** pb);

	 void set_robot_type (const mogs_string & fileurl, MogsAbstractRobot ** pb);

	 void set_scalable_robot_type (const mogs_string & fileurl, MogsAbstractRobot ** pb,
                                const double size, const double weight, const double power);

private:
    bool open_xml();
	bool read_xml();
	void save_xml();

	std::vector<libname> libs_plugins_;
	std::vector<libname> libs_problem_;
	std::vector<libname> libs_robot_;

    //QFile * file_;
	QDomDocument * doc_ = NULL;
	QDomElement root_;
	QDomElement ElPlugins_;
	QDomElement ElProblems_;
	QDomElement ElRobots_;

	void* library_;
	create_problem* create_problem_;
	destroy_problem* destroy_problem_;
};

#endif
