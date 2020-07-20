//      MogsProject.h
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

#ifndef __MOGSPROJECT__
#define __MOGSPROJECT__

#include <string>
#include <iostream>
#include "MogsProblemClassifier.h"
#include "config_mogs_core.h"

#define TMP_PROJECT_NAME "tmp_project.xml"

class MogsProject
{
      public:
	MogsProject ();

	MogsProject(const mogs_string & project_name);

	~MogsProject ();

    /**	Add a object to the project
     * 	\param url   XML/dae/stl file of the object
     * 	\param name  name of the object in the project
     */
	void add_object (   const mogs_string & name,
                        const mogs_string & url);

    /**	Add a problem to the project
     * 	\param xml   XML file of the robot
     * 	\param name  name of the robot in the project
     */
	void add_problem (  const mogs_string & name,
                        const mogs_string & url);

    /**	Add a robot to the project
     * 	\param xml   XML file of the robot
     * 	\param name  name of the robot in the project
     */
	void add_robot (const mogs_string & name,
                    const mogs_string & url);

	void add_robot (const mogs_string & name,
                    const mogs_string & url,
                    const double size,
                    const double weight,
                    const double power = 1.0);

    /**	Remove an object from  the project
     * \param name name of the removed object
     */
	void delete_object (const mogs_string & name);

    /**	Remove a problem from  the project
     * \param name name of the removed problem
     */
	void delete_problem (const mogs_string & name);
//
    /**	Remove a robot from  the project
     * \param name name of the removed robot
     */
	void delete_robot (const mogs_string &name);

	/** Return the url of the robot from its name	*/
	mogs_string get_robot_url( const mogs_string & name);

	/**	Return the url of the problem file */
	mogs_string get_problem_url(const mogs_string & name);

	/** Test if the object is already present in the project
	*  return true if the object is defined in the project
	* else return false*/
    bool object_defined (const mogs_string &name);

    /**	Print the list of all the objects
     */
    void print_objects();

    /**	Print the list of all the problems
     */
	void print_problems ();

    /**	Print the list of all the robots
     */
	void print_robots ();

	bool read_project (const mogs_string & filename);

	void init_problem();

	bool test()
	{
		qDebug()<<"Testing the MogsProject";
	}

	void save ();

	/** Test if the problem is already present in the project
	*  return true if the problem is defined in the project
	* else return false*/
	bool problem_defined (const mogs_string& url);

	/** Test if the robot is already present in the project
	*  return true if the robot is defined in the project
	* else return false*/
	bool robot_defined (const mogs_string& name);

	/** Solve the problem	*/
	void solve(const mogs_string& name);

	bool project_is_read_;

    QFile * file_;
	QDomDocument * doc_;
	QDomElement root_, ElRobots_, ElProblems_, ElObjects_;
    QDomElement  ElRobot_, ElProblem_, ElObject_;

	mogs_string project_file_;

	MogsAbstractProblem *problem_;

};
#endif
