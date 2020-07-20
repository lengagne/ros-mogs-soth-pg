//      MogsAbstractProblem.h
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
//      from 2013:  Université Blaise Pascal / axis : ISPR / theme MACCS
#ifndef __MOGSABSTRACTPROBLEM__
#define __MOGSABSTRACTPROBLEM__
// Library needed to type conversions
#include "MogsTypes.h"
#include "MogsAbstractCollision.h"
/**     This class reads the configuration files,
*       prepare, solve the problem and store the result.
*/
struct additionnal_robot_body_mesh{
    mogs_string robot_name__;
    mogs_string body_name__;
    mogs_string mesh_name__;
    Eigen::Matrix<double,3,1> position__;
    Eigen::Matrix<double,3,1> rotation__;
    MogsGeometry * mesh__;
};

class MogsAbstractProblem
{
      public:
        MogsAbstractProblem();
        ~MogsAbstractProblem();
        
        std::vector<additionnal_robot_body_mesh> get_additionnal_meshes()
        {
            return add_meshes_;            
        }
        
        std::vector<MogsGeometry*> get_objects() const
        {
            return objects_;
        }
        std::vector<mogs_string> get_objects_name() const
        {
                return objects_name_;
        }
        std::vector< Eigen::Matrix < double, 3, 1 >> get_objects_position() const
        {
                return objects_position_;
        }
        std::vector< Eigen::Matrix < double, 3, 1 >> get_objects_rotation() const
        {
                return objects_orientation_;
        }
        std::vector<MogsRobotProperties*> get_robots() const
        {
            return robots_;
        }
        std::vector<mogs_string> get_robots_name() const
        {
                return robots_name_;
        }
        mogs_string get_type() const
        {
                return problem_type_;
        }
        /** Read the xml file
        *  This function contains the reading of the problem properties
        *  The child functions must call this one first !!
        */
        virtual void read_problem (const mogs_string & filename);
        
        void set_additionnal_meshes(const std::vector<additionnal_robot_body_mesh>& in);

        void set_objects(std::vector<MogsGeometry* > objects)
        {
                objects_ = objects;
        }
        void set_robots(std::vector<MogsRobotProperties*> robots);
        /** Solve the problem    */
        virtual void solve() = 0;
      protected:
        mogs_string xml_file_;
        QDomDocument xml_doc_;
        /** File for the xsd schema consistency test.
         *  This variable must be set in the child ckass
         */
        mogs_string xsd_file_;
        /** Node of the xml problem file        */
        QDomElement root_;
        std::vector<QDomNode> el_robots_;
        QDomDocument doc_;
        QFile *file_;
        mogs_string problem_type_;
//      Robots' information
        int nb_robots_;         // number of robots
        std::vector < mogs_string > robots_name_;       // name of the robot
        mogs_string environment_name_;  // xml of the robot
        void* collision_construct_;
        MogsAbstractCollision * collision_;
        create_collision* create_collision_;
        destroy_collision* destroy_collision_;
        /// Variables used for fixed robots or for the initial value of free floatting robots.
        std::vector < Eigen::Matrix < double, 3, 1 > > robot_position_, robot_orientation_;
        std::vector< std::map<mogs_string,mogs_string> > robot_config_files_;
        std::vector<MogsRobotProperties*> robots_;
        
        std::vector<MogsGeometry* > objects_;
        std::vector < mogs_string > objects_name_;
        std::vector < Eigen::Matrix < double, 3, 1 > > objects_position_, objects_orientation_;        
        
        std::vector<additionnal_robot_body_mesh> add_meshes_;
        
};
// the types of the class factories
typedef MogsAbstractProblem* create_problem();
typedef void destroy_problem(MogsAbstractProblem*);
#endif
