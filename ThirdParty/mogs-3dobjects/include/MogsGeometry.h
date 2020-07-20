//      geometry.cpp
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
//      2012-2013: IUT de Beziers/ LIRMM, Beziers, France
//	from 2013 : Université Blaise Pascal / axis : ISPR / theme MACCS

#ifndef _GEOMETRY_H
#define _GEOMETRY_H

#include <string>
#include <vector>
#include <stdlib.h>

#include "MogsTypes.h"

#include "MogsRBDLMathUtils.h"
#include "MogsSpatialAlgebraOperators.h"

#include "serialization.h"
#include "Mesh.h"
#include <QtXml>

using namespace RigidBodyDynamics;

class MogsGeometry
{

	public:
	MogsGeometry ();	// constructeur standard
	MogsGeometry (MogsGeometry *);	// constructeur par recopie

	MogsGeometry (const QString &file);	// constructeur par recopie

	~MogsGeometry ();

    void add_point_inside(double precision = 0.1);

	void affine(double precision = 0.01,bool create_face=true);

	void load (QDomElement Element,
		const mogs_string &  path="./")
	{
		load(Element,1.0,path);
	};

	void load (QDomElement Element,
			double length,
			const mogs_string &  path="./" );

    void load(  const QString & filename);

	void compute_normals(Eigen::Matrix<double,3,1> center, sub_Mesh* sm);

	void convert_box_to_mesh(sub_Mesh *smeh, float X1, float Y1, float Z1, float X2, float Y2, float Z2,
				unsigned int R, unsigned int G, unsigned int B);

	void convert_cylinder_to_mesh(sub_Mesh *smeh, float Radius, float Height,
				Eigen::Matrix<double,3,1> position, Eigen::Matrix<double,3,1> orientation,
				unsigned int R, unsigned int G, unsigned int B);

	void convert_sphere_to_mesh(sub_Mesh *smeh, float X, float Y, float Z, float Radius,
				unsigned int R, unsigned int G, unsigned int B);

    void get_points( std::vector<Eigen::Matrix<double,3,1> > &points ) const;

	void add (Mesh m)
	{
		tab_mesh.push_back (m);
	}

	void add_cube(float X1, float Y1, float Z1, float d=0.01,
				unsigned int R=255, unsigned int G=0, unsigned int B=0  )
        {
            add_box(X1-d/2,Y1-d/2,Y1-d/2,X1+d/2,Y1+d/2,Y1+d/2,R,G,B);
        }

	void add_box(float X1, float Y1, float Z1, float X2, float Y2, float Z2,
				unsigned int R=255, unsigned int G=0, unsigned int B=0 );

        void add_cylinder( float Rayon, float height,
                      Eigen::Matrix<double,3,1> position=Eigen::Matrix<double,3,1>::Zero(), 
                      Eigen::Matrix<double,3,1> orientation=Eigen::Matrix<double,3,1>::Zero(),
                      unsigned int R=255, unsigned int G=0, unsigned int B=0);

        void add_sphere( float X1, float Y1, float Z1, float Rayon,
                     unsigned int R=255, unsigned int G=0, unsigned int B=0);
        
        void get_bounded_box( double *xmin, double *ymin, double *zmin, double *xmax, double *ymax, double *zmax) const;

	void fusion(MogsGeometry* other_geom);

	void fusion ( const SpatialTransform < double >&transform,
			MogsGeometry* other_geom);

        void move(const Eigen::Matrix<double,3,1> pos,
                const Eigen::Matrix<double,3,1> rot);

	void print();

	void scale(const Eigen::Matrix<double,3,1> s);

	void convert_to_point_mesh(MogsGeometry* g, double precision=0.001) const;
        
        void setDiffuseColor(const Eigen::Matrix<double,3,1> & diffuseColor);

	void setSpecularColor(const Eigen::Matrix<double,3,1> & specularColor);

	void setEmissiveColor(const Eigen::Matrix<double,3,1> & emissiveColor);
        
        void setColor(const Eigen::Matrix<double,3,1> & Color);

	std::vector < Mesh > tab_mesh;

	private:

	friend class boost::serialization::access;
        
        friend class GLWidget;

	template < class Archive >
		void serialize (Archive & ar,
				const unsigned int version)
	{
		ar & tab_mesh;
	}
};

#endif
