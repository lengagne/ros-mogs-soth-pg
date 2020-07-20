//      Mesh.h
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
//
//	See README

#ifndef _MESH_H_
#define _MESH_H_

#include <cstdlib>

#include <vector>
#include <functional>

#include "types.h"

#ifdef	Boost_SERIALIZATION_FOUND
#include "MogsSerialization.h"
#endif

#include <QObject>
#include <QMatrix4x4>
#include <QVector3D>
#include <QColor>

#ifdef VISU_TO_DO
#include <GL/gl.h>
#endif
// --- CLASS ----------------------------------------------------

//! [0]
struct Geometry
{
    QVector<unsigned short> faces;
    QVector<QVector3D> vertices;
    QVector<QVector3D> normals;
    void appendSmooth(const QVector3D &a, const QVector3D &n, int from);
    void appendFaceted(const QVector3D &a, const QVector3D &n);
    void finalize();

};
//! [0]

class Patch
{
public:
    enum Smoothing { Faceted, Smooth };
    Patch(Geometry *);
    void setSmoothing(Smoothing s) { sm = s; }
    void translate(const QVector3D &t);
    void rotate(qreal deg, QVector3D axis);
    void addTri(const QVector3D &a, const QVector3D &b, const QVector3D &c, const QVector3D &n);
    void addQuad(const QVector3D &a, const QVector3D &b,  const QVector3D &c, const QVector3D &d);

    unsigned short start;
    unsigned short count;
    unsigned short initv;

//     #if VISU_TO_DO
//     GLfloat faceColor[4]; // dans Visu
//     #endif
    float faceColor[4]; // dans Visu
    QMatrix4x4 mat;
    Smoothing sm;
    Geometry *geom;
};

class Mesh;

class sub_Mesh
{
public:
	sub_Mesh();

	sub_Mesh(Mesh* parent);

	~sub_Mesh();
	Mesh* parent_;

	std::vector<Eigen::Matrix<double,3,1> > points_;
	std::vector<Eigen::Matrix<unsigned int,3,1>  > faces_;
	std::vector<boost::numeric::ublas::vector<double> > texPoints_;
	std::vector<Eigen::Matrix<unsigned int,3,1>  > texFaces_;
	std::vector<Eigen::Matrix<double,3,1> > normals_;
	std::vector<Eigen::Matrix<double,3,1> > normals_faces_;

	float ambientIntensity_;
	Eigen::Matrix<double,3,1>  diffuseColor_;
	Eigen::Matrix<double,3,1>  specularColor_;
	Eigen::Matrix<double,3,1>  emissiveColor_;
	float shininess_;
	float transparency_;

	bool valid_;
	bool attrValid_;

	void affine(double precision, bool create_face=true);

	bool affine_face(const Eigen::Matrix<unsigned int,3,1> face, double precision);

        bool one_step_affine(double precision);


 	void move(	const Eigen::Matrix<double,3,1> & pos,
                const Eigen::Matrix<double,3,3> & rot);

    QList<Patch *> parts;
    Geometry *geom;

#ifdef Boost_SERIALIZATION_FOUND

	friend class boost::serialization::access;

	template < class Archive >
	void serialize (Archive & ar,const unsigned int version)
	{

		// Those values must be recomputed
		ar & points_;
		ar & faces_;
		ar & texPoints_;
		ar & texFaces_;
		ar & normals_;
		ar & normals_faces_;


		ar & ambientIntensity_;
		ar & diffuseColor_;
		ar & specularColor_;
		ar & emissiveColor_;
		ar & shininess_;
		ar & transparency_;
		ar & valid_;
		ar & attrValid_;
//		ar & parts;
//		ar & geom;
	}
#endif
};


class Mesh
{

public:

	Mesh( );

	Mesh(const mogs_string & path,
	     const mogs_string & filename);

	Mesh(const mogs_string & filename);

	~Mesh();

 	void move(	const Eigen::Matrix<double,3,1> & pos,
                const Eigen::Matrix<double,3,3> & rot);

	void parse(	const mogs_string & path,
                const mogs_string & filename);

	void parse(	const mogs_string & filename);

    void parse_dae(const mogs_string & filename);

    void parse_stl(const mogs_string & filename);

	void parse_vrml(const mogs_string & filename);

	void print_info();

	int get_nb_sub_mesh() const
	{
		return subGeometries_.size();
	}

	unsigned int get_nb_point_sub_mesh (unsigned int i) const
	{
		return subGeometries_[i]->points_.size();
	}

	// Creation of display lists

	// if the normals have not been computed yet, compute them
	void compileMesh();
	void compileMesh(unsigned i);

	// parameters for parametric shapes
	bool isParametric(unsigned i) const;
	void setParameters(const std::vector<double>& parameters, unsigned i);
	int getParameters(std::vector<double>& parameters, unsigned i) const;

	// accessors for geometry

	void getPoints(std::vector<Eigen::Matrix<double,3,1> >& points, unsigned i) const;
	void getFaces(std::vector<Eigen::Matrix<unsigned int,3,1> >& faces, unsigned i) const;
	void getNormals(std::vector<Eigen::Matrix<double,3,1> >& normals, unsigned i) const;
	void getTexPoints(std::vector<boost::numeric::ublas::vector<double> >& texPoints,unsigned i) const;
	void getTexFaces(std::vector<Eigen::Matrix<unsigned int,3,1>  >& texFaces, unsigned i) const;

	void setPoints(const std::vector<Eigen::Matrix<double,3,1> >& points, unsigned i);
	void setFaces(const std::vector<Eigen::Matrix<unsigned int,3,1> >& faces, unsigned i);
	void setNormals(const std::vector<Eigen::Matrix<double,3,1> >& normals, unsigned i);
	void setTexPoints(const std::vector<boost::numeric::ublas::vector<double> >& texPoints,	unsigned i);
	void setTexFaces(const std::vector<Eigen::Matrix<unsigned int,3,1>  >& texFaces, unsigned i);

	// creation and manipulation of subgeometries

	void createNewSubGeometries(unsigned size);
	unsigned getNumSubGeometries() const;

	// points/faces/normals of subgeometries

	void addPoint(const Eigen::Matrix<double,3,1> & point, unsigned i);
	void addFace(const Eigen::Matrix<unsigned int,3,1> & face, unsigned i);
	void addNormal(const Eigen::Matrix<double,3,1> & normal, unsigned i);
	void addTexPoint(const boost::numeric::ublas::vector<double>& texPoint, unsigned i);
	void addTexFace(const Eigen::Matrix<unsigned int,3,1> & texFace, unsigned i);

	// appearance of subgeometries

	void setAmbientIntensity(float ambientIntensity, unsigned i);
	float getAmbientIntensity(unsigned i) const;
        
        void setDiffuseColor(const Eigen::Matrix<double,3,1> & diffuseColor);
        void setSpecularColor(const Eigen::Matrix<double,3,1> & specularColor);
        void setEmissiveColor(const Eigen::Matrix<double,3,1> & emissiveColor);

	void setDiffuseColor(const Eigen::Matrix<double,3,1> & diffuseColor, unsigned i);
	Eigen::Matrix<double,3,1>  getDiffuseColor(unsigned i) const;

	void setSpecularColor(const Eigen::Matrix<double,3,1> & specularColor, unsigned i);
	Eigen::Matrix<double,3,1>  getSpecularColor(unsigned i) const;

	void setEmissiveColor(const Eigen::Matrix<double,3,1> & emissiveColor, unsigned i);
	Eigen::Matrix<double,3,1>  getEmissiveColor(unsigned i) const;

	void setShininess(float shininess, unsigned i);
	float getShininess(unsigned i) const;

	void setTransparency(float transparency, unsigned i);
	float getTransparency(unsigned i) const;

	void scale(const Eigen::Matrix<double,3,1> s);

	private:

	struct s_compileSubMeshStructure : std::unary_function<sub_Mesh*, void>
	{
		void operator()(sub_Mesh* geometricStructure);
	} compileSubMeshStructure;

	struct s_computeNormalsOfGeometricStructure : std::unary_function<sub_Mesh*, void>
	{
		void operator()(sub_Mesh* geometricStructure);
	} computeNormalsOfGeometricStructure;

	struct s_deleteSubMesh : std::unary_function<sub_Mesh*, void>
	{
		void operator()(sub_Mesh*& geometricStructure);
	} deleteSubMesh;


#ifdef Boost_SERIALIZATION_FOUND

	friend class boost::serialization::access;

	template < class Archive >
		void serialize (Archive & ar,
				const unsigned int version)
	{
		ar & subGeometries_;
	}

#endif
// protected:	///FIXME
	public:
	std::vector<sub_Mesh*> subGeometries_;
};

#endif
