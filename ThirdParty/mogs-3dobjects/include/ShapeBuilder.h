//      ShapeBuilder.h
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

#ifndef _AF_SHAPE_BUILDER_H_
#define _AF_SHAPE_BUILDER_H_

#include <vector>

#include "types.h"

// --- ShapeBuilder interface -----------------------------------

class ShapeBuilder
{
public:

	ShapeBuilder();
	virtual ~ShapeBuilder() = 0;

	virtual void buildDisplayModel(
		std::vector<Eigen::Matrix<double,3,1> >& points,
		std::vector<Eigen::Matrix<unsigned int,3,1> >& faces,
		std::vector<Eigen::Matrix<double,3,1> >& normals) = 0;

	void setRotation(const Eigen::Matrix<double,3,3> & R);
	void setTranslation(const Eigen::Matrix<double,3,1> & p);

protected:

	Eigen::Matrix<double,3,3> R_;
	Eigen::Matrix<double,3,1> p_;
};


// --- CylinderBuilder class ------------------------------------

class CylinderBuilder
	: public ShapeBuilder
{
public:

	CylinderBuilder(
		unsigned res, double rb, double rt, double length,
		const std::string& covb, const std::string& covt, char axis);

	void buildDisplayModel(
		std::vector<Eigen::Matrix<double,3,1> >& points,
		std::vector<Eigen::Matrix<unsigned int,3,1> >& faces,
		std::vector<Eigen::Matrix<double,3,1> >& normals);

private:

	unsigned res_;
	double rb_, rt_, length_;
	std::string covb_, covt_;
	char axis_;
};

// --- BoxBuilder class -----------------------------------------

class BoxBuilder
	: public ShapeBuilder
{
public:

	BoxBuilder(double length, double width, double height);

	void buildDisplayModel(
		std::vector<Eigen::Matrix<double,3,1> >& points,
		std::vector<Eigen::Matrix<unsigned int,3,1> >& faces,
		std::vector<Eigen::Matrix<double,3,1> >& normals);

private:

	double length_, width_, height_;
};

// --- SphereBuilder class --------------------------------------

class SphereBuilder
	: public ShapeBuilder
{
public:

	SphereBuilder(unsigned res, double ray);
	void buildDisplayModel(
		std::vector<Eigen::Matrix<double,3,1> >& points,
		std::vector<Eigen::Matrix<unsigned int,3,1> >& faces,
		std::vector<Eigen::Matrix<double,3,1> >& normals);

private:

	unsigned res_;
	double ray_;
};

//--- superquadrics class ---------------------------------------

class SuperQuadricBuilder
: public ShapeBuilder
{
public:

	SuperQuadricBuilder(unsigned res, std::string sqType,
		double ax, double ay, double az, double e1, double e2,
		double ar, std::string gds);
	void buildDisplayModel(
		std::vector<Eigen::Matrix<double,3,1> >& points,
		std::vector<Eigen::Matrix<unsigned int,3,1> >& faces,
		std::vector<Eigen::Matrix<double,3,1> >& normals);

private:

	unsigned res_;
	std::string sqType_;
	double ax_, ay_, az_, e1_, e2_, ar_;
	std::string gds_;
};

#endif

// --- DOC --------------------------------------------------------

/*!
* \file ShapeBuilder.h
* \brief Declaration of class afstate::ShapeBuilder.
* \author Abderrahmane Kheddar and Paul Evrard
* \version 0.0.0
*
* Declaration of the afstate::ShapeBuilder class.
*
*/

// SHAPEBUILDER DOC //

/*!
* \class afstate::ShapeBuilder ShapeBuilder.h "Definition"
* \brief builds shapes such as cylinder, box, sphere...
* \var matrix3d afstate::ShapeBuilder::R_
* \brief rotation matrix.
* \var Eigen::Matrix<double,3,1> afstate::ShapeBuilder::p_
* \brief translation vector.
*/

/*!
* \fn 	AFSTATE_API virtual afstate::ShapeBuilder::~ShapeBuilder() = 0;
* \brief virtual destructor of ShapeBuilder.
*/

/*!
* \fn AFSTATE_API void afstate::ShapeBuilder::setRotation(const matrix3d& R)
* \brief sets a rotation matrix \a R to the shape.
* \param R rotation matrix to apply.
*/

/*!
* \fn AFSTATE_API void afstate::ShapeBuilder::setTranslation(const Eigen::Matrix<double,3,1>& p)
* \brief sets a translation vector \a p to the shape.
* \param p translation vector to apply.
*/

/*!
* \fn AFSTATE_API virtual void afstate::ShapeBuilder::buildDisplayModel(
			std::vector<Eigen::Matrix<double,3,1> >& points,
			std::vector<Eigen::Matrix<int,3,1> >& faces,
			std::vector<Eigen::Matrix<double,3,1> >& normals)
* \brief builds a display model for the shape.
* \param points vector of points used for the display model.
* \param faces vector of faces used for the display model.
* \param normals vector of normals used for the display model.
*/

// CYLINDERBUILDER DOC //

/*!
* \class afstate::CylinderBuilder
* \brief builds a cylinder.
* \var unsigned afstate::CylinderBuilder::res_
* \brief resolution.
* \var double afstate::CylinderBuilder::rb_
* \brief bottom radius.
* \var double afstate::CylinderBuilder::rt_
* \brief top radius.
* \var double afstate::CylinderBuilder::length_
* \brief length.
* \var std::string afstate::CylinderBuilder::covb_
* \brief bottom cover (e.g. Disk).
* \var std::string afstate::CylinderBuilder::covt_
* \brief top cover (e.g. Disk).
* \var char afstate::CylinderBuilder::axis_
* \brief Axis of the cylinder
*/

/*!
* \fn AFSTATE_API afstate::CylinderBuilder::CylinderBuilder(
			unsigned res, double rb, double rt, double length,
			const std::string& covb, const std::string& covt)
* \brief constructor of CylinderBuilder.
* \param res resolution.
* \param rb bottom radius.
* \param rt top radius.
* \param length length of the cylinder.
* \param covb bottom cover (e.g. Disk).
* \param covt top cover (e.g. Disk).
*
*/


// BOXBUILDER DOC //

/*!
* \class afstate::BoxBuilder
* \brief builds a box.
* \var double afstate::BoxBuilder::length_
* \brief length of the box.
* \var double afstate::BoxBuilder::width_
* \brief width of the box.
* \var double afstate::BoxBuilder::height_
* \brief height of the box.
*/

/*!
* \fn AFSTATE_API afstate::BoxBuilder::BoxBuilder(double length, double width, double height)
* \brief constructor of BoxBuilder.
* \param length length of the box.
* \param width width of the box.
* \param height height of the box.
*/

// SPHEREBUILDER DOC //

/*!
* \class afstate::SphereBuilder
* \brief builds a sphere.
* \var unsigned afstate::SphereBuilder::res_
* \brief resolution of the sphere.
* \var double afstate::SphereBuilder::ray_
* \brief radius of the sphere.
*/

/*!
* \fn AFSTATE_API afstate::SphereBuilder::SphereBuilder(unsigned res, double ray)
* \brief constructor of SphereBuilder.
* \param res resolution.
* \param ray radius of the sphere.
*
*/

// SUPERQUADRICBUILDER DOC //

/*!
* \class afstate::SuperQuadricBuilder
* \brief builds a superquadric.
* \see For more information : http://en.wikipedia.org/wiki/Superquadrics
* \var unsigned afstate::SuperQuadricBuilder::res_
* \brief resolution.
* \var std::string afstate::SuperQuadricBuilder::sqType_
* \brief type of the superquadric : se (superellipsoid), sh1 (superhyperbolic of 1 sheet),
* sh2 (superhyperbolic of 2 sheet).
* \var double afstate::SuperQuadricBuilder::ax_
* \brief coefficient on x.
* \var double afstate::SuperQuadricBuilder::ay_
* \brief coefficient on y.
* \var double afstate::SuperQuadricBuilder::az_
* \brief coefficient on z.
* \var double afstate::SuperQuadricBuilder::e1_
* \brief first exponent.
* \var double afstate::SuperQuadricBuilder::e2_
* \brief second exponent.
* \var double afstate::SuperQuadricBuilder::ar_
* \brief
* \var std::string afstate::SuperQuadricBuilder::gds_
* \brief
*/

/*!
* \fn AFSTATE_API afstate::SuperQuadricBuilder::SuperQuadricBuilder	(
			unsigned res, std::string sqType,
			double ax, double ay, double az, double e1, double e2,
			double ar, std::string gds)
* \brief constructor of SuperQuadricBuilder.
* \param res resolution.
* \param sqType type of the superquadric : se (superellipsoid), sh1
* (superhyperbolic of 1 sheet), sh2 (superhyperbolic of 2 sheet).
* \param ax coefficient on x.
* \param ay coefficient on y.
* \param az coefficient on z.
* \param e1 first exponent.
* \param e2 second exponent.
* \param ar
* \param gds
*/

