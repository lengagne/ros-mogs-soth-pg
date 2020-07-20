//      ShapeBuilder.cpp
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
//      2012-2013: IUT de Beziers/ LIRMM, Beziers, France
//	from 2013 : Université Blaise Pascal / axis : ISPR / theme MACCS

#include <cmath>

#include "ShapeBuilder.h"

// --------------------------------------------------------------
// --- ShapeBuilder implementation ------------------------------
// --------------------------------------------------------------

ShapeBuilder::ShapeBuilder()
	:	R_(Eigen::Matrix<double,3,3>(Eigen::Matrix<double,3,3>::Identity()))
	,	p_(Eigen::Matrix<double,3,1> (0.,0.,0.))
{
}

ShapeBuilder::~ShapeBuilder()
{
}

void ShapeBuilder::setRotation(const Eigen::Matrix<double,3,3> &R)
{
	R_ = R;
}

void ShapeBuilder::setTranslation(const Eigen::Matrix<double,3,1>  &p)
{
	p_ = p;
}

// --------------------------------------------------------------
// --- CylinderBuilder implementation ---------------------------
// --------------------------------------------------------------

CylinderBuilder::CylinderBuilder(
	unsigned res, double rb, double rt, double length,
	const std::string& covb, const std::string& covt,
	char axis)
	: res_(res)
	, rb_(rb)
	, rt_(rt)
	, length_(length)
	, covb_(covb)
	, covt_(covt)
	, axis_(axis)
{
}

void CylinderBuilder::buildDisplayModel(
	std::vector<Eigen::Matrix<double,3,1> >& points,
	std::vector<Eigen::Matrix<unsigned int,3,1> >& faces,
	std::vector<Eigen::Matrix<double,3,1> >& normals)
{
	Eigen::Matrix<double,3,3> oldR = R_;

	// The cylinder is oriented along the z axis by default.
	//  if the axis is different, we change its orientation.
	if (axis_ == 'x' || axis_ == 'X')
	{
		Eigen::Matrix<double,3,3> orientChange;
		orientChange << 0,0,1, 0,1,0, -1,0,0;
		R_ = R_ * orientChange;
	}
	else if (axis_ == 'y' || axis_ == 'Y')
	{
		Eigen::Matrix<double,3,3> orientChange;
		orientChange << 1,0,0, 0,0,1, 0,-1,0;
		R_ = R_ * orientChange;
	}
	Eigen::Matrix<double,3,1>  c(0.,0.,0.), n(0.,0.,0.);
	double slices = 2.*M_PI / static_cast<double>(res_);
	double angle = 0;

	if((rb_ == 0. && rt_ == 0.) || length_ == 0.)
	{
		std::cerr<<"Parsed cylinder is wrong"<<std::endl;
		exit(-1);
	}

	// two similar treatments for cylinder top or base
	double r = 0., sgn = 0.;

	// vertices + normals of the cylinder
	for(int k = 0; k < 2; ++k) // two same treatments for top or base
	{
		if (k == 0)
		{
			r = rb_;
			sgn = -1.;
		}
		else
		{
			r = rt_;
			sgn = 1.;
		}

		for(unsigned i = 0; i < res_; ++i)
		{
			angle = slices * i;
			// normals
			n = Eigen::Matrix<double,3,1> (sin(angle), cos(angle), (rt_-rb_)/length_);
			// vertices
			c(0) = r*n(0);
			c(1) = r*n(1);
			c(2) = sgn*length_/2.0;
			// static transformations of vertices + normals
			c = R_*c + p_;
			n = R_*n;
			// ensure that the normals are normalized
			if(n.norm() > 1e-24) n.normalize();
			// store vertices and normals
			points.push_back(c);
			normals.push_back(n);
		}
	}

	unsigned i = 0;
	for (; i < res_-1; ++i)
	{
		faces.push_back(Eigen::Matrix<unsigned int,3,1> (res_+i, i+1, i));
		faces.push_back(Eigen::Matrix<unsigned int,3,1> (res_+i+1, i+1, res_+i));
	}
	// close the cylinder
	faces.push_back(Eigen::Matrix<unsigned int,3,1> (res_+i, 0, i));
	faces.push_back(Eigen::Matrix<unsigned int,3,1> (res_, 0, res_+i));

	// Draw the disk covers

	if (covb_ == "Disk" && rb_ != 0.0)
	{
		unsigned k = points.size();
		// duplicate the vertices
		points.insert(points.end(), points.begin(), points.begin()+res_);
		// put related normal (same for all)
		n(0)=0.0; n(1) = 0.0; n(2) = -1.0;
		n = R_*n;
		if(n.norm() > 1e-24) n.normalize(); // ensure the normal is normalized
		normals.insert(normals.end(), res_, n);
		// make faces
		for(i = k+1; i+1 < points.size()-1; i++)
			faces.push_back(Eigen::Matrix<unsigned int,3,1> (i+1, i, k));
	}

	if (covt_ == "Disk" && rt_ != 0.0)
	{
		unsigned k = points.size();
		// duplicate the vertices
		points.insert(points.end(), points.begin()+res_, points.begin()+2*res_);
		// put related normal (same for all)
		n(0)=0.0; n(1) = 0.0; n(2) = 1.0;
		n = R_*n;
		if(n.norm() > 1e-24) n.normalize(); // ensure the normal is normalized
		normals.insert(normals.end(), res_, n);
		// make faces
		for(i = k+1; i < points.size()-1; i++)
			faces.push_back(Eigen::Matrix<unsigned int,3,1> (k, i, i+1));
	}

	if ( points.size() != normals.size())
	{
		std::cerr<<"CylinderBuilder::BuildDisplayModel -> normals and points sizes do not match"<<std::endl;
		exit(-1);
	}

	// Restore old value
	R_ = oldR;
}


// --------------------------------------------------------------
// --- BoxBuilder implementation --------------------------------
// --------------------------------------------------------------

BoxBuilder::BoxBuilder(double length, double width, double height)
	: length_(length)
	, width_(width)
	, height_(height)
{
}

void BoxBuilder::buildDisplayModel(
	std::vector<Eigen::Matrix<double,3,1> >& points,
	std::vector<Eigen::Matrix<unsigned int,3,1> >& faces,
	std::vector<Eigen::Matrix<double,3,1> >& normals)
{

	Eigen::Matrix<double,3,1>  c(0.,0.,0.), n(0.,0.,0.);

	if (width_ == 0. || length_ == 0. || height_ == 0.)
	{
		std::cerr<<"Parsed box is wrong"<<std::endl;
		exit(-1);
	}

	//--- setting the eight vertices
	c = Eigen::Matrix<double,3,1> ( width_/2., -length_/2.,  height_/2.);
	c = R_*c + p_;
	points.push_back(c);
	points.push_back(c);
	points.push_back(c);

	c = Eigen::Matrix<double,3,1> ( width_/2.,  length_/2.,  height_/2.);
	c = R_*c + p_;
	points.push_back(c);
	points.push_back(c);
	points.push_back(c);

	c = Eigen::Matrix<double,3,1> (-width_/2.,  length_/2.,  height_/2.);
	c = R_*c + p_;
	points.push_back(c);
	points.push_back(c);
	points.push_back(c);

	c = Eigen::Matrix<double,3,1> (-width_/2., -length_/2.,  height_/2.);
	c = R_*c + p_;
	points.push_back(c);
	points.push_back(c);
	points.push_back(c);

	c = Eigen::Matrix<double,3,1> ( width_/2., -length_/2., -height_/2.);
	c = R_*c + p_;
	points.push_back(c);
	points.push_back(c);
	points.push_back(c);

	c = Eigen::Matrix<double,3,1> ( width_/2.,  length_/2., -height_/2.);
	c = R_*c + p_;
	points.push_back(c);
	points.push_back(c);
	points.push_back(c);

	c = Eigen::Matrix<double,3,1> (-width_/2.,  length_/2., -height_/2.);
	c = R_*c + p_;
	points.push_back(c);
	points.push_back(c);
	points.push_back(c);

	c = Eigen::Matrix<double,3,1> (-width_/2., -length_/2., -height_/2.);
	c = R_*c + p_;
	points.push_back(c);
	points.push_back(c);
	points.push_back(c);

	normals.resize(points.size());

	//--- drawing faces

	// front
	faces.push_back(Eigen::Matrix<unsigned int,3,1> (12, 3, 0));
	faces.push_back(Eigen::Matrix<unsigned int,3,1> (12, 15, 3));
	n(0) = 1.; n(1)= 0.; n(2) = 0.;
	n = R_*n;
	if(n.norm() > 1e-24) n.normalize(); // in case R is not orthogonal
	normals[0] = n;
	normals[3] = n;
	normals[12] = n;
	normals[15] = n;

	// right side
	faces.push_back(Eigen::Matrix<unsigned int,3,1> (16, 6, 4));
	faces.push_back(Eigen::Matrix<unsigned int,3,1> (16, 18, 6));
	n(0) = 0.; n(1)= 1.; n(2) = 0.;
	n = R_*n;
	if(n.norm() > 1e-24) n.normalize(); // in case R is not orthogonal
	normals[4] = n;
	normals[6] = n;
	normals[16] = n;
	normals[18] = n;

	// back side
	faces.push_back(Eigen::Matrix<unsigned int,3,1> (19, 21, 9));
	faces.push_back(Eigen::Matrix<unsigned int,3,1> (19, 9, 7));
	n(0) = -1.; n(1)= 0.; n(2) = 0.;
	n = R_*n;
	if(n.norm() > 1e-24) n.normalize(); // in case R is not orthogonal
	normals[7] = n;
	normals[9] = n;
	normals[19] = n;
	normals[21] = n;

	// left side
	faces.push_back(Eigen::Matrix<unsigned int,3,1> (22, 13, 1));
	faces.push_back(Eigen::Matrix<unsigned int,3,1> (22, 1, 10));
	n(0) = 0.; n(1)= -1.; n(2) = 0.;
	n = R_*n;
	if(n.norm() > 1e-24) n.normalize(); // in case R is not orthogonal
	normals[1] = n;
	normals[10] = n;
	normals[13] = n;
	normals[22] = n;

	// top side
	faces.push_back(Eigen::Matrix<unsigned int,3,1> (2, 5, 11));
	faces.push_back(Eigen::Matrix<unsigned int,3,1> (5, 8, 11));
	n(0) = 0.; n(1)= 0.; n(2) = 1.;
	n = R_*n;
	if(n.norm() > 1e-24) n.normalize(); // in case R is not orthogonal
	normals[2] = n;
	normals[5] = n;
	normals[8] = n;
	normals[11] = n;

	// bottom side
	faces.push_back(Eigen::Matrix<unsigned int,3,1> (14, 23, 20));
	faces.push_back(Eigen::Matrix<unsigned int,3,1> (14, 20, 17));
	n(0) = 0.; n(1)= 0.; n(2) = -1.;
	n = R_*n;
	if(n.norm() > 1e-24) n.normalize(); // in case R is not orthogonal
	normals[14] = n;
	normals[17] = n;
	normals[20] = n;
	normals[23] = n;
}


// --------------------------------------------------------------
// --- SphereBuilder implementation -----------------------------
// --------------------------------------------------------------

SphereBuilder::SphereBuilder(unsigned res, double ray)
	: res_(res)
	, ray_(ray)
{
}

void SphereBuilder::buildDisplayModel(std::vector<Eigen::Matrix<double,3,1> >& points,
                                      std::vector<Eigen::Matrix<unsigned int,3,1> >& faces,
                                      std::vector<Eigen::Matrix<double,3,1> >& normals)
{
	double slices;
	//double xyslices;

	Eigen::Matrix<double,3,1>  c(0,0,0), n(0,0,0);

	unsigned i = 0, j = 0;

	if(ray_ == .0 || res_ < 3)
	{
		std::cerr<<"Parsed sphere is wrong"<<std::endl;
		exit(-1);
	}

	// slices
	slices = M_PI/((double)(res_));

	// we put the bottom point
	c(0) = 0.; c(1) = 0.; c(2) = -ray_;
	points.push_back(c + p_);
	n(0) = 0.; n(1) = 0.; n(2) = -1.;
	normals.push_back(n);

	for(i = 1; i < res_; i++)
	{
		n(2) = cos(-M_PI+i*slices);
		c(2) = ray_*n(2);
		for(j = 0; j < res_*2; j++)
		{
			double stack = sin(-M_PI+i*slices);
			n(0) = stack*sin(j*slices);
			n(1) = stack*cos(j*slices);
			c(0) = ray_*n(0);
			c(1) = ray_*n(1);
			points.push_back(c + p_);
			normals.push_back(n);
		}
	}

	// we put the top point
	c(0) = 0.; c(1) = 0.; c(2) = ray_;
	points.push_back(c + p_);
	n(0) = 0.; n(1) = 0.; n(2) = 1.;
	normals.push_back(n);

	// Faces, starting from bottom to up
	for(j = 1; j < 2*res_; j++)
		faces.push_back(Eigen::Matrix<unsigned int,3,1> (0, j+1, j));
	faces.push_back(Eigen::Matrix<unsigned int,3,1> (0, 1, j));

	i = 1;
	for(j = 1; j < 2*res_*(res_-2)+1; j++)
	{
		if (j == i*(2*res_))
		{
			faces.push_back(Eigen::Matrix<unsigned int,3,1> (j, j+1, j+(2*res_)));
			faces.push_back(Eigen::Matrix<unsigned int,3,1> (j, j-(2*res_)+1, j+1));
			i++;
		}
		else
		{
			faces.push_back(Eigen::Matrix<unsigned int,3,1> (j, j+(2*res_)+1, j+(2*res_)));
			faces.push_back(Eigen::Matrix<unsigned int,3,1> (j, j+1, j+(2*res_)+1));
		}
	}

	for(i = j; i < j+2*res_-1; i++)
		faces.push_back(Eigen::Matrix<unsigned int,3,1> (i, i+1, 2*res_*(res_-1)+1));
	faces.push_back(Eigen::Matrix<unsigned int,3,1> (i, j, 2*res_*(res_-1)+1));
}

//--- superquadrics methods

SuperQuadricBuilder::SuperQuadricBuilder(unsigned res, std::string sqType, double ax, double ay, double az, double e1, double e2, double ar, std::string gds)
	: res_(res)
	, sqType_(sqType)
	, ax_(ax)
	, ay_(ay)
	, az_(az)
	, e1_(e1)
	, e2_(e2)
	, ar_(ar)
	, gds_(gds)
{
}

double signe(double& x)
{
	if (x >= 0.0) return 1.0;
	else return -1.0;
}

void SuperQuadricBuilder::buildDisplayModel(std::vector<Eigen::Matrix<double,3,1> >& points,
                                            std::vector<Eigen::Matrix<unsigned int,3,1>  >& faces,
                                            std::vector<Eigen::Matrix<double,3,1> >& normals)
{
	Eigen::Matrix<double,3,1>  c, n;
	double cu, su, cv, sv;
	double tc, tn;
	double u, v;
	unsigned i, j;

	if(ax_ == 0.0 || ay_ == 0.0 || az_ == 0.0 || e1_ <= 0.0 || e2_ <= 0.0 || ar_ == 0.0 || res_ < 3)
	{
		std::cerr<<"Parsed superquadrics is wrong"<<std::endl;
		exit(-1);
	}

	// SuperEllipsoid case
	if(sqType_ == "se")
	{
		// build vertices and normals
		for(i = 0; i <= res_; i++)
		{
			u = -M_PI/2.0 + (double)(i)*M_PI/(double)(res_);
			cu = cos(u);
			su = sin(u);

			tc = signe(cu)*pow(fabs(cu), e1_);
			c(2) = az_*signe(su)*pow(fabs(su), e1_);

			tn = signe(cu)*pow(fabs(cu), 2.0 - e1_);
			// not so smart to use again u to store the z normal instead of a new double
			// not able to use directly n(2) = u = signe(su)*pow(fabs(su), 2.0-e1_)/az_;
			// because the value will be changed by normalize operation
			u = signe(su)*pow(fabs(su), 2.0-e1_)/az_;

			for(j = 0; j <= 2*res_; j++)
			{
				v = -M_PI + (double)(j)*M_PI/(double)(res_);
				cv = cos(v);
				sv = sin(v);

				c(0) = ax_*tc*signe(cv)*pow(fabs(cv), e2_);
				c(1) = ay_*tc*signe(sv)*pow(fabs(sv), e2_);

				points.push_back(c + p_);

				n(0) = tn*signe(cv)*pow(fabs(cv), 2.0-e2_)/ax_;
				n(1) = tn*signe(sv)*pow(fabs(sv), 2.0-e2_)/ay_;
				// if I do not use an intermediate variable the value of n(2) stored
				// prior to this loop will be changed by normalizing operation
				n(2) = u;
				if(n.norm() > 1e-24) n.normalize();

				normals.push_back(n);
			}
		}
		// build triangular faces
		for(unsigned i = 0; i < (2*res_+1)*(res_); i++)
		{
			faces.push_back(Eigen::Matrix<unsigned int,3,1> (i, i+1, i+2*res_+1));
			faces.push_back(Eigen::Matrix<unsigned int,3,1> (i, i+2*res_+1, i+2*res_));
		}
	}


	// SuperHyperbolic of 1 sheet
	if(sqType_ == "sh1")
	{
		// build vertices and normals
		for(i = 0; i <= res_; i++)
		{
			u = -M_PI/2.0 + (double)(i)*M_PI/(double)(res_);
			cu = 1/cos(u);
			su = tan(u);

			tc = signe(cu)*pow(fabs(cu), e1_);
			c(2) = az_*signe(su)*pow(fabs(su), e1_);

			tn = signe(cu)*pow(fabs(cu), 2.0 - e1_);
			// not so smart to use again u to store the z normal instead of a new double
			// not able to use directly n(2) = u = signe(su)*pow(fabs(su), 2.0-e1_)/az_;
			// because the value will be changed by normalize operation
			u = signe(su)*pow(fabs(su), 2.0-e1_)/az_;

			for(j = 0; j <= 2*res_; j++)
			{
				v = -M_PI + (double)(j)*M_PI/(double)(res_);
				cv = cos(v);
				sv = sin(v);

				c(0) = ax_*tc*signe(cv)*pow(fabs(cv), e2_);
				c(1) = ay_*tc*signe(sv)*pow(fabs(sv), e2_);

				points.push_back(c + p_);

				n(0) = tn*signe(cv)*pow(fabs(cv), 2.0-e2_)/ax_;
				n(1) = tn*signe(sv)*pow(fabs(sv), 2.0-e2_)/ay_;
				// if I do not use an intermediate variable the value of n(2) stored
				// prior to this loop will be changed by normalizing operation
				n(2) = u;
				if(n.norm() > 1e-24) n.normalize();

				normals.push_back(n);
			}
		}
		// build triangular faces
		for(unsigned i = 0; i < (2*res_+1)*(res_); i++)
		{
			faces.push_back(Eigen::Matrix<unsigned int,3,1> (i, i+1, i+2*res_+1));
			faces.push_back(Eigen::Matrix<unsigned int,3,1> (i, i+2*res_+1, i+2*res_));
		}
	}

	// SuperHyperbolic of 2 sheet
	if(sqType_ == "sh2")
	{
	}

	// SuperToroid
	if(sqType_ == "st")
	{
		// adjust
		/*
		double ax = 1/(ax_+ar_);
		double ay = 1/(ay_+ar_);
		double az = 1/(az_+ar_);
		*/

		// build vertices and normals
		for(unsigned i = 0; i <= 2*res_; i++)
		{
			double u = -M_PI + (double)(i)*M_PI/(double)(res_);
			double cu = cos(u); double su = sin(u);
			c(2) = az_*signe(su)*pow(fabs(su), e1_);

			double tc = 0.1 + 0.5*signe(cu)*pow(fabs(cu), e1_);

			for(unsigned j = 0; j <= 2*res_; j++)
			{
				double v = -M_PI + (double)(j)*M_PI/(double)(res_);

				double cv = cos(v); double sv = sin(v);

				c(0) = ax_*tc*signe(cv)*pow(fabs(cv), e2_);
				c(1) = ay_*tc*signe(sv)*pow(fabs(sv), e2_);
				points.push_back(c + p_);

				double t = signe(cu)*pow(fabs(cu), 2.0 - e1_);
				n(0) = t*signe(cv)*pow(fabs(cv), 2.0-e2_)/ax_;
				n(1) = t*signe(sv)*pow(fabs(sv), 2.0-e2_)/ay_;
				n(2) = signe(su)*pow(fabs(su), 2.0-e1_)/az_;
				if(n.norm() > 1e-24) n.normalize();
				normals.push_back(n);
			}
		}
		// build triangular faces
		for(unsigned i = 0; i < (2*res_+1)*(2*res_); i++)
		{
			faces.push_back(Eigen::Matrix<unsigned int,3,1> (i, i+1, i+2*res_+1));
			faces.push_back(Eigen::Matrix<unsigned int,3,1> (i, i+2*res_+1, i+2*res_));
		}
		/*
		for(unsigned i = 0; i < (res_+1)*(res_); i++)
		{
			faces.push_back(Eigen::Matrix<int,3,1> (i, i+1, i+res_+1));
			faces.push_back(Eigen::Matrix<int,3,1> (i, i+res_+1, i+res_));
		}*/
	}
}
