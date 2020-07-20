//      MogsGeometry.cpp
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

#include "MogsGeometry.h"
#include "MogsOctTree.h"

#include <sstream>
#include <assert.h>
#include <math.h>

// #define DEBUG_VISU 1

// namespace RigidBodyDynamics
// {
// ******************************************************************************
// *********
// ********* MogsGeometry constructors / destructors
// *********
// ******************************************************************************

	MogsGeometry::MogsGeometry ()
	{
		tab_mesh.clear();
	}

	MogsGeometry::MogsGeometry (MogsGeometry * other_geom)
	{
		tab_mesh.clear();

		if (other_geom)
		{
			int n = other_geom->tab_mesh.size();
			for(int i=0;i<n;i++)
			{
				Mesh  newMesh = other_geom->tab_mesh[i];
				add(newMesh);
			}
		}
	}

    MogsGeometry::MogsGeometry (const QString &file)
    {
        tab_mesh.clear();
        load(file);
    }

	MogsGeometry::~MogsGeometry ()
	{
		tab_mesh.clear();
	}

    void MogsGeometry::add_box( float X1, float Y1, float Z1, float X2, float Y2, float Z2,
                                unsigned int R, unsigned int G, unsigned int B )
    {
        Mesh *meh = new Mesh();
        sub_Mesh *smeh = new sub_Mesh(meh);
        convert_box_to_mesh(smeh, X1, Y1, Z1, X2, Y2, Z2, R, G, B);

        // add subMehs in Mesh and Mesh in MogsGeometry
        meh->subGeometries_.push_back(smeh);
        this->add(*meh);
    }

    void MogsGeometry::add_cylinder( float Rayon, float height,
                                     Eigen::Matrix<double,3,1> position, Eigen::Matrix<double,3,1> orientation,
                                     unsigned int R, unsigned int G, unsigned int B)
     {
        Mesh *meh = new Mesh();
        sub_Mesh *smeh = new sub_Mesh(meh);
        convert_cylinder_to_mesh(smeh, Rayon, height, position, orientation,R,G,B);
        meh->subGeometries_.push_back(smeh);
        this->add(*meh);

     }

    void MogsGeometry::add_sphere( float X1, float Y1, float Z1, float Rayon,
                                    unsigned int R, unsigned int G, unsigned int B )
    {
        Mesh *meh = new Mesh();
        sub_Mesh *smeh = new sub_Mesh(meh);
        convert_sphere_to_mesh(smeh, X1, Y1, Z1, Rayon, R, G, B);

        // add subMehs in Mesh and Mesh in MogsGeometry
        meh->subGeometries_.push_back(smeh);
        this->add(*meh);
    }

    void MogsGeometry::add_point_inside(double precision)
    {
//         std::cout<<"MogsGeometry::add_point start"<<std::endl;
        affine(precision,false);
        MogsOctTree * otree = new MogsOctTree(this);

        Eigen::Matrix<double,3,1> min,max;
        otree->get_min_max(min,max);

        Mesh m;
        m.createNewSubGeometries(1);

        for( double x = min(0); x < max(0); x+= precision)
        for( double y = min(1); y < max(1); y+= precision)
        {
            std::vector<double> z;
            z.clear();
            unsigned int nb = otree->get_nb_cross(x,y,z,precision);
            if (nb > 0 && nb %2 ==0)
            {

//                std::cout<<"x = "<< x <<"\ty= "<<y<<" \tnb = "<< nb<<"\t size = "<<z.size() <<std::endl;
                for (int i=0;i<z.size();i+=2)
                {
                    double zz = z[i];
                    double zz_max = z[i+1];
                    for( double zzz = zz; zzz<zz_max; zzz+= precision)
                    {
                        Eigen::Matrix<double,3,1> point(x,y,zzz);
                        m.addPoint(point,0);
//                        std::cout<<"adding point "<< point.transpose()<<std::endl;
                    }
                }
            }
        }

        add(m);

//         std::cout<<"MogsGeometry::add_point stop"<<std::endl;
    }

    void MogsGeometry::affine(double precision,bool create_face)
    {
//        std::cout<<"MogsGeometry::affine start"<<std::endl;
        unsigned int nb = tab_mesh.size();
//        std::cout<<"We are dealing with "<<nb<<" meshes"<<std::endl;
        for (unsigned  int i=0;i<nb;i++)
        {
            Mesh&  Mesh = tab_mesh[i];
            int nb_sub_mesh = Mesh.subGeometries_.size();
//            std::cout<<"mesh "<<i<<" has "<< nb_sub_mesh <<" sub meshes"<<std::endl;
            for (int j=0;j<nb_sub_mesh;j++)
                Mesh.subGeometries_[j]->affine(precision,create_face);
        }
//        std::cout<<"MogsGeometry::affine end"<<std::endl;
    }

	void MogsGeometry::convert_to_point_mesh(MogsGeometry* g, double precision) const
	{
        std::vector<Eigen::Matrix<double,3,1> > points;
        get_points(points);
//         std::cout<<"We will create "<< points.size()<<" points."<<std::endl;
        for (int i=0;i<points.size();i++)
//            g->add_sphere(points[i](0),points[i](1),points[i](2),precision);
            g->add_box(points[i](0)-precision/2,points[i](1)-precision/2,points[i](2)-precision/2,
                        points[i](0)+precision/2,points[i](1)+precision/2,points[i](2)+precision/2);
	}


	void MogsGeometry::get_bounded_box( double *xmin, double *ymin, double *zmin, double *xmax, double *ymax, double *zmax) const
        {
            std::vector<Eigen::Matrix<double,3,1> > points; 
            get_points(points);
            
            *xmin = *ymin = *zmin = 1e6;
            *xmax = *ymax = *zmax = -1e6;
            unsigned int np = points.size();
            for (int i=0;i<np;i++)
            {
                if(points[i](0) < *xmin)    *xmin = points[i](0);
                if(points[i](0) > *xmax)    *xmax = points[i](0);
                if(points[i](1) < *ymin)    *ymin = points[i](1);
                if(points[i](1) > *ymax)    *ymax = points[i](1);
                if(points[i](2) < *zmin)    *zmin = points[i](2);
                if(points[i](2) > *zmax)    *zmax = points[i](2);                
            }
        }

    void MogsGeometry::get_points( std::vector<Eigen::Matrix<double,3,1> > &points ) const
    {
        points.clear();
        int n = tab_mesh.size();
        for(int i=0;i<n;i++)
        {
            const Mesh&  Mesh = tab_mesh[i];
            int nb_sub_mesh = Mesh.subGeometries_.size();
            for (int j=0;j<nb_sub_mesh;j++)
                    {
                        sub_Mesh * sm = Mesh.subGeometries_[j];
                        int nbp = sm->points_.size();
                        for (int k=0;k<nbp;k++)
                        {
                            points.push_back(sm->points_[k]);
                        }
                    }
        }
    }

	void MogsGeometry::fusion ( MogsGeometry* other_geom)
	{
		if (other_geom)
		{
			int n = other_geom->tab_mesh.size();
			for(int i=0;i<n;i++)
			{
				Mesh  newMesh = other_geom->tab_mesh[i];
				add(newMesh);
			}
		}
	}

	void MogsGeometry::fusion ( const SpatialTransform < double >&transform,
                                MogsGeometry* other_geom)
	{
		if (other_geom)
		{
			int n = other_geom->tab_mesh.size();
			for(int i=0;i<n;i++)
			{
				Mesh newMesh = other_geom->tab_mesh[i];
				newMesh.move(transform.r, transform.E);
				add(newMesh);
			}
		}
	}

    void MogsGeometry::move(const Eigen::Matrix<double,3,1> pos,
                            const Eigen::Matrix<double,3,1> rot)
    {
        SpatialTransform < double >T(rot,pos);
        for (int i=0;i<tab_mesh.size();i++)
            tab_mesh[i].move(T.r,T.E);
    }

	void MogsGeometry::print()
	{
		std::cout << "MogsGeometry::print" << std::endl;
		std::cout << "tab_mesh.size() = "<< tab_mesh.size() << std::endl;
	}

	void MogsGeometry::scale(const Eigen::Matrix<double,3,1> s)
	{
        for (int i=0;i<tab_mesh.size();i++)
            tab_mesh[i].scale(s);
	}

// ******************************************************************************
// *********
// *********  XML Parsing
// *********
// ******************************************************************************


	void MogsGeometry::load (	QDomElement Element,
					double length,
					const mogs_string & path)
	{
        for (QDomElement Elbox = Element.firstChildElement("Box"); !Elbox.isNull(); Elbox = Elbox.nextSiblingElement("Box"))
		{
			  float X1, Y1, Z1, X2, Y2, Z2;
			  unsigned int R, G, B;
			  mogs_string stmp;
			  double tval;

              QDomElement El = Elbox.firstChildElement ("Box_X");
			  stmp = El.text();
			  std::istringstream Data_X (stmp.toStdString(), std::ios_base::in);
			  mogs_string unit_X = El.attribute ("unit");   // FIX ME what to do if no attribute
			  if (unit_X.compare ("m") == 0)
			    {
				    Data_X >> tval;
				    X1 = tval;
				    Data_X >> tval;
				    X2 = tval;
			    }
			  else if (unit_X.compare ("mm") == 0)
			    {
				    Data_X >> tval;
				    X1 = tval * 1e-3;
				    Data_X >> tval;
				    X2 = tval * 1e-3;
			    }
			  else if (unit_X.compare ("ratio_length") == 0)
			    {
				    Data_X >> tval;
				    X1 = tval * length;
				    Data_X >> tval;
				    X2 = tval * length;
			    }
			  else
			    {
				    std::cerr << "unknown unit in <Box_X>" << std::endl;
				    exit(-1);
			    }
                El = Elbox.firstChildElement ("Box_Y");
			  stmp = El.text();
			  std::istringstream Data_Y (stmp.toStdString(), std::ios_base::in);
			  mogs_string unit_Y = El.attribute ("unit");
			  if (unit_Y.compare ("m") == 0)
			    {
				    Data_Y >> tval;
				    Y1 = tval;
				    Data_Y >> tval;
				    Y2 = tval;
			    }
			  else if (unit_Y.compare ("mm") == 0)
			    {
				    Data_Y >> tval;
				    Y1 = tval * 1e-3;
				    Data_X >> tval;
				    Y2 = tval * 1e-3;
			    }
			  else if (unit_Y.compare ("ratio_length") == 0)
			    {
				    Data_Y >> tval;
				    Y1 = tval * length;
				    Data_Y >> tval;
				    Y2 = tval * length;
			    }
			  else
			    {
				    std::cerr << "unknown unit in <Box_Y>" << std::endl;
				    exit(-1);
			    }

                El = Elbox.firstChildElement ("Box_Z");
			  stmp = El.text();
			  std::istringstream Data_Z (stmp.toStdString(), std::ios_base::in);
			  mogs_string unit_Z = El.attribute("unit");
			  if (unit_Z.compare ("m") == 0)
			    {
				    Data_Z >> tval;
				    Z1 = tval;
				    Data_Z >> tval;
				    Z2 = tval;
			    }
			  else if (unit_Z.compare ("mm") == 0)
			    {
				    Data_Z >> tval;
				    Z1 = tval * 1e-3;
				    Data_Z >> tval;
				    Z2 = tval * 1e-3;
			    }
			  else if (unit_Z.compare ("ratio_length") == 0)
			    {
				    Data_Z >> tval;
				    Z1 = tval * length;
				    Data_Z >> tval;
				    Z2 = tval * length;
			    }
			  else
			    {
				    std::cerr << "unknown unit in <Box_Z>" << std::endl;
				    exit(-1);
			    }

                El = Elbox.firstChildElement ("Color");
			  if (!El.isElement())   // FIXME test sans couleur
			    {
				    R = 255;
				    G = 255;
				    B = 255;
			    }
			  else
			    {
				    stmp = El.text();
				    std::istringstream Data_RGB (stmp.toStdString(),std::ios_base::in);
				    Data_RGB >> tval;
				    R = tval;
				    Data_RGB >> tval;
				    G = tval;
				    Data_RGB >> tval;
				    B = tval;
			    }

			  Mesh *meh = new Mesh();
			  sub_Mesh *smeh = new sub_Mesh(meh);
			  convert_box_to_mesh(smeh, X1, Y1, Z1, X2, Y2, Z2, R, G, B);

			  // add subMehs in Mesh and Mesh in MogsGeometry
			  meh->subGeometries_.push_back(smeh);
			  this->add(*meh);
#if DEBUG_VISU
			  std::cout << "  Found a box : " << std::endl;
			  std::cout << "        Point 1 : " << X1 << " " << Y1 << " " << Z1 << std::endl;
			  std::cout << "        Point 2 : " << X2 << " " << Y2 << " " << Z2 << std::endl;
			  std::cout << "        Color :   " << (int) R << " " << (int) G << " " << (int) B << std::endl;
#endif

		  }

        for (QDomElement Elcylinder = Element.firstChildElement("Cylinder"); !Elcylinder.isNull(); Elcylinder = Elcylinder.nextSiblingElement("Cylinder"))
		{
			  float Height, Radius;
			  Eigen::Matrix < double, 3, 1 > position, orientation;
			  unsigned int R, G, B;
			  mogs_string stmp;
			  double tval;

              QDomElement El = Elcylinder.firstChildElement ("Radius");
			  assert (!El.isNull());
			  stmp = El.text();
			  std::istringstream Data_Radius (stmp.toStdString(),std::ios_base::in);
			  mogs_string unit_Radius = El.attribute ("unit");
			  if (unit_Radius.compare ("m") == 0)
			    {
				    Data_Radius >> tval;
				    Radius = tval;
			    }
			  else if (unit_Radius.compare ("mm") == 0)
			    {
				    Data_Radius >> tval;
				    Radius = tval * 1e-3;
			    }
			else if (unit_Radius.compare ("ratio_length") == 0)
			    {
				    Data_Radius >> tval;
				    Radius = tval * length;
			    }
			  else
			    {
				    std::cerr << "unknown unit in <Cylinder / Radius>" << std::endl;
				    exit(-1);
			    }

              El = Elcylinder.firstChildElement ("Height");
			  assert (!El.isNull());
			  stmp = El.text();
			  std::istringstream Data_Height (stmp.toStdString(),std::ios_base::in);
			  mogs_string unit_Height = El.attribute ("unit");
			  if (unit_Height.compare ("m") == 0)
			    {
				    Data_Height >> tval;
				    Height = tval;
			    }
			  else if (unit_Height.compare ("mm") == 0)
			    {
				    Data_Height >> tval;
				    Height = tval * 1e-3;
			    }
			  else if (unit_Height.compare ("ratio_length") == 0)
			    {
				    Data_Height >> tval;
				    Height = tval * length;
			    }
			  else
			    {
				    std::cerr << "unknown unit in <Cylinder / Height>" << std::endl;
				    exit(-1);
			    }
              El = Elcylinder.firstChildElement ("Translation");
                stmp = El.text();
                for (int i=0;i<3;i++)
                    position[i] = 0.;
			  if(!El.isNull())
			  {
				stmp = El.text ();
				std::istringstream Data_Translation (stmp.toStdString(),std::ios_base::in);
				mogs_string unit_Translation = El.attribute ("unit");
				if (unit_Translation.compare ("m") == 0)
				{
					for (int i=0;i<3;i++)
					{
						Data_Translation >> tval;
						position[i] = tval;
					}
				}
				else if (unit_Translation.compare ("mm") == 0)
				{
					for (int i=0;i<3;i++)
					{
						Data_Translation >> tval;
						position[i] = tval * 1e-3;
					}
				}
				else if (unit_Translation.compare ("ratio_length") == 0)
				{
					for (int i=0;i<3;i++)
					{
						Data_Translation >> tval;
						position[i] = tval * length;
					}
				}
				else
				{
					std::cerr << "unknown unit in <Cylinder / Translation>" << std::endl;
					exit(-1);
				}
			  }else
			  {
				    for (int i=0;i<3;i++)
					position[i] = 0;
			}

              El = Elcylinder.firstChildElement ("Orientation");
			  stmp = El.text();
			  if (!El.isNull())
			  {
				stmp = El.text ();
				std::istringstream Data_Orientation (stmp.toStdString(),std::ios_base::in);
				mogs_string unit_Orientation = El.attribute ("unit");
				if (unit_Orientation.compare ("rad") == 0)
				{
					for (int i=0;i<3;i++)
					{
						Data_Orientation >> tval;
						orientation[i] = tval;
					}
				}
				else if (unit_Orientation.compare ("degree") == 0)
				{
					for (int i=0;i<3;i++)
					{
						Data_Orientation >> tval;
						orientation[i] = tval * asin(1)/90.0;
					}
				}
				else
				{
					std::cerr << "unknown unit in <Cylinder / Orientation>" << std::endl;
					exit(-1);
				}
			  }else
			  {
				    for (int i=0;i<3;i++)
					orientation[i] = 0;
			}

                El = Elcylinder.firstChildElement ("Color");
			  if (!El.isElement())   // FIXME test sans couleur
			    {
				    R = 255;
				    G = 255;
				    B = 255;
			    }
			  else
			    {
				    stmp = El.text ();
				    std::istringstream Data_RGB (stmp.toStdString(),std::ios_base::in);
				    Data_RGB >> tval;
				    R = tval;
				    Data_RGB >> tval;
				    G = tval;
				    Data_RGB >> tval;
				    B = tval;
			    }

			  // convert to Mesh Object
			  Mesh *meh = new Mesh();
			  sub_Mesh *smeh = new sub_Mesh(meh);

			  convert_cylinder_to_mesh(smeh, Radius, Height, position, orientation, R, G, B);

			  // add subMehs in Mesh and Mesh in MogsGeometry
			  meh->subGeometries_.push_back(smeh);
			  this-> add(*meh);

#if DEBUG_VISU
			  std::cout << "  Found a cylinder : " << std::endl;
			  std::cout << "        Radius :  " << Radius << std::endl;
			  std::cout << "        Height :  " << Height << std::endl;
			  std::cout << "        position    = [" << position[0] << " : " << position[1] << " : " << position[2] << "]"<< std::endl;
			  std::cout << "        orientation = [" << orientation[0] << " : " << orientation[1] << " : " << orientation[2] << "]"<< std::endl;
			  std::cout << "        Color :   " << (int) R << " " << (int) G << " " << (int) B << std::endl;
#endif
		  }

        for (QDomElement Elsphere = Element.firstChildElement("Sphere"); !Elsphere.isNull(); Elsphere = Elsphere.nextSiblingElement("Sphere"))
        {
              assert (!Elsphere.isNull());
              float X, Y, Z, Radius;
              unsigned int R, G, B;
              bool wired = false;

              mogs_string stmp;
              double tval;
              QDomElement El = Elsphere.firstChildElement ("Point_X");
              X = 0.;
              if(!El.isNull())
              {
                  stmp = El.text ();
                  std::istringstream Data_X (stmp.toStdString(), std::ios_base::in);
                  mogs_string unit_X = El.attribute ("unit");
                  if (unit_X.compare ("m") == 0)
                    {
                        Data_X >> tval;
                        X = tval;
                    }
                  else if (unit_X.compare ("mm") == 0)
                    {
                        Data_X >> tval;
                        X = tval * 1e-3;
                    }
                  else if (unit_X.compare ("ratio_length") == 0)
                    {
                        Data_X >> tval;
                        X = tval * length;
                    }
                  else
                    {
                        std::cerr << "unknown unit in <Sphere / Point_X>" << std::endl;
                        exit(-1);
                    }
              }
              El = Elsphere.firstChildElement ("Point_Y");
              Y = 0.;
              if(!El.isNull())
              {
                  stmp = El.text ();
                  std::istringstream Data_Y (stmp.toStdString(), std::ios_base::in);
                  mogs_string unit_Y = El.attribute ("unit");
                  if (unit_Y.compare ("m") == 0)
                    {
                        Data_Y >> tval;
                        Y = tval;
                    }
                  else if (unit_Y.compare ("mm") == 0)
                    {
                        Data_Y >> tval;
                        Y = tval * 1e-3;
                    }
                  else if (unit_Y.compare ("ratio_length") == 0)
                    {
                        Data_Y >> tval;
                        Y = tval * length;
                    }
                  else
                    {
                        std::cerr << "unknown unit in <Sphere / Point_Y>" << std::endl;
                        exit(-1);
                    }
              }

              El = Elsphere.firstChildElement ("Point_Z");
              Z = 0.;
              if(!El.isNull())
              {
                  stmp = El.text ();
                  std::istringstream Data_Z (stmp.toStdString(), std::ios_base::in);
                  mogs_string unit_Z = El.attribute ("unit");
                  if (unit_Z.compare ("m") == 0)
                    {
                        Data_Z >> tval;
                        Z = tval;
                    }
                  else if (unit_Z.compare ("mm") == 0)
                    {
                        Data_Z >> tval;
                        Z = tval * 1e-3;
                    }
                  else if (unit_Z.compare ("ratio_length") == 0)
                    {
                        Data_Z >> tval;
                        Z = tval * length;
                    }
                  else
                    {
                        std::cerr << "unknown unit in <Sphere / Point_Z>" << std::endl;
                        exit(-1);
                    }
              }
              El = Elsphere.firstChildElement ("Radius");
              assert (!El.isNull());
              stmp = El.text ();
              std::istringstream Data_Radius (stmp.toStdString(),std::ios_base::in);
              mogs_string unit_Radius = El.attribute ("unit");
              if (unit_Radius.compare ("m") == 0)
                {
                    Data_Radius >> tval;
                    Radius = tval;
                }
              else if (unit_Radius.compare ("mm") == 0)
                {
                    Data_Radius >> tval;
                    Radius = tval * 1e-3;
                }
              else if (unit_Radius.compare ("ratio_length") == 0)
                {
                    Data_Radius >> tval;
                    Radius = tval * length;
                }
              else
                {
                    std::cerr << "unknown unit in <Sphere / Point_X>" << std::endl;
                    exit(-1);
                }
                El = Elsphere.firstChildElement ("Color");
              if (El.isNull())
                {
                    R = 255;
                    G = 255;
                    B = 255;
                }
              else
                {
                    stmp = El.text ();
                    std::istringstream Data_RGB (stmp.toStdString(),std::ios_base::in);
                    Data_RGB >> tval;
                    R = tval;
                    Data_RGB >> tval;
                    G = tval;
                    Data_RGB >> tval;
                    B = tval;
                }
            El = Elsphere.firstChildElement ("Wired");
            if (El.isNull())
                wired = (El.text() == "Wired");

            // convert to Mesh Object
            Mesh *meh = new Mesh();
            sub_Mesh *smeh = new sub_Mesh(meh);
            convert_sphere_to_mesh(smeh, X, Y, Z, Radius, R, G, B);

            // add subMesh in Mesh and Mesh in MogsGeometry
            meh->subGeometries_.push_back(smeh);
            this-> add(*meh);
#if DEBUG_VISU
			  std::cout << "  Found a Sphere : " << std::endl;
			  std::cout << "        Point : " << X << " " << Y << " " << Z << std::endl;
			  std::cout << "        Radius :  " << Radius << std::endl;
			  std::cout << "        Color :   " << (int) R << " " << (int) G << " " << (int) B << std::endl;
			  std::cout << "        Wired :   " << wired << std::endl;
#endif
		  }


        for (QDomElement Elfile = Element.firstChildElement("File"); !Elfile.isNull(); Elfile = Elfile.nextSiblingElement("File"))
		{
			mogs_string name = Elfile.text ().simplified();
#if DEBUG_VISU
			  qDebug() << "  Found a file : ";
			  qDebug() << "        Filename : " << path<<"/"<<name;
#endif
            this->add(Mesh(path, name));
		}
	}

	void MogsGeometry::load(  const QString & filename)
	{
	    if(filename.endsWith(".xml") )
        {
            // get the QDomElement from the xml
            QFile *file = new QFile(filename);
            if (!file->open(QIODevice::ReadOnly)) {
                std::cerr<<"Error in "<<__FILE__<<" at line : "<< __LINE__<<std::endl;
                std::cerr<<"Cannot open the file "<< filename.toStdString() <<std::endl;
                exit(0);
            }

            QDomDocument doc;
            // Parse file
            if (!doc.setContent(file)) {
                std::cerr<<"Error in "<<__FILE__<<" at line : "<< __LINE__<<std::endl;
               std::cerr<<"Cannot parse the content of "<< filename.toStdString()<<std::endl;
               file->close();
               exit(0);
            }
            file->close();

            QDomElement root = doc.documentElement();
            assert (!root.isNull());
            load(root);
        }else
            this->add(Mesh(filename));
	}

	void MogsGeometry::compute_normals(Eigen::Matrix<double,3,1> center, sub_Mesh* sm)
	{
		int nb_points = sm->points_.size();
		int nb_faces = sm->faces_.size();

		// normals_ and normals_faces_ vectors initialization
		sm->normals_.resize(nb_points);
		sm->normals_faces_.resize(nb_faces);

		Eigen::Matrix<double,3,1> edge1, edge2, n;
		int k, l;

		for (k=0; k<nb_points;++k)
			sm->normals_[k] = Eigen::Matrix<double,3,1>::Zero(3,1);

		for (k=0; k<nb_faces; ++k)
		{
			// compute normals
			edge1 = sm->points_[sm->faces_[k](1)] - sm->points_[sm->faces_[k](0)];
			edge2 = sm->points_[sm->faces_[k](2)] - sm->points_[sm->faces_[k](0)];
			sm->normals_faces_[k] = edge1.cross(edge2);
			if (sm->normals_faces_[k].dot(center - sm->points_[sm->faces_[k](0)]) > 0)
				sm->normals_faces_[k] = edge2.cross(edge1);
			sm->normals_faces_[k].normalize();
			// add normal_face to every triangle point normal
			for (l=0;l<3;++l)
				sm->normals_[sm->faces_[k](l)] += sm->normals_faces_[k];
		}

		// normalize points normals
		for (k=0; k<nb_points;++k)
			sm->normals_[k].normalize();

	}

	void MogsGeometry::convert_box_to_mesh(sub_Mesh *smeh, float X1, float Y1, float Z1, float X2, float Y2,
					   float Z2, unsigned int R, unsigned int G, unsigned int B)
	{
		// add points
		smeh->points_.push_back(Eigen::Matrix<double,3,1>(X1,Y1,Z1)); // point P1 // 0
		smeh->points_.push_back(Eigen::Matrix<double,3,1>(X2,Y1,Z1)); // point q1 // 1
		smeh->points_.push_back(Eigen::Matrix<double,3,1>(X2,Y2,Z1)); // point q2 // 2
		smeh->points_.push_back(Eigen::Matrix<double,3,1>(X1,Y2,Z1)); // point q3 // 3
		smeh->points_.push_back(Eigen::Matrix<double,3,1>(X1,Y2,Z2)); // point q4 // 4
		smeh->points_.push_back(Eigen::Matrix<double,3,1>(X1,Y1,Z2)); // point q5 // 5
		smeh->points_.push_back(Eigen::Matrix<double,3,1>(X2,Y1,Z2)); // point q6 // 6
		smeh->points_.push_back(Eigen::Matrix<double,3,1>(X2,Y2,Z2)); // point P2 // 7
		// add faces
		smeh->faces_.push_back(Eigen::Matrix<unsigned int,3,1>(1,2,6)); // face 0
		smeh->faces_.push_back(Eigen::Matrix<unsigned int,3,1>(2,7,6)); // face 1
		smeh->faces_.push_back(Eigen::Matrix<unsigned int,3,1>(2,3,7)); // face 2
		smeh->faces_.push_back(Eigen::Matrix<unsigned int,3,1>(3,4,7)); // face 3
		smeh->faces_.push_back(Eigen::Matrix<unsigned int,3,1>(3,4,5)); // face 4
		smeh->faces_.push_back(Eigen::Matrix<unsigned int,3,1>(3,5,0)); // face 5
		smeh->faces_.push_back(Eigen::Matrix<unsigned int,3,1>(0,5,6)); // face 6
		smeh->faces_.push_back(Eigen::Matrix<unsigned int,3,1>(0,6,1)); // face 7
		smeh->faces_.push_back(Eigen::Matrix<unsigned int,3,1>(5,6,4)); // face 8
		smeh->faces_.push_back(Eigen::Matrix<unsigned int,3,1>(6,7,4)); // face 9
		smeh->faces_.push_back(Eigen::Matrix<unsigned int,3,1>(0,1,3)); // face 10
		smeh->faces_.push_back(Eigen::Matrix<unsigned int,3,1>(1,2,3)); // face 11

		// add color
		smeh->diffuseColor_ = Eigen::Matrix<double,3,1> (R/255.,G/255.,B/255.);
//  		smeh->specularColor_ = Eigen::Matrix<double,3,1> (R/255.,G,B);
// 	  	smeh->emissiveColor_ = Eigen::Matrix<double,3,1> (R/255.,G,B);

		// other attributes
		smeh->valid_ = false;
		smeh->attrValid_ = false;
//	  	smeh->ambientIntensity_ = 1.;
//	  	smeh->shininess_ = 1.;
		smeh->transparency_ = 0.;

		// compute normals
		Eigen::Matrix<double,3,1> center ((X1+X2)/2.,(Y1+Y2)/2.,(Z1+Z2)/2.);
		compute_normals(center, smeh);
	}

	void MogsGeometry::convert_cylinder_to_mesh(sub_Mesh *smeh, float Radius, float Height,
				      Eigen::Matrix<double,3,1> position, Eigen::Matrix<double,3,1> orientation,
				      unsigned int R, unsigned int G, unsigned int B)
	{
	      int nb_pieces = 40; // number of faces used to represent the cylinder
	      double theta = 2. * 3.14159265358979323846 / (double)nb_pieces; // step

	      smeh->points_.push_back(Eigen::Matrix<double,3,1>(0.,0.,0.)); // point O     // 0
	      smeh->points_.push_back(Eigen::Matrix<double,3,1>(0.,0.,Height)); // point C // 1

	      int k;
	      double Ra = Radius, h = Height;
	      double x, y;
	      for (k = 0; k < nb_pieces; ++k)
	      {
			// add points
			x = Ra * cos(k*theta); y = Ra * sin(k*theta);
			smeh->points_.push_back(Eigen::Matrix<double,3,1>(x,y,0.)); // point P1 // (k*4)+2
			x = Ra * cos((k+1)*theta); y = Ra * sin((k+1)*theta);
			smeh->points_.push_back(Eigen::Matrix<double,3,1>(x,y,0.)); // point P2 // (k*4)+3
			x = Ra * cos(k*theta); y = Ra * sin(k*theta);
			smeh->points_.push_back(Eigen::Matrix<double,3,1>(x,y,h)); // point P3 // (k*4)+4
			x = Ra * cos((k+1)*theta); y = Ra * sin((k+1)*theta);
			smeh->points_.push_back(Eigen::Matrix<double,3,1>(x,y,h)); // point P4 // (k*4)+5
			// add faces
			smeh->faces_.push_back(Eigen::Matrix<unsigned int,3,1>(0,4*k+2,4*k+3));     // bottom
			smeh->faces_.push_back(Eigen::Matrix<unsigned int,3,1>(1,4*k+4,4*k+5));     // top
			smeh->faces_.push_back(Eigen::Matrix<unsigned int,3,1>(4*k+2,4*k+3,4*k+4)); // lateral 1
			smeh->faces_.push_back(Eigen::Matrix<unsigned int,3,1>(4*k+3,4*k+4,4*k+5)); // lateral 2
	      }

		// rotation
		double rot_x = orientation[0], rot_y = orientation[1], rot_z = orientation[2];
		Eigen::Matrix<double,3,3> Rx (1,0,0,0,cos(rot_x),-sin(rot_x),0,sin(rot_x),cos(rot_x));
		Eigen::Matrix<double,3,3> Ry (cos(rot_y),0,sin(rot_y),0,1,0,-sin(rot_y),0,cos(rot_y));
		Eigen::Matrix<double,3,3> Rz (cos(rot_z),-sin(rot_z),0,sin(rot_z),cos(rot_z),0,0,0,1);
		for (k=0; k < smeh->points_.size();++k)
		{
			smeh->points_[k] = smeh->points_[k].transpose() * Rx * Ry * Rz;
			smeh->points_[k].transpose();
		}

		// translation
		for (k=0; k < smeh->points_.size();++k)
			smeh->points_[k] += position;

		// add color
		smeh->diffuseColor_ = Eigen::Matrix<double,3,1> (R/255.,G/255.,B/255.);
// 		smeh->specularColor_ = Eigen::Matrix<double,3,1> (R,G,B);
// 		smeh->emissiveColor_ = Eigen::Matrix<double,3,1> (R,G,B);

		// other attributes
		smeh->valid_ = false;
		smeh->attrValid_ = false;
//		smeh->ambientIntensity_ = 1.;
//		smeh->shininess_ = 1.;
		smeh->transparency_ = 0.;

		// compute normals
		Eigen::Matrix<double,3,1> center (0.,0.,Height/2.);
		center = center.transpose() * Rx * Ry * Rz; // center vector rotation
		center.transpose();
		center += position; // center vector translation
		compute_normals(center, smeh);
	}

	void MogsGeometry::convert_sphere_to_mesh(sub_Mesh *smeh, float X, float Y, float Z, float Radius,
				      unsigned int R, unsigned int G, unsigned int B)
	{
		// FIXME  DO SAME SIZE OF FACES
		int nb_pieces = 40; // number of faces used to represent the sphere : 20 * 20 = 400
		double theta = 2. * 3.14159265358979323846 / (double)nb_pieces; // step

		int k,l, rl = 4*nb_pieces/2;
		double Ra = Radius;
		double x, y, z;
		for (k = 0; k < nb_pieces; ++k) for (l = -nb_pieces/4; l < nb_pieces/4; ++l)
		{
			// add points
			x = Ra * cos(k*theta) * cos(l*theta);
			y = Ra * sin(k*theta) * cos(l*theta);
			z = Ra * sin(l*theta);
			smeh->points_.push_back(Eigen::Matrix<double,3,1>(x,y,z)); // point P1
			x = Ra * cos((k+1)*theta) * cos(l*theta);
			y = Ra * sin((k+1)*theta) * cos(l*theta);
			z = Ra * sin(l*theta);
			smeh->points_.push_back(Eigen::Matrix<double,3,1>(x,y,z)); // point P2
			x = Ra * cos(k*theta) * cos((l+1)*theta);
			y = Ra * sin(k*theta) * cos((l+1)*theta);
			z = Ra * sin((l+1)*theta);
			smeh->points_.push_back(Eigen::Matrix<double,3,1>(x,y,z)); // point P3
			x = Ra * cos((k+1)*theta) * cos((l+1)*theta);
			y = Ra * sin((k+1)*theta) * cos((l+1)*theta);
			z = Ra * sin((l+1)*theta);
			smeh->points_.push_back(Eigen::Matrix<double,3,1>(x,y,z)); // point P4

			l = l + nb_pieces/4;

			// FIXME there will be unused points

			// add faces
			// lateral 1
			if ( 	(smeh->points_[k*rl+4*l] - smeh->points_[k*rl+4*l+1]). norm() > 1e-6 &&
				(smeh->points_[k*rl+4*l+2] - smeh->points_[k*rl+4*l+1]). norm() > 1e-6 &&
				(smeh->points_[k*rl+4*l] - smeh->points_[k*rl+4*l+2]). norm() > 1e-6)
				smeh->faces_.push_back(Eigen::Matrix<unsigned int,3,1>(k*rl+4*l,k*rl+4*l+1,k*rl+4*l+2));
			// lateral 2
			if ( 	(smeh->points_[k*rl+4*l+1] - smeh->points_[k*rl+4*l+2]). norm() > 1e-6 &&
				(smeh->points_[k*rl+4*l+3] - smeh->points_[k*rl+4*l+2]). norm() > 1e-6 &&
				(smeh->points_[k*rl+4*l+1] - smeh->points_[k*rl+4*l+3]). norm() > 1e-6)
			smeh->faces_.push_back(Eigen::Matrix<unsigned int,3,1>(k*rl+4*l+1,k*rl+4*l+2,k*rl+4*l+3));

			l = l - nb_pieces/4;
		}

		// compute normals
		Eigen::Matrix<double,3,1> center (0.,0.,0.);
		compute_normals(center, smeh);

		// translation
		for (k=0; k < smeh->points_.size();++k)
		{
			smeh->points_[k](0) += X;
			smeh->points_[k](1) += Y;
			smeh->points_[k](2) += Z;
		}

		// add color
		smeh->diffuseColor_ = Eigen::Matrix<double,3,1> (R/255.,G/255.,B/255.);
// 		smeh->specularColor_ = Eigen::Matrix<double,3,1> (R,G,B);
// 		smeh->emissiveColor_ = Eigen::Matrix<double,3,1> (R,G,B);

		// other attributes
		smeh->valid_ = false;
		smeh->attrValid_ = false;
// 		smeh->ambientIntensity_ = 1.;
// 		smeh->shininess_ = 1.;
		smeh->transparency_ = 0.;
#ifdef DEBUG
		std::cout<<"There are "<< smeh->faces_.size()<<" faces"<<std::endl;
#endif
	}
	
        void MogsGeometry::setDiffuseColor(const Eigen::Matrix<double,3,1> & diffuseColor)
        {
                int n = tab_mesh.size();
                for(int i=0;i<n;i++)
                        tab_mesh[i].setDiffuseColor(diffuseColor);     
        }

	void MogsGeometry::setSpecularColor(const Eigen::Matrix<double,3,1> & specularColor)
        {
                int n = tab_mesh.size();
                for(int i=0;i<n;i++)
                        tab_mesh[i].setSpecularColor(specularColor);     
        }

	void MogsGeometry::setEmissiveColor(const Eigen::Matrix<double,3,1> & emissiveColor)
        {
                int n = tab_mesh.size();
                for(int i=0;i<n;i++)
                        tab_mesh[i].setEmissiveColor(emissiveColor);     
        }
        
        void MogsGeometry::setColor(const Eigen::Matrix<double,3,1> & Color)
        {
            setDiffuseColor(Color);
            setSpecularColor(Color);
            setEmissiveColor(Color);
        }

