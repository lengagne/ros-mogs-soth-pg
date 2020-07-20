//      Mesh.cpp
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
//
//	See README

#include <algorithm>
#include "Mesh.h"
#include "WRLBodyParser.h"
#include "DAEParser.h"
#include "STLParser.h"

static inline void qSetColor(float colorVec[], QColor c)
{
    colorVec[0] = c.redF();
    colorVec[1] = c.greenF();
    colorVec[2] = c.blueF();
    colorVec[3] = c.alphaF();
}

void Geometry::finalize()
{
    // TODO: add vertex buffer uploading here

    // Finish smoothing normals by ensuring accumulated normals are returned
    // to length 1.0.
    for (int i = 0; i < normals.count(); ++i)
        normals[i].normalize();
}

void Geometry::appendSmooth(const QVector3D &a, const QVector3D &n, int from)
{
    // Smooth normals are acheived by averaging the normals for faces meeting
    // at a point.  First find the point in geometry already generated
    // (working backwards, since most often the points shared are between faces
    // recently added).
    int v = vertices.count() - 1;
    for ( ; v >= from; --v)
        if (qFuzzyCompare(vertices[v], a))
            break;
    if (v < from)
    {
        // The vert was not found so add it as a new one, and initialize
        // its corresponding normal
        v = vertices.count();
        vertices.append(a);
        normals.append(n);
    }
    else
    {
        // Vert found, accumulate normals into corresponding normal slot.
        // Must call finalize once finished accumulating normals
        normals[v] += n;
    }
    // In both cases (found or not) reference the vert via its index
    faces.append(v);
}

void Geometry::appendFaceted(const QVector3D &a, const QVector3D &n)
{
    // Faceted normals are achieved by duplicating the vert for every
    // normal, so that faces meeting at a vert get a sharp edge.
    int v = vertices.count();
    vertices.append(a);
    normals.append(n);
    faces.append(v);
}


Patch::Patch(Geometry *g)
   : start(g->faces.count())
   , count(0)
   , initv(g->vertices.count())
   , sm(Patch::Smooth)
   , geom(g)
{
    #if VISU_TO_DO
    qSetColor(faceColor, QColor(Qt::darkGray));
    #endif
}

void Patch::rotate(qreal deg, QVector3D axis)
{
    mat.rotate(deg, axis);
}

void Patch::translate(const QVector3D &t)
{
    mat.translate(t);
}



void Patch::addTri(const QVector3D &a, const QVector3D &b, const QVector3D &c, const QVector3D &n)
{
    QVector3D norm = n.isNull() ? QVector3D::normal(a, b, c) : n;
    if (sm == Smooth)
    {
        geom->appendSmooth(a, norm, initv);
        geom->appendSmooth(b, norm, initv);
        geom->appendSmooth(c, norm, initv);
    }
    else
    {
        geom->appendFaceted(a, norm);
        geom->appendFaceted(b, norm);
        geom->appendFaceted(c, norm);
    }
    count += 3;
}

void Patch::addQuad(const QVector3D &a, const QVector3D &b,  const QVector3D &c, const QVector3D &d)
{
    QVector3D norm = QVector3D::normal(a, b, c);
    if (sm == Smooth)
    {
        addTri(a, b, c, norm);
        addTri(a, c, d, norm);
    }
    else
    {
        // If faceted share the two common verts
        addTri(a, b, c, norm);
        int k = geom->vertices.count();
        geom->appendSmooth(a, norm, k);
        geom->appendSmooth(c, norm, k);
        geom->appendFaceted(d, norm);
        count += 3;
    }
}

Mesh::Mesh()
{

}

Mesh::Mesh(const mogs_string & path,
	   const mogs_string & filename)
{
	parse(path, filename);
}

Mesh::Mesh(const mogs_string & filename)
{
	parse(filename);
}

Mesh::~Mesh()
{

}

void Mesh::parse(const mogs_string & path,
		const mogs_string & filename)
{
	parse( path +"/" + filename);
}

void Mesh::move(	const Eigen::Matrix<double,3,1> & pos,
			const Eigen::Matrix<double,3,3> & rot)
{
	for (int i=0;i<subGeometries_.size();i++)
		subGeometries_[i]->move(pos,rot);
}

void Mesh::scale(const Eigen::Matrix<double,3,1> s)
{
 	for (int i=0;i<subGeometries_.size();i++)
        for(int j=0;j<subGeometries_[i]->points_.size();j++)
            for(int k=0;k<3;k++)
                subGeometries_[i]->points_[j](k) *= s(k);
}

void Mesh::parse(const mogs_string & filename)
{
	unsigned nSubGeomBefore = getNumSubGeometries();

	if(filename.endsWith(".wrl") || filename.endsWith(".WRL"))
		parse_vrml(filename);
	else if(filename.endsWith(".dae") || filename.endsWith(".DAE"))
		parse_dae(filename);
	else if(filename.endsWith(".stl") || filename.endsWith(".STL"))
		parse_stl(filename);
    else
    {
        qDebug()<<"FIXME :: Cannot read type of "<< filename<<"  for the moment";
        return;
    }
	unsigned nSubGeomAfter = getNumSubGeometries();
	// Compile only the normals of the subgeometries added by parsing
	// this file.
	compileMesh();
}

void Mesh::parse_dae(const mogs_string & filename)
{
// 	std::cout<<"We are loading the file "<< filename << std::endl;
	DAEParser* parser = new DAEParser();
	parser->parse(& (*this), filename);
	delete parser;
// 	std::cout<<"We loaded the file "<< filename << std::endl;
}

void Mesh::parse_stl(const mogs_string & filename)
{
 	qDebug()<<"We are loading the file "<< filename;
	STLParser* parser = new STLParser();
	parser->parse(& (*this), filename);
	delete parser;
// 	std::cout<<"We loaded the file "<< filename << std::endl;
}


void Mesh::parse_vrml(const mogs_string & filename)
{
// 	std::cout<<"We are loading the file "<< filename << std::endl;
	WRLBodyParser* parser;
	parser = new WRLBodyParser(Eigen::Matrix<double,3,1> (1., 1. ,1.));
	parser->parse(& (*this), filename.toAscii());
	delete parser;
// 	std::cout<<"We loaded the file "<< filename << std::endl;
}

void Mesh::compileMesh()
{
	std::for_each(subGeometries_.begin(), subGeometries_.end(), compileSubMeshStructure);
}

void Mesh::compileMesh(unsigned i)
{
	compileSubMeshStructure(subGeometries_[i]);
}

bool Mesh::isParametric(unsigned) const
{
	return false;
}

void Mesh::setParameters(const std::vector<double>& , unsigned)
{

}

int Mesh::getParameters(std::vector<double>& parameters, unsigned) const
{
	parameters.clear();
	return -1;
}

void Mesh::getPoints(std::vector<Eigen::Matrix<double,3,1> >& points, unsigned i) const
{
	points = subGeometries_[i]->points_;
}

void Mesh::getFaces(std::vector<Eigen::Matrix<unsigned int,3,1> >& faces, unsigned i) const
{
	faces = subGeometries_[i]->faces_;
}

void Mesh::getNormals(std::vector<Eigen::Matrix<double,3,1> >& normals, unsigned i) const
{
	normals = subGeometries_[i]->normals_;
}

void Mesh::getTexPoints(std::vector<boost::numeric::ublas::vector<double> >& texPoints,
	unsigned i) const
{
	texPoints = subGeometries_[i]->texPoints_;
}

void Mesh::getTexFaces(std::vector<Eigen::Matrix<unsigned int,3,1>  >& texFaces, unsigned i) const
{
	texFaces = subGeometries_[i]->texFaces_;
}

void Mesh::setPoints(const std::vector<Eigen::Matrix<double,3,1> >& points, unsigned i)
{
	subGeometries_[i]->points_ = points;
}

void Mesh::setFaces(const std::vector<Eigen::Matrix<unsigned int,3,1> >& faces, unsigned i)
{
	subGeometries_[i]->faces_ = faces;
}

void Mesh::setNormals(const std::vector<Eigen::Matrix<double,3,1> >& normals, unsigned i)
{
	subGeometries_[i]->normals_ = normals;
}

void Mesh::setTexPoints(const std::vector<boost::numeric::ublas::vector<double> >& texPoints, unsigned i)
{
	subGeometries_[i]->texPoints_ = texPoints;
}

void Mesh::setTexFaces(const std::vector<Eigen::Matrix<unsigned int,3,1>  >& texFaces, unsigned i)
{
	subGeometries_[i]->texFaces_ = texFaces;
}

void Mesh::createNewSubGeometries(unsigned size)
{
	for(size_t i = 0; i < size; ++i)
	{
		sub_Mesh* newsg = new sub_Mesh(this);
		subGeometries_.push_back(newsg);
	}
}

unsigned Mesh::getNumSubGeometries() const
{
	return static_cast<unsigned>(subGeometries_.size());
}

void Mesh::addPoint(const Eigen::Matrix<double,3,1> & point, unsigned i)
{
	subGeometries_[i]->points_.push_back(point);
}

void Mesh::addFace(const Eigen::Matrix<unsigned int,3,1> & face, unsigned i)
{
	subGeometries_[i]->faces_.push_back(face);
}

void Mesh::addNormal(const Eigen::Matrix<double,3,1> & normal, unsigned i)
{
	subGeometries_[i]->normals_.push_back(normal);
}

void Mesh::addTexPoint(const boost::numeric::ublas::vector<double>& texPoint, unsigned i)
{
	subGeometries_[i]->texPoints_.push_back(texPoint);
}

void Mesh::addTexFace(const Eigen::Matrix<unsigned int,3,1> & texFace, unsigned i)
{
	subGeometries_[i]->texFaces_.push_back(texFace);
}

void Mesh::setAmbientIntensity(float ambientIntensity, unsigned i)
{
	subGeometries_[i]->ambientIntensity_ = ambientIntensity;
}

float Mesh::getAmbientIntensity(unsigned i) const
{
	return subGeometries_[i]->ambientIntensity_;
}

void Mesh::setDiffuseColor(const Eigen::Matrix<double,3,1> & diffuseColor, unsigned i)
{
	for(int j = 0; j < 3; ++j)
		subGeometries_[i]->diffuseColor_[j] = diffuseColor[j];
}

Eigen::Matrix<double,3,1>  Mesh::getDiffuseColor(unsigned i) const
{
	Eigen::Matrix<double,3,1>  diffuseColor;
	for(int j = 0; j < 3; ++j)
		diffuseColor[j] = subGeometries_[i]->diffuseColor_[j];
	return diffuseColor;
}

void Mesh::setSpecularColor(const Eigen::Matrix<double,3,1> & specularColor, unsigned i)
{
	for(int j = 0; j < 3; ++j)
	subGeometries_[i]->specularColor_[j] = specularColor[j];
}

void Mesh::setDiffuseColor(const Eigen::Matrix<double,3,1> & diffuseColor)
{
    unsigned int nb = subGeometries_.size();
    for (int i=0;i<nb;i++)
        setDiffuseColor(diffuseColor,i);    
}

void Mesh::setSpecularColor(const Eigen::Matrix<double,3,1> & specularColor)
{
    unsigned int nb = subGeometries_.size();
    for (int i=0;i<nb;i++)
        setSpecularColor(specularColor,i);    
}

void Mesh::setEmissiveColor(const Eigen::Matrix<double,3,1> & emissiveColor)
{
    unsigned int nb = subGeometries_.size();
    for (int i=0;i<nb;i++)
        setEmissiveColor(emissiveColor,i);    
}


Eigen::Matrix<double,3,1>  Mesh::getSpecularColor(unsigned i) const
{
	Eigen::Matrix<double,3,1>  specularColor;
	for(int j = 0; j < 3; ++j)
		specularColor[j] = subGeometries_[i]->specularColor_[j];
	return specularColor;
}

void Mesh::setEmissiveColor(const Eigen::Matrix<double,3,1> & emissiveColor, unsigned i)
{
	for(int j = 0; j < 3; ++j)
	subGeometries_[i]->emissiveColor_[j] = emissiveColor[j];
}

Eigen::Matrix<double,3,1>  Mesh::getEmissiveColor(unsigned i) const
{
	Eigen::Matrix<double,3,1>  emissiveColor;
	for(int j = 0; j < 3; ++j)
		emissiveColor[j] = subGeometries_[i]->emissiveColor_[j];
	return emissiveColor;
}

void Mesh::setShininess(float shininess, unsigned i)
{
	subGeometries_[i]->shininess_ = shininess;
}

float Mesh::getShininess(unsigned i) const
{
	return subGeometries_[i]->shininess_;
}

void Mesh::setTransparency(float transparency, unsigned i)
{
	subGeometries_[i]->transparency_ = transparency;
}

float Mesh::getTransparency(unsigned i) const
{
	return subGeometries_[i]->transparency_;
}

void Mesh::print_info()
{
	std::cout<<" Mesh::print_info()"<<std::endl;
	std::cout<<" number of geometries = "<< subGeometries_.size()<<std::endl<<std::endl;
}

/////////// Nested structures and classes implementation //////////////

///// CONSTRUCTION /////
sub_Mesh::sub_Mesh( ): geom(new Geometry)
{
	valid_ = false;
	attrValid_ = false;
	points_.clear();
	faces_.clear();
	texFaces_.clear();
	normals_.clear();
	diffuseColor_.setZero();
	emissiveColor_.setZero();
	specularColor_.setZero();
}


sub_Mesh::sub_Mesh(Mesh *parent):
	parent_(parent),
	ambientIntensity_(0.0),
	shininess_(0.0),
	transparency_(0.0),
	valid_(false),
	attrValid_(false),
	geom(new Geometry)
{
	points_.clear();
	faces_.clear();
	texFaces_.clear();
	normals_.clear();
	diffuseColor_.setZero();
	emissiveColor_.setZero();
	specularColor_.setZero();
}

sub_Mesh::~sub_Mesh()
{

}

void sub_Mesh::affine(double precision, bool create_face)
{
    if( create_face)
    {
        while (!one_step_affine(precision));
        std::cout<<"There are "<< faces_.size()<<" triangles"<<std::endl;
    }else
    {
        std::cout<<" we add only points without faces"<<std::endl;
        unsigned int nb_faces = faces_.size();
//        std::cout<<"nb_faces = "<< nb_faces <<std::endl;
        Eigen::Matrix<double,3,1> p;
        Eigen::Matrix<double,3,1> d1,d2,d3;
        Eigen::Matrix<double,3,1> dd1,dd2;
        for (unsigned int i=0;i<nb_faces;i++)
        {

            Eigen::Matrix<unsigned int,3,1> & index = faces_[i];
            unsigned int i1,i2;
            //Une fois on met un référence de p1, p2 ou p3 ça introduit une erreur incompréhensible
            Eigen::Matrix<double,3,1> p1 = points_[index[0]];
            Eigen::Matrix<double,3,1> p2 = points_[index[1]];
            Eigen::Matrix<double,3,1> p3 = points_[index[2]];
//
//            p1(0) = 0;  p1(1) = 0;  p1(2) = 0;
//            p2(0) = 0;  p2(1) = 1;  p2(2) = 0;
//            p3(0) = 1;  p3(1) = 0;  p3(2) = 0;
//

            d1 = p2-p1;
            d2 = p3-p1;
            d3 = p3-p2;

            double w12 = d1.norm();
            double w23 = d2.norm();
            double w13 = d3.norm();

            if( w12 > precision || w23 > precision || w13 > precision)
            {
                // we must add points
                double size = w12;
                if(size < w23) size = w23;
                if(size < w13) size = w13;
                unsigned int nb = size / precision;
                for (int ii=1;ii<nb;ii++)   for (int jj=0;jj<(nb-ii)+1;jj++)
                {
                    dd1 = double(ii)/nb * d1;
                    dd2 = double(jj)/nb * d2;
                    p = p1 +  dd1 + dd2;
                    points_.push_back(p);
                }

            }
        }
    }
}

bool sub_Mesh::one_step_affine(double precision)
{
	unsigned int nb = 0;
	unsigned int nb_faces = faces_.size();
	double all_max = 0;
	for (unsigned int i=0;i<nb_faces;i++)
	{

        Eigen::Matrix<unsigned int,3,1> & index = faces_[i];
        unsigned int i1,i2;
        Eigen::Matrix<double,3,1>& p1 = points_[index[0]];
        Eigen::Matrix<double,3,1>& p2 = points_[index[1]];
        Eigen::Matrix<double,3,1>& p3 = points_[index[2]];
        double max = 0;
        bool cut = false;
        double w = (p1-p2).norm();

        if( w > precision)
        {
//            std::cout<<"w1 = " << w <<std::endl;
            i1 = index[0];
            i2 = index[1];
            cut = true;
            max = w;
        }
        w = (p1-p3).norm();

        if( w> precision)
        {
//            std::cout<<"w2 = " << w <<std::endl;
            if (max <w)
            {
                i1 = index[0];
                i2 = index[2];
                max = w;
                cut = true;
            }
        }
        w = (p2-p3).norm();

        if( w> precision)
        {
//            std::cout<<"w3 = " << w <<std::endl;
            if (max <w)
            {
                i1 = index[1];
                i2 = index[2];
                max = w;
                cut = true;
            }
        }

        if (cut)
        {
//        std::cout<<"p1("<<index[0]<<") = "<< p1.transpose()<<std::endl;
//        std::cout<<"p2("<<index[1]<<") = "<< p2.transpose()<<std::endl;
//        std::cout<<"p3("<<index[2]<<") = "<< p3.transpose()<<std::endl;
//            std::cout<<"max = "<< max <<std::endl;
//            std::cout<<"i = "<< i<< "/"<< nb_faces<<"  Cutting "<< index.transpose()<< "   i1="<<i1<<"  i2="<<i2<<std::endl;
            if (max > all_max)  all_max = max;
            nb++;
            // create a new point
            unsigned int index_new_point = points_.size();
            Eigen::Matrix<double,3,1> p = (points_[i1] + points_[i2])/2.;
            // search and cut tirangle
            for (unsigned int k=i;k<nb_faces;k++)
            {
                Eigen::Matrix<unsigned int,3,1> current_face = faces_[k];
                if( (current_face[0] == i1 || current_face[1] == i1 || current_face[2] == i1 ) &&
                   (current_face[0] == i2 || current_face[1] == i2 || current_face[2] == i2))
                {
//                    std::cout<<"k = "<< k<<"  current face "<< current_face.transpose()<<std::endl;
                    // we found one face
                    Eigen::Matrix<unsigned int,3,1> f1 = current_face;
                    for (int ii=0;ii<3;ii++)
                        if(f1(ii) == i1)    f1(ii) = index_new_point;
                    faces_.push_back(f1);
                    normals_faces_.push_back(normals_faces_[k]);

                    Eigen::Matrix<unsigned int,3,1> f2 = current_face;
                    for (int ii=0;ii<3;ii++)
                        if(f2(ii) == i2)
                        {
                            f2(ii) = index_new_point;
                        }
                    faces_[k] = f2;
                    //faces_.erase(faces_.begin() +k);
                }
            }
//            std::cout<<"adding point "<< p.transpose()<<std::endl;
            points_.push_back(p);
            normals_.push_back( (normals_[i1] + normals_[i2])/2.);
        }
	}
	std::cout<<"We must cut "<< nb <<" triangles with max length of "<< all_max <<std::endl;
	if (nb==0)  return true;
	else return false;
}

void sub_Mesh::move(const Eigen::Matrix<double,3,1> & pos,
                    const Eigen::Matrix<double,3,3> & rot)
{
	for(int i=0;i<points_.size();i++)
	{
		// update points position
		points_[i] = pos + rot.transpose() * points_[i];
		// update normals
		normals_[i] = rot * normals_[i];
//		normals_[i].normalize();
	}

	for(int i=0;i<normals_faces_.size();i++)
	{
		// update normals_faces
		normals_faces_[i] = rot * normals_faces_[i];
//		normals_faces_[i].normalize();
	}
}

///// COMPILE STRUCTURE /////
void Mesh::s_compileSubMeshStructure::operator()(sub_Mesh* geom)
{
	// if the normals have not been computed yet, compute them
	if( (geom->normals_.size() == 0u) || (geom->normals_.size() != geom->points_.size()) )
		geom->parent_->computeNormalsOfGeometricStructure(geom);

    // we update the qt model
    for (int i=0;i<geom->normals_faces_.size();i++)
    {
        int id1 = geom->faces_[i](0);
        int id2 = geom->faces_[i](1);
        int id3 = geom->faces_[i](2);
        QVector3D p1(geom->points_[id1](0),geom->points_[id1](1),geom->points_[id1](2));
        QVector3D p2(geom->points_[id2](0),geom->points_[id2](1),geom->points_[id2](2));
        QVector3D p3(geom->points_[id3](0),geom->points_[id3](1),geom->points_[id3](2));
        QVector3D n(geom->normals_faces_[i](0),geom->normals_faces_[i](1),geom->normals_faces_[i](2));
    }
}

///// COMPUTE NORMALS /////
void Mesh::s_computeNormalsOfGeometricStructure::operator()(sub_Mesh* geom)
{
//    if(!geom)   return;
//	geom->normals_.resize(geom->faces_.size(), Eigen::Matrix<double,3,1> (0,0,0));
    geom->normals_.resize(geom->points_.size(), Eigen::Matrix<double,3,1> (0,0,0));

	const std::vector<Eigen::Matrix<unsigned int,3,1> >::const_iterator facesEndIterator = geom->faces_.end();
	std::vector<Eigen::Matrix<unsigned int,3,1> >::const_iterator facesIterator = geom->faces_.begin();

	Eigen::Matrix<double,3,1>  v1, v2, v1Xv2;

	// for each face, compute the normal to the face and add it to the normals associated to
	// each point of the face.
	for(; facesIterator != facesEndIterator; ++facesIterator)
	{
		const Eigen::Matrix<unsigned int,3,1> & currentFace = *facesIterator;

		for(unsigned i = 0u; i < 3u; ++i)
		{
			v1 = geom->points_[currentFace[(i + 1u) % 3u]] - geom->points_[currentFace[i]];
			v2 = geom->points_[currentFace[(i + 2u) % 3u]] - geom->points_[currentFace[i]];

			v1Xv2 = v1.cross(v2);
			geom->normals_[currentFace[i]] += v1Xv2;
			geom->normals_[currentFace[i]].normalize();
		}
	}

//	for(int i=0;i<geom->normals_.size();i++)
//        geom->normals_[i].normalize();

	const std::vector<Eigen::Matrix<double,3,1>  >::const_iterator normalsEndIterator = geom->normals_.end();
	std::vector<Eigen::Matrix<double,3,1>  >::iterator normalsIterator = geom->normals_.begin();

	// make each normal a unit vector.
	for(; normalsIterator != normalsEndIterator; ++normalsIterator) {
		double l = normalsIterator->norm();
		if(l > 1e-24)
			normalsIterator->normalize();
	}

	int nbf = geom->faces_.size();
	for (int i=0;i<nbf;i++)
	{
		int id1 = geom->faces_[i](0);
		int id2 = geom->faces_[i](1);
		int id3 = geom->faces_[i](2);
		Eigen::Matrix<double,3,1> normal;
		if (id1 >= geom->normals_.size() || id2 >= geom->normals_.size() || id3 >= geom->normals_.size() )
		{
		    std::cout<<" id1 = "<< id1 <<" id2 = "<< id2 <<" id3 = "<< id3<<" geom->normals_.size() = "<< geom->normals_.size() <<std::endl;
			std::cout<<"Error in "<<__FILE__<<" at line "<< __LINE__<<std::endl;
			std::cout<<"We cannot compute the normals of the mesh"<<std::endl;
			exit(0);
		}
		normal = (geom->normals_[id1] + geom->normals_[id2] + geom->normals_[id3]);
		normal.normalize();
		geom->normals_faces_.push_back(normal);
	}
}

///// DESTRUCTION OF SUBGEOMETRY /////
void Mesh::s_deleteSubMesh::operator()(sub_Mesh*& geometricStructure)
{
	geometricStructure->valid_ = false;
	geometricStructure->attrValid_ = false;
	delete geometricStructure;
	geometricStructure = 0x0;
}


