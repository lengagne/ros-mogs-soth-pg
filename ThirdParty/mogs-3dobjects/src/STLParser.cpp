//      STLParser.h
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
//	    from 2016 : Université Blaise Pascal / axis : ISPR / theme MACCS

#include "STLParser.h"
#include <fstream>
#include <iostream>

STLParser::STLParser()
{
    //ctor
}

STLParser::~STLParser()
{
    //dtor
}

void STLParser::parse(Mesh* m, QString filename)
{
//	qDebug()<<"STLPARSER begin";
    std::ifstream myFile (filename.toStdString().c_str(), std::ios::in | std::ios::binary);

    char header_info[80] = "";
    char n_triangles[4];
    unsigned long nTriLong;

    int cpt = 0;

    std::vector<Eigen::Matrix<double,3,1> > n;
    std::vector<Eigen::Matrix<double,3,1> > v;
    std::vector<Eigen::Matrix<unsigned int,3,1> > faces;

    //read 80 byte header
    if (myFile) {
        m->subGeometries_.push_back( new sub_Mesh());
        m->subGeometries_[0]->diffuseColor_ = Eigen::Matrix<double,3,1> (0.8,0.8,0.8);
        m->subGeometries_[0]->specularColor_ = Eigen::Matrix<double,3,1> (1,0,0);
        m->subGeometries_[0]->emissiveColor_ = Eigen::Matrix<double,3,1> (1,0,0);

        myFile.read (header_info, 80);
//        qDebug()<<"Read file " << filename;
//        qDebug()<<header_info;

        myFile.read(n_triangles, 4);
        unsigned int* r = (unsigned int*) n_triangles;
        unsigned int num_triangles = *r;
        for (unsigned int i = 0; i < num_triangles; i++) {
            Eigen::Matrix<double,3,1> normal = parse_point(myFile);

            n.push_back(normal);


            Eigen::Matrix<double,3,1> v1 = parse_point(myFile);
            Eigen::Matrix<double,3,1> v2 = parse_point(myFile);
            Eigen::Matrix<double,3,1> v3 = parse_point(myFile);
            v.push_back(v1);
            v.push_back(v2);
            v.push_back(v3);
//            std::cout<<"parse : = "<< normal.transpose()<<" "<< v1.transpose()<<" "<< v2.transpose()<<" "<< v3.transpose()<<" "<<std::endl;
//            info.triangles.push_back(triangle(normal, v1, v2, v3));
            char dummy[2];
            myFile.read(dummy, 2);

            faces.push_back(Eigen::Matrix<unsigned int,3,1>(cpt,cpt+1,cpt+2));
            cpt+=3;
        }
        m->setPoints(v,0);
        m->setFaces(faces,0);
        m->setNormals(n,0);


    }else
    {
        qDebug()<<"Cannot read file " << filename;
        exit(0);
    }

//	qDebug()<<"STLPARSER end";
}

float STLParser::parse_float(std::ifstream& s) {
    char f_buf[sizeof(float)];
    s.read(f_buf, 4);
    float* fptr = (float*) f_buf;
    return *fptr;
}

Eigen::Matrix<double,3,1> STLParser::parse_point(std::ifstream& s) {
    double x = parse_float(s);
    double y = parse_float(s);
    double z = parse_float(s);
    return Eigen::Matrix<double,3,1>(x, y, z);
}
