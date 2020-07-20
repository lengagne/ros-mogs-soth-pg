//      DAEParser.h
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

#ifndef DAEPARSER_H
#define DAEPARSER_H

#include "Mesh.h"

struct data{
    QString name;
    std::vector<Eigen::Matrix<double,3,1> > values;
};

struct color{
    QString name;
    Eigen::Matrix<double,3,1> diffuse;
    Eigen::Matrix<double,3,1> emissive;
    Eigen::Matrix<double,3,1> ambient;
    Eigen::Matrix<double,3,1> specular;
};

class DAEParser
{
    public:
        DAEParser();
        ~DAEParser();

        void parse(Mesh* m, QString filename);

    protected:
    private:


        unsigned int index_points_datas_;
        unsigned int index_vertex_datas_;

        unsigned int get_data_index(const QString & s);

        void read_mesh(Mesh* m,QDomElement & el);

        void read_polylist(Mesh* m,QDomElement & el);

        std::vector<Eigen::Matrix<double,3,1> > points_, normals_;

        std::vector<data> datas_;
        std::vector<color> colors_;
};

#endif // DAEPARSER_H
