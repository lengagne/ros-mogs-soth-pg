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

#include "DAEParser.h"

DAEParser::DAEParser()
{
    //ctor
}

DAEParser::~DAEParser()
{
    //dtor
}

unsigned int DAEParser::get_data_index(const QString & s)
{
    for (int i=0;i<datas_.size();i++)
    {
//        qDebug()<<"testing : "<<datas_[i].name;
        if(datas_[i].name == s)
            return i;
    }

    qDebug()<<"Cannot find "<< s;
    exit(0);
}

void DAEParser::parse(Mesh* m, QString filename)
{
//     qDebug()<<"Parsing DAE in progress of "<< filename;
	mogs_string url = mogs_get_absolute_link(filename);
	QDomDocument *dom = new QDomDocument(url); // Création de l'objet DOM
	QFile xml_doc(url);// On choisit le fichier contenant les informations XML.
    if(!xml_doc.open(QIODevice::ReadOnly))// Si l'on n'arrive pas à ouvrir le fichier XML.
    {
         qDebug() <<"Erreur à l'ouverture du document XML :"<<url;
         qDebug()<<"Le document XML n'a pas pu être ouvert. Vérifiez que le nom est le bon et que le document est bien placé";
         return;
    }
    dom->setContent(&xml_doc);
    QDomElement root = dom->documentElement();


    // read the colors
    QDomElement El_lib_effect = root.firstChildElement("library_effects");
    if( El_lib_effect.isNull())
    {
        qDebug() <<"Erreur cannot find balise : library_effects";
        return;
    }
    for (QDomElement ElEffect = El_lib_effect.firstChildElement("effect"); ! ElEffect.isNull() ; ElEffect = ElEffect.nextSiblingElement("effect"))
    {
        color tmp_color;
        tmp_color.name =  ElEffect.attribute("id");
        QDomNodeList list_color = ElEffect.elementsByTagName("color");
//         qDebug()<<"count =  "<< list_color.count();
        for (int i=0;i<list_color.count();i++)
        {
            QDomElement node = list_color.at(i).toElement();
            QString text = node.text();
            std::istringstream Data (text.toStdString(), std::ios_base::in);
            Eigen::Matrix<double,3,1> v;
            for(int j=0;j<3;j++)    Data>>v(j);
            QString sid = node.attribute("sid");
            if (sid=="diffuse")    tmp_color.diffuse = v;
            if (sid=="emission")    tmp_color.emissive = v;
            if (sid=="ambient")    tmp_color.ambient = v;
        }
        colors_.push_back(tmp_color);
    }

    // read the meshes
     QDomElement El_lib_geom = root.firstChildElement("library_geometries");
     if( El_lib_geom.isNull())
     {
         qDebug() <<"Erreur cannot find balise : library_geometries";
         return;
     }
    for (; ! El_lib_geom.isNull() ; El_lib_geom = El_lib_geom.nextSiblingElement("library_geometries"))
    {
//        qDebug() <<"We find balise : library_geometries";
        QDomElement El_geom = El_lib_geom.firstChildElement("geometry");
        if( El_geom.isNull())
        {
             qDebug() <<"Erreur cannot find balise : geometry";
             return;
        }
        for (; ! El_geom.isNull() ; El_geom = El_geom.nextSiblingElement("geometry"))
        {
//            qDebug() <<"We find balise : geometry";
            QDomElement El_mesh = El_geom.firstChildElement("mesh");
            if( El_mesh.isNull())
            {
                qDebug() <<"Erreur cannot find balise : mesh";
                return;
            }
            for (; ! El_mesh.isNull() ; El_mesh = El_mesh.nextSiblingElement("MESH"))
            {
//                qDebug() <<"We find balise : mesh";
                read_mesh(m,El_mesh);
            }
        }
    }
}

void DAEParser::read_mesh(Mesh* m,QDomElement & el)
{
    QDomElement El_point = el.firstChildElement("source");
    for (; ! El_point.isNull() ; El_point = El_point.nextSiblingElement("source"))
    {
        data tmp_data;
        tmp_data.name = "#"+El_point.attribute("id");
        QDomElement El_data = El_point.firstChildElement("float_array");
        unsigned int nb = El_data.attribute("count").toInt();
        QString text = El_data.text();
//        qDebug()<<"text = "<<text;
        std::istringstream Data (text.toStdString(), std::ios_base::in);
        unsigned int cpt = 0;
        double x,y,z;
        while(cpt< nb)
        {
            Data>>x;
            Data>>y;
            Data>>z;
            tmp_data.values.push_back(Eigen::Matrix<double,3,1>(x,y,z));
            cpt+=3;
        }
        datas_.push_back(tmp_data);
    }

    QDomElement Elvertices = el.firstChildElement("vertices");
    if(Elvertices.isNull()){
        qDebug()<<"Error cannot find balise vertices";
        exit(0);
    }
    QDomElement Elinput = Elvertices.firstChildElement("input");
    if(Elinput.isNull()){
        qDebug()<<"Error cannot find balise input for vertices";
        exit(0);
    }
    for (QDomElement elinput = Elvertices.firstChildElement("input"); !elinput.isNull(); elinput = elinput.nextSiblingElement("input"))
    {
        QString tmp = elinput.attribute("semantic");
        if(tmp=="POSITION")
        {
            QString name = elinput.attribute("source");
            index_points_datas_ = get_data_index(name);
        }
    }

    for (QDomElement Ellist = el.firstChildElement("polylist"); !Ellist.isNull(); Ellist = Ellist.nextSiblingElement("polylist"))
    {
        read_polylist(m,Ellist);
    }

    for (QDomElement Ellist = el.firstChildElement("triangles"); !Ellist.isNull(); Ellist = Ellist.nextSiblingElement("triangles"))
    {
        read_polylist(m,Ellist);
    }
}

void DAEParser::read_polylist(Mesh* m,QDomElement & Ellist)
{
    std::vector<QString> inputs;
    std::vector< std::vector<int> > indexes;
    std::vector<int> offset;

    unsigned int index_vertex,index_normal;

    QString material = Ellist.attribute("material");

//    qDebug()<<"On a une polylist";
    // get the number of input
    int nb_in = 0;
    for (QDomElement elinput = Ellist.firstChildElement("input"); !elinput.isNull(); elinput = elinput.nextSiblingElement("input"))
    {
        QString tmp = elinput.attribute("semantic");
//        qDebug()<<" input = "<< tmp;
        if(tmp=="VERTEX")
            index_vertex = nb_in;
        if(tmp=="NORMAL")
            index_normal = nb_in;

        inputs.push_back(tmp);
        nb_in++;
    }

    unsigned int nb = Ellist.attribute("count").toInt();
    QDomElement ElV = Ellist.firstChildElement("vcount");
    QString sv = ElV.text();
    QDomElement ElP = Ellist.firstChildElement("p");
    QString p = ElP.text();
    std::istringstream Data_sv (sv.toStdString(), std::ios_base::in);
    std::istringstream Data_p (p.toStdString(), std::ios_base::in);
//    qDebug()<<" sv = "<< sv;
//    qDebug()<<" p = "<< p;
    int nbv;
    for(int i=0;i<nb;i++)
    {
        if(sv.isNull())
            nbv=3;
        else
            Data_sv >> nbv;
        offset.push_back(nbv);
        std::vector<int> sub_indexes(nbv);
        for(int j=0;j<nbv;j++)
        {
            for(int k=0;k<nb_in;k++)
                Data_p >> sub_indexes[k];
            indexes.push_back(sub_indexes);
        }
    }

    sub_Mesh * sm = new sub_Mesh(m);
    sm->points_ = datas_[index_points_datas_].values;
    sm->diffuseColor_ = Eigen::Matrix<double,3,1> (1,1,1);
    sm->emissiveColor_ = Eigen::Matrix<double,3,1> (0,0,0);
    sm->specularColor_ = Eigen::Matrix<double,3,1> (1,1,1);
//    sm->ambientColor_ = Eigen::Matrix<double,3,1> (0,0,0);

    //set the color
    for(int i=0;i<colors_.size();i++)
    {
//         qDebug()<<"material = "<< material<<"  color = "<< colors_[i].name;
        if( material.contains(colors_[i].name.remove("-effect",Qt::CaseInsensitive)))
        {
            sm->diffuseColor_ = colors_[i].diffuse;
            sm->emissiveColor_ = colors_[i].emissive;
            sm->specularColor_ = colors_[i].specular;
//            sm->ambientColor_ = colors_[i].ambient;
//             qDebug()<<"Find the good color";
            break;
        }
    }

    unsigned int cpt = 0;
    for(int i=0;i<nb;i++)
    {
        if(offset[i] == 3)
        {
            sm->faces_.push_back(Eigen::Matrix<unsigned int,3,1> (indexes[cpt][index_vertex],indexes[cpt+1][index_vertex],indexes[cpt+2][index_vertex]));
            cpt+=3;
        }else
        {
            qDebug()<<"We did not implement the case count !=3";
        }
    }
    m->subGeometries_.push_back(sm);
}
