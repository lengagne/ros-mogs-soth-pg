//      WRLBodyParser.h
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

#ifndef _AF_VRML_PARSER_H
#define _AF_VRML_PARSER_H_

#include <boost/shared_ptr.hpp>

#include "WRLBasicParser.h"
#include "Mesh.h"
#include "types.h"

	class WRLBodyParserHandler;

	struct Transformation
	{
		Transformation():
			scale(1., 1., 1.),
			scaleOrientation(Eigen::Matrix<double,3,3>::Identity()),
			translation(0, 0, 0),
			rotation(Eigen::Matrix<double,3,3>::Identity()) 
			{
				
			}
			
			Eigen::Matrix<double,3,1>	scale;		/*!< \brief the scale to apply to the list of vertices */
			Eigen::Matrix<double,3,3>	scaleOrientation; /*!< \brief the frame in which the rotation is applied*/
			Eigen::Matrix<double,3,1>	translation; /*!< \brief the translation to apply to the (sub)list of vertices */
			Eigen::Matrix<double,3,3>	rotation; /*!< \brief the rotation to apply to the (sub)list of vertices */
	};

	struct WRLBodyParserData
	{
		// constructors, for convenience
		WRLBodyParserData():
			handlers(), geom(0x0), transformStack(), eventStack(), ccw (false), folder("")
		{}
		WRLBodyParserData(const Eigen::Matrix<double,3,1> & s):
			handlers(), geom(0x0), transformStack(), eventStack(), ccw (false), folder("")
		{
			Transformation t;
			t.scale = s;
			transformStack.push_back(t);
		}
		WRLBodyParserData(const std::vector<Transformation>& t):
			handlers(), geom(0x0), transformStack(t), eventStack(), ccw (false), folder("")
		{
		}

		~WRLBodyParserData();

		// data
		std::vector<WRLBodyParserHandler*>	handlers;	/*!< \brief the list of keyword handlers */
		Mesh*						geom;		/*!< \brief the geometry being constructed */
		std::vector<Transformation> 	transformStack;	/*!< \brief the stack of transformations encountered */
		std::vector<unsigned>			eventStack; /*!< \brief tool indicating of many bracket have been opened */
		bool							ccw;		/*!< \brief are we reading cw or ccw faces? */
		std::string						folder;		/*!< \brief the path to the vrml files (useful for Inline)*/
		std::map<std::string, int>		coordMap;	/*!< \brief Map linking the labels defined by coord DEF to the corresponding subMesh*/
		std::map<std::string, int>		appearanceMap;	/*!< \brief Map linking the labels defined by appearance DEF to the corresponding subMesh*/
	};

	class WRLBodyParser
	{
	public:

		WRLBodyParser();
		WRLBodyParser(const Eigen::Matrix<double,3,1> & scale);
		WRLBodyParser(const std::vector<Transformation>& t);

		void parse(Mesh* geom, const char* filename);

	private:

		WRLBasicParser parser_;
		boost::shared_ptr<WRLBodyParserData> data_;
	};
// }

// --- DOC --------------------------------------------------------

/*!
* \file WRLBodyParser.h
* \brief Declaration of class afstate::WRLBodyParser.
* \author Paul Evrard
* \version 0.0.0
*
* Declaration of the afstate::WRLBodyParser class.
*
*/

/*!
* \class afstate::WRLBodyParser WRLBodyParser.h "Definition"
* \brief Parses pseudo-vrml files. 
* 
*  WRLBodyParser is looking for some keywords of the VRML language, 
*  and parses the data that comes after these keywords.
*  The parser has been implemented using WRLBasicParser. WRLBasicParser is
*  simply an automaton that will look for specific strings in a text in an efficient
*  way, and trigger a callback when a matching substring is found. WRLBodyParser
*  is built upon WRLBasicParser: it registers some keywords of the VRML language such
*  as 'IndexedFaceSet', 'Shininess', 'Shape' etc. and associates them a callback.
*
*  Note: rotation and translation tags must be added BEFORE the list of points on
*  which they are to be applied.
*
*  WARNING: the reinitialization of the rotation/translation/scale values is not
*  handled correctly yet (only on Transform tag reading). Thus, points defined
*  outside of Transform tags may not be parsed correctly.
* 
*/

/*! 
* \fn AFSTATE_API afstate::WRLBodyParserData::WRLBodyParserData()
* \brief default constructor of WRLBodyParserData.
* Scale vector used : (1., 1., 1.)
*/

/*! 
* \fn AFSTATE_API afstate::WRLBodyParserData::WRLBodyParserData(const Eigen::Matrix<double,3,1> & s)
* \brief constructor of WRLBodyParserData using a scale vector.
* \param s the scale to apply to the vertices.
* If a scale tag is read in the VRML file, the resulting scaling value will be
* read-scale * s.
*/

/*! 
* \fn AFSTATE_API afstate::WRLBodyParser::WRLBodyParser()
* \brief default constructor of WRLBodyParser.
* Scale vector used : (1., 1., 1.)
*/

/*! 
* \fn AFSTATE_API afstate::WRLBodyParser::WRLBodyParser(const Eigen::Matrix<double,3,1> & scale)
* \brief constructor of WRLBodyParser using a scale vector.
* \param scale the scale to apply to the vertices.
*/

/*! 
* \fn AFSTATE_API void afstate::WRLBodyParser::parse(Mesh* geom, const char* filename)
* \brief parses the file \a filename to construct the geometry \a geom.
* \param geom the geometry being constructed.
* \param filename the file to parse.
*/

/*!
* \var WRLBasicParser afstate::WRLBodyParser::parser_
* \brief the parser used to parse files.
*/

/*!
* \var boost::shared_ptr<WRLBodyParserData> afstate::WRLBodyParser::data_
* \brief attribute containing parsing data.
*/

/*!
* \struct afstate::WRLBodyParser::WRLBodyParserData
* \brief structure used by the handlers to keep track of the parsing data.
*
* Contains : the list of keyword handlers, the geometry being constructed, the scale to 
* apply to the list of vertices and whether we read the faces clockwise or 
* counterclockwise.
*/

#endif

