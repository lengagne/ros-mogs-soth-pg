//      WRLBodyParser.cpp
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
//	see README

#include <vector>
#include <memory>
#include <cctype>
#include <iostream>

#include <Eigen/Geometry>

#include "ShapeBuilder.h"
#include "WRLBodyParser.h"


// --- Implementation of the Body parser ---
//
// This file might look a bit complicated, so here are some explanations.
// To handle more VRML keywords, just look at the end of this explanation...
// A short section is dedicated to it.
//
// The WRLBodyParser objects parses pseudo-vrml files. It looks for some keywords
// of the VRML language, and parses the data that comes after these keywords.
//
// The parser has been implemented using WRLBasicParser. WRLBasicParser is
// simply an automaton that will look for specific strings in a text in an efficient
// way, and trigger a callback when a matching substring is found. WRLBodyParser
// is built upon WRLBasicParser: it registers some keywords of the VRML language such
// as 'IndexedFaceSet', 'Shininess', 'Shape' etc. and associates them a callback.
//
// The callback takes a reference to the file as argument so that when it's called, it
// will parse the file starting from the character just after the matching keyword.
//
// Example:
// -------------------------------------
// register "toto", with handler onToto
//
// parse file containing:
// "tralala, i'm happy today because my name is toto [
// 1,2,3,4,5 ] et voila !"
// -------------------------------------
//
// The file is automatically parsed, until the word toto is found. Then,
// the callback onToto, which has the following signature:
//
// class onToto {
// void operator()(std::istream& is) { /* do something */ }
// };
//
// is called. At the point where it is called, the parameter 'is' is the file being
// read and currently points just after toto. Hence, in the callback onToto, the first
// call to is.get() will return ' ', the second one will return '['. Then one can
// do
// int i;
// is >> i;
//
// Then i contains 1, and so on...
//
// WRLBodyParser uses a base class for all the handlers: WRLBodyParserHandler. This class
// derives from BasicHandler, the class which contains the callback declaration.
// It enhances the callback system by adding a boolean, 'execute' that will
// be initialized to true, and set to false after the first call of the callback.
// The callback will do something only if this boolean is set to true, hence
// after the callback has been called once, it won't do anything any more when
// matching strings are found later. The reset method resets the boolean to true.
//
// >>>>>>>>>>>>>>>>>> TO ADD NEW HANDLERS <<<<<<<<<<<<<<<<<<<<
//
// Look at the 'init' function, and add appropriate handlers there.
// Declare the handler and write its code, using the following macros:
// - AF_BODY_HANDLER,
// - AF_BODY_END_HANDLER.
//
// See the documentation of these macros below.
//
// Example: the VRML 'Transform' keyword
// -------------------------------------
//
// This requires to add this line in the init function:
//
// data.handlers.push_back(new TransformHandler("Transform", data));
//
// And the declaration of this handler, in the list of handlers (the big part
// of this file):
//
// AF_BODY_HANDLER(Transform)(std::istream& in, WRLBodyParserData& data)
// {
//   ...
//   return 0; // 0: the handler can be called only once before reset is called,
//             // 1: the handler can be called anytime.
// }
// AF_BODY_END_HANDLER;
//
// If you need additional data, add fields in the WRLBodyParserData below.


// --------------------------------------------------------------
// --- Parser Data ----------------------------------------------
// --------------------------------------------------------------

// Check WRLBodyParserData documentation.

// --------------------------------------------------------------
// --- Helpers --------------------------------------------------
// --------------------------------------------------------------

// Splits a quad v into two ccw triangles v1 and v2.

void splitFace(Eigen::Matrix<unsigned int,3,1> & v1, Eigen::Matrix<unsigned int,3,1> & v2, int* v, bool ccw)
{
	if(ccw == true) {
		v1[0] = v[0];
		v1[1] = v[1];
		v1[2] = v[3];

		v2[0] = v[1];
		v2[1] = v[2];
		v2[2] = v[3];
	}
	else {
		v1[0] = v[0];
		v1[1] = v[3];
		v1[2] = v[1];

		v2[0] = v[1];
		v2[1] = v[3];
		v2[2] = v[2];
	}
}

// separate the folder from the filename of a given path.
void splitPath (const std::string& path, std::string & folder, std::string & filename)
{
	size_t found=path.find_last_of("/\\");
	folder = path.substr(0,found);
	filename = path.substr(found+1);
}

std::string parseName(std::istream& in)
{
	char c = 0;
	std::string s;

	// Skip spaces.
	while(isspace(c = in.peek())){ c = in.get(); }

	// Read the name.
	c = in.get();
	s += c;
	while(isgraph(c = in.peek())){ s += c; c = in.get(); }

	return s;
}

// --------------------------------------------------------------
// --- Handler Base ---------------------------------------------
// --------------------------------------------------------------

// Base Body parser handler.
// Stores the data structure manipulated by all the handlers,
// and send a reference to it when forwarding the callback call.
// This class intercepts the callbacks sent by the parser, and
// will forward the message to the derived classes by the mean
// of the pure virtual 'process' function, only if the boolean
// 'execute' is set to true (which is the case at construction).
// After the callback has been forwarded, the boolean is set to
// false so that future callbacks won't be forwarded, unless
// the boolean is reset to true, using the reset method.

class WRLBodyParserHandler
	: public BasicHandler
{
public:

	WRLBodyParserHandler(const std::string & name, WRLBodyParserData& data)
		: data_(data)
		, name_(name)
		, execute_(true)
	{}

	void reset() { execute_ = true; }

	void operator()(std::istream& in)
	{
		if(!execute_){ return; }
		if(!process(in, data_)){ execute_ = false; }
	}

	const std::string & getName() const { return name_; }

private:

	virtual int process(std::istream& in, WRLBodyParserData& data) = 0;

private:

	WRLBodyParserData& data_;
	std::string name_;
	bool execute_;
};

// tags useful to count how many brackets have been opened,
//  and when the transformations can be poped back
enum
{
	OPENING_TRANSFORM = 0,
	OPENING_INLINE,
	OPENING_RANDOM,
	OPENING_COORDINATE,
	OPENING_TEXTURE_COORDINATE,
	OPENING_SHAPE
};

inline void pushEvent(unsigned event, const std::string & name, std::vector<unsigned> &eventStack)
{
#ifdef DEBUGGING
	for (unsigned i=0; i<eventStack.size(); ++i)
		std::cout << "  ";
	std::cout << "pushing " << name << "  " << (eventStack.size()) <<std::endl;
#endif // DEBUGGING
	eventStack.push_back(event);
}

inline void popEvent(const std::string & name, std::vector<unsigned> &eventStack)
{
	if (eventStack.size() == 0)
		std::cerr << " ERROR when poping the stack of events" << std::endl;
#ifdef DEBUGGING
	for (unsigned i=1; i<eventStack.size(); ++i)
		std::cout << "  ";
	std::cout << "poping " << name << "  " << (eventStack.size()-1) <<std::endl;
#endif // DEBUGGING
	eventStack.pop_back();
}

// --------------------------------------------------------------
// --- Macros ---------------------------------------------------
// --------------------------------------------------------------

// This macro is here for convenience. Each keyword is associated
// to a handler class which derives from WRLBodyParserHandler (defined
// just above). Hence, the declaration of a whole class is necessary
// for each keyword, although the only specific part is the body
// of the 'process' method.
// Hence, two macros are defined:
// - AF_BODY_HANDLER(name) declares a class 'name'Handler with
//   the appropriate mother class constructor call, and declares
//   a process method, without stating the argument list.
// - AF_BODY_END_HANDLER closes the declaration.
//
// Using these macros, the declaration of a handler for a specific
// keyword is most simple:
//
// AF_BODY_HANDLER(SomeHandlerClassName)(std::istream& in, WRLBodyParserData& data)
// {
//   ...
//   return 0; // 0: the handler can be called only once before reset is called,
//             // 1: the handler can be called anytime.
// }
// AF_BODY_END_HANDLER;
//
// And that's all! It just looks like declaring a simple function, but with
// the power of using classes.

#define AF_BODY_HANDLER(name) \
class name##Handler: public WRLBodyParserHandler { \
public: \
name##Handler(const char* name, WRLBodyParserData& data): WRLBodyParserHandler(name, data) {} \
virtual ~name##Handler(){} \
private: \
int process

#define AF_BODY_END_HANDLER }


// --------------------------------------------------------------
// --- Handlers -------------------------------------------------
// --------------------------------------------------------------

AF_BODY_HANDLER(Comment)(std::istream& in, WRLBodyParserData& /*data*/)
{
	// We ignore the comments, else if we find a keyword inside a comment
	// it will be handled by the parser.
	char c = in.get();
	while( c != '\n')
	{
		c = in.get();
	}
	return 1;
}
AF_BODY_END_HANDLER;

AF_BODY_HANDLER(TextureTransform)(std::istream& in, WRLBodyParserData& data)
{
	char c = in.get();
	while(c != '{'){ c = in.get(); }
	pushEvent(OPENING_RANDOM, "OPENING TextureTRANSFORM", data.eventStack);

	// Do all the parsing to avoid meeting the line
	//  translation 0 0
	// that will break the parser
	while(c != '}'){ c = in.get(); }
	popEvent("CLOSING TextureTRANSFORM",data.eventStack);

	return 1;
}
AF_BODY_END_HANDLER;

AF_BODY_HANDLER(Transform)(std::istream& in, WRLBodyParserData& data)
{
	// We add a new layer of transformation with default values for translation/rotation/scaling
	Transformation t;
	data.transformStack.push_back(t);

	// We get the next opening {
	std::string tmp ("");
	char c = in.get();
	while(c != '{'){ tmp +=c; c = in.get(); }

	pushEvent(OPENING_TRANSFORM, "OPENING_TRANSFORM " + tmp, data.eventStack);
	return 1;
}
AF_BODY_END_HANDLER;


AF_BODY_HANDLER(Inline)(std::istream& in, WRLBodyParserData& data)
{
	char c;

	// we get the name of the file
	while((c = in.peek()) != '{'){ c = in.get(); } // skip spaces
	pushEvent(OPENING_INLINE, "OPENING_INLINE", data.eventStack);

	// skip the tag url
	while((c = in.peek()) != '"'){ c = in.get(); } // skip spaces
	c = in.get();
	std::string s;
	// Read the name.
	while((c = in.peek()) != '"'){ s += c; c = in.get(); }
	while((c = in.peek()) != '}'){ c = in.get(); } // skip spaces
	c = in.get(); // remove last }
	popEvent("OPENING_INLINE",data.eventStack);

	// building the whole path.
	std::string path =  data.folder + "/" + s ;

	// and parse the file.
	//  initialize it with the current transformationStack
	// TODO: avoid copying the whole transformation stack by using a
	//  well built homogenous materix
	WRLBodyParser subParser (data.transformStack);
	subParser.parse(data.geom, path.c_str());
	return 1;
}
AF_BODY_END_HANDLER;


AF_BODY_HANDLER(Rotation)(std::istream& in, WRLBodyParserData& data)
{
	// we get the axis
	Eigen::Matrix<double,3,1>  v;
	in >> v;

	// we get the angle
	double a;
	in >> a;

	// we compute the corresponding rotation matrix
	// in order to do that, we need to use quaternions
	Eigen::AngleAxis<double> aa(a, Eigen::Vector3d(v[0], v[1], v[2]));
	Eigen::Quaternion<double> q(aa);
	q.normalize();

	// we convert the quaternion to a rotation matrix
	Eigen::Matrix3d rot = q.toRotationMatrix();
	data.transformStack.back().rotation <<
		rot(0,0), rot(0,1), rot(0,2),
		rot(1,0), rot(1,1), rot(1,2),
		rot(2,0), rot(2,1), rot(2,2)
	;

	return 1;
}
AF_BODY_END_HANDLER;

AF_BODY_HANDLER(Translation)(std::istream& in, WRLBodyParserData& data)
{
	// we get the translation vector
	Eigen::Matrix<double,3,1>  v;
	in >> v;

	data.transformStack.back().translation = v;

	return 1;
}
AF_BODY_END_HANDLER;

AF_BODY_HANDLER(ScaleOrientation)(std::istream& in, WRLBodyParserData& data)
{
	// we get the axis
	Eigen::Matrix<double,3,1>  v;
	in >> v;

	// we get the angle
	double a;
	in >> a;

	// we compute the corresponding rotation matrix
	// in order to do that, we need to use quaternions
	Eigen::AngleAxis<double> aa(a, Eigen::Vector3d(v[0], v[1], v[2]));
	Eigen::Quaternion<double> q(aa);
	q.normalize();

	// we convert the quaternion to a rotation matrix
	Eigen::Matrix3d rot = q.toRotationMatrix();
	data.transformStack.back().scaleOrientation <<
		rot(0,0), rot(0,1), rot(0,2),
		rot(1,0), rot(1,1), rot(1,2),
		rot(2,0), rot(2,1), rot(2,2)
	;

	return 1;
}
AF_BODY_END_HANDLER;

AF_BODY_HANDLER(Scale)(std::istream& in, WRLBodyParserData& data)
{
	// we get the scale vector
	Eigen::Matrix<double,3,1>  v;
	in >> v;

	data.transformStack.back().scale = v;
	return 1;
}
AF_BODY_END_HANDLER;

AF_BODY_HANDLER(Shape)(std::istream& /*in*/, WRLBodyParserData& data)
{
	std::for_each(data.handlers.begin(), data.handlers.end(), std::mem_fun(&WRLBodyParserHandler::reset));
	data.geom->createNewSubGeometries(1);
	data.ccw = true;
	return 1;
}
AF_BODY_END_HANDLER;

// -- Specific shape handlers

void buildSpecificMesh(ShapeBuilder* sb, WRLBodyParserData& data)
{
	std::vector<Eigen::Matrix<double,3,1> > points;
	std::vector<Eigen::Matrix<unsigned int,3,1> > faces;
	std::vector<Eigen::Matrix<double,3,1> > normals;

	sb->buildDisplayModel( points, faces, normals);

	for (unsigned pi=0; pi<points.size(); ++pi)
	{
		Eigen::Matrix<double,3,1>  v = points[pi];
		for (int i=(int)(data.transformStack.size())-1; i>=0;--i)
		{
			const Transformation & transf = data.transformStack[i];
			v = transf.scaleOrientation.transpose() * v; // change the frame of appliance of the scaling
			Eigen::Matrix<double,3,1>  v1 = elt_product (v,transf.scale); // we apply the scaling
			v = transf.scaleOrientation * v1;
			v = transf.rotation * v; //we apply the rotation
			v += transf.translation; // we apply the translation
		}
		points[pi] = v;
	}

	unsigned int i = data.geom->getNumSubGeometries() - 1;
	data.geom->setPoints(points, i);
	data.geom->setFaces(faces, i);
	data.geom->setNormals(normals, i);

	return ;
}

AF_BODY_HANDLER(Box)(std::istream& in, WRLBodyParserData& data)
{
	Eigen::Matrix<double,3,1>  boxSize(.1, 0.02, 0.01);

	std::string tmp;
	// entering the loop
	while (tmp != "{") in >> tmp;
	pushEvent(OPENING_SHAPE, "OPENING_SHAPE_BOX", data.eventStack);

	// entering the loop
	while (tmp != "}")
	{
		if(tmp == "size") in >> boxSize;
		in >> tmp;
	}
	popEvent("OPENING_SHAPE_BOX", data.eventStack);

	BoxBuilder* sb = new BoxBuilder (boxSize[1], boxSize[0], boxSize[2]);
	buildSpecificMesh(sb, data);
	delete sb;

	return 1;
}
AF_BODY_END_HANDLER;

AF_BODY_HANDLER(Cylinder)(std::istream& in, WRLBodyParserData& data)
{
	double radius = 0;
	double height = 0;

	std::string tmp;
	// entering the loop
	while (tmp != "{") in >> tmp;
	pushEvent(OPENING_SHAPE, "OPENING_CYLINDER_SHAPE", data.eventStack);

	// entering the loop
	while (tmp != "}")
	{
		if(tmp == "radius") in >> radius;
		if(tmp == "height") in >> height;
		in >> tmp;
	}
	popEvent("OPENING_CYLINDER_SHAPE", data.eventStack);

	ShapeBuilder* sb = new CylinderBuilder (36, radius, radius, height, "Disk", "Disk", 'y');
	buildSpecificMesh(sb, data);
	delete sb;

	return 1;
}
AF_BODY_END_HANDLER;

AF_BODY_HANDLER(Sphere)(std::istream& in, WRLBodyParserData& data)
{
	double radius = 0;

	std::string tmp;
	// entering the loop
	while (tmp != "{") in >> tmp;
	pushEvent(OPENING_SHAPE, "OPENING_SHAPE_SHERE", data.eventStack);

	// entering the loop
	while (tmp != "}")
	{
		if(tmp == "radius") in >> radius;
		in >> tmp;
	}
	popEvent("OPENING_SHAPE_SHERE", data.eventStack);

	ShapeBuilder* sb = new SphereBuilder (10, radius);
	buildSpecificMesh(sb, data);
	delete sb;

	return 1;
}
AF_BODY_END_HANDLER;



AF_BODY_HANDLER(AmbientIntensity)(std::istream& in, WRLBodyParserData& data)
{
	float f; in >> f;
	data.geom->setAmbientIntensity(f, data.geom->getNumSubGeometries() - 1);
	return 0;
}
AF_BODY_END_HANDLER;


AF_BODY_HANDLER(DiffuseColor)(std::istream& in, WRLBodyParserData& data)
{
	Eigen::Matrix<double,3,1>  v; in >> v;
	data.geom->setDiffuseColor(v, data.geom->getNumSubGeometries() - 1);
	return 0;
}
AF_BODY_END_HANDLER;


AF_BODY_HANDLER(SpecularColor)(std::istream& in, WRLBodyParserData& data)
{
	Eigen::Matrix<double,3,1>  v; in >> v; v /= 5.;
	data.geom->setSpecularColor(v, data.geom->getNumSubGeometries() - 1);
	return 0;
}
AF_BODY_END_HANDLER;


AF_BODY_HANDLER(EmissiveColor)(std::istream& in, WRLBodyParserData& data)
{
	Eigen::Matrix<double,3,1>  v; in >> v;
	data.geom->setEmissiveColor(v, data.geom->getNumSubGeometries() - 1);
	return 0;
}
AF_BODY_END_HANDLER;


AF_BODY_HANDLER(Shininess)(std::istream& in, WRLBodyParserData& data)
{
	float f; in >> f;
	data.geom->setShininess(1.f - f, data.geom->getNumSubGeometries() - 1);
	return 0;
}
AF_BODY_END_HANDLER;


AF_BODY_HANDLER(Transparency)(std::istream& in, WRLBodyParserData& data)
{
	float f; in >> f;
	data.geom->setTransparency(f, data.geom->getNumSubGeometries() - 1);
	return 0;
}
AF_BODY_END_HANDLER;

AF_BODY_HANDLER(Ccw)(std::istream& in, WRLBodyParserData& data)
{
	const size_t bufferSize = 32;
	char c = 0; std::string s; s.reserve(bufferSize);

	while((c = in.peek()) == ' '){ c = in.get(); } // skip spaces

	// read alphanum characters
	c = in.get();
	while( ((c >= 'A') && (c <= 'Z')) || ((c >= 'a') && (c >= 'z')) ) {
		s += c; c = in.get();
	}

	std::transform(s.begin(), s.end(), s.begin(), (int(*)(int))std::tolower);

	if(s == "false"){ data.ccw = false; }
	else{ data.ccw = true; }

	return 0;
}
AF_BODY_END_HANDLER;


AF_BODY_HANDLER(Coordinate)(std::istream& in, WRLBodyParserData& data)
{
	// look for the opening bracket '{'
	char c = in.get();
	while(c != '{'){ c = in.get(); }
	pushEvent(OPENING_COORDINATE, "OPENING_COORDINATE", data.eventStack);

	// look for the opening bracket '['
	while(c != '['){ c = in.get(); }
	while((isdigit(in.peek()) == false) && (in.peek() != '-') && (c != ']')) {
		c = in.get();
	}

	// while the bracket is not closed
	for(Eigen::Matrix<double,3,1>  v; c != ']'; ) {
		// read a point
		in >> v;
		// TODO: avoid useless computation by building a unique homogeneous matrix
		// to multiply v.

		// for each layer, starting from the last one.
		for (int i=data.transformStack.size()-1; i>=0;--i)
		{
			const Transformation & transf = data.transformStack[i];
			Eigen::Matrix<double,3,1>  v1 = elt_product(v, transf.scale); // we apply the scaling
			v = transf.rotation * v1; //we apply the rotation
			v += transf.translation; // we apply the translation
		}

		data.geom->addPoint(v, data.geom->getNumSubGeometries() - 1);

		// look for a next point (separated by a ',' or the closing bracket
		c = in.get();
		while((c != '\n') && (c != ',') && (c != ']')){ c = in.get(); }
		while((c != ']') && (isdigit(in.peek()) == false) && (in.peek() != '-') && (in.peek() != ']')) {
			c = in.get();
		}
		if(in.peek() == ']'){	c = in.get(); }
	}

	// look for the closing bracket '}'
	while(c != '}'){ c = in.get(); }
	c = in.get();

	popEvent("OPENING_COORDINATE",data.eventStack);

	return 0;
}
AF_BODY_END_HANDLER;


AF_BODY_HANDLER(TextureCoordinate)(std::istream& in, WRLBodyParserData& data)
{
	char c = in.get();
	while(c != '{'){ c = in.get(); }
	pushEvent(OPENING_TEXTURE_COORDINATE, "OPENING_TEXTURE_COORDINATE", data.eventStack);

	while(c != '['){ c = in.get(); }
	while((isdigit(in.peek()) == false) && (in.peek() != '-') && (c != ']')) {
		c = in.get();
	}

	for(boost::numeric::ublas::vector<double> v(2); c != ']'; ) {
		in >> v[0] >> v[1];
		data.geom->addTexPoint(v, data.geom->getNumSubGeometries() - 1);

		c = in.get();
		while((c != ',') && (c != ']')){ c = in.get(); }
		while((c != ']') && (isdigit(in.peek()) == false) && (in.peek() != '-') && (in.peek() != ']')) {
			c = in.get();
		}
		if(in.peek() == ']'){ c = in.get(); }
	}

	while(c != '}'){ c = in.get(); }
	popEvent("OPENING_TEXTURE_COORDINATE", data.eventStack);


	return 0;
}
AF_BODY_END_HANDLER;


AF_BODY_HANDLER(CoordIndex)(std::istream& in, WRLBodyParserData& data)
{
	char c = in.get();
	while(c != '['){ c = in.get(); }

	int v[4];
	const size_t count  = 32;
	std::string buffer; buffer.reserve(count);

	while((isdigit(in.peek()) == false) && (c != ']')){ c = in.get();}
	while(c != ']') {
		while(!isdigit(in.peek())){	c = in.get();}

		// load the 3 first values
		for(int i = 0; i < 3; ++i) {
			for(buffer = "", c = in.get(); isdigit(c); c = in.get()){
			buffer += c;

			}
			v[i] = atoi(buffer.c_str());
			if(i < 2) {
				while(isdigit(in.peek()) == false){	c = in.get();}
			}
		}

		while(c != ',' && c != ' '){ c = in.get();}
		while((isdigit(in.peek()) == false) && (in.peek() != '-')){ c = in.get();}

		for(buffer = "", c = in.get(); isdigit(c) || (c == '-'); ) {
			buffer += c;
			c = in.get();
		}
		int tmp = atoi(buffer.c_str());

		while((c != ',') && (c != ']')){ c = in.get();}

		// check if the fourth one is the last one (-1)
		if((tmp == -1) && data.ccw) {
			data.geom->addFace(Eigen::Matrix<unsigned int,3,1> (v[0], v[1], v[2]), data.geom->getNumSubGeometries() - 1);
		}
		else if(tmp == -1) {
			data.geom->addFace(Eigen::Matrix<unsigned int,3,1> (v[2], v[1], v[0]), data.geom->getNumSubGeometries() - 1);
		}
		else {
			v[3] = tmp;
			Eigen::Matrix<unsigned int,3,1>  v1, v2;
			splitFace(v1, v2, v, data.ccw);
			data.geom->addFace(v1, data.geom->getNumSubGeometries() - 1);
			data.geom->addFace(v2, data.geom->getNumSubGeometries() - 1);

			c = in.get();
// 			while((c != ',') && (c != ']')){ c = in.get(); std::cout<<"	"<<c<<std::endl;}
		}

		while((c != ']') && (isdigit(in.peek()) == false) && (in.peek() != ']')) {
			c = in.get();
		}
		if(in.peek() == ']'){ c = in.get();}
	}

	return 0;
}
AF_BODY_END_HANDLER;


AF_BODY_HANDLER(TexCoordIndex)(std::istream& in, WRLBodyParserData& data)
{
	char c = in.get();
	while(c != '['){ c = in.get(); }

	int v[4];
	const size_t count  = 32;
	std::string buffer; buffer.reserve(count);

	while((isdigit(in.peek()) == false) && (c != ']')){ c = in.get(); }

	while(c != ']') {
		while(!isdigit(in.peek())){ c = in.get(); }

		for(unsigned i = 0u; i < 3u; ++i) {
			for(buffer = "", c = in.get(); isdigit(c); ) {
				buffer += c;
				c = in.get();
			}
			v[i] = atoi(buffer.c_str());

			if(i < 2u) {
				while(isdigit(in.peek()) == false){ c = in.get(); }
			}
		}

		while(c != ','){ c = in.get(); }

		while((isdigit(in.peek()) == false) && (in.peek() != '-')){ c = in.get(); }

		for(buffer = "", c = in.get(); isdigit(c) || (c == '-'); ) {
			buffer += c;
			c = in.get();
		}
		int tmp = atoi(buffer.c_str());

		while((c != ',') && (c != ']')){ c = in.get(); }

		if((tmp == -1) && (data.ccw == true)) {
			data.geom->addTexFace(Eigen::Matrix<unsigned int,3,1> (v[0], v[1], v[2]), data.geom->getNumSubGeometries() - 1);
		}
		else if(tmp == -1) {
			data.geom->addTexFace(Eigen::Matrix<unsigned int,3,1> (v[2], v[1], v[0]), data.geom->getNumSubGeometries() - 1);
		}
		else {
			v[3] = tmp;
			Eigen::Matrix<unsigned int,3,1>  v1, v2;
			splitFace(v1, v2, v, data.ccw);
			data.geom->addTexFace(v1, data.geom->getNumSubGeometries() - 1);
			data.geom->addTexFace(v2, data.geom->getNumSubGeometries() - 1);

			c = in.get();
			while((c != ',') && (c != ']')){ c = in.get(); }
		}

		while((c != ']') && (isdigit(in.peek()) == false) && (in.peek() != ']')) {
			c = in.get();
		}
		if(in.peek() == ']'){ c = in.get(); }
	}

	return 0;
}
AF_BODY_END_HANDLER;

AF_BODY_HANDLER(OpeningBracket)(std::istream& /*in*/, WRLBodyParserData& data)
{
	pushEvent(OPENING_RANDOM, "OPENING_RANDOM", data.eventStack);
	return 1;
}
AF_BODY_END_HANDLER;


AF_BODY_HANDLER(ClosingBracket)(std::istream& /*in*/, WRLBodyParserData& data)
{
#ifdef DEBUGGING
	for (unsigned i=1; i<data.eventStack.size(); ++i)
		std::cout << "  ";

	if(data.eventStack.back() == OPENING_TRANSFORM)
		std::cout << "poping OPENING_TRANSFORM";
	else if(data.eventStack.back() == OPENING_INLINE)
		std::cout << "poping OPENING_INLINE";
	else if(data.eventStack.back() == OPENING_COORDINATE)
		std::cout << "poping OPENING_INLINE";
	else if(data.eventStack.back() == OPENING_TEXTURE_COORDINATE)
		std::cout << "poping OPENING_TEXTURE_COORDINATE";
	else
		std::cout << "poping OPENING_RANDOM";
	std::cout << "  " << (data.eventStack.size()- 1)<<std::endl;
#endif // DEBUGGING

	// we are closing a Transform => we remove the associated transformation
	if(data.eventStack.back() == OPENING_TRANSFORM)
		data.transformStack.pop_back();

	data.eventStack.pop_back();
	return 1;
}
AF_BODY_END_HANDLER;



////////////////////////////////////////////////////
////// Flags DEF and USE ///////////////////////////
////////////////////////////////////////////////////

// Parse the name of a defined object.
// Although this is not needed, this allows to have DEF names containing
// reserved words without leading to unexpected errors.
AF_BODY_HANDLER(Def)(std::istream& in, WRLBodyParserData& /*data*/)
{
	std::string s = parseName(in);

#ifdef DEBUGGING
	std::cout << "Encounter Def " << s << std::endl;
#endif // DEBUGGING

	return 1;
}
AF_BODY_END_HANDLER;


// Parse the name of the object before USE
// This is only used for debug purpose, to see what USE flags are not found
AF_BODY_HANDLER(Use)(std::istream& in, WRLBodyParserData& /*data*/)
{
	std::string s = parseName(in);
	std::cout << " Warning: the flag USE " << s << " will not be taken into account " << std::endl;

	return 1;
}
AF_BODY_END_HANDLER;


// Parse coord def flags and store the number of the corresponding geometry
AF_BODY_HANDLER(CoordDef)(std::istream& in, WRLBodyParserData& data)
{
	// get the name of a defined object to use.
	std::string s = parseName(in);

#ifdef DEBUGGING
	std::cout << "CoordDef Encounter Def  **" << s << "**" << std::endl;
#endif // DEBUGGING

	// Save the value of the current sub geometry
	data.coordMap[s] = data.geom->getNumSubGeometries() - 1;
	return 1;
}
AF_BODY_END_HANDLER;

// Copy the points of the corresponding geometry into the current one
AF_BODY_HANDLER(CoordUse)(std::istream& in, WRLBodyParserData& data)
{
	// get the name of a defined object to use.
	std::string s = parseName(in);

#ifdef DEBUGGING
	std::cout << "CoordUse Encounter Def **" << s << "**" << std::endl;
#endif // DEBUGGING

	//// copy the points of the subMesh into the current one

	// get the number of the corresponding subMesh
	std::map<std::string, int>::const_iterator it;
	if(data.coordMap.find(s) == data.coordMap.end())
		std::cerr << "could not find " << s << std::endl;
	unsigned subGeom = data.coordMap[s];

	std::vector<Eigen::Matrix<double,3,1> > points;
	data.geom->getPoints(points, subGeom);

	for(unsigned i=0; i<points.size(); ++i)
		data.geom->addPoint(points[i], data.geom->getNumSubGeometries() - 1);

	return 1;
}
AF_BODY_END_HANDLER;





AF_BODY_HANDLER(AppearanceDef)(std::istream& in, WRLBodyParserData& data)
{
	// get the name of a defined object to use.
	std::string s = parseName(in);

#ifdef DEBUGGING
	std::cout << "CoordDef Encounter Def  **" << s << "**" << std::endl;
#endif // DEBUGGING

	// Save the value of the current sub geometry
	data.appearanceMap[s] = data.geom->getNumSubGeometries() - 1;
	return 1;
}
AF_BODY_END_HANDLER;

AF_BODY_HANDLER(AppearanceUse)(std::istream& in, WRLBodyParserData& data)
{
	// get the name of a defined object to use.
	std::string s = parseName(in);

#ifdef DEBUGGING
	std::cout << "AppearanceUse Encounter Def **" << s << "**" << std::endl;
#endif // DEBUGGING

	//// copy the properties of the subMesh into the current one

	// get the number of the corresponding subMesh
	std::map<std::string, int>::const_iterator it;
	if(data.appearanceMap.find(s) == data.appearanceMap.end())
		std::cerr << "could not find " << s << std::endl;
	unsigned subGeom = data.coordMap[s];

	int targetSubGeom = data.geom->getNumSubGeometries() - 1;

	data.geom->setAmbientIntensity(data.geom->getAmbientIntensity(subGeom), targetSubGeom);
	data.geom->setDiffuseColor(data.geom->getDiffuseColor(subGeom), targetSubGeom);
	data.geom->setEmissiveColor(data.geom->getEmissiveColor(subGeom), targetSubGeom);
	data.geom->setSpecularColor(data.geom->getSpecularColor(subGeom), targetSubGeom);
	data.geom->setShininess(data.geom->getShininess(subGeom), targetSubGeom);
	data.geom->setTransparency(data.geom->getTransparency(subGeom), targetSubGeom);

	return 1;
}
AF_BODY_END_HANDLER;

#undef AF_BODY_END_HANDLER
#undef AF_BODY_HANDLER

// The handlers list. Add new handlers here!

void init(WRLBasicParser& p, WRLBodyParserData& data)
{
	data.handlers.push_back(new ScaleOrientationHandler("scaleOrientation", data));
	data.handlers.push_back(new CommentHandler("#", data));
	data.handlers.push_back(new TransformHandler("Transform", data));
	data.handlers.push_back(new RotationHandler("rotation", data));
	data.handlers.push_back(new TranslationHandler("translation", data));
	data.handlers.push_back(new ScaleHandler("scale ", data));
	data.handlers.push_back(new ShapeHandler("Shape", data));
	data.handlers.push_back(new AmbientIntensityHandler("ambientIntensity", data));
	data.handlers.push_back(new DiffuseColorHandler("diffuseColor", data));
	data.handlers.push_back(new SpecularColorHandler("specularColor", data));
	data.handlers.push_back(new EmissiveColorHandler("emissiveColor", data));
	data.handlers.push_back(new ShininessHandler("shininess", data));
	data.handlers.push_back(new TransparencyHandler("transparency", data));
	data.handlers.push_back(new CcwHandler("ccw", data));
	data.handlers.push_back(new CoordinateHandler("Coordinate", data));
	data.handlers.push_back(new CoordIndexHandler("coordIndex", data));
	data.handlers.push_back(new TextureCoordinateHandler("TextureCoordinate", data));
	data.handlers.push_back(new TexCoordIndexHandler("texCoordIndex", data));
	data.handlers.push_back(new OpeningBracketHandler("{", data));
	data.handlers.push_back(new ClosingBracketHandler("}", data));
	data.handlers.push_back(new InlineHandler("Inline", data));

	// handles particular shapes, such as Sphere and Cylinder.
	data.handlers.push_back(new BoxHandler("geometry Box", data));
	data.handlers.push_back(new CylinderHandler("geometry Cylinder", data));
	data.handlers.push_back(new SphereHandler("geometry Sphere", data));

	// handle the def flags
	data.handlers.push_back(new DefHandler("DEF", data));
	data.handlers.push_back(new UseHandler("USE", data));

	data.handlers.push_back(new CoordDefHandler("coord DEF", data));
	data.handlers.push_back(new CoordUseHandler("coord USE", data));

	data.handlers.push_back(new AppearanceDefHandler("appearance DEF", data));
	data.handlers.push_back(new AppearanceUseHandler("appearance USE", data));

	// a simple flag that should now be confused with Transform
	data.handlers.push_back(new TextureTransformHandler("textureTransform", data));

	for(size_t i = 0; i < data.handlers.size(); ++i) {
		p.addHandler(data.handlers[i]->getName(), data.handlers[i]);
	}
}

WRLBodyParserData::~WRLBodyParserData()
{
	for (unsigned i=0; i<handlers.size(); ++i)
		if (handlers[i] != 0x0)
			delete handlers[i];
}

WRLBodyParser::WRLBodyParser()
	: data_(new WRLBodyParserData)
{
	init(parser_, *data_);
}

WRLBodyParser::WRLBodyParser(const Eigen::Matrix<double,3,1> & scale)
	: data_(new WRLBodyParserData(scale))
{
	init(parser_, *data_);
}

WRLBodyParser::WRLBodyParser(const std::vector<Transformation>& t)
: data_(new WRLBodyParserData(t))
{
	init(parser_, *data_);
}


void WRLBodyParser::parse(Mesh* geom, const char* filename)
{
	//get the relative containing the vrmls.
	std::string path(filename);
	std::string tmp;
	splitPath(path, data_->folder, tmp);

	//parsing...
	data_->geom = geom;
	parser_.parse(filename);

	if (data_->eventStack.size() != 0)
		std::cerr << "Error in " << filename << ": the stack of events has been emptied." << std::endl;
}
