//      WRLBasicParser.cpp
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


#ifndef _AF_BASIC_PARSER_H_
#define _AF_BASIC_PARSER_H_

#include <iosfwd>
#include <vector>

#include "Automaton.h"
#include "WRLBasicParser.h"


class BasicHandler
{
  public:

    virtual ~BasicHandler() {}
    virtual void operator()(std::istream& /*in*/) {}
};

class WRLBasicParser
  {
  public:

    WRLBasicParser();

    void addHandler(const std::string & keyword, BasicHandler* handler);
    void setCharacterHandler(BasicHandler* handler);
    void parse(const char* filename);

  private:

    Automaton<unsigned> dictionary_;
    std::vector<BasicHandler*> handlers_;
    BasicHandler* characterHandler_;
};

#endif
