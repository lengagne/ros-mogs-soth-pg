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
//      2012-2013: IUT de Beziers/ LIRMM, Beziers, France
//	from 2013 : Université Blaise Pascal / axis : ISPR / theme MACCS

#include <list>
#include <algorithm>
#include <cctype>
#include <fstream>
#include <utility>
#include <cstring>
#include "WRLBasicParser.h"
#include <iostream>

class BrowserSet
{
private:

	typedef std::pair<unsigned, unsigned> browser_t;

public:

	BrowserSet(Automaton<unsigned>& dictionary, const std::vector<BasicHandler*>& handlers)
		: dictionary_(dictionary), handlers_(handlers) {}

	~BrowserSet()
	{
		std::list<browser_t>::iterator endIt = browsers_.end();
		std::list<browser_t>::iterator itToDelete = browsers_.begin();

		for(std::list<browser_t>::iterator it = browsers_.begin(); it != endIt;) {
			std::list<browser_t>::iterator itToDelete = it++;
			dictionary_.removeBrowser(itToDelete->first);
			browsers_.erase(itToDelete);
		}
	}

	// Forwards all the automaton browsers using transition (c,0).
	// Remove all the browsers that cannot forward with this transition.
	// Save the callable browsers (the state reached by (c,0) is terminal).
	// The previous callable browsers are erased before doing the transition.
	void feed(char c)
	{
		unsigned i = dictionary_.createBrowser();
		browsers_.push_back(std::make_pair(i, 0));
		callable_.clear();

		for(std::list<browser_t>::iterator it = browsers_.begin(); it != browsers_.end();) {
			if(dictionary_.browseNext(it->first, c, 0)) {
				++it->second;
				if(dictionary_.getValue(it->first) < handlers_.size()){ callable_.push_back(*it); }
				++it;
			}
			else {
				std::list<browser_t>::iterator itToDelete = it++;
				dictionary_.removeBrowser(itToDelete->first);
				browsers_.erase(itToDelete);
			}
		}
	}

	// Calls the handler of the terminal state corresponding to the longest
	// matched character sequence.
	void callLongest(std::istream& in)
	{
		if(!callable_.empty()) {
			std::list<browser_t>::iterator longest = callable_.begin();

			for(std::list<browser_t>::iterator it = callable_.begin(); it != callable_.end(); ++it) {
				if(it->second > longest->second){ longest = it; }
			}

			unsigned h = dictionary_.getValue(longest->first);
			(*handlers_[h])(in);
		}
	}

private:

	Automaton<unsigned>& dictionary_;
	const std::vector<BasicHandler*>& handlers_;
	std::list<browser_t> browsers_;
	std::list<browser_t> callable_;
};


WRLBasicParser::WRLBasicParser()
	: dictionary_(static_cast<unsigned>(std::string::npos))
	, characterHandler_(0x0)
{
}

void WRLBasicParser::addHandler(const std::string & keyword, BasicHandler* handler)
{
	dictionary_.addEntry(keyword.c_str(), keyword.length(), 0, static_cast<unsigned>(handlers_.size()));
	handlers_.push_back(handler);
}

void WRLBasicParser::setCharacterHandler(BasicHandler* handler)
{
	characterHandler_ = handler;
}


void WRLBasicParser::parse(const char* filename)
{
	std::ifstream in(filename);
	in.precision(12u);
	if(!in.is_open()){
		std::cerr<<"Error in "<< __FILE__<<" in line: "<< __LINE__<<std::endl;
		std::cerr<<"Cannot open "<<filename<<std::endl;
		exit(-1);
	}

	BrowserSet browsers(dictionary_, handlers_);
	while(!in.eof()) {
		char c = in.get();
		if(characterHandler_){ (*characterHandler_)(in); }

		browsers.feed(c);
		browsers.callLongest(in);
	}
}
