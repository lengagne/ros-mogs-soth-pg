//      AutomatonNode.h
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

#ifndef _AUTOMATON_H_
#define _AUTOMATON_H_

#include <vector>
#include <map>
#include <iostream>

template<class DataT>
class AutomatonNode
{
public:

	typedef DataT data_t;
	typedef std::pair<char, char> trans_t;
	typedef std::map<trans_t, AutomatonNode<data_t> >	nodes_t;

public:

	AutomatonNode(const data_t& val, bool final): val_(val), final_(final) {}

	AutomatonNode<data_t>* createSucc(char entry, char label, const data_t& val, bool final)
	{
		typename std::pair< typename nodes_t::iterator, bool> p =
			succ_.insert(std::make_pair(std::make_pair(entry, label), AutomatonNode<data_t>(val, final)));

		if(!p.second && (final || !p.first->second.final_)) {
			p.first->second.val_ = val;
			p.first->second.final_ = final;
		}
		return &p.first->second;
	}

	const AutomatonNode<data_t>* getNextNode(char entry, char label) const
	{
		typename nodes_t::const_iterator it = succ_.find(std::make_pair(entry, label));
		if(it != succ_.end()){ return &it->second; }
		return 0x0;
	}

	const data_t& getValue() const { return val_; }

private:

	data_t val_;
	nodes_t succ_;
	bool final_;
};

template<class DataT>
class Automaton
{
public:

	typedef DataT data_t;

public:

	Automaton(const data_t& defaultVal): defaultVal_(defaultVal), start_(defaultVal, false) {}

	void addEntry(const char* entry, size_t size, char label, const data_t& value)
	{
		/*typename*/ AutomatonNode<data_t>* ptr = &start_;
		for(; size > 1; ++entry, --size) {
			ptr = ptr->createSucc(*entry, label, defaultVal_, false);
		}
		ptr->createSucc(*entry, label, value, true);
	}

	data_t getEntry(const char* entry, size_t size, char label) const
	{
		const AutomatonNode<data_t>* ptr = &start_;
		for(; ptr && (size > 0); ++entry, --size){ ptr = ptr->getNextNode(*entry, label); }
		if(ptr){ return ptr->getValue(); }
		return defaultVal_;
	}

	bool browseNext(unsigned id, char entry, char label)
	{
		if((id >= browsers_.size()) || !browsers_[id]) {
			std::cerr<<"invalid browser id"<<std::endl;
			exit(-1);
		}
		const AutomatonNode<data_t>* tmp = browsers_[id]->getNextNode(entry, label);
		if(tmp == 0x0){ return false; }
		browsers_[id] = tmp;
		return true;
	}

	data_t getValue(unsigned id) const
	{
		if((id >= browsers_.size()) || !browsers_[id]) {
			std::cerr<<"invalid browser id"<<std::endl;
			exit(-1);
		}
		return browsers_[id]->getValue();
	}

	unsigned createBrowser()
	{
		typename std::vector<const AutomatonNode<data_t>*>::iterator it =
			std::find(browsers_.begin(), browsers_.end(),
			static_cast<const AutomatonNode<data_t>*>(0x0));

		if(it == browsers_.end()) {
			browsers_.push_back(&start_);
			return (browsers_.size() - 1);
		}

		*it = &start_;
		return std::distance(browsers_.begin(), it);
	}

	void removeBrowser(unsigned id)
	{
		if((id >= browsers_.size()) || !browsers_[id]) {
			std::cerr<<"invalid browser id"<<std::endl;
			exit(-1);
		}
		browsers_[id] = 0x0;
	}

private:

	data_t defaultVal_;
	AutomatonNode<data_t> start_;
	std::vector<const AutomatonNode<data_t>*> browsers_;
};

#endif
