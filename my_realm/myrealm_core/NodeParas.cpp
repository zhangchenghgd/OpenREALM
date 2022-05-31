#include "NodeParas.h"

namespace MyREALM
{

	NodeParas::NodeParas()
	{
	}

	NodeParas::~NodeParas()
	{
	}


	void NodeParas::param(const std::string& key,
		 std::string& value, 
		const std::string& default_value)
	{
		NodeParas::iterator it =
			this->find(key);
		if (it != this->end())
		{
			value = this->at(key);
		}
		else
		{
			value = default_value;
		}
	}

}