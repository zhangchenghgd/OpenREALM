#ifndef MYREALM_NODEPARAS_H
#define MYREALM_NODEPARAS_H


#include "MyREALM_Core_Exports.h"
#include <unordered_map>

namespace MyREALM
{

	class MyREALM_Core_API NodeParas : 
		public std::unordered_map<std::string, std::string>
	{
	public:
		NodeParas();
		~NodeParas();

		void param(const std::string& key, std::string& value,
			const std::string& default_value = "");

	};

}

#endif  // !MYREALM_NODEPARAS_H