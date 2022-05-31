#ifndef MYREALM_MYREALM_H
#define MYREALM_MYREALM_H

#include "MyREALM_Core_Exports.h"
#include "MyPublisher.h"


namespace MyREALM
{
	class MyREALM_Core_API MyRealmSys
	{
	public:
		~MyRealmSys();

		MyRealmSys(const MyRealmSys& other) = delete;

		MyRealmSys& operator=(const MyRealmSys&) = delete;

		static MyRealmSys& get_instance();

		MyPublisher* createrPublisher(const std::string& name);

		MyPublisher* getPublisher(const std::string& name);

		MyPublisher* getOrCreatePublisher(const std::string& name);

		bool ok() const;

	private:
		MyRealmSys();

	private:
		MyPublishers m_publishers;
		std::mutex m_mutex;
		bool m_bOK;
	};
}

#endif // !MYREALM_MYREALM_H
