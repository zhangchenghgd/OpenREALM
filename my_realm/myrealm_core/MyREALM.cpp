#include "MyREALM.h"

namespace MyREALM
{
	MyRealmSys::MyRealmSys():m_bOK(true)
	{
	}

	MyRealmSys::~MyRealmSys()
	{
	}

	MyRealmSys& MyRealmSys::get_instance()
	{
		static MyRealmSys instance;
		return instance;
	}

	MyPublisher* MyRealmSys::createrPublisher(const std::string& name)
	{
		m_mutex.lock();
		MyPublishers::iterator it = m_publishers.find(name);
		if (it == m_publishers.end())
		{
			std::shared_ptr<MyPublisher> pub =
				std::make_shared<MyPublisher>();
			m_publishers[name] = pub;
		}
		m_mutex.unlock();
		return m_publishers[name].get();
	}

	MyPublisher* MyRealmSys::getPublisher(const std::string& name)
	{
		MyPublishers::iterator it = m_publishers.find(name);
		if (it != m_publishers.end())
		{
			return it->second.get();
		}

		return nullptr;
	}

	MyPublisher* MyRealmSys::getOrCreatePublisher(const std::string& name)
	{
		m_mutex.lock();
		MyPublishers::iterator it = m_publishers.find(name);
		if (it == m_publishers.end())
		{
			std::shared_ptr<MyPublisher> pub =
				std::make_shared<MyPublisher>();
			m_publishers[name] = pub;
		}
		m_mutex.unlock();

		return m_publishers[name].get();
	}

	bool MyRealmSys::ok() const
	{
		return m_bOK;
	}
}

