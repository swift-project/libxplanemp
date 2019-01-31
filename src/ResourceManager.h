#ifndef RESOURCEMANAGER_H
#define RESOURCEMANAGER_H

#include <functional>
#include <memory>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <assert.h>

// ResourceManager is the central point to get texture and model resources.
// In case a resource was already loaded and is still in memory, a shared pointer is returned.
// In case the resource was not yet loaded, async loading will be triggered.
// Until the resource loading is completed an empty pointer will be returned.
// ResourceManager itself keeps weak pointers of each loaded resource. As
// soon as all shared pointers are deleted, the resource will be freed automatically.
template <typename T>
class ResourceManager
{
public:
    using ResourceHandle = std::shared_ptr<T>; // C++20: use std::atomic<std::shared_ptr<T>>
    using Factory = std::function<ResourceHandle(std::string)>;
    using Callback = std::function<void(const ResourceHandle &)>;

    ResourceManager(Factory factory) : m_factory(factory) {}

    void loadAsync(const std::string &name, Callback callback)
    {
        std::thread loaderThread ([ = ]
        {
            ResourceHandle resource;
            std::unique_lock<std::mutex> lock(m_mutex);
            auto resourceIt = m_resourceCache.find(name);
            if (resourceIt != m_resourceCache.end())
            {
                resource = resourceIt->second.lock();
            }
            if (! resource)
            {
                resource = m_factory(name);
                m_resourceCache[name] = resource;
            }
            lock.unlock();
            callback(resource);
        });
        
		if (loaderThread.joinable())
		{
			loaderThread.detach();
		}
    }

private:
    Factory m_factory;
    std::mutex m_mutex;
    std::unordered_map<std::string, std::weak_ptr<T>> m_resourceCache;
};

#endif
