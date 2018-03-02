#ifndef RESOURCEMANAGER_H
#define RESOURCEMANAGER_H

#include <functional>
#include <memory>
#include <future>
#include <chrono>
#include <map>
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
    using ResourceHandle = std::shared_ptr<T>;
    using Future = std::future<ResourceHandle>;

    using ResourceCache = std::map<std::string, std::weak_ptr<T>>;
    using FutureCache = std::map<std::string, Future>;

    // Class for improved resurce loading: 
    // Every frame, `get()` was being called, to check if loading finished yet.
    // Every time `get()` was called, it performed at least one map lookup, which is an expensive operation to do every frame.
    // This patch introduces a TransientState object that caches the map iterators from one call of `get()` to the next.
    class TransientState
    {
        friend class ResourceManager;
        typename ResourceCache::iterator m_resourceIt;
        typename FutureCache::iterator m_futureIt;
        bool m_searchedResource = false;
        bool m_searchedFuture = false;
    };

    ResourceManager(std::function<Future(std::string)> factory) : m_factory(factory) {}

    std::shared_ptr<T> get(const std::string &name, TransientState *state)
    {
        auto &resourceIt = state->m_resourceIt;
        auto &futureIt = state->m_futureIt;

        std::shared_ptr<T> resource;
        if (!state->m_searchedResource) { resourceIt = m_resourceCache.find(name); state->m_searchedResource = true; }
        if (resourceIt != m_resourceCache.end())
        {
            resource = resourceIt->second.lock();
        }
        if (resource) { return resource; }

        if (!state->m_searchedFuture) { futureIt = m_futureCache.find(name); state->m_searchedFuture = true; }
        if (futureIt == m_futureCache.end())
        {
            futureIt = m_futureCache.emplace(name, m_factory(name)).first;
        }
        auto &future = futureIt->second;

        assert(future.valid());
        if (future.wait_for(std::chrono::duration<int>::zero()) != std::future_status::ready) // not yet finished loading
        {
            return nullptr;
        }
        resource = future.get();
        m_futureCache.erase(name);
        m_resourceCache[name] = resource;
        *state = {};
        return resource;
    }

private:
    std::function<Future(std::string)> m_factory;
    ResourceCache m_resourceCache;
    FutureCache m_futureCache;
};

#endif
