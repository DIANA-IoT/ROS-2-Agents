#ifndef __MESSAGE_CONTAINER_HPP__
#define __MESSAGE_CONTAINER_HPP__

#include <cstdint>
#include <vector>
#include <iterator>

template <typename T>
class MessageContainer
{
public:
    // Default constructor to set the default TTL
    explicit MessageContainer(int8_t defaultTtl) : defaultTtl(defaultTtl) {}

    void set(const int16_t &id, const T &val)
    {
        for (auto &group : groups)
        {
            if (group.id == id)
            {
                group.value = val;
                group.ttl = defaultTtl; // Reset TTL on set
                return;
            }
        }
        groups.push_back({id, defaultTtl, val});
    }

    T get(const int16_t &id)
    {
        T toReturn;
        for (auto it = groups.begin(); it != groups.end(); ++it)
        {
            if (it->id == id)
            {
                it->ttl--; // Decrement TTL for all groups
                toReturn = it->value;
                if (it->ttl <= 0)
                {
                    it = groups.erase(it); // Remove group if TTL reaches 0
                    if (it == groups.end())
                        break;
                }
            }
        }
        return toReturn; // Return default value if not found
    }

    std::vector<int16_t> getAllIds() const
    {
        std::vector<int16_t> ids;
        for (const auto &group : groups)
        {
            if (group.ttl > 0)
            {
                ids.push_back(group.id);
            }
        }
        return ids;
    }

private:
    struct Group
    {
        int16_t id;
        int8_t ttl;
        T value;
    };

    std::vector<Group> groups;
    int8_t defaultTtl; // Default TTL value
};

#endif
