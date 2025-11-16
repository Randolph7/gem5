/*
 * Copyright (c) 2025 The gem5 Authors
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __MEM_CACHE_PREFETCH_BINGO_HH__
#define __MEM_CACHE_PREFETCH_BINGO_HH__

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iterator>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "base/types.hh"
#include "mem/cache/prefetch/queued.hh"

namespace gem5
{

struct BingoPrefetcherParams;

namespace prefetch
{

class Bingo : public Queued
{
  public:
    enum class PrefetchEvent
    {
        None,
        PCAddress,
        AddressOnly,
        PCOffset,
        PCOnly,
        OffsetOnly
    };

    Bingo(const BingoPrefetcherParams &p);
    ~Bingo() override = default;

    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses,
                           const CacheAccessor &cache) override;

    using EvictionInfo = CacheDataUpdateProbeArg;
    void notifyEvict(const EvictionInfo &info) override;

  private:
    struct RegionKey
    {
        Addr region;
        bool secure;

        bool operator==(const RegionKey &other) const
        {
            return region == other.region && secure == other.secure;
        }
    };

    struct RegionKeyHasher
    {
        std::size_t operator()(const RegionKey &key) const
        {
            return std::hash<Addr>{}(key.region) ^
                   (key.secure ? 0x9e3779b97f4a7c15ULL : 0ULL);
        }
    };

    template <class Value>
    class LRUMap
    {
      public:
        using OptionalValue = std::optional<std::pair<RegionKey, Value>>;

        explicit LRUMap(std::size_t capacity);

        Value *find(const RegionKey &key);

        OptionalValue insert(const RegionKey &key, const Value &value);

        OptionalValue erase(const RegionKey &key);

        bool contains(const RegionKey &key) const;

      private:
        struct Node
        {
            RegionKey key;
            Value value;
        };

        void touch(typename std::list<Node>::iterator it);

        const std::size_t capacity;
        std::list<Node> order;
        std::unordered_map<RegionKey, typename std::list<Node>::iterator,
                           RegionKeyHasher>
            map;
    };

    struct FilterEntry
    {
        uint64_t pc;
        unsigned offset;
    };

    struct AccumEntry
    {
        uint64_t pc;
        unsigned triggerOffset;
        std::vector<bool> pattern;
    };

    class PatternMatcher
    {
      public:
        virtual ~PatternMatcher() = default;
        virtual std::vector<bool> lookup(uint64_t pc, uint64_t block,
                                         bool secure) = 0;
        virtual void insert(uint64_t pc, uint64_t block, bool secure,
                            const std::vector<bool> &pattern) = 0;
        virtual PrefetchEvent lastEvent() const = 0;
    };

    class PatternHistoryTableSingle : public PatternMatcher
    {
      public:
        PatternHistoryTableSingle(unsigned pattern_len, unsigned pc_width,
                                  unsigned min_addr_width,
                                  unsigned max_addr_width,
                                  unsigned total_entries, unsigned assoc,
                                  double vote_threshold);

        std::vector<bool> lookup(uint64_t pc, uint64_t block,
                                 bool secure) override;
        void insert(uint64_t pc, uint64_t block, bool secure,
                    const std::vector<bool> &pattern) override;
        PrefetchEvent lastEvent() const override { return last_event; }

      private:
        struct Entry
        {
            bool valid;
            uint64_t tag;
            std::vector<bool> pattern;
        };

        uint64_t buildKey(uint64_t pc, uint64_t block, bool secure) const;
        void setMRU(unsigned set, unsigned way);
        unsigned selectVictim(unsigned set);
        uint64_t maskBits(unsigned bits) const;
        std::vector<bool> vote(
            const std::vector<std::vector<bool>> &patterns) const;
        std::vector<bool> rotate(const std::vector<bool> &pattern,
                                 int amount) const;

        const unsigned patternLen;
        const unsigned pcWidth;
        const unsigned minAddrWidth;
        const unsigned maxAddrWidth;
        const double voteThreshold;
        const unsigned assoc;
        const unsigned numSets;
        const unsigned indexBits;

        std::vector<std::vector<Entry>> sets;
        std::vector<std::list<unsigned>> lru;
        PrefetchEvent last_event = PrefetchEvent::None;
    };

    class PatternHistoryTableMulti : public PatternMatcher
    {
      public:
        enum class Mode
        {
            PCAddress,
            AddressOnly,
            PCOffset,
            PCOnly,
            OffsetOnly
        };

        struct SubTableConfig
        {
            Mode mode;
        };

        PatternHistoryTableMulti(unsigned pattern_len, unsigned pc_width,
                                 unsigned min_addr_width,
                                 unsigned max_addr_width,
                                 unsigned total_entries, unsigned assoc,
                                 const std::vector<SubTableConfig> &config);

        std::vector<bool> lookup(uint64_t pc, uint64_t block,
                                 bool secure) override;
        void insert(uint64_t pc, uint64_t block, bool secure,
                    const std::vector<bool> &pattern) override;
        PrefetchEvent lastEvent() const override { return last_event; }

      private:
        struct SubTable
        {
            Mode mode;
            unsigned pcBits;
            unsigned addrBits;

            struct Entry
            {
                bool valid;
                uint64_t tag;
                std::vector<bool> pattern;
            };

            unsigned numSets;
            unsigned assoc;
            unsigned indexBits;
            unsigned pcBitsEff;
            unsigned addrBitsEff;
            bool secureInPC;
            bool secureInAddr;
            std::vector<std::vector<Entry>> sets;
            std::vector<std::list<unsigned>> lru;

            std::vector<bool> lookup(unsigned pattern_len, uint64_t pc,
                                     uint64_t block, bool secure) const;
            void insert(unsigned pattern_len, uint64_t pc, uint64_t block,
                        bool secure,
                        const std::vector<bool> &pattern);
            uint64_t buildKey(uint64_t pc, uint64_t block,
                              bool secure) const;
            uint64_t maskBits(unsigned bits) const;
            void setMRU(unsigned set, unsigned way);
            unsigned selectVictim(unsigned set);
            std::vector<bool> rotate(const std::vector<bool> &pattern,
                                     int amount) const;
        };

        std::vector<SubTable> subtables;
        PrefetchEvent last_event = PrefetchEvent::None;
        const unsigned patternLen;
    };

    void commitAccumulation(const RegionKey &key, const AccumEntry &entry);
    std::vector<bool> findPattern(uint64_t pc, uint64_t block_index,
                                  bool secure);

    const unsigned regionSize;
    const unsigned filterTableSize;
    const unsigned accumulationTableSize;
    const unsigned phtEntries;
    const unsigned phtAssociativity;
    const unsigned minAddrWidth;
    const unsigned maxAddrWidth;
    const unsigned pcWidth;
    const double voteThreshold;
    const std::vector<std::string> multiTableModes;

    unsigned blocksPerRegion;

    LRUMap<FilterEntry> filterTable;
    LRUMap<AccumEntry> accumulationTable;
    std::unique_ptr<PatternMatcher> matcher;
};

template <class Value>
Bingo::LRUMap<Value>::LRUMap(std::size_t capacity)
  : capacity(capacity)
{
    assert(capacity > 0);
}

template <class Value>
void
Bingo::LRUMap<Value>::touch(typename std::list<Node>::iterator it)
{
    order.splice(order.begin(), order, it);
}

template <class Value>
Value *
Bingo::LRUMap<Value>::find(const RegionKey &key)
{
    auto iter = map.find(key);
    if (iter == map.end())
        return nullptr;
    touch(iter->second);
    return &(iter->second->value);
}

template <class Value>
typename Bingo::LRUMap<Value>::OptionalValue
Bingo::LRUMap<Value>::insert(const RegionKey &key, const Value &value)
{
    auto iter = map.find(key);
    if (iter != map.end()) {
        iter->second->value = value;
        touch(iter->second);
        return std::nullopt;
    }

    order.push_front(Node{key, value});
    map[key] = order.begin();

    OptionalValue evicted;
    if (map.size() > capacity) {
        auto last = std::prev(order.end());
        evicted = std::make_pair(last->key, last->value);
        map.erase(last->key);
        order.pop_back();
    }

    return evicted;
}

template <class Value>
typename Bingo::LRUMap<Value>::OptionalValue
Bingo::LRUMap<Value>::erase(const RegionKey &key)
{
    auto iter = map.find(key);
    if (iter == map.end())
        return std::nullopt;

    OptionalValue removed = std::make_pair(iter->second->key,
                                           iter->second->value);
    order.erase(iter->second);
    map.erase(iter);
    return removed;
}

template <class Value>
bool
Bingo::LRUMap<Value>::contains(const RegionKey &key) const
{
    return map.count(key) != 0;
}

} // namespace prefetch
} // namespace gem5

#endif // __MEM_CACHE_PREFETCH_BINGO_HH__
