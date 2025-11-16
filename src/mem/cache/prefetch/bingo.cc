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

#include "mem/cache/prefetch/bingo.hh"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "debug/HWPrefetch.hh"
#include "params/BingoPrefetcher.hh"

namespace gem5
{

namespace prefetch
{

namespace
{
constexpr uint64_t
maskBits(unsigned bits)
{
    if (bits == 0)
        return 0;
    if (bits >= 64)
        return std::numeric_limits<uint64_t>::max();
    return (UINT64_C(1) << bits) - 1;
}
} // anonymous namespace

Bingo::PatternHistoryTableSingle::PatternHistoryTableSingle(
    unsigned pattern_len, unsigned pc_width, unsigned min_addr_width,
    unsigned max_addr_width, unsigned total_entries, unsigned assoc,
    double vote_threshold)
  : patternLen(pattern_len), pcWidth(pc_width), minAddrWidth(min_addr_width),
    maxAddrWidth(max_addr_width), voteThreshold(vote_threshold),
    assoc(std::max(1u, assoc)),
    numSets(std::max(1u, total_entries / std::max(1u, assoc))),
    indexBits(numSets > 1 ? floorLog2(numSets) : 0),
    sets(numSets, std::vector<Entry>(this->assoc)),
    lru(numSets)
{
    fatal_if(pattern_len == 0 || !isPowerOf2(pattern_len),
        "Bingo pattern length must be a power-of-two, got %u", pattern_len);
    fatal_if(total_entries == 0,
        "Bingo PHT must contain at least one entry");
    fatal_if(total_entries % this->assoc != 0,
        "Bingo PHT entries (%u) must be divisible by associativity (%u)",
        total_entries, this->assoc);
    fatal_if(numSets == 0, "Bingo PHT requires at least one set");
    fatal_if(numSets > 1 && !isPowerOf2(numSets),
        "Bingo PHT sets must be a power-of-two, got %u", numSets);
    const unsigned pcExtWidth = pcWidth + 1;
    fatal_if(pcExtWidth + minAddrWidth <= indexBits,
        "pc_width (%u) + 1 + min_addr_width (%u) must be > index bits (%u)",
        pcWidth, minAddrWidth, indexBits);
    fatal_if(pcExtWidth + maxAddrWidth <= indexBits,
        "pc_width (%u) + 1 + max_addr_width (%u) must be > index bits (%u)",
        pcWidth, maxAddrWidth, indexBits);

    for (unsigned set = 0; set < numSets; ++set) {
        auto &queue = lru[set];
        for (unsigned way = 0; way < this->assoc; ++way) {
            sets[set][way].valid = false;
            queue.push_back(way);
        }
    }
}

uint64_t
Bingo::PatternHistoryTableSingle::buildKey(uint64_t pc, uint64_t block,
                                           bool secure) const
{
    const unsigned pcExtWidth = pcWidth + 1;

    pc &= maskBits(pcWidth);
    const uint64_t offset_mask = maskBits(minAddrWidth);
    const uint64_t offset = block & offset_mask;
    const uint64_t base = block >> minAddrWidth;
    const uint64_t pc_ext = ((pc << 1) | static_cast<uint64_t>(secure)) &
                            maskBits(pcExtWidth);

    uint64_t key =
        (base << (pcExtWidth + minAddrWidth)) | (pc_ext << minAddrWidth) |
        offset;

    if (indexBits != 0) {
        uint64_t tag = (pc_ext << minAddrWidth) | offset;
        do {
            tag >>= indexBits;
            key ^= tag & maskBits(indexBits);
        } while (tag > 0);
    }

    return key;
}

void
Bingo::PatternHistoryTableSingle::setMRU(unsigned set, unsigned way)
{
    auto &queue = lru[set];
    for (auto it = queue.begin(); it != queue.end(); ++it) {
        if (*it == way) {
            queue.erase(it);
            break;
        }
    }
    queue.push_front(way);
}

unsigned
Bingo::PatternHistoryTableSingle::selectVictim(unsigned set)
{
    auto &queue = lru[set];
    unsigned victim = queue.back();
    queue.pop_back();
    queue.push_front(victim);
    return victim;
}

uint64_t
Bingo::PatternHistoryTableSingle::maskBits(unsigned bits) const
{
    return gem5::prefetch::maskBits(bits);
}

std::vector<bool>
Bingo::PatternHistoryTableSingle::rotate(const std::vector<bool> &pattern,
                                         int amount) const
{
    if (pattern.empty())
        return pattern;
    std::vector<bool> rotated(patternLen, false);
    const int len = static_cast<int>(patternLen);
    for (int i = 0; i < len; ++i) {
        int idx = (i - amount) % len;
        if (idx < 0)
            idx += len;
        rotated[i] = pattern[idx];
    }
    return rotated;
}

std::vector<bool>
Bingo::PatternHistoryTableSingle::vote(
    const std::vector<std::vector<bool>> &patterns) const
{
    if (patterns.empty())
        return {};

    std::vector<unsigned> counts(patternLen, 0);
    for (const auto &pattern : patterns) {
        if (pattern.size() != patternLen)
            continue;
        for (unsigned i = 0; i < patternLen; ++i)
            if (pattern[i])
                counts[i]++;
    }

    std::vector<bool> result(patternLen, false);
    const double total = static_cast<double>(patterns.size());
    if (total == 0.0)
        return result;

    for (unsigned i = 0; i < patternLen; ++i) {
        if (counts[i] / total >= voteThreshold)
            result[i] = true;
    }
    return result;
}

std::vector<bool>
Bingo::PatternHistoryTableSingle::lookup(uint64_t pc, uint64_t block,
                                         bool secure)
{
    uint64_t key = buildKey(pc, block, secure);
    unsigned set = numSets > 1 ? key & maskBits(indexBits) : 0;
    uint64_t tag = numSets > 1 ? key >> indexBits : key;

    const unsigned pcExtWidth = pcWidth + 1;
    const uint64_t min_mask =
        maskBits(pcExtWidth + minAddrWidth - indexBits);
    const uint64_t max_mask =
        maskBits(pcExtWidth + maxAddrWidth - indexBits);

    std::vector<std::vector<bool>> min_matches;
    const int offset =
        static_cast<int>(block & maskBits(minAddrWidth));

    for (unsigned way = 0; way < assoc; ++way) {
        Entry &entry = sets[set][way];
        if (!entry.valid)
            continue;

        const bool min_match =
            ((entry.tag & min_mask) == (tag & min_mask));
        const bool max_match =
            ((entry.tag & max_mask) == (tag & max_mask));

        if (max_match) {
            setMRU(set, way);
            last_event = PrefetchEvent::PCAddress;
            return rotate(entry.pattern, offset);
        }

        if (min_match)
            min_matches.push_back(entry.pattern);
    }

    last_event = PrefetchEvent::None;
    if (!min_matches.empty()) {
        auto voted = vote(min_matches);
        if (!voted.empty()) {
            last_event = PrefetchEvent::PCOffset;
            return rotate(voted, offset);
        }
    }

    return {};
}

void
Bingo::PatternHistoryTableSingle::insert(uint64_t pc, uint64_t block,
                                         bool secure,
                                         const std::vector<bool> &pattern)
{
    if (pattern.size() != patternLen)
        return;

    uint64_t key = buildKey(pc, block, secure);
    unsigned set = numSets > 1 ? key & maskBits(indexBits) : 0;
    uint64_t tag = numSets > 1 ? key >> indexBits : key;

    int offset = static_cast<int>(block & maskBits(minAddrWidth));
    std::vector<bool> stored = rotate(pattern, -offset);

    for (unsigned way = 0; way < assoc; ++way) {
        Entry &entry = sets[set][way];
        if (entry.valid && entry.tag == tag) {
            entry.pattern = stored;
            setMRU(set, way);
            return;
        }
    }

    unsigned victim = assoc;
    for (unsigned way = 0; way < assoc; ++way) {
        if (!sets[set][way].valid) {
            victim = way;
            break;
        }
    }
    if (victim == assoc)
        victim = selectVictim(set);

    Entry &slot = sets[set][victim];
    slot.valid = true;
    slot.tag = tag;
    slot.pattern = stored;
    setMRU(set, victim);
}

Bingo::PatternHistoryTableMulti::PatternHistoryTableMulti(
    unsigned pattern_len, unsigned pc_width, unsigned min_addr_width,
    unsigned max_addr_width, unsigned total_entries, unsigned assoc,
    const std::vector<SubTableConfig> &config)
  : subtables(), patternLen(pattern_len)
{
    fatal_if(config.empty(),
        "Bingo multi-table configuration requires at least one mode");

    for (const auto &cfg : config) {
        SubTable table;
        table.mode = cfg.mode;
        table.assoc = std::max(1u, assoc);
        table.numSets = std::max(1u, total_entries / table.assoc);
        fatal_if(table.numSets > 1 && !isPowerOf2(table.numSets),
            "Bingo multi-table requires power-of-two sets");
        table.indexBits = table.numSets > 1 ? floorLog2(table.numSets) : 0;

        switch (cfg.mode) {
          case Mode::PCAddress:
            table.pcBits = pc_width;
            table.addrBits = max_addr_width;
            break;
          case Mode::AddressOnly:
            table.pcBits = 0;
            table.addrBits = max_addr_width;
            break;
          case Mode::PCOffset:
            table.pcBits = pc_width;
            table.addrBits = min_addr_width;
            break;
          case Mode::PCOnly:
            table.pcBits = pc_width;
            table.addrBits = 0;
            break;
          case Mode::OffsetOnly:
            table.pcBits = 0;
            table.addrBits = min_addr_width;
            break;
        }

        table.secureInAddr = table.addrBits > 0;
        table.secureInPC = !table.secureInAddr;
        table.addrBitsEff = table.addrBits + (table.secureInAddr ? 1 : 0);
        table.pcBitsEff = table.pcBits + (table.secureInPC ? 1 : 0);

        fatal_if(table.pcBitsEff + table.addrBitsEff == 0,
            "Bingo multi-table configuration must provide bits for key");
        fatal_if(table.pcBitsEff + table.addrBitsEff <= table.indexBits,
            "Bingo multi-table key bits (%u) must exceed index bits (%u)",
            table.pcBitsEff + table.addrBitsEff, table.indexBits);

        table.sets.resize(table.numSets,
            std::vector<SubTable::Entry>(table.assoc));
        table.lru.resize(table.numSets);
        for (unsigned set = 0; set < table.numSets; ++set) {
            for (unsigned way = 0; way < table.assoc; ++way) {
                table.sets[set][way].valid = false;
                table.lru[set].push_back(way);
            }
        }

        subtables.push_back(std::move(table));
    }
}

uint64_t
Bingo::PatternHistoryTableMulti::SubTable::maskBits(unsigned bits) const
{
    return gem5::prefetch::maskBits(bits);
}

std::vector<bool>
Bingo::PatternHistoryTableMulti::SubTable::rotate(
    const std::vector<bool> &pattern, int amount) const
{
    if (pattern.empty())
        return pattern;
    std::vector<bool> rotated(pattern.size(), false);
    const int len = static_cast<int>(pattern.size());
    for (int i = 0; i < len; ++i) {
        int idx = (i - amount) % len;
        if (idx < 0)
            idx += len;
        rotated[i] = pattern[idx];
    }
    return rotated;
}

void
Bingo::PatternHistoryTableMulti::SubTable::setMRU(unsigned set, unsigned way)
{
    auto &queue = lru[set];
    for (auto it = queue.begin(); it != queue.end(); ++it) {
        if (*it == way) {
            queue.erase(it);
            break;
        }
    }
    queue.push_front(way);
}

unsigned
Bingo::PatternHistoryTableMulti::SubTable::selectVictim(unsigned set)
{
    auto &queue = lru[set];
    unsigned victim = queue.back();
    queue.pop_back();
    queue.push_front(victim);
    return victim;
}

uint64_t
Bingo::PatternHistoryTableMulti::SubTable::buildKey(uint64_t pc,
                                                    uint64_t block,
                                                    bool secure) const
{
    uint64_t pc_part = pcBits ? (pc & maskBits(pcBits)) : 0;
    uint64_t addr_part = addrBits ? (block & maskBits(addrBits)) : 0;

    if (secureInAddr) {
        addr_part = ((addr_part << 1) | static_cast<uint64_t>(secure)) &
                    maskBits(addrBitsEff);
    } else {
        pc_part = ((pc_part << 1) | static_cast<uint64_t>(secure)) &
                  maskBits(pcBitsEff);
    }

    uint64_t key = (pc_part << addrBitsEff) | addr_part;

    if (indexBits != 0) {
        uint64_t tag = key >> indexBits;
        while (tag > 0) {
            key ^= tag & maskBits(indexBits);
            tag >>= indexBits;
        }
    }

    return key;
}

std::vector<bool>
Bingo::PatternHistoryTableMulti::SubTable::lookup(unsigned pattern_len,
    uint64_t pc, uint64_t block, bool secure) const
{
    if (pattern_len == 0)
        return {};
    uint64_t key = buildKey(pc, block, secure);
    unsigned set = numSets > 1 ? key & maskBits(indexBits) : 0;
    uint64_t tag = numSets > 1 ? key >> indexBits : key;

    for (unsigned way = 0; way < assoc; ++way) {
        Entry const &entry = sets[set][way];
        if (!entry.valid || entry.tag != tag)
            continue;
        const_cast<SubTable*>(this)->setMRU(set, way);
        int offset = static_cast<int>(block % pattern_len);
        return rotate(entry.pattern, offset);
    }
    return {};
}

void
Bingo::PatternHistoryTableMulti::SubTable::insert(unsigned pattern_len,
    uint64_t pc, uint64_t block, bool secure,
    const std::vector<bool> &pattern)
{
    if (pattern.size() != pattern_len)
        return;

    uint64_t key = buildKey(pc, block, secure);
    unsigned set = numSets > 1 ? key & maskBits(indexBits) : 0;
    uint64_t tag = numSets > 1 ? key >> indexBits : key;

    int offset = static_cast<int>(block % pattern_len);
    std::vector<bool> stored = rotate(pattern, -offset);

    for (unsigned way = 0; way < assoc; ++way) {
        Entry &entry = sets[set][way];
        if (entry.valid && entry.tag == tag) {
            entry.pattern = stored;
            setMRU(set, way);
            return;
        }
    }

    unsigned victim = assoc;
    for (unsigned way = 0; way < assoc; ++way) {
        if (!sets[set][way].valid) {
            victim = way;
            break;
        }
    }
    if (victim == assoc)
        victim = selectVictim(set);

    Entry &slot = sets[set][victim];
    slot.valid = true;
    slot.tag = tag;
    slot.pattern = stored;
    setMRU(set, victim);
}

std::vector<bool>
Bingo::PatternHistoryTableMulti::lookup(uint64_t pc, uint64_t block,
                                        bool secure)
{
    for (auto &table : subtables) {
        auto match = table.lookup(patternLen, pc, block, secure);
        if (!match.empty()) {
            switch (table.mode) {
              case Mode::PCAddress: last_event = PrefetchEvent::PCAddress; break;
              case Mode::AddressOnly: last_event = PrefetchEvent::AddressOnly; break;
              case Mode::PCOffset: last_event = PrefetchEvent::PCOffset; break;
              case Mode::PCOnly: last_event = PrefetchEvent::PCOnly; break;
              case Mode::OffsetOnly: last_event = PrefetchEvent::OffsetOnly; break;
            }
            return match;
        }
    }
    last_event = PrefetchEvent::None;
    return {};
}

void
Bingo::PatternHistoryTableMulti::insert(uint64_t pc, uint64_t block,
                                        bool secure,
                                        const std::vector<bool> &pattern)
{
    for (auto &table : subtables)
        table.insert(patternLen, pc, block, secure, pattern);
}

Bingo::Bingo(const BingoPrefetcherParams &p)
  : Queued(p), regionSize(p.region_size), blocksPerRegion(1),
    filterTableSize(p.filter_table_entries),
    accumulationTableSize(p.accumulation_table_entries),
    phtEntries(p.pht_entries), phtAssociativity(p.pht_assoc),
    minAddrWidth(p.min_addr_width), maxAddrWidth(p.max_addr_width),
    pcWidth(p.pc_width),
    voteThreshold(std::max(0.0, std::min(1.0, p.vote_threshold / 100.0))),
    multiTableModes(p.multi_table_modes.begin(), p.multi_table_modes.end()),
    filterTable(filterTableSize), accumulationTable(accumulationTableSize)
{
    fatal_if(regionSize == 0 || regionSize % blkSize != 0,
        "Bingo region size (%u) must be a multiple of the cache block size "
        "(%u)", regionSize, blkSize);
    blocksPerRegion = regionSize >> lBlkSize;
    fatal_if(blocksPerRegion == 0 || !isPowerOf2(blocksPerRegion),
        "Bingo requires region_size / block_size to be power-of-two");
    fatal_if(filterTableSize == 0, "Bingo filter table must have entries");
    fatal_if(accumulationTableSize == 0,
        "Bingo accumulation table must have entries");
    fatal_if(phtEntries == 0,
        "Bingo pattern history table must have entries");
    fatal_if(phtAssociativity == 0,
        "Bingo pattern history table associativity must be non-zero");

    std::vector<PatternHistoryTableMulti::SubTableConfig> configs;
    if (!multiTableModes.empty()) {
        for (const auto &mode_str : multiTableModes) {
            std::string lowered = mode_str;
            std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                           [](unsigned char c) { return std::tolower(c); });
            PatternHistoryTableMulti::Mode mode;
            if (lowered == "pc+addr" || lowered == "pc_address") {
                mode = PatternHistoryTableMulti::Mode::PCAddress;
            } else if (lowered == "addr" || lowered == "address") {
                mode = PatternHistoryTableMulti::Mode::AddressOnly;
            } else if (lowered == "pc+offs" || lowered == "pc_offset") {
                mode = PatternHistoryTableMulti::Mode::PCOffset;
            } else if (lowered == "pc") {
                mode = PatternHistoryTableMulti::Mode::PCOnly;
            } else if (lowered == "offs" || lowered == "offset") {
                mode = PatternHistoryTableMulti::Mode::OffsetOnly;
            } else {
                fatal("Unknown Bingo multi table mode \"%s\"", mode_str);
            }
            configs.push_back({mode});
        }
    }

    if (configs.empty()) {
        matcher = std::make_unique<PatternHistoryTableSingle>(
            blocksPerRegion, pcWidth, minAddrWidth, maxAddrWidth,
            phtEntries, phtAssociativity, voteThreshold);
    } else {
        matcher = std::make_unique<PatternHistoryTableMulti>(
            blocksPerRegion, pcWidth, minAddrWidth, maxAddrWidth,
            phtEntries, phtAssociativity, configs);
    }
}

void
Bingo::commitAccumulation(const RegionKey &key, const AccumEntry &entry)
{
    if (entry.pattern.empty() || entry.pattern.size() != blocksPerRegion)
        return;

    std::vector<bool> pattern = entry.pattern;
    if (entry.triggerOffset < pattern.size())
        pattern[entry.triggerOffset] = true;

    uint64_t block_index =
        (static_cast<uint64_t>(key.region) * blocksPerRegion) +
        entry.triggerOffset;
    matcher->insert(entry.pc, block_index, key.secure, pattern);
}

std::vector<bool>
Bingo::findPattern(uint64_t pc, uint64_t block_index, bool secure)
{
    return matcher ? matcher->lookup(pc, block_index, secure)
                   : std::vector<bool>();
}

void
Bingo::calculatePrefetch(const PrefetchInfo &pfi,
                         std::vector<AddrPriority> &addresses,
                         const CacheAccessor &cache)
{
    if (!pfi.hasPC())
        return;

    const uint64_t pc = pfi.getPC();
    const Addr block_idx = blockIndex(pfi.getAddr());
    const Addr region_idx = block_idx / blocksPerRegion;
    const unsigned offset =
        static_cast<unsigned>(block_idx % blocksPerRegion);
    RegionKey key{region_idx, pfi.isSecure()};

    if (auto *acc = accumulationTable.find(key)) {
        if (offset < acc->pattern.size())
            acc->pattern[offset] = true;
        return;
    }

    if (auto *ft = filterTable.find(key)) {
        if (ft->offset != offset) {
            AccumEntry accum;
            accum.pc = ft->pc;
            accum.triggerOffset = ft->offset;
            accum.pattern.assign(blocksPerRegion, false);
            if (ft->offset < accum.pattern.size())
                accum.pattern[ft->offset] = true;
            if (offset < accum.pattern.size())
                accum.pattern[offset] = true;

            auto evicted = accumulationTable.insert(key, accum);
            filterTable.erase(key);
            if (evicted)
                commitAccumulation(evicted->first, evicted->second);
        }
        return;
    }

    FilterEntry entry{pc, offset};
    filterTable.insert(key, entry);

    auto pattern = findPattern(pc, block_idx, key.secure);
    if (pattern.size() != blocksPerRegion)
        return;
    if (pattern.empty())
        return;

    Addr region_base = region_idx * blocksPerRegion;
    for (unsigned i = 0; i < pattern.size(); ++i) {
        if (!pattern[i] || i == offset)
            continue;

        Addr candidate_block = region_base + i;
        Addr candidate_addr = candidate_block << lBlkSize;
        addresses.emplace_back(candidate_addr, 0);
    }
}

void
Bingo::notifyEvict(const EvictionInfo &info)
{
    const Addr block_idx = blockIndex(info.addr);
    const Addr region_idx = block_idx / blocksPerRegion;
    RegionKey key{region_idx, info.isSecure};

    filterTable.erase(key);
    auto evicted = accumulationTable.erase(key);
    if (evicted)
        commitAccumulation(evicted->first, evicted->second);
}

} // namespace prefetch
} // namespace gem5
