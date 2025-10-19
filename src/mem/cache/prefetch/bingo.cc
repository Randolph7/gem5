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
#include <cassert>
#include <cstdlib>

#include "base/intmath.hh"
#include "debug/HWPrefetch.hh"
#include "params/BingoPrefetcher.hh"

namespace gem5
{

namespace prefetch
{

Bingo::Bingo(const BingoPrefetcherParams &p)
  : Queued(p),
    signatureTableEntries(std::max(1u, p.signature_table_entries)),
    patternTableEntries(std::max(1u, p.pattern_table_entries)),
    maxRegionOffsets(std::max(1u, p.max_region_offsets)),
    confidenceThreshold(p.confidence_threshold),
    maxPrefetchDistance(p.max_prefetch_distance),
    signatureTable(signatureTableEntries),
    patternTable()
{
    patternTable.reserve(patternTableEntries);
    for (unsigned i = 0; i < patternTableEntries; ++i) {
        patternTable.emplace_back(maxRegionOffsets);
    }
}

void
Bingo::calculatePrefetch(const PrefetchInfo &pfi,
                         std::vector<AddrPriority> &addresses,
                         const CacheAccessor &cache)
{
    if (!pfi.hasPC()) {
        DPRINTF(HWPrefetch, "Bingo: ignoring access without PC information\n");
        return;
    }

    auto &sig_entry = lookupSignatureEntry(pfi);
    updateSignature(sig_entry, pfi);

    auto &pat_entry = lookupPatternEntry(sig_entry.signature);
    learnPatterns(pfi, sig_entry, pat_entry, cache);
    generatePrefetchCandidates(pfi, pat_entry, addresses);
}

Bingo::SignatureEntry &
Bingo::lookupSignatureEntry(const PrefetchInfo &pfi)
{
    assert(!signatureTable.empty());
    Addr tag = pfi.hasPC() ? pfi.getPC() : pfi.getAddr();
    size_t index = (tag >> lBlkSize) % signatureTableEntries;
    return signatureTable[index];
}

Bingo::PatternEntry &
Bingo::lookupPatternEntry(uint32_t signature)
{
    assert(!patternTable.empty());
    size_t index = signature % patternTableEntries;
    PatternEntry &entry = patternTable[index];

    if (!entry.valid || entry.signature != signature) {
        entry.signature = signature;
        std::fill(entry.regionOffsets.begin(), entry.regionOffsets.end(), 0);
        std::fill(entry.confidences.begin(), entry.confidences.end(), 0);
        entry.valid = false;
    }

    return entry;
}

void
Bingo::updateSignature(SignatureEntry &entry, const PrefetchInfo &pfi)
{
    entry.valid = true;
    entry.triggerPC = pfi.hasPC() ? pfi.getPC() : 0;
    entry.lastAddress = blockAddress(pfi.getAddr());

    // Placeholder signature update.
    entry.signature = (entry.signature << 1) ^
        static_cast<uint32_t>(entry.lastAddress >> lBlkSize);
}

void
Bingo::learnPatterns(const PrefetchInfo &pfi, SignatureEntry &sig_entry,
                     PatternEntry &pat_entry, const CacheAccessor &cache)
{
    // This method will eventually update pattern entries based on the current
    // trigger context and observed demand stream.  The scaffold simply keeps
    // the bookkeeping in sync so future work can plug the full algorithm.
    pat_entry.signature = sig_entry.signature;

    // TODO: Implement Bingo's learning procedure:
    //  * update regionOffsets/confidences using the spatial region tracked
    //    by sig_entry.lastAddress
    //  * incorporate cache feedback to adapt entries
    // For now we leave the entry marked invalid so no prefetches are issued.
    pat_entry.valid = false;

    DPRINTF(HWPrefetch, "Bingo: learning step placeholder for signature %#x\n",
            pat_entry.signature);
}

void
Bingo::generatePrefetchCandidates(const PrefetchInfo &pfi,
                                  const PatternEntry &pat_entry,
                                  std::vector<AddrPriority> &addresses)
{
    if (!pat_entry.valid) {
        return;
    }

    Addr base = blockAddress(pfi.getAddr());
    for (size_t i = 0; i < pat_entry.regionOffsets.size(); ++i) {
        if (pat_entry.confidences[i] < confidenceThreshold) {
            continue;
        }

        int32_t offset = pat_entry.regionOffsets[i];
        if (std::abs(offset) > static_cast<int32_t>(maxPrefetchDistance)) {
            continue;
        }

        Addr candidate = base + static_cast<Addr>(offset) * blkSize;
        addresses.emplace_back(candidate, 0);
    }
}

} // namespace prefetch
} // namespace gem5
