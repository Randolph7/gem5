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

#include <cstdint>
#include <vector>

#include "base/types.hh"
#include "mem/cache/prefetch/queued.hh"

namespace gem5
{

struct BingoPrefetcherParams;

namespace prefetch
{

/**
 * Skeleton for the Bingo hardware prefetcher.
 *
 * Bingo is a region-based prefetcher that learns trigger signatures and the
 * corresponding stream/offset patterns that historically followed those
 * triggers.  This header only provides the scaffolding so the detailed
 * implementation can be filled in later.
 */
class Bingo : public Queued
{
  public:
    Bingo(const BingoPrefetcherParams &p);
    ~Bingo() override = default;

    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses,
                           const CacheAccessor &cache) override;

  private:
    /** Size of the signature table that tracks the current context. */
    const unsigned signatureTableEntries;
    /** Size of the pattern table storing learned offset vectors. */
    const unsigned patternTableEntries;
    /** Maximum number of offsets per pattern that we consider. */
    const unsigned maxRegionOffsets;
    /** Confidence threshold to emit prefetches for a learned pattern. */
    const unsigned confidenceThreshold;
    /** Maximum prefetch distance (in cache blocks) that Bingo can issue. */
    const unsigned maxPrefetchDistance;

    struct SignatureEntry
    {
        Addr triggerPC;
        Addr lastAddress;
        uint32_t signature;
        bool valid;

        SignatureEntry() : triggerPC(0), lastAddress(0), signature(0),
            valid(false)
        {}
    };

    struct PatternEntry
    {
        uint32_t signature;
        std::vector<int32_t> regionOffsets;
        std::vector<uint8_t> confidences;
        bool valid;

        PatternEntry(unsigned max_offsets)
            : signature(0), regionOffsets(max_offsets, 0),
              confidences(max_offsets, 0), valid(false)
        {}
    };

    std::vector<SignatureEntry> signatureTable;
    std::vector<PatternEntry> patternTable;

    /** Locate or allocate the signature entry associated with this access. */
    SignatureEntry &lookupSignatureEntry(const PrefetchInfo &pfi);
    /** Retrieve the pattern entry trained for a given trigger signature. */
    PatternEntry &lookupPatternEntry(uint32_t signature);

    /** Update the context signature as new accesses are observed. */
    void updateSignature(SignatureEntry &entry, const PrefetchInfo &pfi);
    /** Placeholder for Bingo's learning logic, left for future work. */
    void learnPatterns(const PrefetchInfo &pfi, SignatureEntry &sig_entry,
                       PatternEntry &pat_entry, const CacheAccessor &cache);
    /** Translate a learned pattern into concrete prefetch candidates. */
    void generatePrefetchCandidates(const PrefetchInfo &pfi,
                                    const PatternEntry &pat_entry,
                                    std::vector<AddrPriority> &addresses);
};

} // namespace prefetch
} // namespace gem5

#endif // __MEM_CACHE_PREFETCH_BINGO_HH__
