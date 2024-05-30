/**
 * Copyright (c) 2018-2020 Inria
 * All rights reserved.
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
#include <stdio.h>
#include "mem/cache/replacement_policies/lilru_rp.hh"
#include "mem/cache/base.hh"

#include <cassert>
#include <memory>

#include "params/LILRURP.hh"
#include "sim/cur_tick.hh"
#include "debug/CacheRepl.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(ReplacementPolicy, replacement_policy);
namespace replacement_policy
{
Tick LastMissLatency = 0;
LILRU::LILRU(const Params &p)
  : LRU(p)
{
}

void
LILRU::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    LRU::invalidate(replacement_data);
    // Reset last touch timestamp
    std::static_pointer_cast<LILRUReplData>(
        replacement_data)->lastTouchTick = Tick(0);
    std::static_pointer_cast<LILRUReplData>(
        replacement_data)->reusedFlag = true;
}

// Randolph: Give new parameters when touched
void
LILRU::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    LRU::touch(replacement_data);
    // Update last touch timestamp
    std::static_pointer_cast<LILRUReplData>(
        replacement_data)->lastTouchTick = curTick();
    std::static_pointer_cast<LILRUReplData>(
        replacement_data)->reusedFlag = false;
}

// Randolph: Give new parameters when reset
void 
LILRU::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    LRU::reset(replacement_data);
    // Set last touch timestamp
    std::static_pointer_cast<LILRUReplData>(
        replacement_data)->lastTouchTick = curTick();
    std::static_pointer_cast<LILRUReplData>(
        replacement_data)->missPenalty = LastMissLatency;
    std::static_pointer_cast<LILRUReplData>(
        replacement_data)->reusedFlag = false;

    // Randolph: Print tick
    DPRINTF(CacheRepl, "Randolph: nnnnnn Last miss latency: %lu\n", LastMissLatency);
}

// Randolph: Change sequance test func
ReplaceableEntry*
LILRU::getVictim(const ReplacementCandidates& candidates) const
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    // Initialize variables for the smallest lastTouchTick
    ReplaceableEntry* smallest = nullptr;
    Tick smallestMissPenalty = 0;

    // Find the smallest lastTouchTick candidate
    for (const auto& candidate : candidates) {
        if (smallest == nullptr || std::static_pointer_cast<LILRUReplData>(candidate->replacementData)->lastTouchTick <
            std::static_pointer_cast<LILRUReplData>(smallest->replacementData)->lastTouchTick) {
            smallest = candidate;
        }
    }
    smallestMissPenalty = std::static_pointer_cast<LILRUReplData>(smallest->replacementData)->missPenalty;

    // If the smallest candidate's reusedFlag is true, evict it
    if (std::static_pointer_cast<LILRUReplData>(smallest->replacementData)->reusedFlag) {
        return smallest;
    }

    // Initialize variables to find the next 4 smallest lastTouchTick candidates
    std::vector<ReplaceableEntry*> lastCandidates;
    const size_t maxCandidates = 4;

    // Find the next 4 candidates with the smallest lastTouchTick
    for (const auto& candidate : candidates) {
        if (candidate != smallest) {
            lastCandidates.push_back(candidate);
            if (lastCandidates.size() > maxCandidates) {
                // Find the candidate with the largest lastTouchTick in lastCandidates and remove it
                auto maxIt = std::max_element(lastCandidates.begin(), lastCandidates.end(),
                                              [](const auto& a, const auto& b) {
                                                  return std::static_pointer_cast<LILRUReplData>(a->replacementData)->lastTouchTick <
                                                         std::static_pointer_cast<LILRUReplData>(b->replacementData)->lastTouchTick;
                                              });
                lastCandidates.erase(maxIt);
            }
        }
    }

    // Sort lastCandidates by lastTouchTick
    std::sort(lastCandidates.begin(), lastCandidates.end(),
              [](const ReplaceableEntry* a, const ReplaceableEntry* b) {
                  return std::static_pointer_cast<LILRUReplData>(a->replacementData)->lastTouchTick <
                         std::static_pointer_cast<LILRUReplData>(b->replacementData)->lastTouchTick;
              });

    // Comparing
    Tick missPenaltySum = 0;
    int count = 0;
    for (count=0; count<lastCandidates.size(); count++){
        missPenaltySum += std::static_pointer_cast<LILRUReplData>(lastCandidates[count]->replacementData)->missPenalty;
        if (missPenaltySum > smallestMissPenalty)
        {
            break;
        }
    }

    // Adjust the lastTouchTick and flags accordingly if needed
    if (count > 0) {
        // Set the reusedFlag for the evicted candidate
        std::static_pointer_cast<LILRUReplData>(smallest->replacementData)->reusedFlag = true;
        std::static_pointer_cast<LILRUReplData>(smallest->replacementData)->lastTouchTick = 
            std::static_pointer_cast<LILRUReplData>(lastCandidates[count]->replacementData)->lastTouchTick;
        
        // Perform the necessary adjustments
        for (int i = count; i > 0; --i) {
            std::static_pointer_cast<LILRUReplData>(lastCandidates[i]->replacementData)->lastTouchTick =
                std::static_pointer_cast<LILRUReplData>(lastCandidates[i - 1]->replacementData)->lastTouchTick;
        }

        return lastCandidates[0];
    }

    // Otherwise, evict the smallest candidate
    return smallest;
}

std::shared_ptr<ReplacementData>
LILRU::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new LILRUReplData());
}

} // namespace replacement_policy
} // namespace gem5
