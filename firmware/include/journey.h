#pragma once
#include <stdint.h>
#include "tunables.h"

// The radio and recorded-log replays feed the same time-bucket summaries.
// No packet-order reconstruction or strongest-packet timestamp is required.
struct JourneyBucket {
  uint32_t startMs, endMs;
  uint16_t count;
  int8_t median;
};

struct JourneyEvidence {
  bool active = false, passage = false;
  uint32_t startMs = 0, lastMs = 0, packets = 0;
  uint32_t nearStartMs = 0, nearEndMs = 0, nearPackets = 0;
  uint32_t passEndMs = 0, packetsAtPass = 0;
  uint32_t lastWeakGapMs = 0;
  int8_t passMedian = -127;
  uint16_t tailBuckets = 0, weakBuckets = 0;
  int8_t ending[4] = {};
  uint8_t endingCount = 0, endingIndex = 0;

  void add(const JourneyBucket &b) {
    if (!b.count) return;
    if (!active) { active = true; startMs = b.startMs; }
    const uint32_t gap = b.startMs - lastMs;
    lastMs = b.endMs;
    packets += b.count;
    // A single strong packet cannot establish or move the passage landmark.
    if (b.count >= 2 && b.median >= JOURNEY_PASS_DBM) {
      if (!nearPackets || b.startMs - nearEndMs > JOURNEY_NEAR_GAP_MS) {
        nearStartMs = b.startMs;
        nearPackets = 0;
      }
      nearEndMs = b.endMs;
      nearPackets += b.count;
      if (nearPackets >= JOURNEY_NEAR_PACKETS &&
          nearEndMs - nearStartMs >= APPROACH_NEAR_MIN_MS) {
        passage = true;
        passMedian = b.median;
        passEndMs = b.endMs;
        packetsAtPass = packets;
        tailBuckets = weakBuckets = 0;
        endingCount = endingIndex = 0;
        lastWeakGapMs = 0;
        return;
      }
    }
    if (passage) {
      if (tailBuckets < UINT16_MAX) ++tailBuckets;
      if (b.median <= PRESENCE_FADED_DBM && weakBuckets < UINT16_MAX) ++weakBuckets;
      if (b.median <= PRESENCE_FADED_DBM && gap >= JOURNEY_FADE_GAP_MS)
        lastWeakGapMs = b.startMs;
      ending[endingIndex] = b.median;
      endingIndex = (endingIndex + 1) % 4;
      if (endingCount < 4) ++endingCount;
    }
  }

  uint32_t headMs() const { return passage ? passEndMs - startMs : 0; }
  uint32_t tailMs() const { return passage ? lastMs - passEndMs : 0; }
  uint32_t tailPackets() const { return packets - packetsAtPass; }
  int8_t endingMedian() const {
    int8_t v[4];
    for (unsigned i = 0; i < endingCount; ++i) v[i] = ending[i];
    for (unsigned i = 1; i < endingCount; ++i)
      for (unsigned j = i; j && v[j] < v[j-1]; --j) {
        const int8_t t = v[j]; v[j] = v[j-1]; v[j-1] = t;
      }
    return endingCount ? v[(endingCount - 1) / 2] : 127;
  }
  bool parkedTail() const {
    return passage && weakBuckets >= 3 && endingCount >= 3 &&
           tailMs() >= PRESENCE_PARKED_MIN_MS &&
           tailMs() > headMs() && endingMedian() < RSSI_TRIGGER_DBM;
  }
  bool departureCandidate() const {
    const uint32_t head = headMs(), tail = tailMs();
    // A quick, substantial fade can end before reception becomes sparse.
    // Longer fades instead need low density, a density decrease, and gaps.
    // Both require direction and multiple weak buckets; there is no 15s veto.
    const bool direction = passage && head >= PRESENCE_DEPARTURE_HEAD_MS &&
           head >= tail + PRESENCE_DIRECTION_MARGIN_MS &&
           tail >= JOURNEY_FADE_MIN_MS && weakBuckets >= 2 &&
           endingCount >= 2 && endingMedian() <= PRESENCE_FADED_DBM;
    const bool quickFade = tail <= JOURNEY_FAST_FADE_MS &&
                           passMedian - endingMedian() >= JOURNEY_FADE_DROP_DB;
    const bool thinningFade = lastWeakGapMs && lastMs - lastWeakGapMs <= JOURNEY_ENDING_WINDOW_MS &&
           uint64_t(tailPackets()) * 1000 <= uint64_t(tail) * JOURNEY_FADE_MAX_PPS &&
           uint64_t(tailPackets()) * head * 3 <= uint64_t(packetsAtPass) * tail * 2;
    return direction && (quickFade || thinningFade);
  }
};
