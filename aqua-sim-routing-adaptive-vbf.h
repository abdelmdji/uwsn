/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Adaptive-IoUT-VBF : density- and energy-aware vector-based forwarding
 * for the Internet of Underwater Things.
 *
 * Implements the protocol specified in:
 *   Azzouz, Ben Messaoud, Benarfa, "Adaptive-IoUT-VBF: A Density-Aware
 *   Dynamic Routing Protocol for ... Internet of Underwater Things".
 *
 * Design notes (kept deliberately close to the manuscript):
 *   - R_adaptive = R_base * (0.50 + 0.50*eps) * delta(rho)          [Eq. 11]
 *   - eps  = max(E_i/E_init, 0.20)                                  [Eq. 11]
 *   - delta(rho) piecewise from the one-hop neighbour count         [Eq. 12]
 *   - T_hold = (d_perp / R_adaptive) * T_max                        [Eq. 15]
 *   - counter-based duplicate suppression: cancel after K overheard
 *   - three-stage fallback void recovery                            [Eq. 18]
 *
 * The forwarding decision is taken ENTIRELY at the receiver. A node never
 * schedules a transmission at another node and never modifies another node's
 * state. R_adaptive travels in the packet header so receivers evaluate
 * against the same corridor the forwarder computed.
 */

#ifndef AQUA_SIM_ROUTING_ADAPTIVE_VBF_H
#define AQUA_SIM_ROUTING_ADAPTIVE_VBF_H

#include "aqua-sim-routing.h"
#include "aqua-sim-address.h"
#include "ns3/vector.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/random-variable-stream.h"

#include <map>
#include <set>
#include <vector>

namespace ns3 {

/* Message types carried in VBHeader::MessType for this protocol. */
#define AIV_DATA    41
#define AIV_BEACON  42

/**
 * \brief Per-node bookkeeping for the protocol's own energy accounting.
 *
 * The manuscript specifies fixed per-event costs (E_TX = 0.50 J,
 * E_RX = 0.10 J, E_idle = 0.001 J/s) rather than a distance-dependent
 * physical model. We implement exactly that, so the code and the paper
 * describe the same thing. AquaSimEnergyModel is left untouched and can be
 * swapped in later for the distance-dependent sensitivity run.
 */
class AivEnergyLedger
{
public:
  static AivEnergyLedger& Get ();

  void Configure (uint32_t nNodes, double eInit, double eTx, double eRx,
                  double eIdlePerSec);
  void Reset ();

  double Energy (uint32_t nodeId) const;
  double Fraction (uint32_t nodeId) const;   // E_i / E_init, floored at 0
  bool   Alive (uint32_t nodeId) const;      // E_i >= E_TX

  /* Returns false if the node could not afford the transmission. */
  bool ChargeTx (uint32_t nodeId);
  void ChargeRx (uint32_t nodeId);
  void ChargeBeacon (uint32_t nodeId, double payloadRatio);

  /* Applies idle drain up to 'now' and records first-death time. */
  void ApplyIdleUpTo (double now);

  double FirstDeathTime () const { return m_firstDeath; }
  bool   AnyDeath () const { return m_firstDeath >= 0.0; }

  double ETx () const { return m_eTx; }
  double EInit () const { return m_eInit; }

  /* Jain's fairness index over CONSUMED energy, and the paper's energy gap. */
  double JainFairness () const;
  double EnergyGap () const;
  double MeanConsumed () const;
  std::vector<double> Consumed () const;

private:
  AivEnergyLedger () {}
  void NoteDeath (uint32_t nodeId);

  std::vector<double> m_energy;
  double m_eInit = 100.0;
  double m_eTx = 0.50;
  double m_eRx = 0.10;
  double m_eIdle = 0.001;
  double m_lastIdleApplied = 0.0;
  double m_firstDeath = -1.0;
};

/**
 * \brief Global statistics collector (throughput, PDR, delay, hop count).
 *
 * Duplicates are inevitable with K > 1 forwarders, so delivery is counted
 * on the FIRST arrival of each (source, sequence) pair only.
 */
class AivStats
{
public:
  static AivStats& Get ();
  void Reset ();

  void NoteGenerated (uint32_t src, uint32_t seq);
  /* Returns true if this was the first (i.e. non-duplicate) arrival. */
  bool NoteDelivered (uint32_t src, uint32_t seq, uint32_t bits,
                      double delaySec, uint32_t hops);
  void NoteDropped ()      { m_dropped++; }
  void NoteVoidFallback () { m_fallbackUsed++; }
  void NoteTransmission () { m_transmissions++; }

  uint32_t Generated () const   { return m_generated; }
  uint32_t Delivered () const   { return m_delivered; }
  uint32_t Duplicates () const  { return m_duplicates; }
  uint32_t Dropped () const     { return m_dropped; }
  uint32_t Fallbacks () const   { return m_fallbackUsed; }
  uint32_t Transmissions () const { return m_transmissions; }
  uint64_t DeliveredBits () const { return m_bits; }
  double   MeanDelay () const;
  double   MeanHops () const;
  double   Pdr () const;
  double   ThroughputKbps (double simTime) const;

private:
  AivStats () {}
  std::set<std::pair<uint32_t,uint32_t> > m_seen;
  uint32_t m_generated = 0, m_delivered = 0, m_duplicates = 0;
  uint32_t m_dropped = 0, m_fallbackUsed = 0, m_transmissions = 0;
  uint64_t m_bits = 0;
  double m_delaySum = 0.0;
  uint64_t m_hopSum = 0;
};

/** \brief Neighbour table entry, refreshed by periodic beacons. */
struct AivNeighbour
{
  AquaSimAddress addr;
  Vector pos;
  double energyFraction;
  double lastHeard;
};

/**
 * \ingroup aqua-sim-ng
 * \brief Adaptive-IoUT-VBF routing.
 */
class AquaSimAdaptiveVbf : public AquaSimRouting
{
public:
  AquaSimAdaptiveVbf ();
  virtual ~AquaSimAdaptiveVbf ();

  static TypeId GetTypeId (void);
  int64_t AssignStreams (int64_t stream);

  virtual bool Recv (Ptr<Packet> packet, const Address &dest,
                     uint16_t protocolNumber);

  void SetSinkPosition (Vector p) { m_sinkPos = p; }
  Vector GetSinkPosition () const { return m_sinkPos; }

  /* Called by the traffic generator on a source node. */
  void OriginatePacket (uint32_t seq, uint32_t payloadBytes);

  void StartBeaconing ();

protected:
  virtual void DoDispose ();

private:
  /* --- geometry -------------------------------------------------------- */
  static double PerpDistance (const Vector &cand, const Vector &from,
                              const Vector &sink);
  double Progress (const Vector &cand, const Vector &self) const;

  /* --- adaptive radius, Eq. (11)-(12) ---------------------------------- */
  double DensityFactor (uint32_t rho) const;
  double AdaptiveRadius (double energyFraction, uint32_t rho) const;

  /* --- neighbour table ------------------------------------------------- */
  void SendBeacon ();
  void HandleBeacon (Ptr<Packet> p);
  void PurgeNeighbours ();
  uint32_t NeighbourCount ();

  /* --- forwarding ------------------------------------------------------ */
  void HandleData (Ptr<Packet> p);
  void ForwardAfterHold (Ptr<Packet> p, uint32_t src, uint32_t seq);
  void Broadcast (Ptr<Packet> p);
  Vector SelfPosition ();
  uint32_t SelfId ();

  /* Does at least one neighbour qualify inside 'radius'? Used by the
     forwarder to decide whether fallback expansion is needed. */
  bool AnyCandidate (double radius, double guard);

  /* --- parameters ------------------------------------------------------ */
  double m_rBase;        // R_base           (m)
  double m_rComm;        // R_c              (m)
  double m_tMaxHold;     // T_max            (s)
  uint32_t m_k;          // K forwarders / suppression threshold
  double m_guardNormal;  // gamma_normal     (m)
  double m_guardFallback;// gamma_fallback   (m)
  double m_deliverRadius;// R_deliver        (m)
  uint32_t m_maxHops;
  double m_epsMin;
  double m_soundSpeed;
  double m_beaconInterval;
  uint32_t m_beaconBytes;
  double m_neighbourTimeout;
  bool m_isSink;
  bool m_enableAdaptation;   // false => fixed R_base (ablation switch)

  Vector m_sinkPos;
  std::map<uint32_t, AivNeighbour> m_neighbours;

  /* Pending forwards awaiting their holding timer, keyed by (src,seq). */
  struct Pending { EventId ev; uint32_t heard; };
  std::map<std::pair<uint32_t,uint32_t>, Pending> m_pending;
  /* Packets this node has already forwarded or discarded. */
  std::set<std::pair<uint32_t,uint32_t> > m_handled;

  EventId m_beaconEvent;
  Ptr<UniformRandomVariable> m_rand;
};

} // namespace ns3

#endif /* AQUA_SIM_ROUTING_ADAPTIVE_VBF_H */
