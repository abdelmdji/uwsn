/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Adaptive-IoUT-VBF implementation. See header for the design contract.
 */

#include "aqua-sim-routing-adaptive-vbf.h"
#include "aqua-sim-header.h"
#include "aqua-sim-header-routing.h"
#include "aqua-sim-address.h"
#include "aqua-sim-pt-tag.h"

#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/vector.h"
#include "ns3/simulator.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"

#include <cmath>
#include <algorithm>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("AquaSimAdaptiveVbf");
NS_OBJECT_ENSURE_REGISTERED (AquaSimAdaptiveVbf);

/* ===================================================================== */
/*  Energy ledger                                                        */
/* ===================================================================== */

AivEnergyLedger&
AivEnergyLedger::Get ()
{
  static AivEnergyLedger inst;
  return inst;
}

void
AivEnergyLedger::Configure (uint32_t nNodes, double eInit, double eTx,
                            double eRx, double eIdlePerSec)
{
  m_eInit = eInit;
  m_eTx = eTx;
  m_eRx = eRx;
  m_eIdle = eIdlePerSec;
  m_energy.assign (nNodes, eInit);
  m_lastIdleApplied = 0.0;
  m_firstDeath = -1.0;
}

void
AivEnergyLedger::Reset ()
{
  std::fill (m_energy.begin (), m_energy.end (), m_eInit);
  m_lastIdleApplied = 0.0;
  m_firstDeath = -1.0;
}

double
AivEnergyLedger::Energy (uint32_t id) const
{
  if (id >= m_energy.size ()) return 0.0;
  return m_energy[id];
}

double
AivEnergyLedger::Fraction (uint32_t id) const
{
  if (m_eInit <= 0.0) return 0.0;
  return std::max (0.0, Energy (id) / m_eInit);
}

bool
AivEnergyLedger::Alive (uint32_t id) const
{
  return Energy (id) >= m_eTx;
}

void
AivEnergyLedger::NoteDeath (uint32_t id)
{
  if (m_firstDeath < 0.0 && m_energy[id] <= 0.0)
    {
      m_firstDeath = Simulator::Now ().GetSeconds ();
      NS_LOG_INFO ("First node death: node " << id << " at " << m_firstDeath << "s");
    }
}

bool
AivEnergyLedger::ChargeTx (uint32_t id)
{
  if (id >= m_energy.size ()) return false;
  if (m_energy[id] < m_eTx) return false;
  m_energy[id] -= m_eTx;
  NoteDeath (id);
  return true;
}

void
AivEnergyLedger::ChargeRx (uint32_t id)
{
  if (id >= m_energy.size ()) return;
  m_energy[id] = std::max (0.0, m_energy[id] - m_eRx);
  NoteDeath (id);
}

void
AivEnergyLedger::ChargeBeacon (uint32_t id, double payloadRatio)
{
  if (id >= m_energy.size ()) return;
  m_energy[id] = std::max (0.0, m_energy[id] - m_eTx * payloadRatio);
  NoteDeath (id);
}

void
AivEnergyLedger::ApplyIdleUpTo (double now)
{
  double dt = now - m_lastIdleApplied;
  if (dt <= 0.0) return;
  for (uint32_t i = 0; i < m_energy.size (); i++)
    {
      m_energy[i] = std::max (0.0, m_energy[i] - m_eIdle * dt);
      NoteDeath (i);
    }
  m_lastIdleApplied = now;
}

std::vector<double>
AivEnergyLedger::Consumed () const
{
  std::vector<double> c;
  c.reserve (m_energy.size ());
  for (double e : m_energy) c.push_back (m_eInit - e);
  return c;
}

double
AivEnergyLedger::MeanConsumed () const
{
  std::vector<double> c = Consumed ();
  if (c.empty ()) return 0.0;
  double s = 0.0;
  for (double v : c) s += v;
  return s / c.size ();
}

double
AivEnergyLedger::JainFairness () const
{
  std::vector<double> c = Consumed ();
  if (c.empty ()) return 1.0;
  double sum = 0.0, sumSq = 0.0;
  for (double v : c) { sum += v; sumSq += v * v; }
  if (sumSq <= 0.0) return 1.0;
  double f = (sum * sum) / (c.size () * sumSq);
  return std::min (1.0, std::max (0.01, f));
}

double
AivEnergyLedger::EnergyGap () const
{
  double f = JainFairness ();
  double eAvg = MeanConsumed ();
  return (eAvg / f) - (eAvg * f);
}

/* ===================================================================== */
/*  Statistics                                                           */
/* ===================================================================== */

AivStats&
AivStats::Get ()
{
  static AivStats inst;
  return inst;
}

void
AivStats::Reset ()
{
  m_seen.clear ();
  m_generated = m_delivered = m_duplicates = 0;
  m_dropped = m_fallbackUsed = m_transmissions = 0;
  m_bits = 0;
  m_delaySum = 0.0;
  m_hopSum = 0;
}

void
AivStats::NoteGenerated (uint32_t, uint32_t)
{
  m_generated++;
}

bool
AivStats::NoteDelivered (uint32_t src, uint32_t seq, uint32_t bits,
                         double delaySec, uint32_t hops)
{
  std::pair<uint32_t,uint32_t> key (src, seq);
  if (m_seen.count (key))
    {
      m_duplicates++;
      return false;
    }
  m_seen.insert (key);
  m_delivered++;
  m_bits += bits;
  m_delaySum += delaySec;
  m_hopSum += hops;
  return true;
}

double AivStats::MeanDelay () const
{ return m_delivered ? m_delaySum / m_delivered : 0.0; }

double AivStats::MeanHops () const
{ return m_delivered ? static_cast<double> (m_hopSum) / m_delivered : 0.0; }

double AivStats::Pdr () const
{ return m_generated ? 100.0 * m_delivered / m_generated : 0.0; }

double
AivStats::ThroughputKbps (double simTime) const
{
  if (simTime <= 0.0) return 0.0;
  return (static_cast<double> (m_bits) / simTime) / 1000.0;
}

/* ===================================================================== */
/*  Routing protocol                                                     */
/* ===================================================================== */

AquaSimAdaptiveVbf::AquaSimAdaptiveVbf ()
  : m_rBase (150.0),
    m_rComm (250.0),
    m_tMaxHold (0.60),
    m_k (3),
    m_guardNormal (5.0),
    m_guardFallback (0.0),
    m_deliverRadius (50.0),
    m_maxHops (30),
    m_epsMin (0.20),
    m_soundSpeed (1500.0),
    m_beaconInterval (10.0),
    m_beaconBytes (32),
    m_neighbourTimeout (30.0),
    m_isSink (false),
    m_enableAdaptation (true),
    m_sinkPos (0, 0, 0)
{
  m_rand = CreateObject<UniformRandomVariable> ();
}

AquaSimAdaptiveVbf::~AquaSimAdaptiveVbf () {}

TypeId
AquaSimAdaptiveVbf::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::AquaSimAdaptiveVbf")
    .SetParent<AquaSimRouting> ()
    .AddConstructor<AquaSimAdaptiveVbf> ()
    .AddAttribute ("BaseRadius", "Nominal pipe radius R_base (m).",
                   DoubleValue (150.0),
                   MakeDoubleAccessor (&AquaSimAdaptiveVbf::m_rBase),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("CommRange", "Acoustic communication range R_c (m).",
                   DoubleValue (250.0),
                   MakeDoubleAccessor (&AquaSimAdaptiveVbf::m_rComm),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("MaxHoldTime", "T_max (s).",
                   DoubleValue (0.60),
                   MakeDoubleAccessor (&AquaSimAdaptiveVbf::m_tMaxHold),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("K", "Forwarders per hop / duplicate suppression threshold.",
                   UintegerValue (3),
                   MakeUintegerAccessor (&AquaSimAdaptiveVbf::m_k),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("ProgressGuard", "gamma_normal (m).",
                   DoubleValue (5.0),
                   MakeDoubleAccessor (&AquaSimAdaptiveVbf::m_guardNormal),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("DeliverRadius", "Delivery zone radius (m).",
                   DoubleValue (50.0),
                   MakeDoubleAccessor (&AquaSimAdaptiveVbf::m_deliverRadius),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("MaxHops", "Hop limit.",
                   UintegerValue (30),
                   MakeUintegerAccessor (&AquaSimAdaptiveVbf::m_maxHops),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("BeaconInterval", "T_b (s).",
                   DoubleValue (10.0),
                   MakeDoubleAccessor (&AquaSimAdaptiveVbf::m_beaconInterval),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("IsSink", "True on the surface sink node.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&AquaSimAdaptiveVbf::m_isSink),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableAdaptation",
                   "If false the radius is fixed at R_base (ablation baseline).",
                   BooleanValue (true),
                   MakeBooleanAccessor (&AquaSimAdaptiveVbf::m_enableAdaptation),
                   MakeBooleanChecker ())
  ;
  return tid;
}

int64_t
AquaSimAdaptiveVbf::AssignStreams (int64_t stream)
{
  m_rand->SetStream (stream);
  return 1;
}

void
AquaSimAdaptiveVbf::DoDispose ()
{
  m_beaconEvent.Cancel ();
  for (auto &kv : m_pending) kv.second.ev.Cancel ();
  m_pending.clear ();
  m_rand = 0;
  AquaSimRouting::DoDispose ();
}

/* --------------------------------------------------------------------- */

Vector
AquaSimAdaptiveVbf::SelfPosition ()
{
  Ptr<MobilityModel> m = GetNetDevice ()->GetNode ()->GetObject<MobilityModel> ();
  return m ? m->GetPosition () : Vector (0, 0, 0);
}

uint32_t
AquaSimAdaptiveVbf::SelfId ()
{
  return GetNetDevice ()->GetNode ()->GetId ();
}

/*
 * Orthogonal distance from 'cand' to the line through 'from' in the
 * direction of 'sink'  --  Eq. (10):  ||v x u|| / ||u||
 */
double
AquaSimAdaptiveVbf::PerpDistance (const Vector &cand, const Vector &from,
                                  const Vector &sink)
{
  Vector u (sink.x - from.x, sink.y - from.y, sink.z - from.z);
  Vector v (cand.x - from.x, cand.y - from.y, cand.z - from.z);
  double un = std::sqrt (u.x*u.x + u.y*u.y + u.z*u.z);
  if (un < 1e-9) return 0.0;
  Vector c (v.y*u.z - v.z*u.y,
            v.z*u.x - v.x*u.z,
            v.x*u.y - v.y*u.x);
  double cn = std::sqrt (c.x*c.x + c.y*c.y + c.z*c.z);
  return cn / un;
}

/*
 * Geographic progress, Eq. (18): how much closer to the sink 'cand' is than
 * the node that transmitted the packet.
 */
double
AquaSimAdaptiveVbf::Progress (const Vector &cand, const Vector &self) const
{
  double dSelf = CalculateDistance (self, m_sinkPos);
  double dCand = CalculateDistance (cand, m_sinkPos);
  return dSelf - dCand;
}

/* delta(rho) -- Eq. (12) */
double
AquaSimAdaptiveVbf::DensityFactor (uint32_t rho) const
{
  if (rho < 3)  return 2.00;
  if (rho <= 8) return 0.88;
  if (rho <= 15) return 0.70;
  if (rho <= 25) return 0.60;
  return 0.55;
}

/* R_adaptive -- Eq. (11), bounded by Eq. (13) */
double
AquaSimAdaptiveVbf::AdaptiveRadius (double energyFraction, uint32_t rho) const
{
  if (!m_enableAdaptation) return m_rBase;
  double eps = std::max (energyFraction, m_epsMin);
  double r = m_rBase * (0.50 + 0.50 * eps) * DensityFactor (rho);
  double rMin = 0.10 * m_rBase;
  return std::min (m_rComm, std::max (rMin, r));
}

/* --------------------------------------------------------------------- */
/*  Neighbour table                                                      */
/* --------------------------------------------------------------------- */

void
AquaSimAdaptiveVbf::StartBeaconing ()
{
  double jitter = m_rand->GetValue (0.0, m_beaconInterval);
  m_beaconEvent = Simulator::Schedule (Seconds (jitter),
                                       &AquaSimAdaptiveVbf::SendBeacon, this);
}

void
AquaSimAdaptiveVbf::SendBeacon ()
{
  AivEnergyLedger &led = AivEnergyLedger::Get ();
  uint32_t id = SelfId ();

  if (led.Alive (id))
    {
      /* Beacons are charged in proportion to their size relative to a
         512-byte data payload, as argued in Section V-B of the paper. */
      led.ChargeBeacon (id, static_cast<double> (m_beaconBytes) / 512.0);

      Ptr<Packet> p = Create<Packet> (m_beaconBytes);
      AquaSimHeader ash;
      VBHeader vbh;
      AquaSimPtTag ptag;

      Vector pos = SelfPosition ();
      vbh.SetMessType (AIV_BEACON);
      vbh.SetSenderAddr (AquaSimAddress::ConvertFrom (GetNetDevice ()->GetAddress ()));
      vbh.SetExtraInfo_f (pos);
      /* Residual energy fraction encoded in per-mille in the Range field. */
      vbh.SetRange (static_cast<uint32_t> (led.Fraction (id) * 1000.0));
      vbh.SetPkNum (0);

      ash.SetSAddr (AquaSimAddress::ConvertFrom (GetNetDevice ()->GetAddress ()));
      ash.SetDAddr (AquaSimAddress::GetBroadcast ());
      ash.SetNextHop (AquaSimAddress::GetBroadcast ());
      ash.SetDirection (AquaSimHeader::DOWN);
      ash.SetNumForwards (0);
      ash.SetErrorFlag (false);
      ash.SetSize (m_beaconBytes);
      ash.SetTimeStamp (Simulator::Now ());
      ptag.SetPacketType (AquaSimPtTag::PT_UWVB);

      p->AddPacketTag (ptag);
      p->AddHeader (vbh);
      p->AddHeader (ash);

      SendDown (p, AquaSimAddress::GetBroadcast (), Seconds (0.0));
    }

  m_beaconEvent = Simulator::Schedule (Seconds (m_beaconInterval),
                                       &AquaSimAdaptiveVbf::SendBeacon, this);
}

void
AquaSimAdaptiveVbf::HandleBeacon (Ptr<Packet> p)
{
  AquaSimHeader ash;
  VBHeader vbh;
  p->RemoveHeader (ash);
  p->PeekHeader (vbh);

  AivNeighbour n;
  n.addr = vbh.GetSenderAddr ();
  n.pos = vbh.GetExtraInfo ().f;
  n.energyFraction = vbh.GetRange () / 1000.0;
  n.lastHeard = Simulator::Now ().GetSeconds ();

  uint32_t key = static_cast<uint32_t> (n.addr.GetAsInt ());
  m_neighbours[key] = n;
}

void
AquaSimAdaptiveVbf::PurgeNeighbours ()
{
  double now = Simulator::Now ().GetSeconds ();
  Vector self = SelfPosition ();
  for (auto it = m_neighbours.begin (); it != m_neighbours.end (); )
    {
      bool stale = (now - it->second.lastHeard) > m_neighbourTimeout;
      bool far = CalculateDistance (self, it->second.pos) > m_rComm;
      if (stale || far) it = m_neighbours.erase (it);
      else ++it;
    }
}

uint32_t
AquaSimAdaptiveVbf::NeighbourCount ()
{
  PurgeNeighbours ();
  return static_cast<uint32_t> (m_neighbours.size ());
}

/* --------------------------------------------------------------------- */
/*  Candidate check used by the forwarder for fallback decisions          */
/* --------------------------------------------------------------------- */

bool
AquaSimAdaptiveVbf::AnyCandidate (double radius, double guard)
{
  Vector self = SelfPosition ();
  for (const auto &kv : m_neighbours)
    {
      const AivNeighbour &n = kv.second;
      if (CalculateDistance (self, n.pos) > m_rComm) continue;
      if (PerpDistance (n.pos, self, m_sinkPos) > radius) continue;
      if (Progress (n.pos, self) <= guard) continue;
      return true;
    }
  return false;
}

/* --------------------------------------------------------------------- */
/*  Origination                                                          */
/* --------------------------------------------------------------------- */

void
AquaSimAdaptiveVbf::OriginatePacket (uint32_t seq, uint32_t payloadBytes)
{
  AivEnergyLedger &led = AivEnergyLedger::Get ();
  led.ApplyIdleUpTo (Simulator::Now ().GetSeconds ());

  uint32_t id = SelfId ();
  if (!led.Alive (id)) return;

  AivStats::Get ().NoteGenerated (id, seq);

  Vector self = SelfPosition ();
  uint32_t rho = NeighbourCount ();
  double radius = AdaptiveRadius (led.Fraction (id), rho);
  double guard = m_guardNormal;

  /* Three-stage fallback void recovery, Eq. (18). */
  if (!AnyCandidate (radius, guard))
    {
      const double stages[3] = { 1.5 * radius, 2.0 * m_rBase, 3.0 * m_rBase };
      bool found = false;
      for (int s = 0; s < 3 && !found; s++)
        {
          double r = std::min (stages[s], m_rComm);
          if (AnyCandidate (r, m_guardFallback))
            {
              radius = r;
              guard = m_guardFallback;
              found = true;
              AivStats::Get ().NoteVoidFallback ();
            }
        }
      if (!found)
        {
          AivStats::Get ().NoteDropped ();
          return;
        }
    }

  if (!led.ChargeTx (id)) return;
  AivStats::Get ().NoteTransmission ();

  Ptr<Packet> p = Create<Packet> (payloadBytes);
  AquaSimHeader ash;
  VBHeader vbh;
  AquaSimPtTag ptag;

  vbh.SetMessType (AIV_DATA);
  vbh.SetPkNum (seq);
  vbh.SetSenderAddr (AquaSimAddress::ConvertFrom (GetNetDevice ()->GetAddress ()));
  vbh.SetForwardAddr (AquaSimAddress::ConvertFrom (GetNetDevice ()->GetAddress ()));
  vbh.SetOriginalSource (self);
  vbh.SetExtraInfo_o (self);   // original source
  vbh.SetExtraInfo_f (self);   // this forwarder
  vbh.SetExtraInfo_t (m_sinkPos);
  /* R_adaptive and the active progress guard travel in the header. */
  vbh.SetRange (static_cast<uint32_t> (radius * 100.0));
  vbh.SetToken (static_cast<uint32_t> (guard * 100.0));

  ash.SetSAddr (AquaSimAddress::ConvertFrom (GetNetDevice ()->GetAddress ()));
  ash.SetDAddr (AquaSimAddress::GetBroadcast ());
  ash.SetNextHop (AquaSimAddress::GetBroadcast ());
  ash.SetDirection (AquaSimHeader::DOWN);
  ash.SetNumForwards (0);
  ash.SetErrorFlag (false);
  ash.SetSize (payloadBytes);
  ash.SetTimeStamp (Simulator::Now ());
  ash.SetSeqNum (seq);
  ptag.SetPacketType (AquaSimPtTag::PT_UWVB);

  p->AddPacketTag (ptag);
  p->AddHeader (vbh);
  p->AddHeader (ash);

  m_handled.insert (std::make_pair (id, seq));
  SendDown (p, AquaSimAddress::GetBroadcast (), Seconds (0.0));
}

/* --------------------------------------------------------------------- */
/*  Reception  --  Algorithm 1, Part B                                    */
/* --------------------------------------------------------------------- */

bool
AquaSimAdaptiveVbf::Recv (Ptr<Packet> packet, const Address &dest,
                          uint16_t protocolNumber)
{
  if (packet == 0) return false;

  AivEnergyLedger &led = AivEnergyLedger::Get ();
  led.ApplyIdleUpTo (Simulator::Now ().GetSeconds ());

  VBHeader peek;
  {
    Ptr<Packet> copy = packet->Copy ();
    AquaSimHeader tmp;
    copy->RemoveHeader (tmp);
    copy->PeekHeader (peek);
  }

  if (peek.GetMessType () == AIV_BEACON)
    {
      /* Beacon reception is charged at the same size ratio as its
         transmission, keeping the accounting symmetric. */
      led.ChargeRx (SelfId ());
      HandleBeacon (packet);
      return true;
    }

  if (peek.GetMessType () != AIV_DATA)
    {
      return false;   // not ours
    }

  led.ChargeRx (SelfId ());
  HandleData (packet);
  return true;
}

void
AquaSimAdaptiveVbf::HandleData (Ptr<Packet> p)
{
  AquaSimHeader ash;
  VBHeader vbh;
  p->RemoveHeader (ash);
  p->RemoveHeader (vbh);

  uint32_t srcId = static_cast<uint32_t> (vbh.GetSenderAddr ().GetAsInt ());
  uint32_t seq = vbh.GetPkNum ();
  std::pair<uint32_t,uint32_t> key (srcId, seq);

  Vector self = SelfPosition ();
  uint32_t myId = SelfId ();
  AivEnergyLedger &led = AivEnergyLedger::Get ();

  /* ---- sink delivery ------------------------------------------------- */
  if (m_isSink || CalculateDistance (self, m_sinkPos) < m_deliverRadius)
    {
      double delay = Simulator::Now ().GetSeconds ()
                     - ash.GetTimeStamp ().GetSeconds ();
      AivStats::Get ().NoteDelivered (srcId, seq, ash.GetSize () * 8,
                                      delay, ash.GetNumForwards ());
      return;
    }

  /* ---- already dealt with: count it toward suppression ---------------- */
  auto pend = m_pending.find (key);
  if (pend != m_pending.end ())
    {
      pend->second.heard++;
      if (pend->second.heard >= m_k)
        {
          pend->second.ev.Cancel ();
          m_pending.erase (pend);
          m_handled.insert (key);   // suppressed
        }
      return;
    }
  if (m_handled.count (key)) return;

  /* ---- guards --------------------------------------------------------- */
  if (!led.Alive (myId)) return;
  if (ash.GetNumForwards () >= m_maxHops)
    {
      AivStats::Get ().NoteDropped ();
      m_handled.insert (key);
      return;
    }

  /* ---- eligibility against the corridor carried in the header --------- */
  Vector prevPos = vbh.GetExtraInfo ().f;
  double radius = vbh.GetRange () / 100.0;
  double guard = vbh.GetToken () / 100.0;

  double dPerp = PerpDistance (self, prevPos, m_sinkPos);
  if (dPerp > radius) return;

  double prog = Progress (self, prevPos);
  if (prog <= guard) return;

  /* ---- holding time, Eq. (15) ----------------------------------------- */
  double tHold = (radius > 0.0)
                 ? (dPerp / radius) * m_tMaxHold
                 : m_tMaxHold;
  tHold = std::max (0.0, std::min (m_tMaxHold, tHold));

  Ptr<Packet> copy = p->Copy ();
  copy->AddHeader (vbh);
  copy->AddHeader (ash);

  Pending entry;
  entry.heard = 0;
  entry.ev = Simulator::Schedule (Seconds (tHold),
                                  &AquaSimAdaptiveVbf::ForwardAfterHold,
                                  this, copy, srcId, seq);
  m_pending[key] = entry;
}

void
AquaSimAdaptiveVbf::ForwardAfterHold (Ptr<Packet> p, uint32_t srcId, uint32_t seq)
{
  std::pair<uint32_t,uint32_t> key (srcId, seq);
  m_pending.erase (key);
  m_handled.insert (key);

  AivEnergyLedger &led = AivEnergyLedger::Get ();
  led.ApplyIdleUpTo (Simulator::Now ().GetSeconds ());

  uint32_t myId = SelfId ();
  if (!led.Alive (myId)) return;

  AquaSimHeader ash;
  VBHeader vbh;
  p->RemoveHeader (ash);
  p->RemoveHeader (vbh);

  Vector self = SelfPosition ();
  uint32_t rho = NeighbourCount ();
  double radius = AdaptiveRadius (led.Fraction (myId), rho);
  double guard = m_guardNormal;

  /* Three-stage fallback void recovery, Eq. (18). */
  if (!AnyCandidate (radius, guard))
    {
      const double stages[3] = { 1.5 * radius, 2.0 * m_rBase, 3.0 * m_rBase };
      bool found = false;
      for (int s = 0; s < 3 && !found; s++)
        {
          double r = std::min (stages[s], m_rComm);
          if (AnyCandidate (r, m_guardFallback))
            {
              radius = r; guard = m_guardFallback; found = true;
              AivStats::Get ().NoteVoidFallback ();
            }
        }
      if (!found)
        {
          AivStats::Get ().NoteDropped ();
          return;
        }
    }

  if (!led.ChargeTx (myId)) return;
  AivStats::Get ().NoteTransmission ();

  vbh.SetForwardAddr (AquaSimAddress::ConvertFrom (GetNetDevice ()->GetAddress ()));
  vbh.SetExtraInfo_f (self);
  vbh.SetRange (static_cast<uint32_t> (radius * 100.0));
  vbh.SetToken (static_cast<uint32_t> (guard * 100.0));

  ash.SetNumForwards (ash.GetNumForwards () + 1);
  ash.SetNextHop (AquaSimAddress::GetBroadcast ());
  ash.SetDAddr (AquaSimAddress::GetBroadcast ());
  ash.SetDirection (AquaSimHeader::DOWN);
  ash.SetErrorFlag (false);

  p->AddHeader (vbh);
  p->AddHeader (ash);

  SendDown (p, AquaSimAddress::GetBroadcast (), Seconds (0.0));
}

} // namespace ns3
