/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Experiment driver for Adaptive-IoUT-VBF and its baselines.
 *
 * Every parameter that varies in the six scenarios of the paper is exposed
 * on the command line, so a scenario is a sweep over one flag and nothing
 * in the source needs editing between runs.
 *
 *   ./ns3 run "adaptive-iout-vbf-experiment --protocol=adaptive --nNodes=50"
 *
 * Protocols:  adaptive | vbf | hhvbf | dbr
 *   'vbf'   -> AquaSimVBF with hop-by-hop disabled  (Classic-VBF)
 *   'hhvbf' -> AquaSimVBF with hop-by-hop enabled   (HHVBF)
 *   'dbr'   -> AquaSimDBR
 *
 * One line of CSV is appended per run. Aggregate over seeds with analyze.py.
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/pointer.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/packet-socket-address.h"
#include "ns3/aqua-sim-ng-module.h"

#include "ns3/aqua-sim-helper.h"
#include "ns3/aqua-sim-routing-adaptive-vbf.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("AdaptiveIoutVbfExperiment");

/* --------------------------------------------------------------------- */

/* ---------------------------------------------------------------------
 * Device-layer instrumentation.
 *
 * These handlers sit on the PHY, below routing, so they fire identically
 * for Adaptive-IoUT-VBF, Classic-VBF, HHVBF and DBR. Putting the counters
 * here rather than inside one protocol is what makes the comparison fair:
 * no protocol can be measured on terms the others are not.
 * ------------------------------------------------------------------- */

static void
PhyTx (uint32_t nodeId, Ptr<Packet> p, double)
{
  AivEnergyLedger &led = AivEnergyLedger::Get ();
  led.ApplyIdleUpTo (Simulator::Now ().GetSeconds ());
  led.ChargeTx (nodeId);
  AivStats::Get ().NoteTransmission ();
}

static void
PhyRx (uint32_t nodeId, Ptr<Packet> p, double)
{
  AivEnergyLedger &led = AivEnergyLedger::Get ();
  led.ApplyIdleUpTo (Simulator::Now ().GetSeconds ());
  led.ChargeRx (nodeId);
}

/* Delivery is counted from the sink's own PHY reception, where every
   header is still intact. AquaSimHeader fields used here (source address,
   sequence number, hop count, timestamp, size) are maintained by all
   AquaSim routing protocols, so this works for the baselines too. */
static void
SinkRx (Ptr<Packet> p, double)
{
  Ptr<Packet> copy = p->Copy ();
  AquaSimHeader ash;
  if (copy->GetSize () < ash.GetSerializedSize ()) return;
  copy->RemoveHeader (ash);

  uint32_t src = static_cast<uint32_t> (ash.GetSAddr ().GetAsInt ());
  uint32_t seq = ash.GetSeqNum ();
  if (ash.GetSize () == 0) return;                 // control traffic

  double delay = Simulator::Now ().GetSeconds ()
                 - ash.GetTimeStamp ().GetSeconds ();
  if (delay < 0.0) return;

  AivStats::Get ().NoteDelivered (src, seq, ash.GetSize () * 8,
                                  delay, ash.GetNumForwards ());
}

/* One traffic generator for every protocol. The offered load is therefore
   identical by construction rather than by coincidence -- if the four
   protocols did not generate the same number of packets, no comparison
   between them would mean anything. */
struct TrafficSource
{
  Ptr<AquaSimNetDevice> dev;
  Ptr<AquaSimAdaptiveVbf> routing;
  uint32_t seq;
  Address sinkAddr;
  uint32_t payloadBytes;
  double interval;
  double stopTime;
  Ptr<UniformRandomVariable> jitter;
};

static void
GenerateTraffic (TrafficSource *ts)
{
  if (Simulator::Now ().GetSeconds () >= ts->stopTime) return;

  Ptr<AquaSimNetDevice> dev = ts->dev;
  uint32_t *seq = &ts->seq;
  uint32_t payloadBytes = ts->payloadBytes;
  Address sinkAddr = ts->sinkAddr;

  AivStats::Get ().NoteGenerated (dev->GetNode ()->GetId (), *seq);

  if (ts->routing != 0)
    {
      ts->routing->OriginatePacket (*seq, payloadBytes);
    }
  else
    {
      Ptr<Packet> pkt = Create<Packet> (payloadBytes);
      AquaSimHeader ash;
      ash.SetSize (payloadBytes);
      ash.SetSeqNum (*seq);
      ash.SetTimeStamp (Simulator::Now ());
      ash.SetSAddr (AquaSimAddress::ConvertFrom (dev->GetAddress ()));
      ash.SetDAddr (AquaSimAddress::ConvertFrom (sinkAddr));
      ash.SetNumForwards (0);
      ash.SetErrorFlag (false);
      pkt->AddHeader (ash);
      dev->Send (pkt, sinkAddr, 0);
    }

  (*seq)++;
  double next = ts->interval
                + ts->jitter->GetValue (-0.1 * ts->interval, 0.1 * ts->interval);
  Simulator::Schedule (Seconds (next), &GenerateTraffic, ts);
}

/* Periodically applies the idle drain so first-death time is detected even
   when a node is otherwise inactive. */
static void
IdleTick (double period, double stopTime)
{
  AivEnergyLedger::Get ().ApplyIdleUpTo (Simulator::Now ().GetSeconds ());
  if (Simulator::Now ().GetSeconds () + period <= stopTime)
    Simulator::Schedule (Seconds (period), &IdleTick, period, stopTime);
}

/* --------------------------------------------------------------------- */

int
main (int argc, char *argv[])
{
  /* ---- defaults: the paper's Table II ------------------------------- */
  std::string protocol   = "adaptive";
  uint32_t nNodes        = 50;
  uint32_t nSources      = 5;
  double   fieldSize     = 500.0;    // horizontal extent (m)
  double   waterDepth    = 500.0;    // vertical extent (m)
  double   maxSpeed      = 3.0;      // m/s
  double   basePipeR     = 150.0;    // m
  double   commRange     = 250.0;    // m
  double   packetInterval= 6.0;      // s
  uint32_t payloadBytes  = 512;
  double   simTime       = 200.0;    // s
  double   bitRate       = 10000.0;  // bps  (AquaSim-NG default)
  double   frequency     = 25.0;     // kHz
  double   initialEnergy = 100.0;    // J
  double   eTx           = 0.50;     // J per transmission
  double   eRx           = 0.10;     // J per reception
  double   eIdle         = 0.001;    // J/s
  uint32_t seed          = 1;
  uint32_t k             = 3;
  double   tMaxHold      = 0.60;
  bool     runToDeath    = false;    // if true, ignore simTime cap
  std::string outFile    = "results.csv";
  std::string label      = "";

  CommandLine cmd;
  cmd.AddValue ("protocol",       "adaptive | vbf | hhvbf | dbr", protocol);
  cmd.AddValue ("nNodes",         "Number of sensor nodes", nNodes);
  cmd.AddValue ("nSources",       "Number of traffic-generating nodes", nSources);
  cmd.AddValue ("fieldSize",      "Horizontal field side length (m)", fieldSize);
  cmd.AddValue ("waterDepth",     "Vertical field extent (m)", waterDepth);
  cmd.AddValue ("maxSpeed",       "Maximum node speed (m/s)", maxSpeed);
  cmd.AddValue ("basePipeR",      "Base pipe radius R_base (m)", basePipeR);
  cmd.AddValue ("commRange",      "Acoustic range R_c (m)", commRange);
  cmd.AddValue ("packetInterval", "Packet generation interval (s)", packetInterval);
  cmd.AddValue ("payloadBytes",   "Data payload size (bytes)", payloadBytes);
  cmd.AddValue ("simTime",        "Simulation duration (s)", simTime);
  cmd.AddValue ("bitRate",        "Acoustic bit rate (bps)", bitRate);
  cmd.AddValue ("frequency",      "Carrier frequency (kHz)", frequency);
  cmd.AddValue ("initialEnergy",  "Initial node energy (J)", initialEnergy);
  cmd.AddValue ("eTx",            "Transmission energy (J)", eTx);
  cmd.AddValue ("eRx",            "Reception energy (J)", eRx);
  cmd.AddValue ("eIdle",          "Idle drain (J/s)", eIdle);
  cmd.AddValue ("seed",           "RNG run number", seed);
  cmd.AddValue ("K",              "Forwarders per hop", k);
  cmd.AddValue ("tMaxHold",       "Maximum holding time (s)", tMaxHold);
  cmd.AddValue ("runToDeath",     "Run until first node death", runToDeath);
  cmd.AddValue ("outFile",        "CSV output path", outFile);
  cmd.AddValue ("label",          "Free-text tag for the sweep", label);
  cmd.Parse (argc, argv);

  if (runToDeath) simTime = 20000.0;   // generous ceiling; loop exits on death

  RngSeedManager::SetSeed (12345);
  RngSeedManager::SetRun (seed);

  /* ---- energy + stats ------------------------------------------------ */
  AivEnergyLedger::Get ().Configure (nNodes, initialEnergy, eTx, eRx, eIdle);
  AivStats::Get ().Reset ();

  /* ---- nodes and mobility -------------------------------------------- */
  NodeContainer nodes;
  nodes.Create (nNodes);

  MobilityHelper mobility;
  {
    std::ostringstream xy, z, spd;
    xy  << "ns3::UniformRandomVariable[Min=0|Max=" << fieldSize << "]";
    z   << "ns3::UniformRandomVariable[Min=0|Max=" << waterDepth << "]";
    spd << "ns3::UniformRandomVariable[Min=0|Max=" << maxSpeed << "]";

    mobility.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
                                   "X", StringValue (xy.str ()),
                                   "Y", StringValue (xy.str ()),
                                   "Z", StringValue (z.str ()));
    /* Three-dimensional Random Waypoint, as specified in the paper.
       RandomWaypointMobilityModel draws each successive waypoint from its
       PositionAllocator, so a RandomBoxPositionAllocator gives genuinely
       3-D motion. (ns-3 has no RandomWalk3dMobilityModel; RandomWalk2d
       would pin the depth coordinate, which is exactly the flaw we are
       trying to avoid.) */
    Ptr<RandomBoxPositionAllocator> waypoints =
      CreateObject<RandomBoxPositionAllocator> ();
    {
      Ptr<UniformRandomVariable> ux = CreateObject<UniformRandomVariable> ();
      ux->SetAttribute ("Min", DoubleValue (0.0));
      ux->SetAttribute ("Max", DoubleValue (fieldSize));
      Ptr<UniformRandomVariable> uy = CreateObject<UniformRandomVariable> ();
      uy->SetAttribute ("Min", DoubleValue (0.0));
      uy->SetAttribute ("Max", DoubleValue (fieldSize));
      Ptr<UniformRandomVariable> uz = CreateObject<UniformRandomVariable> ();
      uz->SetAttribute ("Min", DoubleValue (0.0));
      uz->SetAttribute ("Max", DoubleValue (waterDepth));
      waypoints->SetX (ux);
      waypoints->SetY (uy);
      waypoints->SetZ (uz);
    }
    mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                               "Speed", StringValue (spd.str ()),
                               "Pause",
                               StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"),
                               "PositionAllocator", PointerValue (waypoints));
    mobility.Install (nodes);
  }

  /* Node 0 is the surface sink: fixed, centred, at depth 0. */
  Vector sinkPos (fieldSize / 2.0, fieldSize / 2.0, 0.0);
  nodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (sinkPos);
  MobilityHelper stat;
  stat.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  stat.Install (nodes.Get (0));
  nodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (sinkPos);

  /* ---- channel, PHY, MAC --------------------------------------------- */
  AquaSimChannelHelper channelHelper = AquaSimChannelHelper::Default ();
  channelHelper.SetPropagation ("ns3::AquaSimRangePropagation");

  AquaSimHelper asHelper = AquaSimHelper::Default ();
  asHelper.SetChannel (channelHelper.Create ());
  asHelper.SetMac ("ns3::AquaSimBroadcastMac");

  if (protocol == "adaptive")
    {
      asHelper.SetRouting ("ns3::AquaSimAdaptiveVbf",
                           "BaseRadius", DoubleValue (basePipeR),
                           "CommRange", DoubleValue (commRange),
                           "K", UintegerValue (k),
                           "MaxHoldTime", DoubleValue (tMaxHold));
    }
  else if (protocol == "vbf")
    {
      asHelper.SetRouting ("ns3::AquaSimVBF",
                           "Width", DoubleValue (basePipeR),
                           "HopByHop", IntegerValue (0));
    }
  else if (protocol == "hhvbf")
    {
      asHelper.SetRouting ("ns3::AquaSimVBF",
                           "Width", DoubleValue (basePipeR),
                           "HopByHop", IntegerValue (1));
    }
  else if (protocol == "dbr")
    {
      asHelper.SetRouting ("ns3::AquaSimDBR");
    }
  else
    {
      NS_FATAL_ERROR ("Unknown protocol: " << protocol);
    }

  NetDeviceContainer devices;
  std::vector<Ptr<AquaSimAdaptiveVbf> > aivRouting;

  for (uint32_t i = 0; i < nodes.GetN (); i++)
    {
      Ptr<AquaSimNetDevice> dev = CreateObject<AquaSimNetDevice> ();
      devices.Add (asHelper.Create (nodes.Get (i), dev));

      dev->GetPhy ()->SetTransRange (commRange);
      dev->SetAddress (AquaSimAddress (i + 1));

      if (i == 0) dev->SetSinkStatus ();

      /* Energy accounting for every node, every protocol. */
      dev->GetPhy ()->TraceConnectWithoutContext (
          "Tx", MakeBoundCallback (&PhyTx, i));
      dev->GetPhy ()->TraceConnectWithoutContext (
          "Rx", MakeBoundCallback (&PhyRx, i));
      /* Delivery accounting at the sink only. */
      if (i == 0)
        dev->GetPhy ()->TraceConnectWithoutContext ("Rx", MakeCallback (&SinkRx));

      if (protocol == "adaptive")
        {
          Ptr<AquaSimAdaptiveVbf> r =
            DynamicCast<AquaSimAdaptiveVbf> (dev->GetRouting ());
          NS_ASSERT (r != 0);
          r->SetSinkPosition (sinkPos);
          r->AssignStreams (100 + i);
          if (i == 0)
            {
              r->SetAttribute ("IsSink", BooleanValue (true));
            }
          r->StartBeaconing ();
          aivRouting.push_back (r);
        }
      else
        {
          Ptr<AquaSimVBF> v = DynamicCast<AquaSimVBF> (dev->GetRouting ());
          if (v) v->SetTargetPos (sinkPos);
          aivRouting.push_back (0);
        }
    }

  /* Packet sockets are required for the baseline traffic path; without
     this the socket factory is absent and Socket::CreateSocket returns
     null (segfault on Connect). */
  PacketSocketHelper packetSocket;
  packetSocket.Install (nodes);

  /* ---- traffic -------------------------------------------------------- */
  Ptr<UniformRandomVariable> jitter = CreateObject<UniformRandomVariable> ();
  jitter->SetStream (7);

  uint32_t sources = std::min (nSources, nNodes - 1);
  Address sinkAddr = devices.Get (0)->GetAddress ();
  std::vector<TrafficSource *> trafficSources;

  for (uint32_t s = 1; s <= sources; s++)
    {
      uint32_t idx = 1 + (s - 1) * ((nNodes - 1) / sources);
      if (idx >= nNodes) idx = nNodes - 1;

      TrafficSource *ts = new TrafficSource ();
      ts->dev = DynamicCast<AquaSimNetDevice> (devices.Get (idx));
      ts->routing = aivRouting[idx];
      ts->seq = 0;
      ts->sinkAddr = sinkAddr;
      ts->payloadBytes = payloadBytes;
      ts->interval = packetInterval;
      ts->stopTime = simTime;
      ts->jitter = jitter;
      trafficSources.push_back (ts);

      Simulator::Schedule (Seconds (5.0 + 0.37 * s), &GenerateTraffic, ts);
    }

  Simulator::Schedule (Seconds (1.0), &IdleTick, 1.0, simTime);

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  double endTime = Simulator::Now ().GetSeconds ();
  AivEnergyLedger::Get ().ApplyIdleUpTo (endTime);
  Simulator::Destroy ();

  /* ---- results -------------------------------------------------------- */
  AivEnergyLedger &led = AivEnergyLedger::Get ();
  AivStats &st = AivStats::Get ();

  bool censored = !led.AnyDeath ();
  double lifetime = censored ? -1.0 : led.FirstDeathTime ();

  bool header = false;
  {
    std::ifstream probe (outFile.c_str ());
    header = !probe.good () || probe.peek () == std::ifstream::traits_type::eof ();
  }

  std::ofstream out (outFile.c_str (), std::ios_base::app);
  if (header)
    {
      out << "label,protocol,seed,nNodes,nSources,fieldSize,waterDepth,maxSpeed,"
          << "basePipeR,commRange,packetInterval,payloadBytes,bitRate,simTime,"
          << "generated,delivered,duplicates,dropped,transmissions,fallbacks,"
          << "pdr_pct,throughput_kbps,mean_delay_s,mean_hops,"
          << "lifetime_s,lifetime_censored,jain_fairness,energy_gap_J,"
          << "mean_consumed_J\n";
    }

  out << std::fixed << std::setprecision (6)
      << label << "," << protocol << "," << seed << "," << nNodes << ","
      << sources << "," << fieldSize << "," << waterDepth << "," << maxSpeed << ","
      << basePipeR << "," << commRange << "," << packetInterval << ","
      << payloadBytes << "," << bitRate << "," << endTime << ","
      << st.Generated () << "," << st.Delivered () << "," << st.Duplicates () << ","
      << st.Dropped () << "," << st.Transmissions () << "," << st.Fallbacks () << ","
      << st.Pdr () << "," << st.ThroughputKbps (endTime) << ","
      << st.MeanDelay () << "," << st.MeanHops () << ","
      << lifetime << "," << (censored ? 1 : 0) << ","
      << led.JainFairness () << "," << led.EnergyGap () << ","
      << led.MeanConsumed () << "\n";
  out.close ();

  std::cout << protocol << " seed=" << seed
            << " PDR=" << st.Pdr () << "%"
            << " thr=" << st.ThroughputKbps (endTime) << "kbps"
            << " hops=" << st.MeanHops ()
            << " delay=" << st.MeanDelay () << "s"
            << " lifetime=" << (censored ? std::string ("CENSORED")
                                         : std::to_string (lifetime))
            << " gap=" << led.EnergyGap () << "J"
            << std::endl;

  return 0;
}
