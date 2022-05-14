/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 AGH University of Science and Technology
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Author: Lukasz Prasnal <prasnal@kt.agh.edu.pl>
 */

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/propagation-module.h"
#include "ns3/mobility-module.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/config.h"

//for building positioning modelling
#include <ns3/buildings-module.h>
#include <ns3/building.h>
#include <ns3/buildings-helper.h>
#include <ns3/buildings-propagation-loss-model.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/hybrid-buildings-propagation-loss-model.h>
#include <ns3/mobility-building-info.h>

#include <string>

using namespace ns3; 

NS_LOG_COMPONENT_DEFINE ("wifi-qos-test");

class SimulationHelper 
{
public:
	SimulationHelper ();
	static void PopulateArpCache ();
};

SimulationHelper::SimulationHelper () 
{
}

//fullfil the ARP cache prior to simulation run
void
SimulationHelper::PopulateArpCache () 
{
  Ptr<ArpCache> arp = CreateObject<ArpCache> ();
  arp->SetAliveTimeout (Seconds (3600 * 24 * 365) );
	
  for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i) 
    {	
      Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
      NS_ASSERT (ip != 0);
      ObjectVectorValue interfaces;
      ip->GetAttribute ("InterfaceList", interfaces);

      for (ObjectVectorValue::Iterator j = interfaces.Begin (); j != interfaces.End (); j++) 
        {		
          Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface> ();
          NS_ASSERT (ipIface != 0);
          Ptr<NetDevice> device = ipIface->GetDevice ();
          NS_ASSERT (device != 0);
          Mac48Address addr = Mac48Address::ConvertFrom (device->GetAddress () );
      
          for (uint32_t k = 0; k < ipIface->GetNAddresses (); k++) 
            {			
              Ipv4Address ipAddr = ipIface->GetAddress (k).GetLocal();		
              if (ipAddr == Ipv4Address::GetLoopback ()) 
                continue;

              ArpCache::Entry *entry = arp->Add (ipAddr);
              Ipv4Header ipv4Hdr;
              ipv4Hdr.SetDestination (ipAddr);
              Ptr<Packet> p = Create<Packet> (100);  
              entry->MarkWaitReply (ArpCache::Ipv4PayloadHeaderPair (p, ipv4Hdr) );
              entry->MarkAlive (addr);
            }
        }
    }

    for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i) 
      {
        Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
		NS_ASSERT (ip != 0);
		ObjectVectorValue interfaces;
		ip->GetAttribute ("InterfaceList", interfaces);

        for (ObjectVectorValue::Iterator j = interfaces.Begin (); j != interfaces.End (); j ++)
          {
            Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface> ();
            ipIface->SetAttribute ("ArpCache", PointerValue (arp) );
          }
      }
}



/* ===== main function ===== */

int main (int argc, char *argv[])
{
  uint32_t nSTA = 11;
  uint32_t packetSize = 1472;
  Time appsStart = Seconds (0);
  float simTime = 10; // set as calcStart + time 
  float calcStart = 0;
  double Mbps = 54;
  uint32_t seed = 1;
  uint16_t roomsInAxis = 2;
  uint16_t  floors = 3;
  double wallsLoss = 5.0;
  bool rtsCts = false;



/* ===== Command Line parameters ===== */

  CommandLine cmd;
  cmd.AddValue ("nSTA",        "Number of stations",                           nSTA);
  cmd.AddValue ("pSize",       "Packet size [B]",                              packetSize);
  cmd.AddValue ("end",         "simulation time [s]",                          simTime);
  cmd.AddValue ("calcStart",   "start of results analysis [s]",                calcStart);
  cmd.AddValue ("Mbps",        "traffic generated per queue [Mbps]",           Mbps);
  cmd.AddValue ("seed",        "Seed",                                         seed);
  cmd.AddValue ("roomsInAxis", "Number of room in x and y axis in building",   roomsInAxis);
  cmd.AddValue ("floors",      "Number of floors in building",                 floors);
  cmd.AddValue ("wallsLoss",   "Internal walls loss in db",                    wallsLoss);
  cmd.AddValue ("RTSCTS",      "use RTS/CTS?",                                 rtsCts);


  cmd.Parse (argc, argv);

  Time simulationTime = Seconds (simTime);
  //ns3::RngSeedManager::SetSeed (seed);
  ns3::RngSeedManager::SetRun (seed);
 
  Packet::EnablePrinting ();

  // enable or not rts/cts
  if (rtsCts) {
	  Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("0"));
  }

  // AP
  NodeContainer ap;
  ap.Create(1);

  //Stations
  NodeContainer sta;
  sta.Create (nSTA);



/* ======== Positioning / Mobility ======= */
  
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

  //AP
  positionAlloc->Add (Vector (2.0, 12.0, 0.0));

  //Stations
  //Ground floor
  positionAlloc->Add (Vector (2.0, 18.0, 0.0));
  positionAlloc->Add (Vector (2.0, 2.0, 0.0));
  positionAlloc->Add (Vector (8.0, 2.0, 0.0));

  //First floor
  positionAlloc->Add (Vector (2.0, 18.0, 2.0));    
  positionAlloc->Add (Vector (8.0, 18.0, 2.0));
  positionAlloc->Add (Vector (2.0, 2.0, 2.0));
  positionAlloc->Add (Vector (6.0, 8.0, 2.0));
  positionAlloc->Add (Vector (8.0, 5.0, 2.0));
  positionAlloc->Add (Vector (6.0, 2.0, 2.0));

  //Second floor
  positionAlloc->Add (Vector (8.0, 18.0, 4.0));
  positionAlloc->Add (Vector (2.0, 2.0, 4.0));


  MobilityHelper mobility;
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (ap);
  mobility.Install (sta);



/* ===== Propagation Model configuration ===== */

  //building definition:
  Ptr<Building> b = CreateObject <Building> ();
    b->SetBoundaries (Box (0.0, 20.0, 0.0, 30.0, 0.0, 6.0));
    b->SetBuildingType (Building::Office);
    b->SetExtWallsType (Building::ConcreteWithWindows);
    b->SetNFloors (floors);
    b->SetNRoomsX (roomsInAxis);
    b->SetNRoomsY (roomsInAxis);

  BuildingsHelper::Install (sta);
  BuildingsHelper::Install (ap);
  
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");


/* ===== MAC and PHY configuration ===== */

  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel.Create ());

  // set building propagation loss model
  // see https://www.nsnam.org/doxygen/classns3_1_1_hybrid_buildings_propagation_loss_model.html#details
  Ptr<HybridBuildingsPropagationLossModel> buildingLossModel = CreateObject<HybridBuildingsPropagationLossModel>();
  Config::Set ("/ChannelList/*/$ns3::YansWifiChannel/PropagationLossModel", PointerValue (buildingLossModel));
  Config::Set ("/ChannelList/*/$ns3::YansWifiChannel/PropagationLossModel/$ns3::HybridBuildingsPropagationLossModel/Frequency", DoubleValue (5 * 1e9));
  Config::Set ("/ChannelList/*/$ns3::YansWifiChannel/PropagationLossModel/$ns3::HybridBuildingsPropagationLossModel/Environment", StringValue("Urban"));
  Config::Set ("/ChannelList/*/$ns3::YansWifiChannel/PropagationLossModel/$ns3::HybridBuildingsPropagationLossModel/CitySize", StringValue("Small"));
  Config::Set ("/ChannelList/*/$ns3::YansWifiChannel/PropagationLossModel/$ns3::HybridBuildingsPropagationLossModel/ShadowSigmaOutdoor", DoubleValue (7.0));
  Config::Set ("/ChannelList/*/$ns3::YansWifiChannel/PropagationLossModel/$ns3::HybridBuildingsPropagationLossModel/ShadowSigmaIndoor", DoubleValue (8.0));
  Config::Set ("/ChannelList/*/$ns3::YansWifiChannel/PropagationLossModel/$ns3::HybridBuildingsPropagationLossModel/ShadowSigmaExtWalls", DoubleValue (1.0));
  Config::Set ("/ChannelList/*/$ns3::YansWifiChannel/PropagationLossModel/$ns3::HybridBuildingsPropagationLossModel/InternalWallLoss",  DoubleValue (wallsLoss));
  

  WifiHelper wifi;
  WifiMacHelper mac; //802.11ac
  wifi.SetStandard (WIFI_PHY_STANDARD_80211ac);

  //WiFi Remote Station Manager
  //MINSTREL rate manager for 802.11n/ac - see Attributes on https://www.nsnam.org/doxygen/classns3_1_1_minstrel_ht_wifi_manager.html#pri-attribs
  wifi.SetRemoteStationManager ("ns3::MinstrelHtWifiManager");

  // MAC parameters
  // see Attributes on https://www.nsnam.org/doxygen/classns3_1_1_adhoc_wifi_mac.html#pri-methods				    "DataMode", StringValue ("OfdmRate54Mbps") ); 
  Ssid ssid = Ssid ("TEST");

  mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid));
  NetDeviceContainer staDevices = wifi.Install (phy, mac, sta);

  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid));
  NetDeviceContainer apDevice = wifi.Install (phy, mac, ap);




/* ===== Internet stack ===== */

  InternetStackHelper stack;
  stack.Install (sta);
  stack.Install (ap);

  Ipv4AddressHelper address;

  address.SetBase ("192.168.1.0", "255.255.255.0");

  Ipv4InterfaceContainer apIf;
  apIf = address.Assign (apDevice);

  Ipv4InterfaceContainer staIf;
  staIf = address.Assign (staDevices);




/* ===== Setting applications ===== */

  //Configure traffic destination (sink)
  //uint32_t destinationSTANumber = nSTA; //for one common traffic destination
  Ipv4Address destination = apIf.GetAddress (0);
  Ptr<Node> dest = ap.Get (0);

  PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (destination, 1000) );
  sink.Install (dest);

  // Print ap position
  std::cout << "----------------------AP position--------------------" << std::endl;
  std::cout << "AP position: (" << dest->GetObject<MobilityModel>()->GetPosition().x << ", ";
  std::cout << dest->GetObject<MobilityModel>()->GetPosition().y << ", ";
  std::cout << dest->GetObject<MobilityModel>()->GetPosition().z << ")" << std::endl;


  //Configure CBR traffic sources
  DataRate dataRate = DataRate (1000000 * Mbps);

  std::cout << "----------------------Stations position--------------------" << std::endl;

  for (uint32_t i = 0; i < nSTA; i++) 
    {
      Ptr<Node> node = sta.Get(i);

      OnOffHelper cbr  ("ns3::UdpSocketFactory", InetSocketAddress (destination, 1000) );
      cbr.SetConstantRate (dataRate, packetSize);
      cbr.SetAttribute ("StartTime",  TimeValue (appsStart) );
      cbr.SetAttribute ("StopTime",   TimeValue (simulationTime) );

      cbr.Install (node);

      // Print station position
      std::cout << "Station " << std::to_string(i) << " position: (" << node->GetObject<MobilityModel>()->GetPosition().x <<", ";
      std::cout << node->GetObject<MobilityModel>()->GetPosition().y <<", ";
      std::cout << node->GetObject<MobilityModel>()->GetPosition().z << ")" << std::endl;

    }


/* ===== tracing configuration ====== */

  //phy.EnablePcap ("out", nSTA-1, 0); // sniffing to PCAP file

  //AsciiTraceHelper ascii;
  //phy.EnableAsciiAll (ascii.CreateFileStream ("out.tr"));
  //phy.EnableAscii (ascii.CreateFileStream ("out.tr"), sta.Get (0)->GetDevice (0));

  FlowMonitorHelper flowmon_helper;
  Ptr<FlowMonitor> monitor = flowmon_helper.InstallAll ();
  monitor->SetAttribute ("StartTime", TimeValue (Seconds (calcStart) ) ); //Time from which flowmonitor statistics are gathered.
  monitor->SetAttribute ("DelayBinWidth", DoubleValue (0.001));
  monitor->SetAttribute ("JitterBinWidth", DoubleValue (0.001));
  monitor->SetAttribute ("PacketSizeBinWidth", DoubleValue (20));



/* ===== running simulation ========= */

  SimulationHelper::PopulateArpCache ();
  Simulator::Stop (simulationTime);
  Simulator::Run ();
  Simulator::Destroy ();



/* ===== printing results ===== */

  monitor->CheckForLostPackets ();

  //monitor->SerializeToXmlFile ("out.xml", true, true); // sniffing to XML file
  
  std::string proto;
  //initialize variables for overall results calculation
  uint64_t txBytes = 0, rxBytes = 0, txPackets = 0, rxPackets = 0, lostPackets = 0;
  double throughput;
  Time delaySum = Seconds (0), jitterSum = Seconds (0);

  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon_helper.GetClassifier ());
  //iterate over traffic flows
  std::map< FlowId, FlowMonitor::FlowStats > stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::iterator flow = stats.begin (); flow != stats.end (); flow++)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (flow->first);

      //print results for given traffic flow
      switch (t.protocol)
        {
          case (6):
            proto = "TCP";
            break;
          case (17):
            proto = "UDP";
            break;
          default:
            exit (1);
        }
      std::cout << "FlowID: " << flow->first << "(" << proto << " "
                << t.sourceAddress << "/" << t.sourcePort << " --> "
                << t.destinationAddress << "/" << t.destinationPort << ")" <<
      std::endl;

      std::cout << "  Tx bytes:\t"     << flow->second.txBytes << std::endl;
      std::cout << "  Rx bytes:\t"     << flow->second.rxBytes << std::endl;
      std::cout << "  Tx packets:\t"   << flow->second.txPackets << std::endl;
      std::cout << "  Rx packets:\t"   << flow->second.rxPackets << std::endl;
      std::cout << "  Lost packets:\t" << flow->second.lostPackets << std::endl;
      if (flow->second.rxPackets > 0)
        {
          //std::cout << "  Throughput:\t"   << flow->second.rxBytes * 8.0 / (flow->second.timeLastRxPacket.GetSeconds ()-flow->second.timeFirstTxPacket.GetSeconds ()) / 1000000  << " Mb/s" << std::endl;
          std::cout << "  Throughput:\t"   << flow->second.rxBytes * 8.0 / (simulationTime - Seconds (calcStart)).GetMicroSeconds ()  << " Mb/s" << std::endl;
          std::cout << "  Mean delay:\t"   << (double)(flow->second.delaySum / (flow->second.rxPackets)).GetMicroSeconds () / 1000 << " ms" << std::endl;    
          if (flow->second.rxPackets > 1)
            std::cout << "  Mean jitter:\t"  << (double)(flow->second.jitterSum / (flow->second.rxPackets - 1)).GetMicroSeconds () / 1000 << " ms" << std::endl;   
          else
            std::cout << "  Mean jitter:\t---"   << std::endl;
        }
      else
        {
          std::cout << "  Throughput:\t0 Mb/s" << std::endl;
          std::cout << "  Mean delay:\t---"    << std::endl;    
          std::cout << "  Mean jitter:\t---"   << std::endl;
        }

      //increase variables for overall results calculation
      txBytes     += flow->second.txBytes;
      rxBytes     += flow->second.rxBytes;
      txPackets   += flow->second.txPackets;
      rxPackets   += flow->second.rxPackets;
      lostPackets += flow->second.lostPackets;
      //throughput  += (flow->second.rxPackets > 0 ? flow->second.rxBytes * 8.0 / (flow->second.timeLastRxPacket.GetSeconds ()-flow->second.timeFirstTxPacket.GetSeconds ()) / 1000000 : 0);
      throughput  += (flow->second.rxPackets > 0 ? flow->second.rxBytes * 8.0 / (simulationTime - Seconds (calcStart)).GetMicroSeconds () : 0);
      delaySum    += flow->second.delaySum;
      jitterSum   += flow->second.jitterSum;
    }


  //print overall results
  std::cout << "=======================Total: =====================================" << std::endl;

  std::cout << "  Tx bytes:\t"     << txBytes     << std::endl;
  std::cout << "  Rx bytes:\t"     << rxBytes     << std::endl;
  std::cout << "  Tx packets:\t"   << txPackets   << std::endl;
  std::cout << "  Rx packets:\t"   << rxPackets   << std::endl;
  std::cout << "  Lost packets:\t" << lostPackets << std::endl;
  std::cout << "  Throughput:\t"   << throughput  << " Mb/s" << std::endl;
  if (rxPackets > 0)
    {
      std::cout << "  Mean delay:\t"   << (double)(delaySum / (rxPackets)).GetMicroSeconds () / 1000 << " ms" << std::endl;    
      if (rxPackets > 1)  
        std::cout << "  Mean jitter:\t"  << (double)(jitterSum / (rxPackets - 1)).GetMicroSeconds () / 1000  << " ms" << std::endl;   
      else
        std::cout << "  Mean jitter:\t---"   << std::endl;
    }
  else
    {
      std::cout << "  Mean delay:\t---"    << std::endl;    
      std::cout << "  Mean jitter:\t---"   << std::endl;
    }


  return 0;
}
