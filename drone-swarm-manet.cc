#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/olsr-module.h"
#include "ns3/yans-wifi-helper.h"
#include <algorithm>
#include <ctime>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DroneSwarmMANET");

// Structure to store simulation results
struct SimulationResult {
  double throughput;
  double lossRate;
};

// Main simulation class
class DroneSwarmExperiment {
public:
  DroneSwarmExperiment()
      : port(9), nLevels(2), nClusters1(2), nNodesPerCluster1(3), width(500),
        height(500), simulationTime(30) {
    NS_LOG_INFO("DroneSwarmExperiment initialized.");
  }

  SimulationResult Run() {
    NS_LOG_INFO("Starting simulation...");
    RngSeedManager::SetSeed(std::time(nullptr));

    // Wireless channel configuration
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    channel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    NS_LOG_INFO("Wireless channel configured.");

    // Physical layer configuration
    YansWifiPhyHelper phy = YansWifiPhyHelper();
    phy.SetChannel(channel.Create());
    phy.Set("TxPowerStart", DoubleValue(100.0));
    phy.Set("TxPowerEnd", DoubleValue(100.0));
    NS_LOG_INFO("Physical layer configured.");

    // WiFi configuration
    WifiHelper wifi;
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
                                 StringValue("HtMcs7"), "ControlMode",
                                 StringValue("HtMcs0"));
    NS_LOG_INFO("WiFi configured with ConstantRateWifiManager.");

    // MAC configuration
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    // Mobility setup
    MobilityHelper mobility;
    mobility.SetPositionAllocator(
        "ns3::RandomRectanglePositionAllocator", "X",
        StringValue("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"), "Y",
        StringValue("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    NS_LOG_INFO("Mobility model configured.");

    // Routing protocol
    InternetStackHelper internet;
    OlsrHelper olsr;
    internet.SetRoutingHelper(olsr);
    NS_LOG_INFO("OLSR routing protocol configured.");

    // IP addressing
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.0.0.0", "255.255.255.0");

    // Cluster creation function
    auto createCluster = [&](int nNodes, NodeContainer &clusterNodes) {
      clusterNodes.Create(nNodes);
      NS_LOG_INFO("Created cluster with " << nNodes << " nodes.");
      mobility.Install(clusterNodes);
      NetDeviceContainer devices = wifi.Install(phy, mac, clusterNodes);
      internet.Install(clusterNodes);
      Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);
      NS_LOG_INFO("Assigned IP addresses to cluster.");
      return interfaces;
    };

    // First level clusters
    NodeContainer firstLevelClusters[nClusters1];
    Ipv4InterfaceContainer firstLevelInterfaces[nClusters1];

    for (int i = 0; i < nClusters1; ++i) {
      firstLevelInterfaces[i] =
          createCluster(nNodesPerCluster1, firstLevelClusters[i]);
      ipv4.NewNetwork();
    }

    // Second level (inter-cluster) communication
    NodeContainer secondLevelCluster;
    secondLevelCluster.Add(firstLevelClusters[0].Get(0));
    secondLevelCluster.Add(firstLevelClusters[1].Get(0));
    Ipv4InterfaceContainer secondLevelInterfaces =
        createCluster(2, secondLevelCluster);

    // Function to set up traffic generation
    auto connectNodes = [&](NodeContainer &nodes,
                            Ipv4InterfaceContainer &interfaces, int port) {
      NS_LOG_INFO("Connecting nodes for traffic generation.");
      Ipv4Address destAddress = interfaces.GetAddress(1);
      OnOffHelper onOff("ns3::UdpSocketFactory",
                        InetSocketAddress(destAddress, port));
      onOff.SetAttribute("PacketSize", UintegerValue(1024));
      onOff.SetAttribute("DataRate", StringValue("1Mbps"));
      ApplicationContainer app = onOff.Install(nodes.Get(0));
      app.Start(Seconds(1.0));
      app.Stop(Seconds(simulationTime));

      PacketSinkHelper sink("ns3::UdpSocketFactory",
                            InetSocketAddress(Ipv4Address::GetAny(), port));
      ApplicationContainer sinkApp = sink.Install(nodes.Get(1));
      sinkApp.Start(Seconds(1.0));
      sinkApp.Stop(Seconds(simulationTime));

      return std::make_pair(app, sinkApp);
    };

    // Connect first-level clusters
    auto apps1 =
        connectNodes(firstLevelClusters[0], firstLevelInterfaces[0], port);
    auto apps2 =
        connectNodes(firstLevelClusters[1], firstLevelInterfaces[1], port);

    // Enable tracing
    AsciiTraceHelper ascii;
    phy.EnableAsciiAll(ascii.CreateFileStream("wifi-trace.tr"));

    // Flow monitor setup
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(simulationTime));
    NS_LOG_INFO("Running simulation...");
    Simulator::Run();

    monitor->SerializeToXmlFile("flow-monitor-results.xml", true, true);
    NS_LOG_INFO("Simulation finished. Processing results...");

    // Calculate throughput and loss rate
    Ptr<PacketSink> sink1 = DynamicCast<PacketSink>(apps1.second.Get(0));
    Ptr<PacketSink> sink2 = DynamicCast<PacketSink>(apps2.second.Get(0));

    int sent = 2 * nNodesPerCluster1 * simulationTime;
    int received = sink1->GetTotalRx() + sink2->GetTotalRx();
    Simulator::Destroy();

    double throughput = received / simulationTime;
    double lossRate = (sent > 0) ? (sent - received) / static_cast<double>(sent) : 0.0;

    NS_LOG_INFO("Throughput: " << throughput << " Pkt/s");
    NS_LOG_INFO("Loss Rate: " << lossRate);

    return {throughput, lossRate};
  }

private:
  int port, nLevels, nClusters1, nNodesPerCluster1;
  double width, height, simulationTime;
};

int main(int argc, char **argv) {
  int simTime = 60;

  CommandLine cmd;
  cmd.AddValue("simulationTime", "Simulation duration in seconds", simTime);
  cmd.Parse(argc, argv);

  // Habilitar logs de información
  LogComponentEnable("DroneSwarmMANET", LOG_LEVEL_INFO);

  DroneSwarmExperiment experiment;
  SimulationResult result = experiment.Run();

  std::cout << "Throughput: " << result.throughput << " Pkt/s" << std::endl;
  std::cout << "Loss Rate: " << result.lossRate << std::endl;

  return 0;
}
