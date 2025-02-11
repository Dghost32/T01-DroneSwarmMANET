#include <fstream>
#include <iostream>
#include <ctime>
#include <vector>
#include <algorithm>
#include <numeric>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/olsr-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/applications-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DroneSwarmMANET");

struct SimulationResult {
    double throughput;
    double lossRate;
};

class DroneSwarmExperiment {
public:
    DroneSwarmExperiment() : port(9), nLevels(2), nClusters1(2), nNodesPerCluster1(3), width(500), height(500), simulationTime(30) {}

    SimulationResult Run() {
        RngSeedManager::SetSeed(std::time(nullptr));

        YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
        channel.AddPropagationLoss("ns3::FriisPropagationLossModel");
        channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

        YansWifiPhyHelper phy;
        phy.Set("TxPowerStart", DoubleValue(100.0));
        phy.Set("TxPowerEnd", DoubleValue(100.0));

        WifiHelper wifi;
        wifi.SetRemoteStationManager("ns3::AarfWifiManager");

        MobilityHelper mobility;
        mobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                                      "X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=" + std::to_string(width) + "]"),
                                      "Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=" + std::to_string(height) + "]"));

        InternetStackHelper internet;
        OlsrHelper olsr;
        internet.SetRoutingHelper(olsr);

        Ipv4AddressHelper ipv4;
        ipv4.SetBase("10.0.0.0", "255.255.255.0");

        auto createCluster = [&](int index, int nNodes, NodeContainer& clusterNodes) {
            clusterNodes.Create(nNodes);
            mobility.Install(clusterNodes);
            NetDeviceContainer devices = wifi.Install(phy, WifiMacHelper::Default(), clusterNodes);
            internet.Install(clusterNodes);
            ipv4.Assign(devices);
            ipv4.NewNetwork();
            return devices;
        };

        NodeContainer firstLevelClusters[nClusters1];
        NetDeviceContainer firstLevelDevices[nClusters1];

        for (int i = 0; i < nClusters1; ++i) {
            firstLevelDevices[i] = createCluster(i, nNodesPerCluster1, firstLevelClusters[i]);
        }

        NodeContainer secondLevelCluster;
        secondLevelCluster.Add(firstLevelClusters[0].Get(0));
        secondLevelCluster.Add(firstLevelClusters[1].Get(0));

        NetDeviceContainer secondLevelDevices = createCluster(nClusters1, 2, secondLevelCluster);

        auto connectNodes = [&](NodeContainer& nodes, int port) {
            OnOffHelper onOff("ns3::UdpSocketFactory", InetSocketAddress(nodes.Get(1)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal(), port));
            onOff.SetAttribute("PacketSize", UintegerValue(1024));
            onOff.SetAttribute("DataRate", DataRateValue(DataRate("1Mbps")));
            ApplicationContainer app = onOff.Install(nodes.Get(0));
            app.Start(Seconds(1.0));
            app.Stop(Seconds(simulationTime));
            return app;
        };

        connectNodes(firstLevelClusters[0], port);
        connectNodes(firstLevelClusters[1], port);

        Simulator::Stop(Seconds(simulationTime));
        Simulator::Run();

        int sent = 0, received = 0;
        for (int i = 0; i < nClusters1; ++i) {
            sent += firstLevelClusters[i].Get(0)->GetObject<OnOffApplication>()->GetTotalTx();
            received += firstLevelClusters[i].Get(1)->GetObject<PacketSink>()->GetTotalRx();
        }

        Simulator::Destroy();

        double throughput = received / simulationTime;
        double lossRate = (sent - received) / static_cast<double>(sent);

        return {throughput, lossRate};
    }

private:
    int port;
    int nLevels;
    int nClusters1;
    int nNodesPerCluster1;
    double width;
    double height;
    double simulationTime;
};

int main(int argc, char **argv) {
    DroneSwarmExperiment experiment;
    SimulationResult result = experiment.Run();

    std::cout << "Throughput: " << result.throughput << " Pkt/s" << std::endl;
    std::cout << "Loss Rate: " << result.lossRate << std::endl;

    return 0;
}
