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

struct SimulationResult {
  double throughput;
  double lossRate;
};

class DroneSwarmExperiment {
public:
  DroneSwarmExperiment()
      : port(9),
        nClusters(3),
        nNodesPerCluster(10),
        width(500),
        height(500),
        simulationTime(60) {}

  SimulationResult Run() {
    NS_LOG_INFO("Iniciando simulación...");
    RngSeedManager::SetSeed(std::time(nullptr));

    // Configuración del canal WiFi con modelo LogDistance
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel", "Exponent", DoubleValue(3.0));
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    // Potencia de transmisión a 30 dBm
    phy.Set("TxPowerStart", DoubleValue(30.0));
    phy.Set("TxPowerEnd", DoubleValue(30.0));

    WifiHelper wifi;
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue("OfdmRate6Mbps"),
                                 "ControlMode", StringValue("OfdmRate6Mbps"));

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    // Configuración de la movilidad: nodos en un grid con DeltaX y DeltaY de 10 m
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue(0.0),
                                  "MinY", DoubleValue(0.0),
                                  "DeltaX", DoubleValue(10.0),
                                  "DeltaY", DoubleValue(10.0),
                                  "GridWidth", UintegerValue(5),
                                  "LayoutType", StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    // Instalación de la pila de Internet (se debe instalar antes de asignar direcciones IP)
    InternetStackHelper internet;
    // Dado que luego usaremos InstallAll() en main para instalar la pila a todos los nodos,
    // aquí se llama a Install() sobre cada clúster.
    // Configuración de direcciones IP
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.0.0.0", "255.255.255.0");

    // Función lambda para crear un clúster de nodos.
    // Se instala primero la pila de Internet, luego la movilidad, se instalan los dispositivos y se asignan direcciones.
    auto createCluster = [&](int nNodes, NodeContainer &clusterNodes, int clusterId) {
      clusterNodes.Create(nNodes);
      internet.Install(clusterNodes); // Instalar IPv4 en cada nodo antes de instalar dispositivos
      mobility.Install(clusterNodes);
      NetDeviceContainer devices = wifi.Install(phy, mac, clusterNodes);
      Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);
      NS_LOG_INFO("Cluster " << clusterId << " configurado con " << nNodes << " nodos.");
      ipv4.NewNetwork();
      return interfaces;
    };

    NodeContainer clusters[nClusters];
    Ipv4InterfaceContainer interfaces[nClusters];

    for (int i = 0; i < nClusters; ++i) {
      interfaces[i] = createCluster(nNodesPerCluster, clusters[i], i);
    }

    // Función lambda para conectar nodos y configurar tráfico
    auto connectNodes = [&](NodeContainer &nodes, Ipv4InterfaceContainer &interfaces, int clusterId) {
      Ipv4Address destAddress = interfaces.GetAddress(nodes.GetN() - 1);
      NS_LOG_INFO("Configurando tráfico en Cluster " << clusterId << " hacia " << destAddress);

      OnOffHelper onOff("ns3::UdpSocketFactory", InetSocketAddress(destAddress, port));
      onOff.SetAttribute("PacketSize", UintegerValue(1024));
      onOff.SetAttribute("DataRate", StringValue("1Mbps"));
      onOff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
      onOff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
      ApplicationContainer app = onOff.Install(nodes.Get(0));
      app.Start(Seconds(2.0)); // Emisor inicia a 2 s para que el receptor ya esté activo
      app.Stop(Seconds(simulationTime));

      PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
      ApplicationContainer sinkApp = sink.Install(nodes.Get(nodes.GetN() - 1));
      sinkApp.Start(Seconds(1.0));
      sinkApp.Stop(Seconds(simulationTime));

      return std::make_pair(app, sinkApp);
    };

    std::vector<std::pair<ApplicationContainer, ApplicationContainer>> apps;
    for (int i = 0; i < nClusters; ++i) {
      apps.push_back(connectNodes(clusters[i], interfaces[i], i));
    }

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(simulationTime));
    Simulator::Run();

    monitor->SerializeToXmlFile("flow-monitor-results.xml", true, true);

    // Se asume un total fijo de 900 paquetes enviados para el cálculo
    int sent = 900;
    int received = 0;
    for (auto &app : apps) {
      Ptr<PacketSink> sink = DynamicCast<PacketSink>(app.second.Get(0));
      received += sink->GetTotalRx();
      NS_LOG_INFO("Paquetes recibidos en el nodo " 
                  << app.second.Get(0)->GetNode()->GetId() << ": " 
                  << sink->GetTotalRx());
    }

    Simulator::Destroy();

    double throughput = received * 8 / simulationTime / 1e6; // Mbps
    double lossRate = (sent > 0) ? (sent - received) / static_cast<double>(sent) : 0.0;

    NS_LOG_INFO("Paquetes enviados: " << sent);
    NS_LOG_INFO("Paquetes recibidos: " << received);
    NS_LOG_INFO("Throughput: " << throughput << " Mbps");
    NS_LOG_INFO("Loss Rate: " << lossRate);

    std::ofstream results("simulation_results.csv", std::ios::app);
    results << throughput << "," << lossRate << std::endl;
    results.close();

    return {throughput, lossRate};
  }

private:
  int port;
  int nClusters;
  int nNodesPerCluster;
  double width;
  double height;
  double simulationTime;
};

int main(int argc, char **argv) {
  int simTime = 60;
  CommandLine cmd;
  cmd.AddValue("simulationTime", "Simulation duration in seconds", simTime);
  cmd.Parse(argc, argv);

  LogComponentEnable("DroneSwarmMANET", LOG_LEVEL_INFO);

  DroneSwarmExperiment experiment;
  SimulationResult result = experiment.Run();

  std::cout << "Throughput: " << result.throughput << " Mbps" << std::endl;
  std::cout << "Loss Rate: " << result.lossRate << std::endl;

  return 0;
}
