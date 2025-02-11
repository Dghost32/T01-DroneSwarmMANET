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
#include "ns3/flow-monitor-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DroneSwarmMANET");

// Estructura para almacenar los resultados de la simulación
struct SimulationResult {
    double throughput;  // Tasa de transmisión de paquetes
    double lossRate;    // Tasa de pérdida de paquetes
};

// Clase principal para la simulación del enjambre de drones
class DroneSwarmExperiment {
public:
    // Constructor con valores por defecto
    DroneSwarmExperiment() : port(9), nLevels(2), nClusters1(2), nNodesPerCluster1(3), width(500), height(500), simulationTime(30) {}

    // Método principal para ejecutar la simulación
    SimulationResult Run() {
        RngSeedManager::SetSeed(std::time(nullptr));  // Inicializa la semilla aleatoria basada en el tiempo actual

        // Configuración del canal inalámbrico
        YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
        channel.AddPropagationLoss("ns3::FriisPropagationLossModel");  // Pérdida de propagación basada en el modelo Friis
        channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");  // Modelo de retraso de propagación constante

        // Configuración del PHY (capa física)
        YansWifiPhyHelper phy;
        phy.Set("TxPowerStart", DoubleValue(100.0));
        phy.Set("TxPowerEnd", DoubleValue(100.0));

        // Configuración de WiFi
        WifiHelper wifi;
        wifi.SetRemoteStationManager("ns3::AarfWifiManager");

        // Configuración de la movilidad usando variables aleatorias uniformes
        MobilityHelper mobility;
        Ptr<UniformRandomVariable> randomX = CreateObject<UniformRandomVariable>();
        randomX->SetAttribute("Min", DoubleValue(0.0));
        randomX->SetAttribute("Max", DoubleValue(width));
        Ptr<UniformRandomVariable> randomY = CreateObject<UniformRandomVariable>();
        randomY->SetAttribute("Min", DoubleValue(0.0));
        randomY->SetAttribute("Max", DoubleValue(height));

        mobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                                      "X", PointerValue(randomX),
                                      "Y", PointerValue(randomY));

        // Configuración del protocolo de enrutamiento OLSR
        InternetStackHelper internet;
        OlsrHelper olsr;
        internet.SetRoutingHelper(olsr);

        // Configuración de direcciones IP
        Ipv4AddressHelper ipv4;
        ipv4.SetBase("10.0.0.0", "255.255.255.0");

        // Función lambda para crear un cluster de nodos
        auto createCluster = [&](int nNodes, NodeContainer& clusterNodes) {
            clusterNodes.Create(nNodes);
            mobility.Install(clusterNodes);
            NetDeviceContainer devices = wifi.Install(phy, WifiMacHelper::Default(), clusterNodes);
            internet.Install(clusterNodes);
            return ipv4.Assign(devices);  // Asigna direcciones IP y devuelve la interfaz
        };

        // Creación de los clusters de primer nivel
        NodeContainer firstLevelClusters[nClusters1];
        Ipv4InterfaceContainer firstLevelInterfaces[nClusters1];

        for (int i = 0; i < nClusters1; ++i) {
            firstLevelInterfaces[i] = createCluster(nNodesPerCluster1, firstLevelClusters[i]);
            ipv4.NewNetwork();  // Configurar una nueva red para evitar superposición de direcciones
        }

        // Creación del segundo nivel de clusters (enlace entre clusters)
        NodeContainer secondLevelCluster;
        secondLevelCluster.Add(firstLevelClusters[0].Get(0));
        secondLevelCluster.Add(firstLevelClusters[1].Get(0));
        Ipv4InterfaceContainer secondLevelInterfaces = createCluster(2, secondLevelCluster);

        // Función lambda para conectar dos nodos con una aplicación OnOff
        auto connectNodes = [&](NodeContainer& nodes, Ipv4InterfaceContainer& interfaces, int port) {
            Ipv4Address destAddress = interfaces.GetAddress(1);
            OnOffHelper onOff("ns3::UdpSocketFactory", InetSocketAddress(destAddress, port));
            onOff.SetAttribute("PacketSize", UintegerValue(1024));
            onOff.SetAttribute("DataRate", DataRateValue(DataRate("1Mbps")));
            ApplicationContainer app = onOff.Install(nodes.Get(0));
            app.Start(Seconds(1.0));
            app.Stop(Seconds(simulationTime));

            // Configurar un PacketSink para recibir los datos
            PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
            ApplicationContainer sinkApp = sink.Install(nodes.Get(1));
            sinkApp.Start(Seconds(1.0));
            sinkApp.Stop(Seconds(simulationTime));

            return std::make_pair(app, sinkApp);
        };

        // Conectar los clusters con OnOff y PacketSink
        auto apps1 = connectNodes(firstLevelClusters[0], firstLevelInterfaces[0], port);
        auto apps2 = connectNodes(firstLevelClusters[1], firstLevelInterfaces[1], port);

        // Configuración de rastreo ASCII
        AsciiTraceHelper ascii;
        phy.EnableAsciiAll(ascii.CreateFileStream("wifi-trace.tr"));

        // Configuración de FlowMonitor para análisis de tráfico
        FlowMonitorHelper flowmon;
        Ptr<FlowMonitor> monitor = flowmon.InstallAll();

        Simulator::Stop(Seconds(simulationTime));
        Simulator::Run();

        // Guardar los resultados del monitoreo de flujo
        monitor->SerializeToXmlFile("flow-monitor-results.xml", true, true);

        // Recuperar los datos recibidos desde PacketSink
        Ptr<PacketSink> sink1 = DynamicCast<PacketSink>(apps1.second.Get(0));
        Ptr<PacketSink> sink2 = DynamicCast<PacketSink>(apps2.second.Get(0));

        int sent = 2 * nNodesPerCluster1 * simulationTime;  // Paquetes enviados
        int received = sink1->GetTotalRx() + sink2->GetTotalRx();  // Paquetes recibidos

        Simulator::Destroy();

        // Cálculo del rendimiento (throughput) y la tasa de pérdida de paquetes
        double throughput = received / simulationTime;
        double lossRate = (sent > 0) ? (sent - received) / static_cast<double>(sent) : 0.0;

        // Guardar los resultados en un archivo CSV
        std::ofstream results("simulation_results.csv", std::ios::app);
        results << throughput << "," << lossRate << std::endl;
        results.close();

        return {throughput, lossRate};
    }

private:
    int port;  // Puerto de comunicación
    int nLevels;  // Niveles jerárquicos
    int nClusters1;  // Número de clusters de primer nivel
    int nNodesPerCluster1;  // Número de nodos por cluster
    double width;  // Ancho del área de simulación
    double height;  // Alto del área de simulación
    double simulationTime;  // Tiempo de simulación en segundos
};

int main(int argc, char **argv) {
    int simTime = 30;

    // Permitir cambiar la duración de la simulación desde la línea de comandos
    CommandLine cmd;
    cmd.AddValue("simulationTime", "Tiempo de ejecución de la simulación en segundos", simTime);
    cmd.Parse(argc, argv);

    // Ejecutar la simulación
    DroneSwarmExperiment experiment;
    SimulationResult result = experiment.Run();

    // Imprimir los resultados en la consola
    std::cout << "Throughput: " << result.throughput << " Pkt/s" << std::endl;
    std::cout << "Loss Rate: " << result.lossRate << std::endl;

    return 0;
}
