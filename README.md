# T01-DroneSwarmMANET

# Drone Swarm MANET Simulation with ns-3

This project simulates a **drone swarm-based Mobile Ad Hoc Network (MANET)** using the **ns-3** simulator. It sets up a hierarchical wireless network where drones communicate using the **OLSR (Optimized Link State Routing) protocol**.

## 🛠 Features
- **Hierarchical Cluster-Based Network**: Two-level structure with multiple drone clusters.
- **Wi-Fi Communication**: Uses `AarfWifiManager` for rate control.
- **OLSR Routing Protocol**: Optimized routing for dynamic topology.
- **Mobility Model**: Random placement of drones in a 500x500 area.
- **Performance Metrics**: Measures **throughput** and **packet loss rate**.

## 🚀 Installation & Usage
1. **Install ns-3** (if not already installed):
   ```sh
   git clone https://gitlab.com/nsnam/ns-3-dev.git
   cd ns-3-dev
   ./ns3 configure --enable-examples
   ./ns3 build
   ```
2. **Clone this repository**:
   ```sh
   git clone https://github.com/dghost32/T01-DroneSwarmMANET.git
   cd drone-swarm-manet
   ```
3. **Compile and run the simulation**:
   ```sh
   ./waf --run "drone-swarm-manet"
   ```

## 📊 Output
The simulation prints:
```sh
Throughput: X Pkt/s
Loss Rate: Y
```

where **X** is the average packet throughput and **Y** is the packet loss rate.
