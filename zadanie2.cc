#include "ns3/command-line.h"
#include "ns3/string.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/qos-txop.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/olsr-helper.h"
#include "ns3/csma-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/gnuplot.h"
#include "ns3/rng-seed-manager.h"
#include <math.h>

using namespace ns3;

void runSim(double);
void fillGnuplotData(std::vector<int> meassurements[]);
void fillGnuplotData(std::vector<double> meassurements[]);
void createXAxis(std::vector<double> meassurements[]);

void createXAxisAndConvertToDouble(std::vector<int> meassurements[], std::vector<double> xValues);
void fillGnuplotData(std::vector<double> meassurements[], std::vector<double> xValues);
void countPacketForEverySecond(std::vector<double> packetArrivalTimes, std::vector<int> meassurementsArray[], uint64_t index);
void computeDataAndSetGraph(std::vector<double> meassurements[], std::vector<double> xValues);
CommandLine addValuesCmd();
AnimationInterface createAnimFile(NodeContainer apNodes, NodeContainer serverNodes, NodeContainer uavNodes);
Gnuplot setupGraphAndData(int graphNumber);


// global variables / simulation settings
bool printLogs = false;
bool createAnim = false;
int graphNumber = 0;
int simTime = 30;
int distance = 15;
int helloInterval = 2000; //ms
int uavSpeed = 25;
int packetSize = 0;

// Variables for Graphs
Gnuplot2dDataset data;
// True if we got some data for plot
bool someData = false;
Gnuplot2dDataset errorBars;
Gnuplot2dDataset data2;
Gnuplot2dDataset errorBars2;


// Sink Application packets meassurments
// Variable for save time when sink app receive packet
std::vector<double> receiveTimes = {};
std::vector<int> packetsPerSec[10];
// Packets received on SinkApp
int receivedPackets = 0;

int uavID = 21;
// Simulation will run nRuns
uint64_t nRuns = 1;

// all packets meassurements
int allPacketsRecieved = 0;
std::vector<double> allPacketsArrivalTimes = {};
std::vector<int> allPacketsMeassurements[10];
// All packets, which have been sent by UAV
int allSendedPackets = 0;
std::vector<double> allPacketsSendedTimes = {};
std::vector<int> allSendedPacketsAllRuns[5];
std::vector<int> allSendedPacketsPerSec[5];

// For graph 2
std::vector<double> intervalHelloPackets = {};
// For graph 3
std::vector<double> uavSpeedAggregateGraph = {};

// Type of movement (navigate to StartPoint or RandomWalk)
bool commingBack = false;
// Position to go for Mobility models
Ptr<RandomRectanglePositionAllocator> randWaypointAllocator;
Ptr<RandomRectanglePositionAllocator> startPointAllocator;

// Count received packets of Sink App

void receivedPacketsCallback(Ptr< const Packet > packet, const Address &address) {
    receivedPackets++;
    receiveTimes.push_back(Simulator::Now().GetSeconds());
}

void receivePacketOnCsmaCallback(Ptr< const Packet> packet) {
    ++allPacketsRecieved;
    allPacketsArrivalTimes.push_back(Simulator::Now().GetSeconds());
}

void appSendPacketCallback(Ptr< const Packet> packet) {
    ++allSendedPackets;
    allPacketsSendedTimes.push_back(Simulator::Now().GetSeconds());
}

static void changeOffTimeApp() {

    Config::Set("NodeList/" + std::to_string(uavID) + "/ApplicationList/0/$ns3::OnOffApplication/OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.6]"));
}

void backToStartPointCallback(Ptr< const MobilityModel> mobilityModel) {
    Vector position = mobilityModel->GetPosition();

    if (printLogs) {
        std::cout << Simulator::Now().GetSeconds() << "s " << "x: " << position.x << "; y: " << position.y << std::endl;
    }

    if (!commingBack) {
        // is the uav out of area?
        if (position.x < 0.0 || position.x > (6 * distance) || position.y < 0.0 || position.y > (6 * distance)) {
            commingBack = true;
            Config::Set("/NodeList/" + std::to_string(uavID) + "/$ns3::MobilityModel/$ns3::RandomWaypointMobilityModel/PositionAllocator", PointerValue(startPointAllocator));

            if (printLogs) {
                std::cout << Simulator::Now().GetSeconds() << "s " << "Robot can not send dat. He will go back to start point" << std::endl;
            }
        }
    } else {
        // Is uav home?
        if (position.x > (3.5 * distance - 0.5) && position.x < (3.5 * distance + 0.5) && position.y > (3.5 * distance - 0.5) && position.y < (3.5 * distance + 0.5)) {
            commingBack = false;
            Config::Set("/NodeList/" + std::to_string(uavID) + "/$ns3::MobilityModel/$ns3::RandomWaypointMobilityModel/PositionAllocator", PointerValue(randWaypointAllocator));

            if (printLogs) {
                std::cout << Simulator::Now().GetSeconds() << "s " << "Robot reached start point and will walk randomly" << std::endl;
            }
        }
    }
}

void setUavSpeed(int speed) {

    Config::Set("NodeList/" + std::to_string(uavID) + "/$ns3::MobilityModel/$ns3::RandomWaypointMobilityModel/Speed", StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(speed) + "]"));
}

static void speedUpRobot() {

    Config::Set("NodeList/" + std::to_string(uavID) + "/$ns3::MobilityModel/$ns3::RandomWaypointMobilityModel/Speed", StringValue("ns3::ConstantRandomVariable[Constant=60]"));
}

static void doSimulation(double simulationTime) {
    // Server Node
    NodeContainer serverNodes;
    serverNodes.Create(1);
    Ptr<Node> server = serverNodes.Get(0);

    // AP Nodes
    int numberAPs = 30;
    NodeContainer apNodes;
    apNodes.Create(numberAPs);

    // UAV Node
    NodeContainer uavNodes;
    uavNodes.Create(1);
    Ptr<Node> uav = uavNodes.Get(0);
    uavID = uav->GetId();

    // Container helps to install nodes more easily
    // ApNodes and server communicate with each other
    NodeContainer ethernetNodes;
    ethernetNodes.Add(server);
    ethernetNodes.Add(apNodes);
    // Robot communicate with all apNodes
    NodeContainer wifiNodes;
    wifiNodes.Add(apNodes);
    wifiNodes.Add(uav);

    // -------------------------------------------------------------------//
    //                                                                    //
    // Set up the wifi network                                            //
    //                                                                    //
    // -------------------------------------------------------------------//

    // Create the wifi 
    WifiHelper wifi;
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
            "DataMode", StringValue("OfdmRate48Mbps")); // The highest possible value for Ofdm error model

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    // Create a channel object which implements the YANS channel model.  
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel",
            "MaxRange", DoubleValue(16.0));

    // Helper use to create Physical layer model easily
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());


    // Ns-3 the net device consist of the software driver to control the hardware
    // and the simulated hardware(Network Interface Cards, or NICs.).
    // Ensures connection of Node to network
    NetDeviceContainer wifiDevices = wifi.Install(wifiPhy, mac, wifiNodes);

    // Add the IPv4 protocol and choose routing type
    InternetStackHelper internet;

    OlsrHelper olsr;
    internet.SetRoutingHelper(olsr);
    internet.Install(wifiNodes);

    // Assign IPv4 addresses to the device drivers that we just created.
    Ipv4AddressHelper ipAddrs;
    ipAddrs.SetBase("192.168.0.0", "255.255.255.0");
    ipAddrs.Assign(wifiDevices);

    // -------------------------------------------------------------------//
    //                                                                    //
    // Create mobility models                                             //
    //                                                                    //
    // -------------------------------------------------------------------//

    // Create static mobility model for Server 
    MobilityHelper serverMobilityModel;
    serverMobilityModel.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    serverMobilityModel.SetPositionAllocator("ns3::GridPositionAllocator",
            "MinX", DoubleValue(7 * distance), // We move server to right 
            "MinY", DoubleValue(3.5 * distance));
    serverMobilityModel.Install(server);

    // Create static mobility model for APs 
    MobilityHelper apMobilityModel;
    apMobilityModel.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    apMobilityModel.SetPositionAllocator("ns3::GridPositionAllocator",
            "MinX", DoubleValue(20.0), // start X position
            "MinY", DoubleValue(20.0), // start X position
            "DeltaX", DoubleValue(distance), // space between nodes
            "DeltaY", DoubleValue(distance), // space between nodes
            "GridWidth", UintegerValue(5),
            "LayoutType", StringValue("RowFirst"));
    apMobilityModel.Install(apNodes);

    // When UAV leave APs than we change randWaypointAllocator to startPointAllocator
    Ptr<ConstantRandomVariable> generateRandStartPoint = CreateObject<ConstantRandomVariable>();
    generateRandStartPoint->SetAttribute("Constant", DoubleValue(3.5 * distance));

    startPointAllocator = CreateObject<RandomRectanglePositionAllocator>();
    startPointAllocator->SetX(generateRandStartPoint);
    startPointAllocator->SetY(generateRandStartPoint);

    // Create dynamic mobility model for UAV 
    Ptr<UniformRandomVariable> generateRandWayPoint = CreateObject<UniformRandomVariable>();
    generateRandWayPoint->SetAttribute("Min", DoubleValue(-3 * distance));
    generateRandWayPoint->SetAttribute("Max", DoubleValue(8 * distance));

    randWaypointAllocator = CreateObject<RandomRectanglePositionAllocator>();
    randWaypointAllocator->SetX(generateRandWayPoint);
    randWaypointAllocator->SetY(generateRandWayPoint);

    MobilityHelper uavMobility;
    uavMobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
            "Speed", StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(uavSpeed) + "]"),
            "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"),
            "PositionAllocator", PointerValue(randWaypointAllocator));
    // Start position of UAV
    uavMobility.SetPositionAllocator("ns3::GridPositionAllocator",
            "MinX", DoubleValue(3.5 * distance),
            "MinY", DoubleValue(3.5 * distance));
    uavMobility.Install(uav);

    // -------------------------------------------------------------------//
    //                                                                    //
    // Set up the LAN network                                             //
    //                                                                    //
    // -------------------------------------------------------------------//

    // The CSMA networks will be in the "67.86.52. address space
    ipAddrs.SetBase("67.86.52.0", "255.255.255.0");

    // Create the CSMA network
    CsmaHelper csmaNetwork;
    csmaNetwork.SetChannelAttribute("DataRate",
            DataRateValue(DataRate(5000 * 1000)));
    csmaNetwork.SetChannelAttribute("Delay", TimeValue(MilliSeconds(2)));
    NetDeviceContainer csmaDevices = csmaNetwork.Install(ethernetNodes);

    // Add the IPv4 protocol stack to the LAN nodes (We have only one node - server)
    internet.Install(serverNodes);
    // Assign IPv4 address to the new LAN nodes (only for server)
    Ipv4InterfaceContainer lanInterfaceContainer = ipAddrs.Assign(csmaDevices);

    // -------------------------------------------------------------------//
    //                                                                    //
    // Create OnOff Application for generate data                         //
    //                                                                    //
    // -------------------------------------------------------------------//

    uint16_t port = 10;

    // Let's fetch the IP address of the Server
    Ipv4Address remoteAddr = lanInterfaceContainer.GetAddress(server->GetId());

    OnOffHelper onoff("ns3::UdpSocketFactory",
            Address(InetSocketAddress(remoteAddr, port)));

    ApplicationContainer apps = onoff.Install(uav);
    apps.Start(Seconds(2));
    apps.Stop(Seconds(simulationTime - 1));

    // Create a packet sink to receive these packets
    PacketSinkHelper sink("ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), port));
    apps = sink.Install(server);

    // -------------------------------------------------------------------//
    //                                                                    //
    // Set up callbacks                                                   //
    //                                                                    //
    // ------------------------------------------------------------------ //

    if (graphNumber == 0) {
        Simulator::Schedule(Seconds(7.0), &changeOffTimeApp);
        Simulator::Schedule(Seconds(12.0), &speedUpRobot);
    }

    std::ostringstream pathMobilityTraceSource;
    pathMobilityTraceSource << "/NodeList/"
            << wifiNodes.Get(numberAPs)->GetId() // The last added node in wifiNodes is UAV
            << "/$ns3::MobilityModel/CourseChange";
    // If UAV move make callback
    Config::ConnectWithoutContext(pathMobilityTraceSource.str(), MakeCallback(&backToStartPointCallback));

    if (graphNumber >= 1 && graphNumber <= 3) {
        // Rx - A packet has been received on Sink app
        Config::ConnectWithoutContext("/NodeList/0/ApplicationList/0/$ns3::PacketSink/Rx", MakeCallback(&receivedPacketsCallback));
        //Tracking received packets on csma devices
        Config::ConnectWithoutContext("/NodeList/0/DeviceList/0/$ns3::CsmaNetDevice/MacRx", MakeCallback(&receivePacketOnCsmaCallback));
        // Tracking of sending packets from UAV 
        Config::ConnectWithoutContext("/NodeList/" + std::to_string(uavID) + "/ApplicationList/0/$ns3::OnOffApplication/Tx", MakeCallback(&appSendPacketCallback));
    }

    if (printLogs) {
        std::cout << "Address for UAV " << std::to_string(uavID) << std::endl;
    }

    // -------------------------------------------------------------------//
    //                                                                    //
    // Create NetAnim  file                                               //
    //                                                                    //
    // -------------------------------------------------------------------//

    if (createAnim) {
        AnimationInterface anim = createAnimFile(apNodes, serverNodes, uavNodes);

        runSim(simulationTime);
    } else {
        runSim(simulationTime);
    }
}

void runSim(double simulationTime) {
    Simulator::Stop(Seconds(simulationTime));
    Simulator::Run();
    Simulator::Destroy();
}

int main(int argc, char *argv[]) {
    // Simulation defaults are typically set before command line arguments are parsed.
    Config::SetDefault("ns3::OnOffApplication::PacketSize", StringValue("1472"));
    Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue("100kb/s"));

    CommandLine cmd = addValuesCmd();
    cmd.Parse(argc, argv);

    // Set up Hello interval for OLSR routing
    Config::SetDefault("ns3::olsr::RoutingProtocol::HelloInterval", StringValue(std::to_string(helloInterval) + "ms"));
    Gnuplot graph;
    // prvotne nastavenia v hl.funkcii
    if (graphNumber) {
        graph = setupGraphAndData(graphNumber);
    }
    // Set up program for type of graph
    if (graphNumber == 1) {
        nRuns = 5;
    } else if (graphNumber == 2) {
        nRuns = 3;
    } else if (graphNumber == 3) {
        nRuns = 5;
    } else if (graphNumber == 0) {
        nRuns = 1;
    } else {
        std::cerr << "GraphNumber has to be from interval <0; 3>" << std::endl;
        return -1;
    }

    // Manage RNG
    RngSeedManager seedManager;
    seedManager.SetRun(nRuns);

    //For graphs, increase hello intervals or speed of UAV,...
    int outerRuns = 1;

    if (graphNumber == 2) {
        outerRuns = 10;
        intervalHelloPackets.clear();
    }
    if (graphNumber == 3) {
        outerRuns = 10;
        uavSpeedAggregateGraph.clear();
    }

    // For graph 11 Hello interval
    int lastComputedInterval = 250;
    // For graph 12 Hello interval
    int lastComputedSpeed = 10;
    // only for logs
    int countLoop = 1;
    int allLoops = outerRuns * nRuns;

    // Simulation
    for (int outer = 0; outer < outerRuns; ++outer) {
        if (graphNumber == 2) {
            Config::SetDefault("ns3::olsr::RoutingProtocol::HelloInterval", StringValue(std::to_string(lastComputedInterval) + "ms"));
            lastComputedInterval += 250;
            intervalHelloPackets.push_back(lastComputedInterval);
        }

        if (graphNumber == 3) {
            setUavSpeed(lastComputedSpeed);
            lastComputedSpeed += 20;
            uavSpeedAggregateGraph.push_back(lastComputedSpeed);
        }

        for (uint64_t i = 0; i < nRuns; i++) {

            if (graphNumber >= 1 && graphNumber <= 3) {
                // For On/Off app (UAV)
                allSendedPackets = 0;
                allPacketsSendedTimes.clear();
                // For CSMA
                allPacketsRecieved = 0;
                allPacketsArrivalTimes.clear();
                // For Sink app
                receivedPackets = 0;
                receiveTimes.clear();
            }
            if (printLogs) {
                std::cout << "Num. " << countLoop << "/" << allLoops << std::endl;
            }

            doSimulation((double) simTime);

            if (graphNumber == 1) {
                countPacketForEverySecond(receiveTimes, packetsPerSec, i);
                countPacketForEverySecond(allPacketsSendedTimes, allSendedPacketsPerSec, i);
                if (printLogs) {
                    std::cout << "All sent packets from UAV " << allSendedPackets << std::endl;
                    std::cout << "All received packets on Sink app " << receivedPackets << std::endl;
                }
            }

            if (graphNumber == 2) {
                allPacketsMeassurements[i].push_back(allPacketsArrivalTimes.size());
                std::cout << "All sent packets from UAV " << allSendedPackets << std::endl;
                std::cout << "allPacketsArrivalTimes size " << allPacketsArrivalTimes.size() << std::endl;
            }
            if (graphNumber == 3) {
                allPacketsMeassurements[i].push_back(receiveTimes.size());
                std::cout << "All received packets on Sink app " << receivedPackets << std::endl;
                std::cout << "receiveTimes size " << receiveTimes.size() << std::endl;
            }
            countLoop++;
        }
    }

    if (graphNumber == 1) {
        std::vector<double> quotient[nRuns];
        for (int i = 0; i < nRuns; ++i) {
            for (int j = 0; j < packetsPerSec[i].size(); ++j) {
                if (packetsPerSec[i][j] != 0) {
                    // push_back() - adds a new element at the end of the vector, after its current last element.
                    quotient[i].push_back(packetsPerSec[i][j]);
                    someData = true;
                } else {
                    quotient[i].push_back(0);
                }
            }
        }


        fillGnuplotData(quotient);

        std::vector<double> quotient2[nRuns];
        for (int i = 0; i < nRuns; ++i) {
            for (int j = 0; j < allSendedPacketsPerSec[i].size(); ++j) {
                if (allSendedPacketsPerSec[i][j] != 0) {
                    quotient2[i].push_back(allSendedPacketsPerSec[i][j]);
                } else {
                    quotient2[i].push_back(0);
                }
            }
        }
        createXAxis(quotient2);

        graph.AddDataset(errorBars2);
        graph.AddDataset(data2);
    }

    if (graphNumber == 2) {
        someData = true;
        createXAxisAndConvertToDouble(allPacketsMeassurements, intervalHelloPackets);
    }

    if (graphNumber == 3) {
        someData = true;
        createXAxisAndConvertToDouble(allPacketsMeassurements, uavSpeedAggregateGraph);
    }

    if (graphNumber) {
        // zaverecne spustenie
        if (someData) {
            graph.AddDataset(errorBars);
            graph.AddDataset(data);
            std::ofstream plotFile("graf" + std::to_string(graphNumber) + ".plt");
            graph.GenerateOutput(plotFile);
            plotFile.close();
            std::string pltName = "gnuplot graf" + std::to_string(graphNumber) + ".plt";
            if (system(pltName.c_str()));
        } else {
            std::cout << "No data for plot, graph will not be generated" << std::endl;
        }

    }
    return 0;
}

void countPacketForEverySecond(std::vector<double> packetArrivalTimes, std::vector<int> meassurementsArray[], uint64_t index) {
    // we loop through all times when we received packets
    std::cout << "Index: " << index << " packetArrivalTimes.size() " << packetArrivalTimes.size() << std::endl;

    if (packetArrivalTimes.size() == 0) {
        // fill - no data
        for (int k = 0; k < simTime; k++) {
            meassurementsArray[index].push_back(0);
        }
    }
    for (int j = 0; j < packetArrivalTimes.size(); j++) {
        for (int k = 0; k < simTime; k++) {
            // In time 0 we received 0 packets
            if (j == 0)
                meassurementsArray[index].push_back(0);
            if (packetArrivalTimes.size() > 0) {
                // We got times as double, but for graph we need time as integer. 
                // We find correct second in simulation
                if (packetArrivalTimes[j] >= k && packetArrivalTimes[j] < k + 1) {
                    meassurementsArray[index][k]++;
                    // when time was find break and go to the next 
                    if (j != 0)
                        break;
                }
            }
        }
    }
}

void fillGnuplotData(std::vector<int> meassurements[]) {
    // convert the integers to doubles and call the other function
    std::vector<double> doubles[nRuns];
    for (int i = 0; i < nRuns; ++i) {
        doubles[i] = std::vector<double>(meassurements[i].begin(), meassurements[i].end());
    }
    fillGnuplotData(doubles);
}

void fillGnuplotData(std::vector<double> meassurements[]) {
    std::vector<double> xVals;
    for (int i = 0; i < meassurements[0].size(); ++i) {
        xVals.push_back((double) i);
    }
    fillGnuplotData(meassurements, xVals);
}

void createXAxis(std::vector<double> measuredData[]) {
    std::vector<double> xValues;
    for (int i = 0; i < measuredData[0].size(); ++i) {
        xValues.push_back((double) i);
    }
    computeDataAndSetGraph(measuredData, xValues);
}

void createXAxisAndConvertToDouble(std::vector<int> meassurements[], std::vector<double> xValues) {
    std::vector<double> doubles[nRuns];
    for (int i = 0; i < nRuns; ++i) {
        doubles[i] = std::vector<double>(meassurements[i].begin(), meassurements[i].end());
    }
    fillGnuplotData(doubles, xValues);
}

void fillGnuplotData(std::vector<double> meassurements[], std::vector<double> xValues) {
    for (int i = 0; i < meassurements[0].size(); ++i) {

        double avg = 0.0;
        for (uint32_t j = 0; j < nRuns; ++j) {
            avg += meassurements[j].at(i);
        }
        avg /= nRuns;

        double deviation = 0.0;
        for (uint64_t j = 0; j < nRuns; ++j) {
            double k = meassurements[j].at(i) - avg;
            deviation += k*k;
        }
        deviation /= nRuns;
        deviation = sqrt(deviation);

        data.Add(xValues[i], avg);
        errorBars.Add(xValues[i], avg, deviation);
    }
}

// Compute avarage and deviation

void computeDataAndSetGraph(std::vector<double> measuredData[], std::vector<double> xValues) {
    for (std::vector<double>::size_type i = 0; i < measuredData[0].size(); ++i) {
        double avg = 0.0;
        for (uint64_t j = 0; j < nRuns; ++j) {
            avg += measuredData[j].at(i);
        }
        avg = avg / nRuns;

        double deviation = 0.0;
        for (uint64_t j = 0; j < nRuns; ++j) {
            double k = measuredData[j].at(i) - avg;
            deviation = deviation + k*k;
        }
        deviation /= nRuns;
        deviation = sqrt(deviation);

        data2.Add(xValues[i], avg);
        errorBars2.Add(xValues[i], avg, deviation);
    }
}

CommandLine addValuesCmd() {
    // CommandLine arguments
    CommandLine cmd;
    cmd.AddValue("anim", "Create NetAnim file", createAnim);
    cmd.AddValue("simulTime", "Total simulation time", simTime);
    cmd.AddValue("printLogs", "Enable logging", printLogs);
    cmd.AddValue("graph", "[0-3], Choose number of graph [Default: 0 (none)]", graphNumber);
    cmd.AddValue("distance", "Set distance between APs", distance);
    cmd.AddValue("helloInterval", "Interval of Hello packets [Default: 2000ms]", helloInterval);
    cmd.AddValue("uavSpeed", "Starting speed of UAV", uavSpeed);
    cmd.AddValue("pktSize", "Set size of packets", packetSize);

    return cmd;
}

// Create anim file, setup color and size of nodes 

AnimationInterface createAnimFile(NodeContainer apNodes, NodeContainer serverNodes, NodeContainer uavNodes) {
    AnimationInterface anim("anim.xml");
    // APs
    for (uint32_t i = 0; i < apNodes.GetN(); ++i) {
        anim.UpdateNodeColor(apNodes.Get(i)->GetId(), 0, 0, 255);
        anim.UpdateNodeDescription(apNodes.Get(i), "AP " + std::to_string(i + 1));
        anim.UpdateNodeSize(apNodes.Get(i)->GetId(), 2.0, 2.0);
    }
    // server
    anim.UpdateNodeColor(serverNodes.Get(0)->GetId(), 255, 0, 0);
    anim.UpdateNodeDescription(serverNodes.Get(0), "Server");
    anim.UpdateNodeSize(serverNodes.Get(0)->GetId(), 2.0, 2.0);

    // uav
    anim.UpdateNodeColor(uavNodes.Get(0)->GetId(), 0, 255, 0);
    anim.UpdateNodeDescription(uavNodes.Get(0), "UAV");
    anim.UpdateNodeSize(uavNodes.Get(0)->GetId(), 3.0, 3.0);

    anim.EnablePacketMetadata();

    return anim;
}

Gnuplot setupGraphAndData(int graphNumber) {
    Gnuplot graph("graf" + std::to_string(graphNumber) + ".svg");

    graph.SetTerminal("svg");
    switch (graphNumber) {
        case 1:
            graph.SetTitle("Porovnanie odoslanych paketov s prijatymi paketmi v case (5 spusteni)");
            graph.SetLegend("Cas [s]", "Priemerny pocet odoslanych/prijatych paketov ");

            data.SetTitle("Prijate pakety (Sink App)");
            errorBars.SetTitle("Smerodajna odchylka (Sink App)");

            data2.SetTitle("Odoslane pakety (UAV)");
            data2.SetStyle(Gnuplot2dDataset::LINES);

            errorBars2.SetTitle("Smerodajna odchylka (UAV)");
            errorBars2.SetStyle(Gnuplot2dDataset::POINTS);
            errorBars2.SetErrorBars(Gnuplot2dDataset::Y);

            break;
        case 2:
            graph.SetTitle("Pocet prijatych paketov vzhladom k intervalu Hello paketov");
            graph.SetLegend("Interval odosielania Hello paketov [ms]", "Pocet prijatych paketov za celu simulaciu (3x spustene)");
            data.SetTitle("Pocet prijatych paketov (Sink app)");
            errorBars.SetTitle("Smerodajna odchylka (Sink App)");
            break;
        case 3:
            graph.SetTitle("Pocet prijatych paketov vzhladom k rychlosti UAV");
            graph.SetLegend("Rychlost UAV", "Pocet prijatych paketov za celu simulaciu (5x spustene)");
            data.SetTitle("Pocet prijatych paketov");
            errorBars.SetTitle("Smerodajna odchylka ");
            break;
    }

    data.SetStyle(Gnuplot2dDataset::LINES);
    errorBars.SetStyle(Gnuplot2dDataset::POINTS);
    errorBars.SetErrorBars(Gnuplot2dDataset::Y);

    return graph;

}
