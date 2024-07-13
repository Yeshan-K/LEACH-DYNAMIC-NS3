#include "ns3/applications-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/enum.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/gnuplot.h"
#include "ns3/internet-module.h"
#include "ns3/li-ion-energy-source.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/okumura-hata-propagation-loss-model.h"
#include "ns3/point-to-point-module.h"
#include "ns3/vector.h"
#include "ns3/wifi-module.h"

#include <cfloat>
#include <cstdlib>
#include <iomanip>
#include <math.h>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;
using namespace std;

/*
cp proj/clientserver.cc scratch/main.cc && NS_LOG="LteCustom=level_info" ./ns3 run scratch/main.cc --cwd="/home/yeshan/Downloads/psc-ns3-psc-7.0/output_customCS"
*/

/*
python3 /home/yeshan/Downloads/psc-ns3-psc-7.0/src/flow-monitor/examples/flowmon-parse-results.py /home/yeshan/Downloads/psc-ns3-psc-7.0/output_customCS/flowmonitor.xml
*/

NS_LOG_COMPONENT_DEFINE("LteCustom");

double GetPolynomialModelResult(double x, double p0, double p1, double e);
double CalcRxBB(double RRx);
double CalcTxBB(double RTx);
double CalcRxRF(double SRx);
double CalcTxRF1(double TxRF1);
double CalcTxRF2(double TxRF2);

class EnergyLte : public Object
{
  public:
    // An Energy Source is created for every device
    Ptr<LiIonEnergySource> m_energySourceNode;
    Ptr<OkumuraHataPropagationLossModel> m_pathLossModel;

    EnergyLte();
    ~EnergyLte();

    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("ns3::EnergyLte").SetParent<Object>().AddConstructor<EnergyLte>();
        return tid;
    }

    // An LTE UE can be in transmit mode or reception mode
    int64_t m_time_last_tx; // Time since a transmission is observed
    int64_t m_time_last_rx; // Time since a reception is observed
    double initialEnergy;

    // Only downlink is enabled
    void Only_downlink_rx(double idleTime, double rxPower, Ptr<OutputStreamWrapper> stream);
    // Only uplink is enabled
    void Only_uplink_tx(double idleTime, double txPower, Ptr<OutputStreamWrapper> stream);
    // Decrease if the device is idle all time
    void Only_idle_decrease(double idleTime);
    // Decrease if the device is used simultaneous both uplink and downlink
    double both_downlink_and_uplink(double idleTime);
    // check remaining energy
};

EnergyLte::EnergyLte()
    : m_time_last_tx(4000),
      m_time_last_rx(0)
{
    m_energySourceNode = CreateObject<LiIonEnergySource>();
    m_energySourceNode->SetAttribute("InitialCellVoltage", DoubleValue(4.17));   // Qfull
    m_energySourceNode->SetAttribute("RatedCapacity", DoubleValue(2.33));        // Q
    m_energySourceNode->SetAttribute("NominalCellVoltage", DoubleValue(3.57));   // Vnom
    m_energySourceNode->SetAttribute("NomCapacity", DoubleValue(2.14));          // QNom
    m_energySourceNode->SetAttribute("ExpCellVoltage", DoubleValue(3.714));      // Vexp
    m_energySourceNode->SetAttribute("ExpCapacity", DoubleValue(1.74));          // Qexp
    m_energySourceNode->SetAttribute("InternalResistance", DoubleValue(0.0830)); // R
    m_energySourceNode->SetAttribute("TypCurrent", DoubleValue(0.466));          // i typical
    m_energySourceNode->SetAttribute("ThresholdVoltage", DoubleValue(3.0));      // End of charge.

    initialEnergy = m_energySourceNode->GetRemainingEnergy();

    m_pathLossModel = CreateObject<OkumuraHataPropagationLossModel>();
    m_pathLossModel->SetAttribute("CitySize", EnumValue(0));
    m_pathLossModel->SetAttribute(
        "Frequency",
        DoubleValue(LteSpectrumValueHelper::GetCarrierFrequency(uint32_t(5330))));
}

EnergyLte::~EnergyLte()
{
}

void
EnergyLte::Only_uplink_tx(double idleTime, double txPower, Ptr<OutputStreamWrapper> stream)
{
    // Transmit power = Pcon + PTx + PTxRF(STx) + PTxBB(RTx)
    double txPowerWatts;
    if (txPower >= 10)
    {
        txPowerWatts = 0.55 + CalcTxRF2(txPower) + CalcTxBB(6);
    }
    else
    {
        txPowerWatts = 0.55 + CalcTxRF1(txPower) + CalcTxBB(6);
    }

    Callback<void, double> only_tx;
    only_tx = MakeCallback(&LiIonEnergySource::DecreaseRemainingEnergy, m_energySourceNode);
    DoubleValue supplyVoltage;
    this->m_energySourceNode->GetAttribute("InitialCellVoltage", supplyVoltage);
    double energyDecreasedIdle = 1.53 * idleTime / 1000;
    // Energy to be decrease in joules
    double energyDecreasedTx = txPowerWatts * 0.001;
    m_time_last_tx = Simulator::Now().GetMilliSeconds();

    // NS_LOG_INFO("TX IDLE ENERGY DRAIN = " << energyDecreasedIdle
    //                                       << ", TX Packet Energy Drain = " << energyDecreasedTx);

    if (m_time_last_rx == m_time_last_tx)
    {
        only_tx(energyDecreasedTx);
        double dlUlEnergy = both_downlink_and_uplink(idleTime);
        // *stream->GetStream() << (energyDecreasedTx + dlUlEnergy) << "\t";
    }
    else
    {
        only_tx(energyDecreasedTx + energyDecreasedIdle);
        NS_LOG_DEBUG("Transmit chain is consuming energy: "
                     << (energyDecreasedTx + energyDecreasedIdle) << " joules");
        // *stream->GetStream() << (energyDecreasedTx + energyDecreasedIdle) << "\t";
    }
}

void
EnergyLte::Only_downlink_rx(double idleTime, double rxPower, Ptr<OutputStreamWrapper> stream)
{
    Callback<void, double> only_tx;
    only_tx = MakeCallback(&LiIonEnergySource::DecreaseRemainingEnergy, m_energySourceNode);

    double rxPowerWatts = 0.55 + CalcRxRF(rxPower);

    double energyDecreasedBB = 512 * CalcRxBB(6) / 1000000;
    double supplyVoltage = 5.0;
    double energyDecreasedIdle = 1.53 * idleTime / 1000;
    double energyDecreasedRx = rxPowerWatts * 0.001 + energyDecreasedBB;

    // NS_LOG_INFO("RX IDLE ENERGY DRAIN = " << energyDecreasedIdle
    //                                       << ", RX Packet Energy Drain = " << energyDecreasedRx);

    m_time_last_rx = Simulator::Now().GetMilliSeconds();

    if (m_time_last_rx == m_time_last_tx)
    {
        only_tx(energyDecreasedRx);
        double dlUlEnergy = both_downlink_and_uplink(idleTime);
        // *stream->GetStream() << (energyDecreasedRx + dlUlEnergy) << "\t";
    }
    else
    {
        only_tx(energyDecreasedRx + energyDecreasedIdle);
        NS_LOG_DEBUG("Receive chain is consuming energy"
                     << (energyDecreasedRx + energyDecreasedIdle) << " joules");
        // *stream->GetStream() << (energyDecreasedRx + energyDecreasedIdle) << "\t";
    }
}

void
EnergyLte::Only_idle_decrease(double idleTime)
{
    Callback<void, double> only_tx;
    only_tx = MakeCallback(&LiIonEnergySource::DecreaseRemainingEnergy, m_energySourceNode);

    double supplyVoltage = 5.0;
    double energyDecreasedIdle = 1.53 * idleTime / 1000000000;
    only_tx(energyDecreasedIdle);
}

double
EnergyLte::both_downlink_and_uplink(double idleTime)
{
    Callback<void, double> only_tx;
    only_tx = MakeCallback(&LiIonEnergySource::DecreaseRemainingEnergy, m_energySourceNode);
    double supplyVoltage = 5.0;
    double energyDecreasedIdle = (1.53 + 0.16) * 0.001;
    only_tx(energyDecreasedIdle);
    return energyDecreasedIdle;
    NS_LOG_DEBUG("Both Transmit and Receive chain is consuming power ");
}

class LeachNode : public Object
{
  public:
    LeachNode(){};

    LeachNode(int32_t umberOfCompletedPeriods,
              int32_t expectedPeriods,
              int32_t isClusterHead,
              int32_t lastRound)
    {
        this->umberOfCompletedPeriods = umberOfCompletedPeriods;
        this->expectedPeriods = expectedPeriods;
        this->isClusterHead = isClusterHead;
        this->lastRound = lastRound;
        this->destNode = NULL;
        this->energyTracker = CreateObject<EnergyLte>();
    };

    virtual ~LeachNode(){};

    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("ns3::LeachNode").SetParent<Object>().AddConstructor<LeachNode>();
        return tid;
    }

    void setUmberOfCompletedPeriods(int32_t umberOfCompletedPeriods)
    {
        this->umberOfCompletedPeriods = umberOfCompletedPeriods;
    }

    void setExpectedPeriods(int32_t expectedPeriods)
    {
        this->expectedPeriods = expectedPeriods;
    }

    void setIsClusterHead(int32_t isClusterHead)
    {
        this->isClusterHead = isClusterHead;
    }

    void setLastRound(int32_t lastRound)
    {
        this->lastRound = lastRound;
    }

    void setDestNode(Ptr<Node> node)
    {
        this->destNode = node;
    }

    int32_t getUmberOfCompletedPeriods()
    {
        return this->umberOfCompletedPeriods;
    }

    int32_t getExpectedPeriods()
    {
        return this->expectedPeriods;
    }

    int32_t getIsClusterHead()
    {
        return this->isClusterHead;
    }

    int32_t getLastRound()
    {
        return this->lastRound;
    }

    Ptr<Node> getDestNode()
    {
        return this->destNode;
    }

    Ptr<EnergyLte> getEnergyTracker()
    {
        return this->energyTracker;
    }

    void setClusterId(uint32_t clusterId)
    {
        this->clusterId = clusterId;
    }

    uint32_t getClusterId()
    {
        return this->clusterId;
    }

  private:
    int32_t umberOfCompletedPeriods; // number of periods completed
    int32_t expectedPeriods;         // expected number of periods node should stay alive for
    int32_t isClusterHead;           // boolean for clusterHead or not
    int32_t lastRound;               // last round where node acted as cluster head
    Ptr<Node> destNode;
    Ptr<EnergyLte> energyTracker;
    uint32_t clusterId;
};

//
// Function Declarations
//

void setEnergyLevels(Ptr<Node> node,
                     Ptr<Node> destNode,
                     Ptr<OutputStreamWrapper> stream,
                     std::string context);
double calculateAverageEnergy(NodeContainer ueNodes);
void UePacketTrace(Ptr<OutputStreamWrapper> stream,
                   Ptr<Node> txUeNodePtr,
                   Ptr<Node> rxUeNodePtr,
                   std::string context,
                   Ptr<const Packet> p,
                   const Address& srcAddrs,
                   const Address& dstAddrs);
void GenerateTopologyPlotFile(NodeContainer enbNode, NodeContainer allUeNodes, double gridWidth);
void GenerateTopologyPlotFileWithCluster(NodeContainer enbNode,
                                         NodeContainer allUeNodes,
                                         double gridWidth);
void TraceSinkPC5SignalingPacketTrace(Ptr<OutputStreamWrapper> stream,
                                      uint32_t srcL2Id,
                                      uint32_t dstL2Id,
                                      Ptr<Packet> p);
Ipv6Address FetchUeIp(Ptr<Node> node);
void ChangeUdpEchoClientRemote(Ptr<UdpClient> app, uint16_t port, Ipv6Address address);
Ptr<UdpClient> GetUdpEchoClientObject(Ptr<Node> remoteNode);
Ptr<UdpServer> GetUdpEchoServerObjectWithPort(Ptr<Node> remoteNode, uint32_t port);
void AddServerToNode(Ptr<Node> txNode,
                     Ptr<Node> rxNode,
                     Ptr<Node> enbNode,
                     NodeContainer allUeNodes,
                     uint32_t port,
                     Ptr<OutputStreamWrapper> packetOutputStream);
void SetClientAndServer(Ptr<Node> txNode,
                        Ptr<Node> rxNode,
                        Ptr<Node> enbNode,
                        NodeContainer ueNodes,
                        Ptr<OutputStreamWrapper> packetOutputStream);
double getDistance(Ptr<Node> a, Ptr<Node> b);
int electClusterHead(NodeContainer allUeNodes, NodeContainer enbNode, int32_t currentRound);
void selectClusterHead(Ptr<Node> clusterNode, NodeContainer allUeNodes);
void resetNodes(NodeContainer ueNodes);
void avgEnergyUpdate(NodeContainer ueNodes);
void Update(NodeContainer allNodes,
            Ptr<FlowMonitor> &flowMonitor,
            uint32_t currentRound,
            Ptr<OutputStreamWrapper> packetOutputStream);

/*
 * Main Function
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * Main Function
 */

int
main(int argc, char* argv[])
{
    srand(time(0));
    double simTime = 1.0; // s // Simulation time (in seconds) updated automatically based on
                          // number of remoteNodes
    uint32_t nRelayUes = 1;
    uint32_t nRemoteUesPerRelay = 19;
    uint32_t totalUeNodes = nRelayUes * nRemoteUesPerRelay + nRelayUes;
    bool remoteUesOoc = true;
    uint32_t totalRounds = 10;
    std::string echoServerNode("RemoteHost");
    double gridSquareWidth = 10.0;
    double gridWidth = sqrt(totalUeNodes) * gridSquareWidth + 50;
    double enbZ = 2000;
    double ueZ = 20;

    // LogComponentEnable("LteUeRrc", logLevel);
    // LogComponentEnable("LteSlUeRrc", logLevel);
    // LogComponentEnable("LteUeMac", logLevel);
    // LogComponentEnable("LteEnbRrc", logLevel);
    // LogComponentEnable("LteSlEnbRrc", logLevel);
    // LogComponentEnable("UdpEchoClient", LOG_LEVEL_INFO);

    // LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_DEBUG);
    // LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_DEBUG);

    double startTimeUes = 1.0;

    // Calculate simTime based on relay service starts and give 10 s of traffic for the last one
    simTime = 3.0 + startTimeUes - 0.100 + 30 * (totalRounds + 1); // s
    NS_LOG_INFO("Simulation time = " << simTime << " s");

    NS_LOG_INFO("Configuring default parameters...");

    // Configure the UE for UE_SELECTED scenario
    Config::SetDefault("ns3::LteUeMac::SlGrantMcs", UintegerValue(16));
    Config::SetDefault("ns3::LteUeMac::SlGrantSize",
                       UintegerValue(100)); // The number of RBs allocated per UE for Sidelink
    Config::SetDefault("ns3::LteUeMac::Ktrp", UintegerValue(1));
    Config::SetDefault("ns3::LteUeMac::UseSetTrp", BooleanValue(false));
    Config::SetDefault("ns3::LteUeMac::SlScheduler",
                       StringValue("Random")); // Values include Fixed, Random, MinPrb, MaxCoverage

    // Set the frequency
    Config::SetDefault("ns3::LteEnbNetDevice::DlEarfcn", UintegerValue(5330));
    Config::SetDefault("ns3::LteUeNetDevice::DlEarfcn", UintegerValue(5330));
    Config::SetDefault("ns3::LteEnbNetDevice::UlEarfcn", UintegerValue(23330));
    Config::SetDefault("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(100));
    Config::SetDefault("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(100));

    // Reduce frequency of CQI report to allow for sidelink transmissions
    Config::SetDefault("ns3::LteUePhy::DownlinkCqiPeriodicity", TimeValue(MilliSeconds(79)));

    // Increase SRS periodicity to allow larger number of UEs in the system
    Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(320));

    // Set the UEs power in dBm
    Config::SetDefault("ns3::LteUePhy::TxPower", DoubleValue(23.0));
    // Set the eNBs power in dBm
    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(46.0));

    // Create the lte helper
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();

    // Create and    set the EPC helper
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    // Create Sidelink helper and set lteHelper
    Ptr<LteSidelinkHelper> proseHelper = CreateObject<LteSidelinkHelper>();
    proseHelper->SetLteHelper(lteHelper);

    // Connect Sidelink controller and Sidelink Helper
    Config::SetDefault("ns3::LteSlBasicUeController::ProseHelper", PointerValue(proseHelper));

    // Set pathloss model
    // lteHelper->SetAttribute("PathlossModel",
    // StringValue("ns3::OkumuraHataPropagationLossModel"));

    // Enable Sidelink
    lteHelper->SetAttribute("UseSidelink", BooleanValue(true));

    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // Create a single RemoteHost
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // Create the Internet
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("500Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    // Create remoteNodes (eNb + UEs)
    NodeContainer enbNode;
    enbNode.Create(1);
    Ptr<EnergyLte> enbNodeEnergy = CreateObject<EnergyLte>();
    enbNode.Get(0)->AggregateObject(enbNodeEnergy);

    NS_LOG_INFO("eNb remoteNode id = [" << enbNode.Get(0)->GetId() << "]");

    NodeContainer relayUeNodes;
    relayUeNodes.Create(nRelayUes);
    for (uint32_t relayIdx = 0; relayIdx < relayUeNodes.GetN(); relayIdx++)
    {
        NS_LOG_INFO("aggregated to relayIdx = " << relayIdx);
        relayUeNodes.Get(relayIdx)->AggregateObject(CreateObject<LeachNode>(0, 0, 0, 0));
        // relayUeNodes.Get(relayIdx)->AggregateObject(CreateObject<PacketLossCounter>(16));
    }

    NodeContainer remoteUeNodes;
    remoteUeNodes.Create(nRelayUes * nRemoteUesPerRelay);

    for (uint32_t remoteIdx = 0; remoteIdx < remoteUeNodes.GetN(); remoteIdx++)
    {
        NS_LOG_INFO("aggregated to remoteIdx = " << remoteIdx);
        remoteUeNodes.Get(remoteIdx)->AggregateObject(CreateObject<LeachNode>(0, 0, 0, 0));
        // remoteUeNodes.Get(remoteIdx)->AggregateObject(CreateObject<PacketLossCounter>(16));
    }

    for (uint32_t ry = 0; ry < relayUeNodes.GetN(); ry++)
    {
        NS_LOG_INFO("Relay UE " << ry + 1 << " remoteNode id = [" << relayUeNodes.Get(ry)->GetId()
                                << "]");
    }

    for (uint32_t rm = 0; rm < remoteUeNodes.GetN(); rm++)
    {
        NS_LOG_INFO("Remote UE " << rm + 1 << " remoteNode id = [" << remoteUeNodes.Get(rm)->GetId()
                                 << "]");
    }

    NodeContainer allUeNodes = NodeContainer(relayUeNodes, remoteUeNodes);

    // Position of the remoteNodes
    // eNodeB
    // Ptr<ListPositionAllocator> positionAllocEnb = CreateObject<ListPositionAllocator>();
    // positionAllocEnb->Add(Vector(0.0, 0.0, 30.0));

    // // UEs
    // Ptr<ListPositionAllocator> positionAllocRelays = CreateObject<ListPositionAllocator>();
    // Ptr<ListPositionAllocator> positionAllocRemotes = CreateObject<ListPositionAllocator>();
    // for (uint32_t ry = 0; ry < relayUeNodes.GetN(); ++ry)
    // {
    //     // Relay UE
    //     double ry_angle = ry * (360.0 / relayUeNodes.GetN()); // degrees
    //     double ry_pos_x = std::floor(relayRadius * std::cos(ry_angle * M_PI / 180.0));
    //     double ry_pos_y = std::floor(relayRadius * std::sin(ry_angle * M_PI / 180.0));

    //     positionAllocRelays->Add(Vector(ry_pos_x, ry_pos_y, 1.5));

    //     NS_LOG_INFO("Relay UE " << ry + 1 << " remoteNode id = [" <<
    //     relayUeNodes.Get(ry)->GetId()
    //                             << "]"
    //                                " x "
    //                             << ry_pos_x << " y " << ry_pos_y);
    //     // Remote UEs
    //     for (uint32_t rm = 0; rm < nRemoteUesPerRelay; ++rm)
    //     {
    //         double rm_angle = rm * (360.0 / nRemoteUesPerRelay); // degrees
    //         double rm_pos_x =
    //             std::floor(ry_pos_x + remoteRadius * std::cos(rm_angle * M_PI / 180.0));
    //         double rm_pos_y =
    //             std::floor(ry_pos_y + remoteRadius * std::sin(rm_angle * M_PI / 180.0));

    //         positionAllocRemotes->Add(Vector(rm_pos_x, rm_pos_y, 1.5));

    //         uint32_t remoteIdx = ry * nRemoteUesPerRelay + rm;
    //         NS_LOG_INFO("Remote UE " << remoteIdx << " remoteNode id = ["
    //                                  << remoteUeNodes.Get(remoteIdx)->GetId()
    //                                  << "]"
    //                                     " x "
    //                                  << rm_pos_x << " y " << rm_pos_y);
    //     }
    // }

    // Install mobility
    // eNodeB
    MobilityHelper mobilityeNodeB;
    mobilityeNodeB.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityeNodeB.SetPositionAllocator("ns3::GridPositionAllocator",
                                        "MinX",
                                        DoubleValue(0),
                                        "MinY",
                                        DoubleValue(0),
                                        "DeltaX",
                                        DoubleValue(5.0),
                                        "DeltaY",
                                        DoubleValue(5.0),
                                        "GridWidth",
                                        UintegerValue(3),
                                        "Z",
                                        DoubleValue(enbZ),
                                        "LayoutType",
                                        StringValue("RowFirst"));
    mobilityeNodeB.Install(enbNode);

    // to enb = 0.0037
    // to ue = 0.0021

    MobilityHelper mobilityUe;
    mobilityUe.SetPositionAllocator("ns3::GridPositionAllocator",
                                    "MinX",
                                    DoubleValue(-20.0),
                                    "MinY",
                                    DoubleValue(-20.0),
                                    "DeltaX",
                                    DoubleValue(20.0),
                                    "DeltaY",
                                    DoubleValue(20.0),
                                    "GridWidth",
                                    UintegerValue(10),
                                    "Z",
                                    DoubleValue(ueZ),
                                    "LayoutType",
                                    StringValue("RowFirst"));
    mobilityUe.SetMobilityModel(
        "ns3::RandomDirection2dMobilityModel",
        "Bounds",
        RectangleValue(Rectangle(-1 * gridWidth, gridWidth, -1 * gridWidth, gridWidth)));

    mobilityUe.Install(relayUeNodes);
    mobilityUe.Install(remoteUeNodes);

    Vector originPos = enbNode.Get(0)->GetObject<MobilityModel>()->GetPosition();
    uint32_t ringLevel = 0;
    uint32_t ringUeIdx = 0;
    while (ringUeIdx < allUeNodes.GetN())
    {
        double curX = originPos.x + (-1 * gridSquareWidth * (ringLevel + 1));
        double curY = originPos.y + gridSquareWidth * (ringLevel + 1);
        for (uint32_t xtop = 0; xtop < (ringLevel * 2 + 1) + 1; xtop++)
        {
            if (ringUeIdx >= allUeNodes.GetN())
            {
                break;
            }
            allUeNodes.Get(ringUeIdx++)
                ->GetObject<MobilityModel>()
                ->SetPosition(Vector(curX, curY, ueZ));
            curX += gridSquareWidth;
        }

        for (uint32_t yright = 0; yright < (ringLevel * 2 + 1) + 1; yright++)
        {
            if (ringUeIdx >= allUeNodes.GetN())
            {
                break;
            }
            allUeNodes.Get(ringUeIdx++)
                ->GetObject<MobilityModel>()
                ->SetPosition(Vector(curX, curY, ueZ));
            curY -= gridSquareWidth;
        }

        for (uint32_t xbottom = 0; xbottom < (ringLevel * 2 + 1) + 1; xbottom++)
        {
            if (ringUeIdx >= allUeNodes.GetN())
            {
                break;
            }
            allUeNodes.Get(ringUeIdx++)
                ->GetObject<MobilityModel>()
                ->SetPosition(Vector(curX, curY, ueZ));
            curX -= gridSquareWidth;
        }

        for (uint32_t yleft = 0; yleft < (ringLevel * 2 + 1) + 1; yleft++)
        {
            if (ringUeIdx >= allUeNodes.GetN())
            {
                break;
            }
            allUeNodes.Get(ringUeIdx++)
                ->GetObject<MobilityModel>()
                ->SetPosition(Vector3D(curX, curY, ueZ));
            curY += gridSquareWidth;
        }

        ringLevel++;
    }

    // Generate gnuplot file with the script to generate the topology plot
    GenerateTopologyPlotFile(enbNode, allUeNodes, gridWidth);
    // Install LTE devices to the remoteNodes
    NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(enbNode);
    NetDeviceContainer relayUeDevs = lteHelper->InstallUeDevice(relayUeNodes);
    NetDeviceContainer remoteUeDevs = lteHelper->InstallUeDevice(remoteUeNodes);
    NetDeviceContainer allUeDevs = NetDeviceContainer(relayUeDevs, remoteUeDevs);

    // Configure eNodeB for Sidelink
    Ptr<LteSlEnbRrc> enbSidelinkConfiguration = CreateObject<LteSlEnbRrc>();
    enbSidelinkConfiguration->SetSlEnabled(true);

    //-Configure Sidelink communication pool
    enbSidelinkConfiguration->SetDefaultPool(
        proseHelper->GetDefaultSlCommTxResourcesSetupUeSelected());
    enbSidelinkConfiguration->SetDiscEnabled(true);

    //-Configure Sidelink discovery pool
    enbSidelinkConfiguration->AddDiscPool(
        proseHelper->GetDefaultSlDiscTxResourcesSetupUeSelected());

    //-Configure UE-to-Network Relay parameters
    enbSidelinkConfiguration->SetDiscConfigRelay(proseHelper->GetDefaultSib19DiscConfigRelay());

    // Install eNodeB Sidelink configuration on the eNodeB devices
    lteHelper->InstallSidelinkConfiguration(enbDevs, enbSidelinkConfiguration);

    // Preconfigure UEs for Sidelink
    Ptr<LteSlUeRrc> ueSidelinkConfiguration = CreateObject<LteSlUeRrc>();
    ueSidelinkConfiguration->SetSlEnabled(true);
    //-Configure Sidelink preconfiguration
    // Relay UEs: Empty configuration
    LteRrcSap::SlPreconfiguration preconfigurationRelay;
    // Remote UEs: Empty configuration if in-coverage
    //             Custom configuration (see below) if out-of-coverage
    LteRrcSap::SlPreconfiguration preconfigurationRemote;

    if (true || remoteUesOoc)
    {
        // Configure general preconfiguration parameters
        preconfigurationRemote.preconfigGeneral.carrierFreq =
            enbDevs.Get(0)->GetObject<LteEnbNetDevice>()->GetUlEarfcn();
        preconfigurationRemote.preconfigGeneral.slBandwidth =
            enbDevs.Get(0)->GetObject<LteEnbNetDevice>()->GetUlBandwidth();

        //-Configure preconfigured Sidelink communication pool
        preconfigurationRemote.preconfigComm = proseHelper->GetDefaultSlPreconfigCommPoolList();

        //-Configure preconfigured Sidelink discovery pool
        preconfigurationRemote.preconfigDisc = proseHelper->GetDefaultSlPreconfigDiscPoolList();

        //-Configure preconfigured UE-to-Network Relay parameters
        preconfigurationRemote.preconfigRelay = proseHelper->GetDefaultSlPreconfigRelay();
    }

    //-Enable Sidelink discovery
    ueSidelinkConfiguration->SetDiscEnabled(true);
    //-Set frequency for Sidelink discovery messages monitoring
    ueSidelinkConfiguration->SetDiscInterFreq(
        enbDevs.Get(0)->GetObject<LteEnbNetDevice>()->GetUlEarfcn());

    // Install UE Sidelink configuration on the UE devices with the corresponding
    // preconfiguration ueSidelinkConfiguration->SetSlPreconfiguration(preconfigurationRelay);
    // lteHelper->InstallSidelinkConfiguration(relayUeDevs, ueSidelinkConfiguration);

    // ueSidelinkConfiguration->SetSlPreconfiguration(preconfigurationRemote);
    // lteHelper->InstallSidelinkConfiguration(remoteUeDevs, ueSidelinkConfiguration);

    ueSidelinkConfiguration->SetSlPreconfiguration(preconfigurationRemote);
    lteHelper->InstallSidelinkConfiguration(allUeDevs, ueSidelinkConfiguration);

    // Install the IP stack on the UEs and assign network IP addresses

    internet.Install(relayUeNodes);
    internet.Install(remoteUeNodes);
    Ipv6InterfaceContainer ueIpIfaceRelays;
    Ipv6InterfaceContainer ueIpIfaceRemotes;
    ueIpIfaceRelays = epcHelper->AssignUeIpv6Address(relayUeDevs);
    ueIpIfaceRemotes = epcHelper->AssignUeIpv6Address(remoteUeDevs);

    // Set the default gateway for the UEs
    Ipv6StaticRoutingHelper Ipv6RoutingHelper;
    for (uint32_t u = 0; u < allUeNodes.GetN(); ++u)
    {
        Ptr<Node> ueNode = allUeNodes.Get(u);
        Ptr<Ipv6StaticRouting> ueStaticRouting =
            Ipv6RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv6>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress6(), 1);
    }

    for (uint32_t u = 0; u < relayUeNodes.GetN(); ++u)
    {
        NS_LOG_INFO("Relay Node " << u << " " << ueIpIfaceRelays.GetAddress(u, 1));
    }

    for (uint32_t u = 0; u < remoteUeNodes.GetN(); ++u)
    {
        NS_LOG_INFO("Remote Node " << u << " " << ueIpIfaceRemotes.GetAddress(u, 1));
    }

    // Configure IP for the remoteNodes in the Internet (PGW and RemoteHost)
    Ipv6AddressHelper ipv6h;
    ipv6h.SetBase(Ipv6Address("6001:db80::"), Ipv6Prefix(64));
    Ipv6InterfaceContainer internetIpIfaces = ipv6h.Assign(internetDevices);
    internetIpIfaces.SetForwarding(0, true);
    internetIpIfaces.SetDefaultRouteInAllNodes(0);

    // Set route for the Remote Host to join the LTE network remoteNodes
    Ipv6StaticRoutingHelper ipv6RoutingHelper;
    Ptr<Ipv6StaticRouting> remoteHostStaticRouting =
        ipv6RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv6>());
    remoteHostStaticRouting
        ->AddNetworkRouteTo("7777:f000::", Ipv6Prefix(60), internetIpIfaces.GetAddress(0, 1), 1, 0);

    proseHelper->SetIpv6BaseForRelayCommunication("7777:f00e::", Ipv6Prefix(48));

    uint32_t groupL2Address = 255;
    Ipv6Address groupAddress6("7777:f000::"); // use multicast address as destination
    Ptr<LteSlTft> slTft = Create<LteSlTft>(LteSlTft::BIDIRECTIONAL, groupAddress6, groupL2Address);

    // Attach Relay UEs to the eNB
    lteHelper->Attach(allUeDevs);

    // If the Remote UEs are not OOC attach them to the eNodeB as well
    if (false && !remoteUesOoc)
    {
        lteHelper->Attach(remoteUeDevs);
    }

    ///*** Configure applications ***///
    // For each Remote UE, we have a pair (UpdEchoClient, UdpEchoServer)
    // Each Remote UE has an assigned port
    // UdpEchoClient installed in the Remote UE, sending to the echoServerAddr
    // and to the corresponding Remote UE port
    // UdpEchoServer installed in the echoServerNode, listening to the
    // corresponding Remote UE port

    Ipv6Address echoServerAddr;
    if (false && echoServerNode == "RemoteHost")
    {
        echoServerAddr = internetIpIfaces.GetAddress(1, 1);
    }
    else if (true || echoServerNode == "RemoteUE")
    {
        // We use a dummy IP address for initial configuration as we don't know the
        // IP address of the 'Remote UE (0)' before it connects to the Relay UE
        echoServerAddr = Ipv6Address::GetOnes();
    }
    Ipv6Address relayEchoServerAddr = internetIpIfaces.GetAddress(1, 1);

    uint16_t echoPortBase = 50000;
    ApplicationContainer serverApps;
    ApplicationContainer clientApps;
    AsciiTraceHelper ascii;

    std::ostringstream oss;
    Ptr<OutputStreamWrapper> packetOutputStream = ascii.CreateFileStream("AppPacketTrace.txt");
    *packetOutputStream->GetStream()
        << "time(sec)\tposition\ttx/rx\tC/S\tNodeID\tIP[src]\tIP[dst]\tPktSize(bytes)\tEnergy "
           "Remaining\tEnergy Drained by Packet"
        << std::endl;

    Ptr<OutputStreamWrapper> energyOutputStream = ascii.CreateFileStream("AverageEnergyTrace.txt");
    *energyOutputStream->GetStream() << "time(sec)\tAverage Energy(J)\t" << std::endl;

    for (uint32_t remUeIdx = 0; remUeIdx < remoteUeNodes.GetN(); remUeIdx++)
    {
        NS_LOG_INFO("Loop Remote UeIdx = " << remUeIdx);
        uint32_t destIdx = remUeIdx / nRemoteUesPerRelay;
        uint16_t remUePort = echoPortBase + remUeIdx;

        UdpClientHelper echoClientHelper(echoServerAddr);
        echoClientHelper.SetAttribute("MaxPackets", UintegerValue(1500000));
        echoClientHelper.SetAttribute("Interval", TimeValue(Seconds(0.001)));
        echoClientHelper.SetAttribute("PacketSize", UintegerValue(32));
        echoClientHelper.SetAttribute("RemotePort", UintegerValue(remUePort));
        ApplicationContainer singleClientApp =
            echoClientHelper.Install(remoteUeNodes.Get(remUeIdx));
        // Start the application 3.0 s after the remote UE started the relay service
        // normally it should be enough time to connect
        singleClientApp.Start(Seconds(3.0 + startTimeUes));
        // Stop the application after 10.0 s
        singleClientApp.Stop(Seconds(simTime));

        // Tracing packets on the UdpEchoClient (C)
        oss << "tx\tC\t" << remoteUeNodes.Get(remUeIdx)->GetId();
        singleClientApp.Get(0)->TraceConnect("TxWithAddresses",
                                             oss.str(),
                                             MakeBoundCallback(&UePacketTrace,
                                                               packetOutputStream,
                                                               remoteUeNodes.Get(remUeIdx),
                                                               relayUeNodes.Get(destIdx)));

        oss.str("");
        oss << "rx\tC\t" << remoteUeNodes.Get(remUeIdx)->GetId();
        singleClientApp.Get(0)->TraceConnect("RxWithAddresses",
                                             oss.str(),
                                             MakeBoundCallback(&UePacketTrace,
                                                               packetOutputStream,
                                                               remoteUeNodes.Get(remUeIdx),
                                                               relayUeNodes.Get(destIdx)));

        oss.str("");

        clientApps.Add(singleClientApp);

        SetClientAndServer(remoteUeNodes.Get(remUeIdx),
                           relayUeNodes.Get(destIdx),
                           enbNode.Get(0),
                           allUeNodes,
                           packetOutputStream);
    }

    NodeContainer allNodes = NodeContainer(pgw, remoteHost, enbNode, relayUeNodes, remoteUeNodes);
    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> flowMonitor = flowHelper.Install(allNodes);

    for (uint32_t roundIdx = 0; roundIdx < totalRounds; roundIdx++)
    {
        NS_LOG_INFO("Round " << roundIdx);

        Simulator::Schedule(Seconds(3.0 + startTimeUes - 0.100 + 30 * (roundIdx + 1)),
                            &Update,
                            allNodes,
                            flowMonitor,
                            roundIdx,
                            packetOutputStream);

    }

    for (double time = 5.0; time < simTime; time = time + 5.0)
    {
        Simulator::Schedule(Seconds(time), &avgEnergyUpdate, allUeNodes);
    }

    for (uint32_t relayUeIdx = 0; relayUeIdx < relayUeNodes.GetN(); relayUeIdx++)
    {
        NS_LOG_INFO("Loop Relay UeIdx = " << relayUeIdx);
        uint16_t relayUePort = echoPortBase + relayUeIdx + 1000;
        uint32_t echoServerNodeId = 0;
        // 32 bytes in a packet from Remote - 20 in log
        // 64 bytes in relay - 52 in log
        // UdpEchoClient in the Remote UE
        UdpClientHelper echoClientHelper(relayEchoServerAddr);
        echoClientHelper.SetAttribute("MaxPackets", UintegerValue(1500000));
        echoClientHelper.SetAttribute("Interval", TimeValue(Seconds(0.001)));
        echoClientHelper.SetAttribute("PacketSize", UintegerValue(32));
        echoClientHelper.SetAttribute("RemotePort", UintegerValue(relayUePort));

        ApplicationContainer singleClientApp =
            echoClientHelper.Install(relayUeNodes.Get(relayUeIdx));
        // Start the application 3.0 s after the remote UE started the relay service
        // normally it should be enough time to connect
        singleClientApp.Start(Seconds(3.0 + startTimeUes));
        // Stop the application after 10.0 s
        // singleClientApp.Stop(Seconds(simTime));

        // Tracing packets on the UdpEchoClient (C)
        oss << "tx\tC\t" << relayUeNodes.Get(relayUeIdx)->GetId();
        singleClientApp.Get(0)->TraceConnect("TxWithAddresses",
                                             oss.str(),
                                             MakeBoundCallback(&UePacketTrace,
                                                               packetOutputStream,
                                                               relayUeNodes.Get(relayUeIdx),
                                                               enbNode.Get(0)));
        oss.str("");

        oss << "rx\tC\t" << relayUeNodes.Get(relayUeIdx)->GetId();
        singleClientApp.Get(0)->TraceConnect("RxWithAddresses",
                                             oss.str(),
                                             MakeBoundCallback(&UePacketTrace,
                                                               packetOutputStream,
                                                               relayUeNodes.Get(relayUeIdx),
                                                               enbNode.Get(0)));
        oss.str("");

        clientApps.Add(singleClientApp);

        SetClientAndServer(relayUeNodes.Get(relayUeIdx),
                           remoteHost,
                           enbNode.Get(0),
                           allUeNodes,
                           packetOutputStream);
    }

    proseHelper->ActivateSidelinkBearer(Seconds(3), allUeDevs, slTft);

    ///*** Configure Relaying ***///

    // Setup dedicated bearer for the All (changed from Relay) UEs
    Ptr<EpcTft> tft = Create<EpcTft>();
    EpcTft::PacketFilter dlpf;
    dlpf.localIpv6Address = proseHelper->GetIpv6NetworkForRelayCommunication();
    dlpf.localIpv6Prefix = proseHelper->GetIpv6PrefixForRelayCommunication();
    tft->Add(dlpf);
    EpsBearer bearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
    lteHelper->ActivateDedicatedEpsBearer(allUeDevs, bearer, tft);

    // Tracing PC5 signaling messages
    Ptr<OutputStreamWrapper> PC5SignalingPacketTraceStream =
        ascii.CreateFileStream("PC5SignalingPacketTrace.txt");
    *PC5SignalingPacketTraceStream->GetStream() << "time(s)\ttxId\tRxId\tmsgType" << std::endl;

    for (uint32_t ueDevIdx = 0; ueDevIdx < relayUeDevs.GetN(); ueDevIdx++)
    {
        Ptr<LteUeRrc> rrc = relayUeDevs.Get(ueDevIdx)->GetObject<LteUeNetDevice>()->GetRrc();
        PointerValue ptrOne;
        rrc->GetAttribute("SidelinkConfiguration", ptrOne);
        Ptr<LteSlUeRrc> slUeRrc = ptrOne.Get<LteSlUeRrc>();
        slUeRrc->TraceConnectWithoutContext(
            "PC5SignalingPacketTrace",
            MakeBoundCallback(&TraceSinkPC5SignalingPacketTrace, PC5SignalingPacketTraceStream));
    }
    for (uint32_t ueDevIdx = 0; ueDevIdx < remoteUeDevs.GetN(); ueDevIdx++)
    {
        Ptr<LteUeRrc> rrc = remoteUeDevs.Get(ueDevIdx)->GetObject<LteUeNetDevice>()->GetRrc();
        PointerValue ptrOne;
        rrc->GetAttribute("SidelinkConfiguration", ptrOne);
        Ptr<LteSlUeRrc> slUeRrc = ptrOne.Get<LteSlUeRrc>();
        slUeRrc->TraceConnectWithoutContext(
            "PC5SignalingPacketTrace",
            MakeBoundCallback(&TraceSinkPC5SignalingPacketTrace, PC5SignalingPacketTraceStream));
    }

    lteHelper->EnablePdcpTraces();

    NS_LOG_INFO("Simulation time " << simTime << " s");
    NS_LOG_INFO("Starting simulation...");

    // NodeContainer allFmNodes = NodeContainer(allNodes, remoteHostContainer, pgw);
    // FlowMonitorHelper flowHelper;
    // Ptr<FlowMonitor> flowMonitor = flowHelper.Install(allFmNodes);
    NS_LOG_INFO("Probes: ");
    NS_LOG_INFO(flowMonitor->GetAllProbes().size());
    NS_LOG_INFO(ns3::NodeList::GetNNodes());

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    flowMonitor->SerializeToXmlFile("flowmonitor.xml", true, true);

    Simulator::Destroy();
    return 0;
}

//
//
//

void
setEnergyLevels(Ptr<Node> node,
                Ptr<Node> destNode,
                Ptr<OutputStreamWrapper> stream,
                std::string context)
{
    // NS_LOG_INFO(" " << node->GetId() << " " << destNode->GetId());
    double minimumRxPower = -40;
    // NS_LOG_INFO("Distance: " << node->GetObject<MobilityModel>()->GetDistanceFrom(
    //                 destNode->GetObject<MobilityModel>()));
    // NS_LOG_INFO(
    //     "Path Loss: " <<
    //     node->GetObject<LeachNode>()->getEnergyTracker()->m_pathLossModel->GetLoss(
    //         node->GetObject<MobilityModel>(),
    //         destNode->GetObject<MobilityModel>()));
    // NS_LOG_INFO("Current energy of node " << node->GetId() << ": "
    //                                       << node->GetObject<LeachNode>()
    //                                              ->getEnergyTracker()
    //                                              ->m_energySourceNode->GetRemainingEnergy());

    // NS_LOG_INFO(node->GetDevice(0)->GetObject<LteUeNetDevice>()->GetRrc()->GetState());
    *stream->GetStream() << std::setprecision(8) << std::fixed
                         << node->GetObject<LeachNode>()
                                ->getEnergyTracker()
                                ->m_energySourceNode->GetRemainingEnergy()
                         << "\t";

    /// Minimum Received Signal Strength Indicator = -100 dBm
    // Get needed TxPower from RxPower, inverse of CalcRxPower
    // CalcRxPower() {return txPower - GetLoss()}
    // txPower = rxPower + GetLoss()
    // CalcTxPower() {return rxPower + GetLoss()}
    double txPower =
        minimumRxPower + node->GetObject<LeachNode>()->getEnergyTracker()->m_pathLossModel->GetLoss(
                             node->GetObject<MobilityModel>(),
                             destNode->GetObject<MobilityModel>());

    if (txPower > 23.0)
    {
        NS_LOG_DEBUG("txPower exceeded 23.0");
    }

    txPower = std::min(23.0, txPower);
    double rxPower = node->GetObject<LeachNode>()->getEnergyTracker()->m_pathLossModel->CalcRxPower(
        txPower,
        node->GetObject<MobilityModel>(),
        destNode->GetObject<MobilityModel>());

    NS_LOG_DEBUG("Rx Received Power: " << rxPower);

    if (context[0] == 'r')
    {
        double idleTime = Simulator::Now().GetMilliSeconds() -
                          max(node->GetObject<LeachNode>()->getEnergyTracker()->m_time_last_tx,
                              node->GetObject<LeachNode>()->getEnergyTracker()->m_time_last_rx);

        node->GetObject<LeachNode>()->getEnergyTracker()->Only_downlink_rx(idleTime,
                                                                           rxPower,
                                                                           stream);
    }
    else if (context[0] == 't')
    {
        double idleTime = Simulator::Now().GetMilliSeconds() -
                          max(node->GetObject<LeachNode>()->getEnergyTracker()->m_time_last_tx,
                              node->GetObject<LeachNode>()->getEnergyTracker()->m_time_last_rx);
        node->GetObject<LeachNode>()->getEnergyTracker()->Only_uplink_tx(idleTime, txPower, stream);
    }
}

double
calculateAverageEnergy(NodeContainer ueNodes)
{
    double initialSum = 0, currentSum = 0;
    for (int32_t nodeIdx = 0; nodeIdx < ueNodes.GetN(); nodeIdx++)
    {
        currentSum += ueNodes.Get(nodeIdx)
                          ->GetObject<LeachNode>()
                          ->getEnergyTracker()
                          ->m_energySourceNode->GetRemainingEnergy();
        // initialSum +=
        //     ueNodes.Get(nodeIdx)->GetObject<LeachNode>()->getEnergyTracker()->initialEnergy;
    }

    return currentSum / ueNodes.GetN();
}

/*
 * Trace sink function for logging when a packet is transmitted or received
 * at the application layer
 */
void
UePacketTrace(Ptr<OutputStreamWrapper> stream,
              Ptr<Node> txUeNodePtr,
              Ptr<Node> rxUeNodePtr,
              std::string context,
              Ptr<const Packet> p,
              const Address& srcAddrs,
              const Address& dstAddrs)
{
    std::ostringstream oss;
    // stream->GetStream()->precision(6);

    // *stream->GetStream() << Simulator::Now().GetNanoSeconds() / (double)1e9 << "\t"
    //                      << txUeNodePtr->GetObject<MobilityModel>()->GetPosition() << "\t"
    //                      << context << "\t" <<
    //                      Inet6SocketAddress::ConvertFrom(srcAddrs).GetIpv6()
    //                      << ":" << Inet6SocketAddress::ConvertFrom(srcAddrs).GetPort() << "\t"
    //                      << Inet6SocketAddress::ConvertFrom(dstAddrs).GetIpv6() << ":"
    //                      << Inet6SocketAddress::ConvertFrom(dstAddrs).GetPort() << "\t"
    //                      << p->GetSize() << "\t";

    setEnergyLevels(txUeNodePtr,
                    txUeNodePtr->GetObject<LeachNode>()->getDestNode(),
                    stream,
                    context);
    // *stream->GetStream() << std::endl;
}

/**
 * Function that generates a gnuplot script file that can be used to plot the
 * topology of the scenario access network (eNBs, Relay UEs and Remote UEs)
 */

void
GenerateTopologyPlotFile(NodeContainer enbNode, NodeContainer allUeNodes, double gridWidth)
{
    std::string fileNameWithNoExtension = "topology";
    std::string graphicsFileName = fileNameWithNoExtension + ".png";
    std::string gnuplotFileName = fileNameWithNoExtension + ".plt";
    std::string plotTitle = "Topology (Labels = Node IDs)";

    Gnuplot plot(graphicsFileName);
    plot.SetTitle(plotTitle);
    plot.SetTerminal("png size 1280,1024");
    plot.SetLegend("X", "Y"); // These are the axis, not the legend
    std::ostringstream plotExtras;
    int range = gridWidth;
    plotExtras << "set xrange [-" << range << ":+" << range << "]" << std::endl;
    plotExtras << "set yrange [-" << range << ":+" << range << "]" << std::endl;
    plotExtras << "set linetype 1 pt 3 ps 2 " << std::endl;
    plotExtras << "set linetype 2 lc rgb \"green\" pt 2 ps 2" << std::endl;
    plotExtras << "set linetype 3 pt 1 ps 2" << std::endl;
    plot.AppendExtra(plotExtras.str());

    // eNB
    Gnuplot2dDataset datasetEremoteNodeB;
    datasetEremoteNodeB.SetTitle("eNodeB");
    datasetEremoteNodeB.SetStyle(Gnuplot2dDataset::POINTS);

    double x = enbNode.Get(0)->GetObject<MobilityModel>()->GetPosition().x;
    double y = enbNode.Get(0)->GetObject<MobilityModel>()->GetPosition().y;
    std::ostringstream strForLabel;
    strForLabel << "set label \"" << enbNode.Get(0)->GetId() << "\" at " << x << "," << y
                << " textcolor rgb \"grey\" center front offset 0,1";
    plot.AppendExtra(strForLabel.str());
    datasetEremoteNodeB.Add(x, y);
    plot.AddDataset(datasetEremoteNodeB);

    // Relay UEs
    Gnuplot2dDataset datasetRelays;
    datasetRelays.SetTitle("Relay UEs");
    datasetRelays.SetStyle(Gnuplot2dDataset::POINTS);
    for (uint32_t ry = 0; ry < allUeNodes.GetN(); ry++)
    {
        double x = allUeNodes.Get(ry)->GetObject<MobilityModel>()->GetPosition().x;
        double y = allUeNodes.Get(ry)->GetObject<MobilityModel>()->GetPosition().y;
        std::ostringstream strForLabel;
        strForLabel << "set label \"" << allUeNodes.Get(ry)->GetId() << "\" at " << x << "," << y
                    << " textcolor rgb \"grey\" center front offset 0,1";
        plot.AppendExtra(strForLabel.str());
        datasetRelays.Add(x, y);
    }
    plot.AddDataset(datasetRelays);

    std::ofstream plotFile(gnuplotFileName.c_str());
    plot.GenerateOutput(plotFile);
    plotFile.close();
}

void
GenerateTopologyPlotFileWithCluster(NodeContainer enbNode,
                                    NodeContainer allUeNodes,
                                    double gridWidth)
{
    std::string fileNameWithNoExtension = "topology_" + to_string(Simulator::Now().GetSeconds());
    std::string graphicsFileName = fileNameWithNoExtension + ".png";
    std::string gnuplotFileName = fileNameWithNoExtension + ".plt";
    std::string plotTitle = "Topology (Labels = Node IDs)";

    Gnuplot plot(graphicsFileName);
    plot.SetTitle(plotTitle);
    plot.SetTerminal("png size 1280,1024");
    plot.SetLegend("X", "Y"); // These are the axis, not the legend
    std::ostringstream plotExtras;
    int range = gridWidth;
    plotExtras << "set xrange [-" << range << ":+" << range << "]" << std::endl;
    plotExtras << "set yrange [-" << range << ":+" << range << "]" << std::endl;
    plotExtras << "set linetype 1 pt 3 ps 2 " << std::endl;
    plotExtras << "set linetype 2 lc rgb \"green\" pt 2 ps 2" << std::endl;
    plotExtras << "set linetype 3 pt 1 ps 2" << std::endl;
    plot.AppendExtra(plotExtras.str());

    // eNB
    Gnuplot2dDataset datasetEremoteNodeB;
    datasetEremoteNodeB.SetTitle("eNodeB");
    datasetEremoteNodeB.SetStyle(Gnuplot2dDataset::POINTS);

    double x = enbNode.Get(0)->GetObject<MobilityModel>()->GetPosition().x;
    double y = enbNode.Get(0)->GetObject<MobilityModel>()->GetPosition().y;
    std::ostringstream strForLabel;
    strForLabel << "set label \"" << enbNode.Get(0)->GetId() << "\" at " << x << "," << y
                << " textcolor rgb \"grey\" center front offset 0,1";
    plot.AppendExtra(strForLabel.str());
    datasetEremoteNodeB.Add(x, y);
    plot.AddDataset(datasetEremoteNodeB);

    // all UEs
    Gnuplot2dDataset datasetRelays;
    datasetRelays.SetTitle("All UEs");
    datasetRelays.SetStyle(Gnuplot2dDataset::POINTS);
    for (uint32_t ry = 0; ry < allUeNodes.GetN(); ry++)
    {
        double x = allUeNodes.Get(ry)->GetObject<MobilityModel>()->GetPosition().x;
        double y = allUeNodes.Get(ry)->GetObject<MobilityModel>()->GetPosition().y;
        std::ostringstream strForLabel;
        strForLabel << "set label \"" << allUeNodes.Get(ry)->GetId()
                    << " in: " << allUeNodes.Get(ry)->GetObject<LeachNode>()->getClusterId()
                    << "\" at " << x << "," << y
                    << " textcolor rgb \"grey\" center front offset 0,1";
        plot.AppendExtra(strForLabel.str());
        datasetRelays.Add(x, y);
    }
    plot.AddDataset(datasetRelays);

    std::ofstream plotFile(gnuplotFileName.c_str());
    plot.GenerateOutput(plotFile);
    plotFile.close();
}

/*
 * Trace sink function for logging when PC5 signaling messages are received
 */
void
TraceSinkPC5SignalingPacketTrace(Ptr<OutputStreamWrapper> stream,
                                 uint32_t srcL2Id,
                                 uint32_t dstL2Id,
                                 Ptr<Packet> p)
{
    LteSlPc5SignallingMessageType lpc5smt;
    p->PeekHeader(lpc5smt);
    *stream->GetStream() << Simulator::Now().GetSeconds() << "\t" << srcL2Id << "\t" << dstL2Id
                         << "\t" << lpc5smt.GetMessageName() << std::endl;
}

Ipv6Address
FetchUeIp(Ptr<Node> node)
{
    return node->GetObject<Ipv6>()->GetAddress(1, 1).GetAddress();
}

void
ChangeUdpEchoClientRemote(Ptr<UdpClient> app, uint16_t port, Ipv6Address address)
{
    NS_LOG_INFO("" << Simulator::Now().GetNanoSeconds() / 1e9 << " Node id = ["
                   << app->GetNode()->GetId() << "] changed the UdpEchoClient Remote Ip Address to "
                   << address);
    app->SetRemote(address, port);
}

// Gets the UdpEchoClient Object from the first Application with UdpEchoClient Object, installed
// into remoteNode.

Ptr<UdpClient>
GetUdpEchoClientObject(Ptr<Node> remoteNode)
{
    for (uint32_t appIdx = 0; appIdx < remoteNode->GetNApplications(); appIdx++)
    {
        if (remoteNode->GetApplication(appIdx)->GetObject<UdpClient>())
        {
            return remoteNode->GetApplication(appIdx)->GetObject<UdpClient>();
        }
    }
    return NULL;
}

// Gets the UdpEchoServer Object with the port number aggregated to an Application installed into
// remoteNode.

Ptr<UdpServer>
GetUdpEchoServerObjectWithPort(Ptr<Node> remoteNode, uint32_t port)
{
    for (uint32_t appIdx = 0; appIdx < remoteNode->GetNApplications(); appIdx++)
    {
        if (remoteNode->GetApplication(appIdx)->GetObject<UdpServer>())
        {
            Ptr<UdpServer> udpEchoServer =
                remoteNode->GetApplication(appIdx)->GetObject<UdpServer>();

            UintegerValue serverPort;
            udpEchoServer->GetAttribute("Port", serverPort);
            if (port == serverPort.Get())
            {
                return udpEchoServer;
            }
        }
    }

    return nullptr;
}

// Installs UdpEchoClient Server that listens on port = [50000 + Transmitter Node Id] and sets up
// tracing. rxNode is where the server is installed txNode is where the client will be

void
AddServerToNode(Ptr<Node> txNode,
                Ptr<Node> rxNode,
                Ptr<Node> enbNode,
                NodeContainer allUeNodes,
                uint32_t port,
                Ptr<OutputStreamWrapper> packetOutputStream)
{
    if (GetUdpEchoServerObjectWithPort(rxNode, port))
    {
        return;
    }

    UdpServerHelper echoServerHelper(port);
    ApplicationContainer singleServerApp;

    singleServerApp.Add(echoServerHelper.Install(rxNode));
    uint32_t echoServerNodeId = rxNode->GetId();

    singleServerApp.Start(Simulator::Now() + Seconds(0.1));

    std::ostringstream oss;

    NS_LOG_INFO(" " << txNode->GetId() << " " << rxNode->GetId());

    Ptr<Node> traceRxNode = rxNode;
    if (rxNode->GetId() == 3)
    {
        traceRxNode = enbNode;
    }

    NodeContainer* txRxPtrPair = new NodeContainer(txNode);
    txRxPtrPair->Add(traceRxNode);

    NS_LOG_INFO("main check: " << txRxPtrPair->Get(0)->GetId());

    oss << "rx\tS\t" << echoServerNodeId;
    singleServerApp.Get(0)->TraceConnect(
        "RxWithAddresses",
        oss.str(),
        MakeBoundCallback(&UePacketTrace, packetOutputStream, txNode, traceRxNode));
    oss.str("");

    //

    oss << "tx\tS\t" << echoServerNodeId;
    singleServerApp.Get(0)->TraceConnect(
        "TxWithAddresses",
        oss.str(),
        MakeBoundCallback(&UePacketTrace, packetOutputStream, txNode, traceRxNode));
    oss.str("");

    //
}

// If receiver node is remoteHost, enbNode is used for position tracing of the destination.

void
SetClientAndServer(Ptr<Node> txNode,
                   Ptr<Node> rxNode,
                   Ptr<Node> enbNode,
                   NodeContainer ueNodes,
                   Ptr<OutputStreamWrapper> packetOutputStream)
{
    // Add receiver node as destnode to leachNode object aggregated to tx Node
    NS_LOG_INFO("Setting client and server for " << txNode->GetId() << " to " << rxNode->GetId());

    if (rxNode->GetId() == 3)
    {
        txNode->GetObject<LeachNode>()->setDestNode(enbNode);
    }
    else
    {
        txNode->GetObject<LeachNode>()->setDestNode(rxNode);
    }

    uint32_t echoPortBase = 50000;
    uint32_t port = echoPortBase + txNode->GetId();
    AddServerToNode(txNode, rxNode, enbNode, ueNodes, port, packetOutputStream);
    ChangeUdpEchoClientRemote(GetUdpEchoClientObject(txNode), port, FetchUeIp(rxNode));
    // rxNode->GetObject<PacketLossCounter>()->NotifyReceived();
}

double
getDistance(Ptr<Node> a, Ptr<Node> b)
{
    return a->GetObject<MobilityModel>()->GetDistanceFrom(b->GetObject<MobilityModel>());
}

int
electClusterHead(NodeContainer allUeNodes, NodeContainer enbNode, int32_t currentRound)
{
    NS_LOG_INFO("Current Round: " << currentRound);
    double clusterPercent = 0.25;
    double roundGap = 1 / clusterPercent;
    double roundThreshold =
        clusterPercent / 1 - clusterPercent * (currentRound % 1 / clusterPercent);
    NS_LOG_INFO("round threshold = " << roundThreshold);
    int clusterHeadCount = 0;

    std::vector<uint32_t> idxArr(allUeNodes.GetN()); // 0, 1, 2, 3, 4, 5, 7, . . N
    for (uint32_t i = 0; i < allUeNodes.GetN(); i++)
    {
        idxArr[i] = i;
    }

    std::sort(idxArr.begin(), idxArr.end(), [allUeNodes, enbNode](uint32_t x, uint32_t y) {
        if (getDistance(allUeNodes.Get(x), enbNode.Get(0)) <
            getDistance(allUeNodes.Get(y), enbNode.Get(0)))
        {
            return true;
        }
        else
        {
            return false;
        }
    });

    NS_LOG_INFO("TIME = " << Simulator::Now().GetSeconds());

    for (uint32_t ueIdx = 0; ueIdx < allUeNodes.GetN(); ueIdx++)
    {
        NS_LOG_INFO("Current Node being checked: " << allUeNodes.Get(idxArr[ueIdx])->GetId());
        if (clusterHeadCount > clusterPercent * allUeNodes.GetN())
        {
            NS_LOG_INFO("breaking");
            break;
        }
        if (allUeNodes.Get(idxArr[ueIdx])->GetObject<LeachNode>()->getLastRound() <=
                (currentRound - roundGap) ||
            (currentRound - roundGap < 0))
        {
            NS_LOG_INFO(allUeNodes.Get(idxArr[ueIdx])->GetId() << " Passed first check");
            double randomNumber = .00001 * (rand() % 100000);
            NS_LOG_INFO("random number = " << randomNumber);
            if (randomNumber <= roundThreshold)
            {
                NS_LOG_INFO(allUeNodes.Get(idxArr[ueIdx])->GetId()
                            << allUeNodes.Get(idxArr[ueIdx])->GetId() << "Passed second check");
                allUeNodes.Get(idxArr[ueIdx])->GetObject<LeachNode>()->setIsClusterHead(1);
                allUeNodes.Get(idxArr[ueIdx])->GetObject<LeachNode>()->setLastRound(currentRound);
                allUeNodes.Get(idxArr[ueIdx])
                    ->GetObject<LeachNode>()
                    ->setClusterId(++clusterHeadCount);

                NS_LOG_INFO(
                    "Node id "
                    << allUeNodes.Get(idxArr[ueIdx])->GetId()
                    << " has been elected relay node and given cluster ID "
                    << allUeNodes.Get(idxArr[ueIdx])->GetObject<LeachNode>()->getClusterId());
            }
        }
    }
    if (clusterHeadCount > 0)
        return 1;
    else
        return 0;
}

void
selectClusterHead(Ptr<Node> clusterNode, NodeContainer allUeNodes)
{
    Ptr<Node> minDistance;

    for (uint32_t i = 0; i < allUeNodes.GetN(); i++)
    {
        if (allUeNodes.Get(i)->GetObject<LeachNode>()->getIsClusterHead())
        {
            minDistance = allUeNodes.Get(i);
            break;
        }
    }

    for (uint32_t ueIdx = 1; ueIdx < allUeNodes.GetN(); ueIdx++)
    {
        if (allUeNodes.Get(ueIdx)->GetObject<LeachNode>()->getIsClusterHead())
        {
            if (getDistance(minDistance, clusterNode) >
                getDistance(allUeNodes.Get(ueIdx), clusterNode))
            {
                minDistance = allUeNodes.Get(ueIdx);
            }
        }
    }

    clusterNode->GetObject<LeachNode>()->setClusterId(
        minDistance->GetObject<LeachNode>()->getClusterId());

    NS_LOG_INFO("Node id " << clusterNode->GetId() << " has selected the node  "
                           << minDistance->GetId() << " with cluster ID "
                           << minDistance->GetObject<LeachNode>()->getClusterId());
}

void
resetNodes(NodeContainer ueNodes)
{
    for (uint32_t ueIdx = 0; ueIdx < ueNodes.GetN(); ueIdx++)
    {
        ueNodes.Get(ueIdx)->GetObject<LeachNode>()->setIsClusterHead(0);
        ueNodes.Get(ueIdx)->GetObject<LeachNode>()->setClusterId(-1);
    }
}

void
avgEnergyUpdate(NodeContainer ueNodes)
{
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> energyOutputStream =
        ascii.CreateFileStream("AverageEnergyTrace.txt", ios::app);

    double relEnergy = calculateAverageEnergy(ueNodes);
    *energyOutputStream->GetStream()
        << std::setprecision(8) << std::fixed << Simulator::Now().GetNanoSeconds() / 1e9 << "\t"
        << relEnergy << "\t" << std::endl;
}

// Update(allNodes, RemoteHost)

void
Update(NodeContainer allNodes,
       Ptr<FlowMonitor> &flowMonitor,
       uint32_t currentRound,
       Ptr<OutputStreamWrapper> packetOutputStream)
{
    // record flowmonitor stats into xml file
       NS_LOG_INFO("" << Simulator::Now().GetNanoSeconds() / 1e9 << " Starting Update");

    NS_LOG_INFO("DEREFING ZERO POINTER BELOW: ");
    flowMonitor->SerializeToXmlFile("flowmonitor_" + to_string(currentRound - 2) + ".xml", true, true);
    NS_LOG_INFO("DEREFING ZERO POINTER ABOVE: ");


    Ptr<Node> pgw = allNodes.Get(0);
    Ptr<Node> remoteHost = allNodes.Get(1);
    NodeContainer enbNode = NodeContainer(allNodes.Get(2));
    NodeContainer ueNodes;
    for (uint32_t i = 3; i < allNodes.GetN(); i++)
    {
        ueNodes.Add(allNodes.Get(i));
    }

    resetNodes(ueNodes);
    while (1)
    {
        if (electClusterHead(ueNodes, enbNode, currentRound))
            break;
    }

    for (uint32_t remoteIdx = 0; remoteIdx < ueNodes.GetN(); remoteIdx++)
    {
        if (!ueNodes.Get(remoteIdx)->GetObject<LeachNode>()->getIsClusterHead())
        {
            selectClusterHead(ueNodes.Get(remoteIdx), ueNodes);
        }
    }

    for (uint32_t relayIdx = 0; relayIdx < ueNodes.GetN(); relayIdx++)
    {
        if (ueNodes.Get(relayIdx)->GetObject<LeachNode>()->getIsClusterHead())
        {
            for (uint32_t remoteIdx = 0; remoteIdx < ueNodes.GetN(); remoteIdx++)
            {
                if ((ueNodes.Get(remoteIdx)->GetObject<LeachNode>()->getClusterId() ==
                     ueNodes.Get(relayIdx)->GetObject<LeachNode>()->getClusterId()) &&
                    (relayIdx != remoteIdx))
                {
                    NS_LOG_INFO("Node id "
                                << ueNodes.Get(remoteIdx)->GetId()
                                << "     has joined the cluster of node  "
                                << ueNodes.Get(relayIdx)->GetId() << " with cluster ID "
                                << ueNodes.Get(relayIdx)->GetObject<LeachNode>()->getClusterId());
                    SetClientAndServer(ueNodes.Get(remoteIdx),
                                       ueNodes.Get(relayIdx),
                                       enbNode.Get(0),
                                       ueNodes,
                                       packetOutputStream);
                }
            }

            SetClientAndServer(ueNodes.Get(relayIdx),
                               remoteHost,
                               enbNode.Get(0),
                               ueNodes,
                               packetOutputStream);
        }
    }

    GenerateTopologyPlotFileWithCluster(enbNode, ueNodes, sqrt(ueNodes.GetN()) * 10.0 + 50);

    NS_LOG_INFO("HERE 1: ");
    FlowMonitorHelper flowHelperNew;
    flowMonitor = flowHelperNew.Install(allNodes);
    NS_LOG_INFO("HERE 2: ");

    // SetClientAndServer(ueNodes.Get(2), ueNodes.Get(1), enbNode.Get(0));
    // SetClientAndServer(ueNodes.Get(3), ueNodes.Get(0), enbNode.Get(0));
}

// y = p0 + p1*x + e
double
GetPolynomialModelResult(double x, double p0, double p1, double e)
{
    return p0 + p1 * x + e;
}

// RxBB x = RRx[Mbit/s] 1923 2.89 0.08
double
CalcRxBB(double RRx)
{
    return 0.001 * GetPolynomialModelResult(RRx, 1923, 2.89, 0.08);
}

// TxBB x = RTx[Mbit/s] 2110 0.87 0.01
double
CalcTxBB(double RTx)
{
    return 0.001 * GetPolynomialModelResult(RTx, 2110, 0.87, 0.01);
}

// RxRF x = SRx[dBm] 1889 -1.11 0.06
double
CalcRxRF(double SRx)
{
    return 0.001 * GetPolynomialModelResult(SRx, 1889, -1.11, 0.06);
}

// TxRF1 x = STx[dBm] 2004 5.50 0.60
double
CalcTxRF1(double TxRF1)
{
    return 0.001 * GetPolynomialModelResult(TxRF1, 2004, 5.50, 0.60);
}

// TxRF2 x = STx[dBm] 1132 117 6.87
double
CalcTxRF2(double TxRF2)
{
    return 0.001 * GetPolynomialModelResult(TxRF2, 1132, 117, 6.87);
}
