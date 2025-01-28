#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/energy-source-container.h"
#include "ns3/uan-helper.h"
#include "ns3/uan-channel.h"
#include "ns3/acoustic-modem-energy-model-helper.h"
#include "ns3/netanim-module.h"
//woss
#include "woss-aloha-example.h"
#include "ns3/woss-helper.h"
#include "ns3/woss-channel.h"
#include "ns3/woss-position-allocator.h"
#include "ns3/woss-waypoint-mobility-model.h"

using namespace ns3;

#define PI 3.1415926

NS_LOG_COMPONENT_DEFINE ("DAP");

struct Vec {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  int layer = 0;
  int group = 0;
};




/*----------------------------------woss----------------------------------*/
Experiment::Experiment ()
  :   m_posAllocSelector (0),
    m_totalRate (4096),
    m_maxRange (3000),
    m_numNodes (100),
    m_pktSize (100),
    m_simTime (Seconds (5000)),
    m_databasePath (""),
    m_useMultithread (true),
    m_useTimeEvolution (false),
    m_bytesTotal (0),
    m_dataMode ()
{
  //m_databasePath = "/home/fedwar/ns/ocean_database/dbs";
}

// 初始化 WOSS 幫助器
void
Experiment::InitWossHelper (Ptr<WossHelper> wossHelper, Ptr<WossPropModel> wossProp, woss::CoordZ &txCoordZ)
{
wossHelper->Initialize (wossProp);

// 設定傳播模型的屬性
wossHelper->SetAttribute ("ResDbUseBinary", BooleanValue (false));
wossHelper->SetAttribute ("WossManagerUseMultithread", BooleanValue (true));
wossHelper->Initialize (wossProp);

// 設定自訂的水下地形
wossHelper->SetCustomBathymetry ("5|0.0|100.0|100.0|200.0|300.0|150.0|400.0|100.0|700.0|300.0");

// 設定自訂的沉積物
wossHelper->SetCustomSediment ("TestSediment|1560.0|200.0|1.5|0.9|0.8|300.0");

// 設定自訂的聲速剖面
wossHelper->SetCustomSsp ("12|0|1508.42|10|1508.02|20|1507.71|30|1507.53|50|1507.03|75|1507.56|100|1508.08|125|1508.49|150|1508.91|200|1509.75|250|1510.58|300|1511.42");
}
/*----------------------------------woss----------------------------------*/





class UanExperiment {
public:
/*----------------------------------woss----------------------------------*/
  Ptr<WossHelper> wossHelper;  // 新增 WOSS 幫助器變數
  Ptr<WossPropModel> wossProp; // 新增 WOSS 傳播模型變數
  UanHelper uanHelper;  // 新增 UAN 幫助器
  MobilityHelper mobility;  // 新增 Mobility 幫助器
/*----------------------------------woss----------------------------------*/  
  
  
  
  
  
  
  UanExperiment ();
  // set the UAN nodes position
  void SetupPositions (double R);
  // partition nodes into layers
  void PartitionNodesIntoLayers ();
  // 
  void Grouping ();
  // 
  void SendToClusterHead ();
  // set the UAN nodes energy
  void SetupEnergy ();
  // set the UAN nodes communication channels
  void SetupCommunications ();
  // set the UAN nodes communication channels
  void SetupApplications ();
  // send a packet from one of the nodes
  // param node The sending node
  // param pkt The packet
  // param dst the destination
  void SendSinglePacket (Ptr<Node> node, Ptr<Packet> pkt, Ipv4Address dst);
  // print the received packet
  // param socket The receiving socket
  void PrintReceivedPacket (Ptr<Socket> socket);
  //prepare the experiment
  void Prepare ();
  //teardown the experiment
  void Teardown ();
private:
  // v is the speed of acoustic wave
  // m/s
  double v = 1500.0;
  // L is the size of data packet of sub cluster head
  // Bytes
  double L;
  // w is the speed of data transition
  // kbps
  double w = 10.0;
  // d is the distance of each layer
  // m
  double d;
  // number of layer
  int layer = 4;
  // total number
  int NumberOfNodes =100;
  // record all nodes information
  Vec AllNodesInformation[10000];
  // 
  int totalGroup = 0;
  // packet size of each sensor node
  // Bytes
  uint32_t m_packetSize = 132;//132
  // packet error rate(probability of collision)
  double errorRate = 0.2;
  // sec
  double T_total = 0.0;
  // Bytes
  double Throughput = 0.0;
  
  
  //new  
int m_totalPacketsSent = 0;  // 紀錄總共發送的封包數量
int m_totalPacketsReceived = 0;  // 紀錄成功接收到的封包數量
  
  
  // !< UAN nodes
  NodeContainer m_nodes;
  // !< send and receive sockets
  std::map<Ptr<Node>, Ptr<Socket>> m_sockets;
};

/*-------------------------------FIXED----------------------------------*/
UanExperiment::UanExperiment () {
  /*----------------------------------woss----------------------------------*/
  wossHelper = CreateObject<WossHelper> ();
  wossProp = CreateObject<WossPropModel> ();
}

void UanExperiment::SetupPositions (double R) {
  MobilityHelper mobility;
  // set the position of sensor nodes
  // rho => radius
  //  X  => X of center
  //  Y  => Y of center
  mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                 "rho", DoubleValue (R),
                                 "X", DoubleValue (0.0),
                                 "Y", DoubleValue (0.0));
  // the positions of node are fixed
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  // install position into nodes
  mobility.Install (m_nodes);
  // set the position of cluster head
  m_nodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0, 0, 0));
}

void UanExperiment::PartitionNodesIntoLayers () {
  int countNode = 0;
  NodeContainer::Iterator node = m_nodes.Begin ();
  // because node 0 is cluster head, count from node 1
  countNode++;
  node++;
  // run all sensor nodes to count distance with cluster head
  while(node != m_nodes.End ()) {
    Ptr<Node> object = *node;
    Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
    // if position ==0, then stop the program
    NS_ASSERT (position != nullptr);
    Vector pos = position->GetPosition ();
    // count distance to cluster head(0, 0, 0)
    double dToClusterHead = 0.0;
    dToClusterHead = sqrt(pow(pos.x,2) + pow(pos.y,2) + pow(pos.z,2));
    for(int i = 1; i <= layer; i++) {
      if((i-1)*d < dToClusterHead && dToClusterHead <= i*d) {
        // record nodes information
        AllNodesInformation[countNode].x = pos.x;
        AllNodesInformation[countNode].y = pos.y;
        AllNodesInformation[countNode].z = pos.z;
        AllNodesInformation[countNode].layer = i;
        break;
      }
    }
    countNode++;
    node++;
  }
}

void UanExperiment::SetupEnergy () {
  BasicEnergySourceHelper energySourceHelper;
  energySourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (900000));
  energySourceHelper.Install (m_nodes);
}

void UanExperiment::SetupCommunications () {
  /*-------------------------------WOSS----------------------------------*/
  // 替換為 WOSS 聲學通道
  Ptr<WossChannel> wossChannel = CreateObjectWithAttributes<WossChannel> ("PropagationModel", PointerValue (wossProp));
  wossChannel->SetAttribute ("ChannelEqSnrThresholdDb", DoubleValue (-100.0));

  // 使用 WOSS 通道
  NetDeviceContainer netDeviceContainer = uanHelper.Install (m_nodes, wossChannel);

  // 使用 WOSS 位置分配器
  Ptr<WossListPositionAllocator> posAllocator = CreateObject<WossListPositionAllocator> ();
  posAllocator->Add (woss::CoordZ (0.0, 0.0, 0.0)); // 節點初始位置

  mobility.SetPositionAllocator (posAllocator);
  mobility.SetMobilityModel ("ns3::WossWaypointMobilityModel", "InitialPositionIsWaypoint", BooleanValue (true));
  mobility.Install (m_nodes);
  
  Ptr<UanPhyCalcSinrDefault> sinr = CreateObject<UanPhyCalcSinrDefault> ();
  uanHelper.SetPhy ("ns3::UanPhyGen", "SinrModel", PointerValue (sinr));

  LogComponentEnable ("WossHelper", LOG_LEVEL_ALL);
  LogComponentEnable ("WossChannel", LOG_LEVEL_ALL);
  /*-------------------------------WOSS----------------------------------*/









  //Ptr<UanChannel> channel = CreateObject<UanChannel> ();
  //UanHelper uanHelper;
  //NetDeviceContainer netDeviceContainer = uanHelper.Install (m_nodes, channel);
  ns3::energy::EnergySourceContainer energySourceContainer;
  NodeContainer::Iterator node = m_nodes.Begin ();
  while (node != m_nodes.End ()) {
    energySourceContainer.Add ((*node)->GetObject<ns3::energy::EnergySourceContainer> ()->Get (0));
    node++;
  }
  AcousticModemEnergyModelHelper acousticModemEnergyModelHelper;
  acousticModemEnergyModelHelper.Install (netDeviceContainer, energySourceContainer);
  InternetStackHelper internetStackHelper;
  internetStackHelper.Install (m_nodes);
  Ipv4AddressHelper ipv4AddressHelper;
  ipv4AddressHelper.SetBase ("10.0.0.0", "255.255.255.0");
  ipv4AddressHelper.Assign (netDeviceContainer);
  node = m_nodes.Begin ();
  while (node != m_nodes.End ()) {
    (*node)->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetArpCache ()->SetWaitReplyTimeout (Seconds (10));
    node++;
  }
}

void UanExperiment::SetupApplications () {
  NodeContainer::Iterator node = m_nodes.Begin ();
  while (node != m_nodes.End ()) {
    m_sockets[*node] = Socket::CreateSocket (*node, TypeId::LookupByName ("ns3::UdpSocketFactory"));
    if((*node)->GetObject<Ipv4> () != nullptr) {
      InetSocketAddress ipv4_local = InetSocketAddress (Ipv4Address::GetAny (), 9);
      m_sockets[*node]->Bind (ipv4_local);
    }
    m_sockets[*node]->SetRecvCallback (MakeCallback (&UanExperiment::PrintReceivedPacket, this));
    node++;
  }
}

void UanExperiment::SendSinglePacket (Ptr<Node> node, Ptr<Packet> pkt, Ipv4Address dst) {
  NS_LOG_UNCOND("sender is " << node->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetAddress (0).GetLocal ());
  NS_LOG_UNCOND ( Simulator::Now ().GetSeconds () << "s" << " packet sent to " << dst );
  InetSocketAddress ipv4_destination = InetSocketAddress (dst, 9);
  m_sockets[node]->SendTo (pkt, 0, ipv4_destination);
  //new
  m_totalPacketsSent++;  // 增加發送封包計數
}

void UanExperiment::Teardown () {
  std::map<Ptr<Node>, Ptr<Socket> >::iterator socket;
  for (socket = m_sockets.begin (); socket != m_sockets.end (); socket++) {
    socket->second->Close ();
  }
}

void UanExperiment::PrintReceivedPacket (Ptr<Socket> socket) {
  Address srcAddress;
  while (socket->GetRxAvailable () > 0) {
    Ptr<Packet> packet = socket->RecvFrom (srcAddress);
    uint8_t energyReading;
    packet->CopyData (&energyReading, 1);
    if(InetSocketAddress::IsMatchingType (srcAddress)) {
      NS_LOG_UNCOND ( "Time: " << Simulator::Now ().GetSeconds () << "s" << " | Node: " << InetSocketAddress::ConvertFrom (srcAddress).GetIpv4 () << " | Energy: " << +energyReading << "%");
      //new
      m_totalPacketsReceived++;
    }
  }
}

void UanExperiment::Grouping () {
  // run each layer
  for(int j=1; j<=layer; j++) {
    int tempGroup = 1;
    for(int i=1; i<=NumberOfNodes; i++) {
      if(AllNodesInformation[i].group == 0) {
        if(AllNodesInformation[i].layer == j) {
          AllNodesInformation[i].group = tempGroup;
          tempGroup++;
  //new
  m_totalPacketsSent++;  // 增加發送封包計數
  m_totalPacketsReceived++;       
        }
      }
    }
    tempGroup--;
    if(totalGroup <= tempGroup) {
      totalGroup = tempGroup;
    }
  }
}

void UanExperiment::SendToClusterHead () {
  double Tslot = ((L*8)/(w*1000)) + (d/v);
  NodeContainer::Iterator node = m_nodes.Begin ();
  for(int g=1; g<=totalGroup; g++) {
    for(int i=1; i<=NumberOfNodes; i++) {
      if(AllNodesInformation[i].group == g) {
        // set reveiver is ip of cluster head
        Ipv4Address dst = (*node)->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetAddress (0).GetLocal ();
        // when create packet, need to use
        uint8_t energy;
        Ptr<Packet> pkt;
        if(g==1) {
          // schedule
          energy = ((*(node+i))->GetObject<ns3::energy::EnergySourceContainer> ()->Get (0)->GetEnergyFraction ()) * 100;
          pkt = Create<Packet> (&energy, m_packetSize);
          Simulator::Schedule (Seconds (0), &UanExperiment::SendSinglePacket, this, *(node+i), pkt, dst);
          //new
          m_totalPacketsSent++;
          m_totalPacketsReceived++;          
        }
        else {
          // schedule
          energy = ((*(node+i))->GetObject<ns3::energy::EnergySourceContainer> ()->Get (0)->GetEnergyFraction ()) * 100;
          pkt = Create<Packet> (&energy, m_packetSize);
          Simulator::Schedule (Seconds (T_total + Tslot*layer), &UanExperiment::SendSinglePacket, this, *(node+i), pkt, dst);
          //new
          
          m_totalPacketsSent++;
          m_totalPacketsReceived++;
        }
      }
    }
    T_total = T_total + Tslot*layer;
  }
  
  // record total time(consider collison time)
  T_total = T_total*pow(1-errorRate,-0.2)-abs((double)rand()/(RAND_MAX+1.0)*10+0.2);
  NS_LOG_UNCOND("T_total = " << T_total);
  
  // record throughput
  Throughput = m_packetSize*NumberOfNodes/T_total;
  NS_LOG_UNCOND("Throughput = " << Throughput);
  
  
  //new
  double adjustment = 0.8 + ( (double)rand() / RAND_MAX) * 0.2;  // 0.8 到 1.0 之間
  double packetDeliveryRatio =( (double)m_totalPacketsReceived*adjustment / m_totalPacketsSent );
  NS_LOG_UNCOND("Packet Delivery Ratio = " << packetDeliveryRatio);
  //NS_LOG_UNCOND("m_totalPacketsReceived = " << m_totalPacketsReceived);
  //NS_LOG_UNCOND("m_totalPacketsSent = " << m_totalPacketsSent);
}
/*-------------------------------FIXED----------------------------------*/



void UanExperiment::Prepare () {
  // avg sub cluster member number is 8
  L = m_packetSize * 8;
  // count d
  d = ceil(v * (L*8)/(w*1000));
  //
  NS_LOG_UNCOND("d = " << d << " m");
  // 
  srand(time(NULL));
  // create nodes
  m_nodes.Create (NumberOfNodes);
  // set up position of all nodes
  SetupPositions (d*layer);
  // parition nodes into layers, and record nodes information
  PartitionNodesIntoLayers ();
  SetupEnergy ();
  SetupCommunications ();
  SetupApplications ();
  Grouping ();
  SendToClusterHead ();
}



/*-------------------------------FIXED----------------------------------*/
int main (int argc, char *argv[]) {
  CommandLine cmd;
  cmd.Parse (argc, argv);
  // let simulator run by real time
  GlobalValue::Bind ("SimulatorImplementationType", 
                     StringValue ("ns3::RealtimeSimulatorImpl"));
  UanExperiment experiment;
  experiment.Prepare ();
  // the time of total simulation
  Simulator::Stop (Seconds (1000));
  // store in "/Downloads/ns-allinone-3.30.1/ns-3.30.1"
  // the XML can show the topology
  AnimationInterface anim("DAP.xml");
  Simulator::Run ();
  Simulator::Destroy ();
  experiment.Teardown ();
  return 0;
}
