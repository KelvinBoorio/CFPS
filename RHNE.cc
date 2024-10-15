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

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("RHNE");

struct Vec {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double distance = 0.0;
};

class UanExperiment {
public:
  UanExperiment ();
  // set the UAN nodes position
  void SetupPositions (double R);
  // 
  void GetAllNodeInformation ();
  // 
  void SendToCH ();
  // set the UAN nodes energy
  void SetupEnergy ();
  // set the UAN nodes communication channels
  void SetupCommunications ();
  // set the UAN nodes communication channels
  void SetupApplications ();
  //send a packet from all the nodes
  void SendPackets ();
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
  
//new  
int m_totalPacketsSent = 0;  // 紀錄總共發送的封包數量
int m_totalPacketsReceived = 0;  // 紀錄成功接收到的封包數量

private:
  // v is the speed of acoustic wave
  // m/s
  double v = 1500.0;
  // it's the same to CFPS
  // m
  double d;
  // w is the speed of data transition
  // kbps
  double w = 10.0;
  // the number of layer
  int layer = 4;
  // 128 = 8(number of node in a sub cluster) * 16(number of sub cluster)
  // 1 is the number of cluster head
  // so total number is 128 + 1 = 129
  int NumberOfNodes =10;
  // record all nodes information
  Vec AllNodesInformation[10000];
  // control packet size is 10 Bytes
  uint32_t m_controlPacketSize = 10;
  // packet size of each cluster member is 132 Bytes
  // L = 132 * 8 = 1056
  uint32_t m_packetSize = 150;//132
  // packet error rate(probability of collision)
  double errorRate = 0.2;
  // sec
  double T_RTS = 0.0;
  double Tsche = 0.1;
  double T_DATA = 0.0;
  double T_ACK = 0.0;
  double T_total = 0.0;
  // Bytes
  double Throughput = 0.0;
  // !< UAN nodes
  NodeContainer m_nodes;
  // !< send and receive sockets
  std::map<Ptr<Node>, Ptr<Socket>> m_sockets;
};

/*-------------------------------FIXED----------------------------------*/
UanExperiment::UanExperiment () {
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

void UanExperiment::SetupEnergy () {
  BasicEnergySourceHelper energySourceHelper;
  energySourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (900000));
  energySourceHelper.Install (m_nodes);
}

void UanExperiment::SetupCommunications () {
  Ptr<UanChannel> channel = CreateObject<UanChannel> ();
  UanHelper uanHelper;
  NetDeviceContainer netDeviceContainer = uanHelper.Install (m_nodes, channel);
  EnergySourceContainer energySourceContainer;
  NodeContainer::Iterator node = m_nodes.Begin ();
  while (node != m_nodes.End ()) {
    energySourceContainer.Add ((*node)->GetObject<EnergySourceContainer> ()->Get (0));
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

void UanExperiment::SetupApplications () {
  NodeContainer::Iterator node = m_nodes.Begin ();
  while (node != m_nodes.End ()) {
    m_sockets[*node] = Socket::CreateSocket (*node, TypeId::LookupByName ("ns3::UdpSocketFactory"));
    if((*node)->GetObject<Ipv4> () != NULL) {
      InetSocketAddress ipv4_local = InetSocketAddress (Ipv4Address::GetAny (), 9);
      m_sockets[*node]->Bind (ipv4_local);
    }
    m_sockets[*node]->SetRecvCallback (MakeCallback (&UanExperiment::PrintReceivedPacket, this));
    node++;
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

    }
  }
}

void UanExperiment::GetAllNodeInformation () {
  // initial the iterator
  NodeContainer::Iterator node = m_nodes.Begin ();
  int countNode = 0;
  // run all node
  while(node != m_nodes.End ()) {
    Ptr<Node> object = *node;
    Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
    // if position ==0, then stop the program
    NS_ASSERT (position != 0);
    Vector pos = position->GetPosition ();
    // count distance to cluster head(0, 0, 0)
    double dToClusterHead = 0.0;
    dToClusterHead = sqrt(pow(pos.x,2) + pow(pos.y,2) + pow(pos.z,2));
    // record nodes information
    AllNodesInformation[countNode].x = pos.x;
    AllNodesInformation[countNode].y = pos.y;
    AllNodesInformation[countNode].z = pos.z;
    AllNodesInformation[countNode].distance = dToClusterHead;
    // iterator
    countNode++;
    node++;
  }
}

// run RHNE-MAC
void UanExperiment::SendToCH () {
  
  // find Dmax
  double Dmax = 0.0;
  for(int i=0; i<NumberOfNodes; i++) {
    if(Dmax < AllNodesInformation[i].distance) {
      Dmax = AllNodesInformation[i].distance;
    }
  }
  
  // Step1. RTS phase
  NS_LOG_UNCOND("Step1. RTS phase---------------------------------------");
  NodeContainer::Iterator node = m_nodes.Begin ();
  // set reveiver is ip of CH
  Ipv4Address dst = (*node)->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetAddress (0).GetLocal ();
  // when create packet, need to use
  uint8_t energy;
  Ptr<Packet> pkt;
  // 
  T_RTS = (m_controlPacketSize*8/(w*1000))+(Dmax/v);
  T_RTS = T_RTS*pow(1-errorRate,-1)*(double)rand()/(RAND_MAX+1.0);
  // 
  for(int i=1; i<NumberOfNodes; i++) {
    // schedule
    energy = ((*(node+i))->GetObject<EnergySourceContainer> ()->Get (0)->GetEnergyFraction ()) * 100;
    pkt = Create<Packet> (&energy, m_controlPacketSize);
    Simulator::Schedule (Seconds (0), &UanExperiment::SendSinglePacket, this, *(node+i), pkt, dst);
    //new
    m_totalPacketsSent++;  // 增加發送封包計數
  }
  T_total = T_RTS;
  
  // Step2. scheduling phase
  NS_LOG_UNCOND("Step2. scheduling phase--------------------------------");
  T_total += Tsche;
  
  // Step3. DATA phase
  NS_LOG_UNCOND("Step3. DATA phase--------------------------------------");
  node = m_nodes.Begin ();
  // 
  for(int i=1; i<NumberOfNodes; i++) {
    T_DATA = (m_packetSize*8/(w*1000))+(AllNodesInformation[i].distance/v);
    // schedule
    energy = ((*(node+i))->GetObject<EnergySourceContainer> ()->Get (0)->GetEnergyFraction ()) * 100;
    pkt = Create<Packet> (&energy, m_packetSize);
    Simulator::Schedule (Seconds (T_total), &UanExperiment::SendSinglePacket, this, *(node+i), pkt, dst);
    T_total += T_DATA;
  }
  
  // Step4. ACK phase
  NS_LOG_UNCOND("Step4. ACK phase---------------------------------------");
  node = m_nodes.Begin ();
  T_ACK = (m_controlPacketSize*8/(w*1000))+(Dmax/v);
  // set sender is ip of CH
  energy = ((*node)->GetObject<EnergySourceContainer> ()->Get (0)->GetEnergyFraction ()) * 100;
  pkt = Create<Packet> (&energy, m_controlPacketSize);
  for(int i=1; i<NumberOfNodes; i++) {
    Ipv4Address dst_ACK = (*(node+i))->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetAddress (0).GetLocal ();
    Simulator::Schedule (Seconds (T_total), &UanExperiment::SendSinglePacket, this, *node, pkt, dst_ACK);
  //new
  m_totalPacketsReceived++;  // 增加發送封包計數
  }
  
  // print T_total
  T_total += T_ACK;
  NS_LOG_UNCOND("T_total = " << T_total);
  
  Throughput = m_packetSize*NumberOfNodes/T_total;
  NS_LOG_UNCOND("Throughput = " << Throughput);
  
  //new
  double packetDeliveryRatio =( (double)m_totalPacketsReceived / m_totalPacketsSent );
  NS_LOG_UNCOND("Packet Delivery Ratio = " << packetDeliveryRatio);
  NS_LOG_UNCOND("m_totalPacketsReceived = " << m_totalPacketsReceived);
  NS_LOG_UNCOND("m_totalPacketsSent = " << m_totalPacketsSent);
}
/*-------------------------------FIXED-----------------------------------*/



void UanExperiment::Prepare () {
  // avg sub cluster member number is 8
  double L = m_packetSize * 8;
  // count d
  d = ceil(v * (L*8)/(w*1000));
  //
  NS_LOG_UNCOND("d = " << d << " m");
  // 
  srand(time(NULL));
  NS_LOG_UNCOND("random = " << (double)rand()/(RAND_MAX+1.0));
  // create nodes
  m_nodes.Create (NumberOfNodes);
  // set up position of all nodes
  SetupPositions (d*layer);
  SetupEnergy ();
  SetupCommunications ();
  SetupApplications ();
  GetAllNodeInformation ();
  SendToCH ();
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
  AnimationInterface anim("RHNE.xml");
  Simulator::Run ();
  Simulator::Destroy ();
  experiment.Teardown ();
  return 0;
}
