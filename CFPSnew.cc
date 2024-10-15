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
#include <cstdlib>
#include <ctime>

using namespace ns3;

#define PI 3.1415926

NS_LOG_COMPONENT_DEFINE ("practice");

struct Vec {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  int layer = 0;
  int SubCluster = 0;
  double MinDistanceToRealSCH = 10000.0;
};

struct TreeNode {
  Vec NodeInformation;
  // parent
  TreeNode *parent = NULL;
  // left
  TreeNode *left = NULL;
  //
  TreeNode *mid = NULL;
  // right
  TreeNode *right = NULL;
  // the number of packets which ready to send
  int PacketNumber = 1;
};

struct ElementInSchedulingTable {
  int senderNumber = 0;
  int receiverNumber = 0;
  int sendTimeSlot = 0;
  double sendTime = 0.0;
};

class UanExperiment {
public:
  UanExperiment ();
  // set the UAN nodes position
  void SetupPositions (double R);
  // partition nodes into layers
  void PartitionNodesIntoLayers ();
  // choose sub cluster head
  void ChooseSubClusterHead ();
  // sub clustering
  void SubClustering ();  
  // building initial forwarding tree
  void BuildingIFT (TreeNode *root);
  // 
  void VisitTree(TreeNode *current);
  // 
  void PrintST (ElementInSchedulingTable **ST);
  // 
  void FindSenderInForwardingPhase (TreeNode *current, 
                                    int RestSlot, 
                                    int *minID);
  // building initial scheduling table
  void BuildingIST (ElementInSchedulingTable **ST, TreeNode *root);
  //
  void BuildingRST (ElementInSchedulingTable **ST);
  // 
  void SCHCollectData ();
  //
  void SendToSCH (int SC_ID, int SCH_ID);
  // 
  void RunRST (ElementInSchedulingTable **ST);
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
  // L is the size of data packet of sub cluster head
  // Bytes
  double L;
  // w is the speed of data transition
  // kbps
  double w = 10.0;
  // d is the distance of each layer
  // m
  double d;
  // the number of layer
  int layer = 4;
  // 128 = 8(number of node in a sub cluster) * 16(number of sub cluster)
  // 1 is the number of cluster head
  // so total number is 128 + 1 = 129
  int NumberOfNodes = 100;
  // record all nodes information
  Vec AllNodesInformation[10000];
  // the number of sub cluster head
  int NumberOfSubClusterHead = 0;
  // store the sub cluster head is which node
  int SubClusterHeadToNode[10000]={0};
  // be used in forwarding phase 
  int minID = 10000;
  // sec
  double deltaTp = 0.1;
  // deltaDp(m) = deltaTp(sec) * v
  double deltaDp = 0.0;
  // new
  double deltaDtol = 60.0;
  // 
  double idealSCHdegree[10000];
  // control packet size is 10 Bytes
  uint32_t m_controlPacketSize = 10;
  // packet size of each subcluster member is 132 Bytes
  // L = 132 * 8 = 1056
  uint32_t m_packetSize = 150;//132
  // each SCH packet size
  uint32_t bytesTotalOfSCH[10000]={0};
  // the time of SCH collect data
  // sec
  double Tscd = 0.0;
  // the time of RHNE-MAC scheduling
  // sec
  double Tsche = 0.1;
  // packet error rate(probability of collision)
  double errorRate = 0.2;
  // sec
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
    NS_ASSERT (position != 0);
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

void UanExperiment::ChooseSubClusterHead () {
  // Step1. count the ideal position of sub cluster head
  // the ideal position of sub cluster head
  Vec IdealSubClusterHeadPosition[10000];
  // to record which layer now in for-loop
  int IndexLayer = 1;
  // if IndexLayer%2==0
  //   then clockwise, IndexOfAngle from 0 to (2*IndexLayer-1)-1
  // if IndexLayer%2==1
  //   then counterclockwise, IndexOfAngle from (2*IndexLayer-1)-1 to 0
  int IndexOfAngle = 0;
  // count from 1, for example
  // IdealSubClusterHeadPosition[1] is the position of number"1"
  // IdealSubClusterHeadPosition[NumberOfSubClusterHead] is the position
  //   of number"NumberOfSubClusterHead"
  
  for(int i=1; i<=NumberOfSubClusterHead; i++) {
    // number 1 is in layer 1
    if(i==1) {
      IdealSubClusterHeadPosition[i].x = cos(0) * d / 2;
      IdealSubClusterHeadPosition[i].y = sin(0) * d / 2;
      IdealSubClusterHeadPosition[i].z = 0.0;
      IdealSubClusterHeadPosition[i].layer = IndexLayer;
      IdealSubClusterHeadPosition[i].SubCluster = i;
      IndexLayer++;
      idealSCHdegree[i] = 0.0;
    }
    else {
      // in layer 2, 1 < i <= 4
      // in layer 3, 4 < i <= 9
      // in layer 4, 9 < i <= 16
      if(pow(IndexLayer-1,2)<i && i<=pow(IndexLayer,2)) {
        // set ideal position
        idealSCHdegree[i] = PI/(2*IndexLayer-1)-PI/2 + 
                            IndexOfAngle*2*PI/(2*IndexLayer-1);
        IdealSubClusterHeadPosition[i].x = cos(idealSCHdegree[i]);
        IdealSubClusterHeadPosition[i].y = sin(idealSCHdegree[i]);
        IdealSubClusterHeadPosition[i].x *= (2*IndexLayer-1)*d / 2;
        IdealSubClusterHeadPosition[i].y *= (2*IndexLayer-1)*d / 2;
        IdealSubClusterHeadPosition[i].z = 0.0;
        IdealSubClusterHeadPosition[i].layer = IndexLayer;
        IdealSubClusterHeadPosition[i].SubCluster = i;
        // if IndexLayer%2==0, then clockwise
        if(IndexLayer%2==0) {
          IndexOfAngle++;
        }
        // if IndexLayer%2==1, then counterclockwise
        else {
          IndexOfAngle--;
        }
        // if i==4, 9, 16..., then IndexLayer += 1
        if(i==pow(IndexLayer,2)) {
          IndexLayer++;
          // start to clockwise
          if(IndexLayer%2==0) {
            IndexOfAngle = 0;
          }
          // start to counterclockwise
          else {
            IndexOfAngle = (2*IndexLayer-1)-1;
          }
        }
      }
    }
  }

  // Step2. choose sub cluster head which nearest to ideal position
  int countNode = 0;
  NodeContainer::Iterator node = m_nodes.Begin ();
  // because node 0 is cluster head, count from node 1
  countNode++;
  node++;
  // SCH is sub cluster head
  // record the min distance from ideal SCH to real SCH
  // for example, MinDistanceToIdealSCH[1] = 2.0
  // the min distance from number 1 ideal SCH is 2.0
  double MinDistanceToIdealSCH[10000];
  for(int i=0; i<10000; i++) {
    MinDistanceToIdealSCH[i] = 10000.0;
  }
  // from node 1, run all sensor nodes
  while(node != m_nodes.End ()) {
    // run all ideal SCH
    for(int i=1; i<=NumberOfSubClusterHead; i++) {
      // judge the layer of ideal SCH if equal to the layer of node
      if(AllNodesInformation[countNode].layer == 
         IdealSubClusterHeadPosition[i].layer) {
        double tempDistance = 0.0;
        tempDistance = sqrt(
          pow(AllNodesInformation[countNode].x-
              IdealSubClusterHeadPosition[i].x,2) + 
          pow(AllNodesInformation[countNode].y-
              IdealSubClusterHeadPosition[i].y,2));
        if(tempDistance <= MinDistanceToIdealSCH[i]) {
          MinDistanceToIdealSCH[i] = tempDistance;
          SubClusterHeadToNode[i] = countNode;
          //NS_LOG_UNCOND("tempDistance : " << tempDistance);
        }    
      }
    }
    countNode++;
    node++;
  }
}
/*
void UanExperiment::SubClustering () {
  int countNode = 0;
  NodeContainer::Iterator node = m_nodes.Begin ();
  // because node 0 is cluster head, count from node 1
  countNode++;
  node++;
  // run all sensor nodes
  while (node != m_nodes.End ()) {
    // run all RealSCH
    for(int i=1 ; i<=NumberOfSubClusterHead; i++) {
      // count the distance between node"countNode" to SCH"i"
      double tempDistance = 10000.0;
      tempDistance = sqrt(
        pow(AllNodesInformation[countNode].x - 
            AllNodesInformation[SubClusterHeadToNode[i]].x,2) + 
        pow(AllNodesInformation[countNode].y - 
            AllNodesInformation[SubClusterHeadToNode[i]].y,2));
      
      // judge the layer whether equal
      if(AllNodesInformation[countNode].layer == 
         AllNodesInformation[SubClusterHeadToNode[i]].layer) {
        // if distance closer
        if(tempDistance <= 
           AllNodesInformation[countNode].MinDistanceToRealSCH) {
          // update the distance
          AllNodesInformation[countNode].MinDistanceToRealSCH = 
            tempDistance;
          // update the sub cluster
          AllNodesInformation[countNode].SubCluster = i;
        }
      }
    }
    countNode++;
    node++;
  }
}
*/


void UanExperiment::SubClustering () {
  int countNode = 0;
  NodeContainer::Iterator node = m_nodes.Begin ();
  // 因為節點0是集群頭，所以從節點1開始計數
  countNode++;
  node++;
  // 遍歷所有的感測節點
  while (node != m_nodes.End ()) {
    // 遍歷所有的子集群頭
    for(int i = 1; i <= NumberOfSubClusterHead; i++) {
      // 計算節點"countNode"到子集群頭"SCH i"的距離
      double tempDistance = 10000.0;
      tempDistance = sqrt(
        pow(AllNodesInformation[countNode].x - 
            AllNodesInformation[SubClusterHeadToNode[i]].x, 2) + 
        pow(AllNodesInformation[countNode].y - 
            AllNodesInformation[SubClusterHeadToNode[i]].y, 2));
      
      // 判斷兩者是否處於同一層
      if (AllNodesInformation[countNode].layer == 
          AllNodesInformation[SubClusterHeadToNode[i]].layer) {
        // 如果距離更近
        if (tempDistance <= 
            AllNodesInformation[countNode].MinDistanceToRealSCH) {
          // 更新最短距離
          AllNodesInformation[countNode].MinDistanceToRealSCH = tempDistance;
          // 更新所屬的子集群
          AllNodesInformation[countNode].SubCluster = i;
        }
      }
    }
    // 輸出結果到終端
    std::cout << "Node " << countNode << ": "
              << "Min Distance to SCH = " 
              << AllNodesInformation[countNode].MinDistanceToRealSCH 
              << ", SubCluster = " 
              << AllNodesInformation[countNode].SubCluster << std::endl;

    countNode++;
    node++;
  }
}
/*
void UanExperiment::SubClustering () {
  int countNode = 0;
  NodeContainer::Iterator node = m_nodes.Begin ();
  // 因為節點0是集群頭，所以從節點1開始計數
  countNode++;
  node++;
  // 遍歷所有的感測節點
  while (node != m_nodes.End ()) {
    double firstMinDistance = 10000.0;  // 最短距離
    double secondMinDistance = 10000.0; // 第二短距離
    int firstSCH = -1;  // 所屬的最近子集群頭
    int secondSCH = -1; // 第二近的子集群頭

    // 遍歷所有的子集群頭
    for(int i = 1; i <= NumberOfSubClusterHead; i++) {
      // 計算節點"countNode"到子集群頭"SCH i"的距離
      double tempDistance = sqrt(
        pow(AllNodesInformation[countNode].x - 
            AllNodesInformation[SubClusterHeadToNode[i]].x, 2) + 
        pow(AllNodesInformation[countNode].y - 
            AllNodesInformation[SubClusterHeadToNode[i]].y, 2));
      
      // 判斷兩者是否處於同一層
      if (AllNodesInformation[countNode].layer == 
          AllNodesInformation[SubClusterHeadToNode[i]].layer) {
        // 更新最短和第二短距離
        if (tempDistance < firstMinDistance) {
          // 更新第二短為之前的最短
          secondMinDistance = firstMinDistance;
          secondSCH = firstSCH;
          // 更新最短距離
          firstMinDistance = tempDistance;
          firstSCH = i;
        } else if (tempDistance < secondMinDistance) {
          // 更新第二短距離
          secondMinDistance = tempDistance;
          secondSCH = i;
        }
      }
    }
    
    // 更新節點的信息
    AllNodesInformation[countNode].MinDistanceToRealSCH = firstMinDistance;
    AllNodesInformation[countNode].SubCluster = firstSCH;

    // 輸出結果到終端
    std::cout << "Node " << countNode << ": "
              << "Min Distance to SCH = " 
              << firstMinDistance << ", SubCluster = " 
              << firstSCH << std::endl;

    std::cout << "Node " << countNode << ": "
              << "Second Min Distance to SCH = " 
              << secondMinDistance << ", Second SubCluster = " 
              << secondSCH << std::endl;

    countNode++;
    node++;
  }
}
*/

















void UanExperiment::BuildingIFT (TreeNode *root) {
  // run all SCH to store in TreeNode
  for(int i=1; i<=NumberOfSubClusterHead; i++) {
    root[i].NodeInformation = AllNodesInformation[SubClusterHeadToNode[i]];
  }
  // run all SCH to build link between TreeNodes
  for(int i=1; i<=NumberOfSubClusterHead; i++) {
    // layer 1
    if(i==1) {
      root[i].parent = NULL;
      root[i].left = &root[2];
      root[i].mid = &root[3];
      root[i].right = &root[4];
    }
    // layer 2
    else if(i==2) {
      root[i].parent = &root[1];
      root[i].left = &root[9];
      root[i].mid = &root[8];
      root[i].right = NULL;
    }
    else if(i==3) {
      root[i].parent = &root[1];
      root[i].left = &root[7];
      root[i].mid = NULL;
      root[i].right = NULL;
    }
    else if(i==4) {
      root[i].parent = &root[1];
      root[i].left = &root[6];
      root[i].mid = &root[5];
      root[i].right = NULL;
    }
    // layer 3
    else if(i==5) {
      root[i].parent = &root[4];
      root[i].left = &root[15];
      root[i].mid = &root[16];
      root[i].right = NULL;
    }
    else if(i==6) {
      root[i].parent = &root[4];
      root[i].left = &root[14];
      root[i].mid = NULL;
      root[i].right = NULL;
    }
    else if(i==7) {
      root[i].parent = &root[3];
      root[i].left = &root[13];
      root[i].mid = NULL;
      root[i].right = NULL;
    }
    else if(i==8) {
      root[i].parent = &root[2];
      root[i].left = &root[12];
      root[i].mid = NULL;
      root[i].right = NULL;
    }
    else if(i==9) {
      root[i].parent = &root[2];
      root[i].left = &root[10];
      root[i].mid = &root[11];
      root[i].right = NULL;
    }
    // layer 4
    else if(i==10) {
      root[i].parent = &root[9];
      root[i].left = NULL;
      root[i].mid = NULL;
      root[i].right = NULL;
    }
    else if(i==11) {
      root[i].parent = &root[9];
      root[i].left = NULL;
      root[i].mid = NULL;
      root[i].right = NULL;
    }
    else if(i==12) {
      root[i].parent = &root[8];
      root[i].left = NULL;
      root[i].mid = NULL;
      root[i].right = NULL;
    }
    else if(i==13) {
      root[i].parent = &root[7];
      root[i].left = NULL;
      root[i].mid = NULL;
      root[i].right = NULL;
    }
    else if(i==14) {
      root[i].parent = &root[6];
      root[i].left = NULL;
      root[i].mid = NULL;
      root[i].right = NULL;
    }
    else if(i==15) {
      root[i].parent = &root[5];
      root[i].left = NULL;
      root[i].mid = NULL;
      root[i].right = NULL;
    }
    else if(i==16) {
      root[i].parent = &root[5];
      root[i].left = NULL;
      root[i].mid = NULL;
      root[i].right = NULL;
    }
    // layer 5
    else {
      NS_LOG_UNCOND("the number of sub cluster head is over 16.");
      NS_LOG_UNCOND("please add more 'else-if' in code.");
    }
  }
  /* the final IFT will be below
              01
            /  |  \
          02  03  04
        /  |   |   |  \   
      09  08  07  06  05
    /  |   |   |   |   |  \
  10  11  12  13  14  15  16
  */
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

void UanExperiment::VisitTree(TreeNode *current) {
  if(current) {
    NS_LOG_UNCOND(current->NodeInformation.SubCluster);
    VisitTree(current->left);
    VisitTree(current->mid);
    VisitTree(current->right);
  }
}

void UanExperiment::PrintST (ElementInSchedulingTable **ST) {
  for(int i=layer; i>=1; i--) {
    NS_LOG_UNCOND("layer = " << i);
    for(int j=1; j<=NumberOfSubClusterHead; j++) {
      NS_LOG_UNCOND("[" << j << "] " << ST[i][j].senderNumber << "," << ST[i][j].receiverNumber);
    }
  }
}

void UanExperiment::FindSenderInForwardingPhase(TreeNode *current, 
                                                int RestSlot, 
                                                int *minID) {
  if(current) {
    // if in the outer layer
    if(RestSlot == 0) {
      // if have packet to send
      if(current->PacketNumber) {
        // compare whose ID is smaller
        if(*minID > current->NodeInformation.SubCluster) {
          *minID = current->NodeInformation.SubCluster;
        }
      }
    }
    else {
      FindSenderInForwardingPhase(current->left, RestSlot-1, minID);
      FindSenderInForwardingPhase(current->mid, RestSlot-1, minID);
      FindSenderInForwardingPhase(current->right, RestSlot-1, minID);
    }
  }
}

void UanExperiment::BuildingIST (ElementInSchedulingTable **ST, 
                                 TreeNode *root) {
  // the number of total packet, each SCH need to send one packet
  int TotalPacketNumber = NumberOfSubClusterHead;
  // count time slot from 1
  int slot = 1;
  // there are packet need to send
  while(TotalPacketNumber > 0) {
    
    // Step1. Find the Tk
    // Tk is the outer layer which have node want to send packet
    // run layer from 4 to 1
    int Tk = 0;
    for(int i=layer; i>=1; i--) {
      // judge is whether has packet to send in layers
      bool IsHasPacket = false;
      // run all SCH in layer"i"
      // for example, when i=4
      // run SCH"10" to "16"
      for(int j=pow(i-1,2)+1; j<=pow(i,2); j++) {
        if(root[j].PacketNumber) {
          IsHasPacket = true;
          break;
        }
      }
      if(IsHasPacket) {
        Tk = i;
        break;
      }
    }
    
    // Step2. schedeling
    // record the node which end directly send
    int *EndDirectlySendNode = new int[Tk+1];
    // according to pseudocode to name "s"
    for(int s=1; s<=Tk; s++) {
      // set initial value
      minID = 10000;
      // if s is the first slot in Tk
      if(s==1) {
        // the node with largest ID in each layer can
        // directly send to cluster head
        // Tk is also the outer layer
        for(int i=Tk; i>=1; i--) {
          for(int j=pow(i,2); j>=pow(i-1,2)+1; j--) {
            // if node "j" have packet
            if(root[j].PacketNumber > 0) {
              // now, ID "j" is the node with largest ID in layer "i"
              // set sender
              ST[i][slot].senderNumber = j;
              // set receiver(0 is cluster head)
              ST[i][slot].receiverNumber = 0;
              // set transtion time slot
              ST[i][slot].sendTimeSlot = slot;
              // packet = packet - 1
              root[j].PacketNumber -= 1;
              // set end directly send
              EndDirectlySendNode[i] = j;
              // break to next layer
              break;
            }
          }
        }
      } // end if s==1
      else {
        // let x be the node just finish sending packet
        // x is EndDirectlySendNode[s-1], for example
        // 1 is EndDirectlySendNode[2-1] at slot 1, start receive at slot 2
        // 4 is EndDirectlySendNode[3-1] at slot 2, start receive at slot 3
        // 9 is EndDirectlySendNode[4-1] at slot 3, start receive at slot 4
        int x = EndDirectlySendNode[s-1];
        // find the sender node "i"
        // node "i" is the leaf with smallest ID of node "x" and 
        // node "i" have packet to send
        TreeNode *receiverX = &root[x];
        // because a node send a packet into one layer need a slot, so
        // Tk - root[x].NodeInformation.layer
        // is the rest slots in Tk, for example
        // the rest of slot with 1 is 4-1, so OuterLayerOfNodeX is 1+4-1
        // the rest of slot with 4 is 4-2, so OuterLayerOfNodeX is 2+4-2
        // the rest of slot with 9 is 4-3, so OuterLayerOfNodeX is 3+4-3
        int RestSlot = Tk - root[x].NodeInformation.layer;
        // get node "i", i is minID
        FindSenderInForwardingPhase(receiverX, RestSlot, &minID);
        // if node "i" exists, then minID < 10000
        if(minID < 10000) {
          // set sender
          ST[Tk][slot].senderNumber = minID;
          // set receiver
          ST[Tk][slot].receiverNumber = x;
          // set transtion time slot
          ST[Tk][slot].sendTimeSlot = slot;
          // sender packet - 1
          root[minID].PacketNumber -= 1;
          // receiver packet + 1
          root[x].PacketNumber += 1;
        }
        else {
          // let node y be the neighbor node of node x
          // get the layer of node x
          int LayerOfX = root[x].NodeInformation.layer;
          // the ID of node y
          int y = 0;
          // choose the smallest neighbor ID, for example
          for(y=pow(LayerOfX-1,2)+1; y<=pow(LayerOfX,2); y++) {
            FindSenderInForwardingPhase(&root[y], RestSlot, &minID);
            // if node "i" exists, then minID < 10000
            if(minID < 10000) {
              break;
            }
          }
          // if node "i" exists, then minID < 10000
          if(minID < 10000) {
            // set sender
            ST[Tk][slot].senderNumber = minID;
            // set receiver
            ST[Tk][slot].receiverNumber = y;
            // set transtion time slot
            ST[Tk][slot].sendTimeSlot = slot;
            // packet = packet - 1
            root[minID].PacketNumber -= 1;
            // receiver packet + 1
            root[y].PacketNumber += 1;
          }
          else {
            // because it's "y++" in the end of for-loop
            // so y = y - 1
            y -= 1;
            // let node z be the parent node of node y
            // get the layer of node z
            int LayerOfZ = root[y].parent->NodeInformation.layer;
            // the ID of node z
            int z = 0;
            // z from smallest to largest
            for(z=pow(LayerOfZ-1,2)+1; z<=pow(LayerOfZ,2); z++) {
              FindSenderInForwardingPhase(&root[z], RestSlot, &minID);
              // if node "i" exists, then minID < 10000
              if(minID < 10000) {
                break;
              }
            }
            // if node "i" exists, then minID < 10000
            if(minID < 10000) {
              // set sender
              ST[Tk][slot].senderNumber = minID;
              // set receiver
              ST[Tk][slot].receiverNumber = z;
              // set transtion time slot
              ST[Tk][slot].sendTimeSlot = slot;
              // packet = packet - 1
              root[minID].PacketNumber -= 1;
              // receiver packet + 1
              root[z].PacketNumber += 1;
            }
          } // end if receiver is z
        } // end if receiver is y
      } // end if receiver is x
      // count slot
      slot++;
    } // end for s to Tk
    // delete
    delete [] EndDirectlySendNode;
    // 
    TotalPacketNumber -= Tk;
  } // end while
  PrintST(ST);
}

void UanExperiment::BuildingRST (ElementInSchedulingTable **ST) {
  deltaDp = deltaTp * v;
  // visit all element in ST
  for(int i=1; i<=layer; i++) {
    for(int j=1; j<=NumberOfSubClusterHead; j++) {
      // if it's forwarding phase, then judge the distance
      if(ST[i][j].senderNumber!=0 && ST[i][j].receiverNumber!=0) {
        // judge the distance
        int senderLayer = AllNodesInformation[SubClusterHeadToNode[ST[i]
                                              [j].senderNumber]].layer;
        int receiverLayer = AllNodesInformation[SubClusterHeadToNode[ST[i]
                                               [j].receiverNumber]].layer;
        int deltaLayer = senderLayer - receiverLayer;
        double distanceBetweenSenderAndReceiver = sqrt(
pow(AllNodesInformation[SubClusterHeadToNode[ST[i][j].senderNumber]].x - AllNodesInformation[SubClusterHeadToNode[ST[i][j].receiverNumber]].x,2) + 
pow(AllNodesInformation[SubClusterHeadToNode[ST[i][j].senderNumber]].y - AllNodesInformation[SubClusterHeadToNode[ST[i][j].receiverNumber]].y,2));
        // 
        if(abs(distanceBetweenSenderAndReceiver-deltaLayer*d) > 
           deltaLayer*deltaDp) {
          // now, we must to choose the new receiver
          
          // Step1. find location w
          // count the cos/sin of sender
          // cos = (2*sender_x)/(d(2*sender_layer-1))
          // sin = (2*sender_y)/(d(2*sender_layer-1))
          double cos = (2*AllNodesInformation[SubClusterHeadToNode[ST[i][j].senderNumber]].x)/(d*(2*senderLayer-1));
          double sin = (2*AllNodesInformation[SubClusterHeadToNode[ST[i][j].senderNumber]].y)/(d*(2*senderLayer-1));
          // get x/y of location w
          double w_x = cos * (2*receiverLayer-1) * (d/2);
          double w_y = sin * (2*receiverLayer-1) * (d/2);
          
          // Step2. find node ja(k)
          double R = deltaLayer * (d + deltaDp);
          double minDistanceToW = 10000.0;
          // run all nodes, for example, NumberOfNodes is 129
          // NumberOfNodes 0 cluster head
          // so run from 1 to 128
          for(int k=1; k<NumberOfNodes; k++) {
            // judge node k SCH 
            if(AllNodesInformation[k].SubCluster == 
               AllNodesInformation[SubClusterHeadToNode[
                 ST[i][j].receiverNumber]].SubCluster) {
              // judge the distance between node k and sender
              if(sqrt(pow(AllNodesInformation[k].x - AllNodesInformation[SubClusterHeadToNode[ST[i][j].senderNumber]].x,2) +
                      pow(AllNodesInformation[k].y - AllNodesInformation[SubClusterHeadToNode[ST[i][j].senderNumber]].y,2)) < R) {
                // if node k is nearest to w
                if(sqrt(pow(AllNodesInformation[k].x - w_x,2) + 
                        pow(AllNodesInformation[k].y - w_y,2)) < 
                        minDistanceToW) {
                  // then node k is new receiver
                  ST[i][j].receiverNumber = k;
                  ST[i][j].senderNumber = 
                    SubClusterHeadToNode[ST[i][j].senderNumber];
                  // update the minDistanceToW
                  minDistanceToW = sqrt(
                    pow(AllNodesInformation[k].x - w_x,2) + 
                    pow(AllNodesInformation[k].y - w_y,2));
                }
              } // end if the distance from k to sender < R
            } // end if k in same SC
          }  // end for-loop k
        }
        else {
          // then change SCH ID to node ID
          ST[i][j].senderNumber = 
            SubClusterHeadToNode[ST[i][j].senderNumber];
          ST[i][j].receiverNumber = 
            SubClusterHeadToNode[ST[i][j].receiverNumber];
        }
      }
      // if it's direct sending phase
      else if(ST[i][j].senderNumber!=0 && ST[i][j].receiverNumber==0) {
        // then change SCH ID to node ID
        ST[i][j].senderNumber = 
          SubClusterHeadToNode[ST[i][j].senderNumber];
      }
    } // end for-loop j
  } // end for-loop i
  PrintST(ST);
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

void UanExperiment::SCHCollectData () {
  // each sub cluster head collect data
  for(int i=1; i<=NumberOfSubClusterHead; i++) {
    SendToSCH (i, SubClusterHeadToNode[i]);
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
      m_totalPacketsReceived++;  // 增加成功接收封包計數
    }
  }
}

// each SC run RHNE-MAC
void UanExperiment::SendToSCH (int SC_ID, int SCH_ID) {
  // count SC_ID have how many nodes
  int countSCmember = 0;
  for(int i=1; i<=NumberOfNodes; i++) {
    if(AllNodesInformation[i].SubCluster == SC_ID) {
      countSCmember += 1;
    }
  }
  // can't count SCH
  countSCmember--;
  // send RTS
  // now, all SCM(sub cluster member) send control packet to SCH
  NodeContainer::Iterator node = m_nodes.Begin ();
  for(int i=1; i<=NumberOfNodes; i++) {
    if(AllNodesInformation[i].SubCluster == SC_ID) {
      // SCH can't send to itself
      if(i != SCH_ID) {
        // set reveiver is ip of SCH_ID
        Ipv4Address dst = (*(node+SCH_ID))->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetAddress (0).GetLocal ();
        // when create packet, need to use
        uint8_t energy;
        Ptr<Packet> pkt;
        // schedule
        energy = ((*(node+i))->GetObject<EnergySourceContainer> ()->Get (0)->GetEnergyFraction ()) * 100;
        pkt = Create<Packet> (&energy, m_controlPacketSize);
        Simulator::Schedule (Seconds (0), &UanExperiment::SendSinglePacket, this, *(node+i), pkt, dst);
        //new
        m_totalPacketsSent++;  // 增加發送封包計數
      }
    }
  }
  // count Dmax
  double Dmax = 0.0;
  for(int i=1; i<=NumberOfNodes; i++) {
    if(AllNodesInformation[i].SubCluster == SC_ID) {
      double tempDmax;
      tempDmax = sqrt(
      pow(AllNodesInformation[i].x - AllNodesInformation[SCH_ID].x,2) + 
      pow(AllNodesInformation[i].y - AllNodesInformation[SCH_ID].y,2));
      if(Dmax < tempDmax) {
        Dmax = tempDmax;
      }
    }
  }
  // count Tscd
  double tempTscd = 0.0;
  tempTscd = Tsche + 
    countSCmember*((m_packetSize*8/(w*1000))+(Dmax/v)) + 
    2*((m_controlPacketSize*8/(w*1000))+(Dmax/v));
  if(Tscd < tempTscd) {
    Tscd = tempTscd;
  }
  // the SCM with successful transition can send data in order
  bytesTotalOfSCH[SubClusterHeadToNode[SC_ID]] = m_packetSize * countSCmember * (1-errorRate);
  // 
  int index = 1;
  node = m_nodes.Begin ();
  for(int i=1; i<=NumberOfNodes; i++) {
    if(AllNodesInformation[i].SubCluster == SC_ID) {
      // SCH can't send to itself
      if(i != SCH_ID) {
        // set reveiver is ip of SCH_ID
        Ipv4Address dst = (*(node+SCH_ID))->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetAddress (0).GetLocal ();
        // when create packet, need to use
        uint8_t energy;
        Ptr<Packet> pkt;
        // schedule
        energy = ((*(node+i))->GetObject<EnergySourceContainer> ()->Get (0)->GetEnergyFraction ()) * 100;
        pkt = Create<Packet> (&energy, m_packetSize);
        Simulator::Schedule (Seconds ((Tscd/countSCmember)*index), &UanExperiment::SendSinglePacket, this, *(node+i), pkt, dst);
        index++;
        //new
        m_totalPacketsReceived++;
      }
    }
  }
}

// run RST!!
void UanExperiment::RunRST (ElementInSchedulingTable **ST) {
  NS_LOG_UNCOND("Tscd = " << Tscd << "sec");
  
  // count the time of a slot
  double Tslot = 0.0;
  Tslot = ((L*8)/(w*1000)) + (d/v) + deltaTp;
  NS_LOG_UNCOND("Tslot = " << Tslot);
  
  // run all element in ST
  NodeContainer::Iterator node = m_nodes.Begin ();
  for(int i=1; i<=NumberOfSubClusterHead; i++) {
    for(int j=1; j<=layer; j++) {
      // directly send phase
      if(ST[j][i].receiverNumber==0 && ST[j][i].senderNumber!=0) {
        NS_LOG_UNCOND("ST[" << i << "][" << j << "] is sent end at " << Tscd+i*Tslot << " sec");
        // set reveiver is ip of cluster head
        Ipv4Address dst = (*node)->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetAddress (0).GetLocal ();
        // when create packet, need to use
        uint8_t energy;
        Ptr<Packet> pkt;
        // schedule
        energy = ((*(node+ST[j][i].senderNumber))->GetObject<EnergySourceContainer> ()->Get (0)->GetEnergyFraction ()) * 100;
        pkt = Create<Packet> (&energy, bytesTotalOfSCH[ST[j][i].senderNumber]);
        Simulator::Schedule (Seconds (Tscd+i*Tslot), &UanExperiment::SendSinglePacket, this, *(node+ST[j][i].senderNumber), pkt, dst);
      }
      // forwarding phase
      else if(ST[j][i].receiverNumber!=0 && ST[j][i].senderNumber!=0) {
        NS_LOG_UNCOND("ST[" << i << "][" << j << "] is scheduled at " << Tscd+i*Tslot << " sec");
        // set reveiver is ip of cluster head
        Ipv4Address dst = (*(node+ST[j][i].receiverNumber))->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetAddress (0).GetLocal ();
        // when create packet, need to use
        uint8_t energy;
        Ptr<Packet> pkt;
        // schedule
        energy = ((*(node+ST[j][i].senderNumber))->GetObject<EnergySourceContainer> ()->Get (0)->GetEnergyFraction ()) * 100;
        pkt = Create<Packet> (&energy, bytesTotalOfSCH[ST[j][i].senderNumber]);
        Simulator::Schedule (Seconds (Tscd+i*Tslot), &UanExperiment::SendSinglePacket, this, *(node+ST[j][i].senderNumber), pkt, dst);
        // change packet size
        bytesTotalOfSCH[ST[j][i].receiverNumber] = 
          bytesTotalOfSCH[ST[j][i].senderNumber];
      }
    } // end for-loop j
    T_total = Tscd+i*Tslot;
  } // end for-loop i
  NS_LOG_UNCOND("T_total = " << T_total);
  
  Throughput = m_packetSize*NumberOfNodes/T_total;
  NS_LOG_UNCOND("Throughput = " << Throughput);
  
  
  //new
  double packetDeliveryRatio =( (double)m_totalPacketsReceived / m_totalPacketsSent );
  NS_LOG_UNCOND("Packet Delivery Ratio = " << packetDeliveryRatio);
  //NS_LOG_UNCOND("m_totalPacketsReceived = " << m_totalPacketsReceived);
  //NS_LOG_UNCOND("m_totalPacketsSent = " << m_totalPacketsSent);

}
/*-------------------------------FIXED----------------------------------*/



void UanExperiment::Prepare () {
  // initialize idealSCHdegree
  for(int i=0; i<10000; i++) {
    idealSCHdegree[i] = 0.0;
  }
  // avg sub cluster member number is 8
  L = m_packetSize * 8;
  // count d
  d = ceil(v * (L*8)/(w*1000) + 2*deltaDtol);  //new    + 2 * deltaDtol
  //
  NS_LOG_UNCOND("d = " << d << " m");
  // create nodes
  m_nodes.Create (NumberOfNodes);
  // set up position of all nodes
  SetupPositions (d*layer);
  // parition nodes into layers, and record nodes information
  PartitionNodesIntoLayers ();
  // use the equal area cutting method to count number of sub cluster head
  
  
  
  for(int i=1; i<=layer; i++) {
    NumberOfSubClusterHead += 2 * i - 1;
  }
  // choose sub cluster head
  ChooseSubClusterHead ();
  // sub clustering
  SubClustering ();
  // building initial forwarding tree
  // root is used from 1, so NumberOfSubClusterHead+1
  TreeNode *root = new TreeNode[NumberOfSubClusterHead+1];
  BuildingIFT (root);
  // building initial scheduling table
  // index 0 is not to be used, so "layer+1" and "NumberOfSubClusterHead+1"
  // for example, if layer=4 and NumberOfSubClusterHead=16
  // ST will be below
  //   0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16
  // 0 x x x x x x x x x x  x  x  x  x  x  x  x
  // 1 x x x x x x x x x x  x  x  x  x  x  x  x
  // 2 x x x x x x x x x x  x  x  x  x  x  x  x
  // 3 x x x x x x x x x x  x  x  x  x  x  x  x
  // 4 x x x x x x x x x x  x  x  x  x  x  x  x
  ElementInSchedulingTable **ST = new ElementInSchedulingTable*[layer+1];
  for(int i=0; i<layer+1; i++) {
    ST[i] = new ElementInSchedulingTable[NumberOfSubClusterHead+1];
  }
  BuildingIST (ST, root);
  BuildingRST (ST);
  SetupEnergy ();
  SetupCommunications ();
  SetupApplications ();
  SCHCollectData ();
  // run RST
  RunRST (ST);
  
  
  
  // 
  
  
  
  // delete to return the space of memory
  delete [] root;
  for(int i=0; i<layer+1; i++) {
    delete [] ST[i];
  }
  delete [] ST;
  NS_LOG_UNCOND("end of delete");
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
  Simulator::Stop (Seconds (100));
  // store in "/Downloads/ns-allinone-3.30.1/ns-3.30.1"
  // the XML can show the topology
  AnimationInterface anim("practice.xml");
  Simulator::Run ();
  Simulator::Destroy ();
  experiment.Teardown ();
  return 0;
}
