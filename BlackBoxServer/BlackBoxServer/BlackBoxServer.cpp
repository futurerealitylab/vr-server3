/*
BlackBoxServer 4.0
Includes some code from OptiTrack.
*/

#include "stdafx.h"

#define LCL_BROADCAST
//#define RMT_BROADCAST // TODO: uncomment
//#define RMT_RCV // TODO: comment

using namespace std;

// IP for this host computer
#ifdef LCL_BROADCAST
static const string IP_ADDR = "192.168.1.44";
#elif defined RMT_BROADCAST
static const string IP_ADDR = ""; // TODO: Enter MAGNET computer IP
#elif defined RMT_RCV
static const string IP_ADDR = "128.122.47.161";
#endif


// ADD YOUR WIIMOTE HARDWARE ADDRESSES HERE
char *mote_id_to_label(QWORD id) {
	switch (id) {
	//case 0x9da2a2838788:
	//	return "VR5_wand";
	case 0x9898977f7f7d://0x9da09e838483:
		return "VR4_wand";
	case 0x9898977d7f7f:
		return "VR2_wand";
	case 0x96979d7c7d83:
		return "VR3_wand";
	case 0x9da2a2838788://0x9b9d9a828280:
		return "VR1_wand";
	default:
		return "???";
	}
}

map<std::string, wiimote*> motes;
unsigned detected = 0;

class Stream {
	SOCKET s;
	struct sockaddr_in addr;
	struct sockaddr_in bind_addr;
	public:
	Stream(PCSTR ip, int server_port, bool multicast) {
		WSADATA wd;
		WSAStartup(0x02, &wd);
		int err;
		if ((s = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET) {
			printf("[VR] Could not create socket : %d", WSAGetLastError());
		}

		int opt_val = 1;
		if (multicast) {
			err = setsockopt(s, SOL_SOCKET, SO_BROADCAST, (char*)&opt_val, sizeof(opt_val));
			if (err == SOCKET_ERROR) {
				printf("SOCKET ERROR; HIT ANY KEY TO ABORT\n");
				getchar();
				exit(0);
			}
		}

		opt_val = 1;
		err = setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (char*)&opt_val, sizeof(opt_val));
		if (err == SOCKET_ERROR) {
			printf("SOCKET ERROR; HIT ANY KEY TO ABORT\n");
			getchar();
			exit(0);
		}

		// Bind to correct NIC
		bind_addr.sin_family = AF_INET;
		err = inet_pton(AF_INET, IP_ADDR.c_str(), &bind_addr.sin_addr); // S_ADDR of our IP for the WiFi interface
		//err = inet_pton(AF_INET, "128.122.47.25", &bind_addr.sin_addr); // S_ADDR of our IP for the WiFi interface
		if (err == SOCKET_ERROR) {
			printf("SOCKET ERROR; HIT ANY KEY TO ABORT\n");
			getchar();
			exit(0);
		}
		bind_addr.sin_port = 0;
		err = ::bind(s, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
		if (err == SOCKET_ERROR) {
			printf("SOCKET ERROR; HIT ANY KEY TO ABORT\n");
			getchar();
			exit(0);
		}

		memset(&addr, 0, sizeof(addr));
		addr.sin_family = AF_INET;
		addr.sin_port = htons(server_port);
		// Proper mutlicast group
		err = inet_pton(AF_INET, ip, &addr.sin_addr);
		if (err == SOCKET_ERROR) {
			printf("SOCKET ERROR; HIT ANY KEY TO ABORT\n");
			getchar();
			exit(0);
		}
	}
	void send(char* packet, int length) {
		sendto(s, packet, length, 0, (struct sockaddr*) &addr, sizeof(addr));
	}
	~Stream() {
		closesocket(s);
		WSACleanup();
	}
};

#pragma warning( disable : 4996 )

void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData);
// For NatNet error mesages
void __cdecl MessageHandler(int msgType, char* msg);
void resetClient();
int CreateClient(int iConnectionType);
int PacketServingThread();
int PacketReceivingThread();
void checkForWiimotes();

update_protocol_v3::Update *recvUpdate;
std::mutex recvUpdateLock;

unsigned int MyServersDataPort = 1511;
unsigned int MyServersCommandPort = 1510;

NatNetClient* theClient;
FILE* fp;

sDataDescriptions* pDataDefs = NULL;
// Rigid body labels, etc.
map<int, string> idToLabel;
void GetDataDescriptions() {
	printf("\n\n[VRServer3] Requesting Data Descriptions...\n");
	int nBodies = theClient->GetDataDescriptions(&pDataDefs);
	if (!pDataDefs)
	{
		printf("[VRServer3] Unable to retrieve Data Descriptions.");
		return;
	}
	printf("[VRServer3] Received %d Data Descriptions.\n", pDataDefs->nDataDescriptions);
	for (int i = 0; i < pDataDefs->nDataDescriptions; i++)
	{
		if (pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet) {
			// TODO: Process descriptions for MarkerSets
		}
		else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody) {
			sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
			idToLabel[pRB->ID] = pRB->szName;
		}
		else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton) {
			for (sRigidBodyDescription pRB : pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies) {
				idToLabel[pRB.ID] = pRB.szName;
			}
		}
		else {
			printf("Unknown data type.");
			continue;
		}
	}
}

class PacketGroup {
private:
	/* static fields */
	const static int max_packet_bytes = 872;
	static vector<PacketGroup*> packet_groups;
	static PacketGroup *head;
	static string last_label;
	static std::mutex packet_groups_lock;
	static char buffer[max_packet_bytes];
#ifdef LCL_BROADCAST
	static Stream multicast_stream;
#elif defined RMT_BROADCAST || defined RMT_RCV
	static Stream unicast_stream;
#endif

	// This static field should only be accessed by PacketGroup instances
	// It should not be accessed by the static class, multiple threads,
	// or more than one packet group... for now.
	static int mod_version;
	/* instance fields */
	google::protobuf::Arena arena;
	vector<update_protocol_v3::Update * > packets;
	vector<update_protocol_v3::Update * > ::iterator next_packet;
	int timestamp;
	bool recording;
	bool models_changed;
	string label;
	bool all_sent;
	/* private instance methods */
	update_protocol_v3::Update *newPacket() {
		update_protocol_v3::Update *packet = new update_protocol_v3::Update();
		addPacket(packet);
		return packet;
	}
public:
	/* PacketGroup instance methods allow a user to construct and fill a PacketGroup.
	 * After a PacketGroup is constructed it is set as the latest packet group using setHead.
	 * When a PacketGroup is set as the latest/head packet group it should not be modified and 
	 * its instance methods should no longer be invoked by an outside class.
	*/
	PacketGroup(int _timestamp, bool _recording, bool _models_changed, string _label) {
		timestamp = _timestamp;
		recording = _recording;
		models_changed = _models_changed;
		label = _label;
		all_sent = false;
		packets = vector<update_protocol_v3::Update * > ();
		// Add the first packet
		update_protocol_v3::Update *packet = newPacket();
		assert(packet->ByteSize() < max_packet_bytes);
		// Start the iterator
		next_packet = packets.begin();
	}
	void addPacket(update_protocol_v3::Update *packet) {
		packet->set_mod_version(mod_version++);
		packet->set_time(timestamp);
		packet->set_label(label);
		packets.push_back(packet);
		next_packet = packets.begin();
	}

	void addLiveObject(update_protocol_v3::LiveObject o, bool lhs) {
		addLiveObject(o.label(), true, o.x(), o.y(), o.z(), o.qx(), o.qy(), o.qz(), o.qw(), lhs, o.button_bits());
	}

	void addLiveObject(string label, bool tracking_valid, float x, float y, float z, float qx, float qy, float qz, float qw, bool lhs, int button_bits) {
		update_protocol_v3::LiveObject *liveObj = new update_protocol_v3::LiveObject();
		
		liveObj->set_label(label);

		if (tracking_valid) {
			liveObj->set_x(x);
			liveObj->set_y(y);
			liveObj->set_z(z);

			liveObj->set_qx(qx);
			liveObj->set_qy(qy);
			liveObj->set_qz(qz);
			liveObj->set_qw(qw);
		}
		else {
			liveObj->set_x(0);
			liveObj->set_y(0);
			liveObj->set_z(0);
			liveObj->set_qx(0);
			liveObj->set_qy(0);
			liveObj->set_qz(0);
			liveObj->set_qw(1);
		}

		// TODO: Wiimote/Other interface buttons
		liveObj->set_button_bits(button_bits);
		//if (button_bits>0)
		//	cout << label << ": " << button_bits << endl;
		//liveObj->set_button_bits();

		// not sure about the repeated axisbutton
		//int axis_buttons_size = liveObj->axis_buttons_size();
		//for (int i = 0; i < axis_buttons_size; i++) {
		//	update_protocol_v3::AxisButton * axisBtn = liveObj->add_axis_buttons();
		//}
			
		// not sure about the repeated extradata
		int extra_data_size = liveObj->extra_data_size();
		for (int i = 0; i < extra_data_size; i++) {
			update_protocol_v3::ExtraData * extraDt = liveObj->add_extra_data();
			extraDt->set_label(liveObj->extra_data(i).label());
			extraDt->set_bool_val(liveObj->extra_data(i).bool_val());
			extraDt->set_int_val(liveObj->extra_data(i).int_val());
			extraDt->set_float_val(liveObj->extra_data(i).float_val());
			extraDt->set_double_val(liveObj->extra_data(i).double_val());
			extraDt->set_string_val(liveObj->extra_data(i).string_val());
			extraDt->set_bytes_val(liveObj->extra_data(i).bytes_val());
		}

		update_protocol_v3::Update *current_packet = packets.back();
		if (!(current_packet->ByteSize() + liveObj->ByteSize() < max_packet_bytes)) {
			// Create a new packet to hold the rigid body
			//cout << "creating new packet" << endl;
			current_packet = newPacket();
		}
		current_packet->set_lhs_frame(lhs);
		assert(current_packet->ByteSize() + liveObj->ByteSize() < max_packet_bytes + 100);
		current_packet->mutable_live_objects()->AddAllocated(liveObj);
		assert(current_packet->ByteSize() < max_packet_bytes + 100);
		//cout << "current packet size: " << current_packet->ByteSize() << endl;
	}

	update_protocol_v3::Update *getNextPacketToSend() {
		assert(packets.size() > 0);
		update_protocol_v3::Update *packet = *next_packet;
		// Step forward and reset to the beginning if we are at the end
		next_packet++;
		if (next_packet == packets.end()) {
			all_sent = true;
			next_packet = packets.begin();
		}
		return packet;
	}

	/* Packet group oriented methods */
	/* These methods are thread safe and operator on packets that should not be modified */
	static void send() {
		// Ensure a packet group exists to send
		if (!head) {
			return;
		}
		// Ensure our packet group is not free'd while we send one of its packets
		packet_groups_lock.lock();
		// Get the current packet of the packet group
		update_protocol_v3::Update *packet = head->getNextPacketToSend();
		//cout << head->label << ": sending packet " << head->next_packet-head->packets.begin()+1 << " out of " << head->packets.size() << endl;
		// TODO: Check for wiimotes in this packet and update

		// Fill the buffer
		//cout << "packet size: " << packet->ByteSize() << endl;
		assert(packet->ByteSize() < max_packet_bytes + 128);
		packet->SerializePartialToArray(buffer, max_packet_bytes);
		// Send the buffer
		//std::cout << "sending packet of type: " << packet->label() << std::endl;
		//std::cout << "sending packet: " << packet->mod_version() << std::endl;
		//cout << packet->label() << ": sending " << packet->live_objects_size() << " live objects" << endl;
		//for (update_protocol_v3::LiveObject o : packet->live_objects()) {
		//	cout << o.label() << ", ";
		//}
		//cout << endl;
#ifdef LCL_BROADCAST
		multicast_stream.send(buffer, packet->ByteSize());
#elif defined RMT_BROADCAST || defined RMT_RCV
		unicast_stream.send(buffer, packet->ByteSize());
#endif
		last_label = head->label;

		

		packet_groups_lock.unlock();
	}

	// Important: A PacketGroup must not be modified after it is set as the head
	static void setHead(PacketGroup *newHead) {
		packet_groups_lock.lock();
		packet_groups.push_back(newHead);
		if (head == NULL || head->all_sent) {
			if (newHead->label != last_label) {
				head = newHead;
			}
			else {
				bool set_head = true;
				for (PacketGroup *pg : packet_groups) {
					if (pg->label != newHead->label) {
						set_head = false;
					}
				}
				if (set_head) {
					head = newHead;
				}
			}
		}
		packet_groups_lock.unlock();
	}

	static void clearPacketGroupsBeforeHead() {
		packet_groups_lock.lock();
		vector<int> indexes_to_delete = vector<int>();
		// Find packet groups to delete
		for (int i = 0; i < packet_groups.size(); i++) {
			if (packet_groups[i] != head && packet_groups[i]->label == head->label) {
				indexes_to_delete.push_back(i);
			}
		}
		// Delete them in backwards order
		for (int i = indexes_to_delete.size() - 1; i >= 0; i--) {
			int group_index = indexes_to_delete[i];
			PacketGroup *pg = packet_groups[group_index];
			for (int pi = pg->packets.size() - 1; pi >= 0; pi--) {
				update_protocol_v3::Update *p = pg->packets[pi];
				delete(p);
				pg->packets.erase(pg->packets.begin() + pi);
			}
			delete(packet_groups[group_index]);
			packet_groups.erase(packet_groups.begin() + group_index);
		}
		packet_groups_lock.unlock();
	}
};
/* Initialize PacketGroup static fields */
vector<PacketGroup*> PacketGroup::packet_groups = vector<PacketGroup*>();
string PacketGroup::last_label = "";
PacketGroup * PacketGroup::head = NULL;
std::mutex PacketGroup::packet_groups_lock;
char PacketGroup::buffer[PacketGroup::max_packet_bytes];
#ifdef LCL_BROADCAST
Stream PacketGroup::multicast_stream = Stream("224.1.1.1", 1611, true);
#elif defined RMT_BROADCAST || defined RMT_RCV
Stream PacketGroup::unicast_stream = Stream("128.122.47.161", 1611, false);
#endif

int PacketGroup::mod_version = 0;

int PacketServingThread() {
	while (true) {
		PacketGroup::send();
		PacketGroup::clearPacketGroupsBeforeHead();
		Sleep(1);
	}
	return 0;
}

int PacketReceivingThread() {
	WSAData version;        //We need to check the version.
	WORD mkword = MAKEWORD(2, 2);
	int what = WSAStartup(mkword, &version);
	if (what != 0){
		std::cout << "This version is not supported! - \n" << WSAGetLastError() << std::endl;
	}

	SOCKET soc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	
	BOOL sockoptval = TRUE;
	int sockoptres = setsockopt(soc, SOL_SOCKET, SO_REUSEADDR, (char*)&sockoptval, sizeof(BOOL));
	if (sockoptres != 0) {
		std::cout << "Error in Setting Socket Option: " << WSAGetLastError() << std::endl;
	}
	if (soc == INVALID_SOCKET)
		std::cout << "Creating socket fail\n";
	
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(1615);
	int err = inet_pton(AF_INET, IP_ADDR.c_str(), &addr.sin_addr); // S_ADDR of our IP for the WiFi interface
	//int err = inet_pton(AF_INET, "128.122.47.25", &addr.sin_addr); // S_ADDR of our IP for the WiFi interface
	if (err == SOCKET_ERROR) {
		printf("error assigning address\n");
	}

	int conn = ::bind(soc, (sockaddr*)&addr, sizeof(addr));
	if (conn == SOCKET_ERROR){
		std::cout << "Error - when connecting " << WSAGetLastError() << std::endl;
		closesocket(soc);
		WSACleanup();
	}
	
	int len = 65507;
	char *buf = (char*)malloc(sizeof(char) * len);
	int flags = 0;

	sockaddr_in from_addr;
	from_addr.sin_family = AF_INET;
	from_addr.sin_port = htons(1615);
	err = inet_pton(AF_INET, "172.22.31.129", &from_addr.sin_addr); // S_ADDR of our IP for the WiFi interface
	//err = inet_pton(AF_INET, "128.122.47.25", &from_addr.sin_addr); // S_ADDR of our IP for the WiFi interface
	if (err == SOCKET_ERROR) {
		printf("error assigning address\n");
	}
	
	while (true) {
		int addr_len = sizeof(from_addr);
		int recv_status = recvfrom(soc, buf, len, flags, (sockaddr*)&from_addr, &addr_len);
		if (recv_status == SOCKET_ERROR){
			std::cout << "Error in Receiving: " << WSAGetLastError() << std::endl;
		}

		update_protocol_v3::Update *update = new update_protocol_v3::Update();
		update->ParseFromArray(buf, recv_status);
		delete recvUpdate;
		recvUpdate = update;
		// cout << "received " << recvUpdate->live_objects_size() << " objects." << endl;
		PacketGroup *pg = new PacketGroup(recvUpdate->time(), false, true, update->label());
		if (recvUpdate != NULL) {
			for (int i = 0; i < recvUpdate->live_objects_size(); i++) {
				update_protocol_v3::LiveObject o = recvUpdate->live_objects(i);
				pg->addLiveObject(o, update->lhs_frame());
			}
		}
		PacketGroup::setHead(pg);
	}
	
}
// A NatNet packet has been received
void HandleNatNetPacket(sFrameOfMocapData *data, void *pUserData)
{
	if (idToLabel.size() == 0) {
		//printf("No data descriptions received yet...\n");
		return;
	}
	NatNetClient* pClient = (NatNetClient*)pUserData;
	bool bIsRecording = data->params & 0x01;
	bool bTrackedModelsChanged = data->params & 0x02;
	PacketGroup *pg = new PacketGroup((int)(data->fTimestamp * 1000), false, true, "motive");

	// Rigid Bodies
	for (int i = 0; i < data->nRigidBodies; i++)
	{
		sRigidBodyData rb = data->RigidBodies[i];

		int button_bits = 0;
		string label = idToLabel[rb.ID];
		if (motes.count(label) > 0) {
			motes[label]->RefreshState();
			button_bits = motes[label]->Button.Bits;
		}

		pg->addLiveObject(idToLabel[rb.ID],
			rb.params & 0x01, // tracking valid param
			rb.x, rb.y, rb.z,
			rb.qx, rb.qy, rb.qz, rb.qw,
			true,
			button_bits);
	}
	for (int i = 0; i < data->nSkeletons; i++) {
		for (int j = 0; j < data->Skeletons[i].nRigidBodies; j++) {
			sRigidBodyData rb = data->Skeletons[i].RigidBodyData[j];
			pg->addLiveObject(idToLabel[rb.ID],
				rb.params & 0x01, // tracking valid param
				rb.x, rb.y, rb.z,
				rb.qx, rb.qy, rb.qz, rb.qw,
				true,
				0);
		}
	}
	/*
	for (int i = 0; i < data->nOtherMarkers; i++) {
		float* m = data->OtherMarkers[i];
		pg->addLiveObject("marker",
			1, // tracking valid param
			m[0], m[1], m[2],
			0, 0, 0, 1,
			true,
			0);
		
	}
	*/
	PacketGroup::setHead(pg);
}

int _tmain(int argc, _TCHAR* argv[])
{
	printf("== Holojam server =======---\n");

	// Detects and connects to Wiimotes
	checkForWiimotes();

	// Protobuf setup
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	
	int iResult;
	int iConnectionType = ConnectionType_Multicast;

	// Create NatNet Client
	iResult = CreateClient(iConnectionType);
	if (iResult != ErrorCode_OK)
	{
		printf("Error initializing client.  See log for details.  Exiting");
		return 1;
	}
	else
	{
		printf("Client initialized and ready.\n");
	}

	// Send/receive test request
	printf("[VRServer3] Sending Test Request\n");
	void* response;
	int nBytes;
	iResult = theClient->SendMessageAndWait("TestRequest", &response, &nBytes);
	if (iResult == ErrorCode_OK)
	{
		printf("[VRServer3] Received: %s", (char*)response);
	}
	GetDataDescriptions();

	printf("\nServer is connected to Motive and listening for data...\n");
	int c;
	bool bExit = false;
	int clientsI = 0;
	std::string in_str;
	
	// Start the packet serving thread
	thread packet_serving_thread(PacketServingThread);
	thread packet_receiving_thread(PacketReceivingThread);

	while (!bExit)
	{
		printf("(press the 'h' key for help)\n");
		c = _getch();
		switch (c)
		{
		case 'h':
			printf("r: reset\nq: quit\np: print server info\nd: refresh data descriptions\n");
			break;
		case 'q':
			bExit = true;
			break;
		case 'r':
			resetClient();
			break;
		case 'p':
			sServerDescription ServerDescription;
			memset(&ServerDescription, 0, sizeof(ServerDescription));
			theClient->GetServerDescription(&ServerDescription);
			if (!ServerDescription.HostPresent)
			{
				printf("Unable to connect to server. Host not present. Exiting.");
				return 1;
			}
			break;
		case 'd':
			checkForWiimotes();
			GetDataDescriptions();
			break;
		default:
			printf("unrecognized keycode: %c", c);
			break;
		}
	}

	// Done - clean up.
	packet_receiving_thread.detach();
	packet_serving_thread.detach();
	theClient->Uninitialize();
	return ErrorCode_OK;
}

void checkForWiimotes() {

	if (!motes.empty()) {

		auto itr = motes.begin();
		while (itr != motes.end()) {
			auto toErase = itr;
			++itr;
			delete(toErase->second);
			motes.erase(toErase);
		}
	}

	/* WiiMotes */
	printf("\nLooking for wiimotes...");
	detected = 0;
	while (detected < 7)
	{
		wiimote *next = new wiimote;
		if (!next->Connect(wiimote::FIRST_AVAILABLE)) {
			break;
		}
		detected += 1;
		string label = mote_id_to_label(next->UniqueID);
		motes[label] = next;
		next->SetLEDs(0x0f);
		printf("\nConnected to wiimote #%u: %" PRIx64, detected - 1, next->UniqueID);
		printf("\nname: %s", mote_id_to_label(next->UniqueID));
	}
	printf("\nNo more remotes found\n");
}



/* MOTIVE */
// Establish a NatNet Client connection
int CreateClient(int iConnectionType)
{
	// release previous server
	if (theClient)
	{
		theClient->Uninitialize();
		delete theClient;
	}

	// create NatNet client
	theClient = new NatNetClient(iConnectionType);
	unsigned char ver[4];
	theClient->NatNetVersion(ver);
	printf("(NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

	theClient->SetMessageCallback(MessageHandler);
	theClient->SetVerbosityLevel(Verbosity_Error);
	theClient->SetDataCallback(DataHandler, theClient);

	// Init Client and connect to NatNet server
	// to use NatNet default port assigments
	int retCode = theClient->Initialize("127.0.0.1", "127.0.0.1");
//	int retCode = theClient->Initialize("128.122.47.25", "128.122.47.25");
	// to use a different port for commands and/or data:
	//int retCode = theClient->Initialize(szMyIPAddress, szServerIPAddress, MyServersCommandPort, MyServersDataPort);
	if (retCode != ErrorCode_OK)
	{
		printf("Unable to connect to server.  Error code: %d. Exiting", retCode);
		return ErrorCode_Internal;
	}
	else
	{
		// Print server info
		sServerDescription ServerDescription;
		memset(&ServerDescription, 0, sizeof(ServerDescription));
		theClient->GetServerDescription(&ServerDescription);

		printf("Motive Server application info:\n");
		printf("Application: %s (ver. %d.%d.%d.%d)\n", ServerDescription.szHostApp, ServerDescription.HostAppVersion[0],
			ServerDescription.HostAppVersion[1], ServerDescription.HostAppVersion[2], ServerDescription.HostAppVersion[3]);
		printf("NatNet Version: %d.%d.%d.%d\n", ServerDescription.NatNetVersion[0], ServerDescription.NatNetVersion[1],
			ServerDescription.NatNetVersion[2], ServerDescription.NatNetVersion[3]);
		//printf("Client IP:%s\n", "128.122.47.25");
		//printf("Server IP:%s\n", "128.122.47.25");
		printf("Client IP:%s\n", "192.168.0.1");
		printf("Server IP:%s\n", "192.168.0.1");
		printf("Server Name:%s\n\n", ServerDescription.szHostComputerName);
	}

	return ErrorCode_OK;

}

/* Motive error handling */
void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData)
{
	NatNetClient* pClient = (NatNetClient*)pUserData;
	HandleNatNetPacket(data, pUserData);
}
void __cdecl MessageHandler(int msgType, char* msg)
{
	printf("\n%s\n", msg);
}
void resetClient()
{
	int iSuccess;
	printf("\n\nre-setting Client\n\n.");
	iSuccess = theClient->Uninitialize();
	if (iSuccess != 0) {
		printf("error un-initting Client\n");
	}
	//iSuccess = theClient->Initialize("128.122.47.25", "128.122.47.25");
	iSuccess = theClient->Initialize("127.0.0.1", "127.0.0.1");
	if (iSuccess != 0) {
		printf("error re-initting Client\n");
	}
}

