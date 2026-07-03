#include "wifi_server.h"
#include "math.h"

WiFiServerManager::WiFiServerManager()
{
  wifi_connection_state = false;
  tcp_connection_state = false;
  tcp_state = DISCONNECTED;
  udp_pairing_state = false;

  memset(incomingPacket, 0, sizeof(incomingPacket));    
}

RecData_t WiFiServerManager::read_data(String data)
{
  String rec_data[7];
  String data_update;
  RecData_t rec;
  uint8_t index = 0;
  // const char charArray;
  data_update = data;

  while (data_update.indexOf('|') != -1)
  {
    rec_data[index] = data_update.substring(0, data_update.indexOf('|'));  /* extract the string */
    data_update = data_update.substring(data_update.indexOf('|') + 1);       /* update the string, removing the extracted substring and the separator */
    index++;      /* update the index */
  }
  rec_data[index] = data_update;
  
  // char charArray = rec_data[0].c_str();      /* convert to C string form */

  if(rec_data[0] == "C") {
    rec.mode = MINIHEXA_MOVING_CONTROL;
    for(uint8_t i = 0; i < 3; i++) {
      rec.data[i] = static_cast<uint8_t>(atoi(rec_data[i + 1].c_str()));
    }
  }
  else if(rec_data[0] == "F") {
    rec.mode = MINIHEXA_POSE_CONTROL;
    for(uint8_t i = 0; i < 6; i++) {
      rec.data[i] = static_cast<uint8_t>(atoi(rec_data[1 + i].c_str()));
    }    
  }
  else {
    rec.mode = MINIHEXA_NULL;
  }

  return rec;
}

/* returns false if not connected to the corresponding wifi; returns true and enters WIFI data-receive mode if connected */
bool WiFiServerManager::begin()
{
  if((WiFi.status() == WL_CONNECTED) && (wifi_connection_state == false))
  {
    ESP_LOGI("WIFI", "Connected to WIFI!\n");
    String macAddress = WiFi.macAddress();
    macAddress.replace(":", "");           // remove the colons from the MAC address to get 240AC4123456
    clientAddress = "MINIHEXA:" + macAddress;
    cClientAddress = clientAddress.c_str();
    udpClient.begin(UDP_LISTENER_PORT);
    tcpServer.begin(TCP_LISTENER_PORT);
    tcpClient.setTimeout(0);
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    selfIP = WiFi.localIP();
    wifi_connection_state = true;
    ESP_LOGI("WIFI", "Local IPAddress: %d.%d.%d.%d\n", selfIP[0], selfIP[1], selfIP[2], selfIP[3]);
  }
  return wifi_connection_state;
}

/* after entering WIFI mode, if the UDP data pairing succeeds, enter data transfer mode; after entering data transfer mode, udp_pairing_state and wifi_connection_state must be reset to false*/
bool WiFiServerManager::udp_server()
{
  int packetSize = 0;
  static uint32_t tick_start = 0;
  
  if(millis() - tick_start > 1000) {
    // check whether there is new UDP broadcast data
    packetSize = udpClient.parsePacket();
    tick_start = millis();
  }

  if (packetSize)
  {
    int len = udpClient.read(incomingPacket, 255);    
    if (len > 0)
    {
        incomingPacket[len] = 0; // ensure the string is terminated
    }
    if (strcmp(incomingPacket, "LOBOT_NET_DISCOVER") == 0)
    {
      // get the IP address of the broadcast sender
      clientIP = udpClient.remoteIP();
      ESP_LOGI("WIFI", "Client IPAddress: %d.%d.%d.%d\n", clientIP[0], clientIP[1], clientIP[2], clientIP[3]);
      udpClient.beginPacket(clientIP, UDP_SEND_PORT);
      udpClient.print(clientAddress);  // send the MAC address
      udpClient.endPacket();
      // memset(incomingPacket, 0, sizeof(incomingPacket));   
      udp_pairing_state = true;
    }
  }
  return udp_pairing_state;
}

bool WiFiServerManager::tcp_server()
{
  // check whether there is a new client connection
  if (!tcpClient && (tcp_connection_state == false))
  {
    tcpClient = tcpServer.available();  // check whether a client is connected
    
    if (tcpClient)
    {
      parameters_reset();
      ESP_LOGI("WIFI", "TCP Client connected!\n");
      tcp_connection_state = true;
    }
    else
    {
      tcp_state = DISCONNECTED;
      return false;
    }
  }  
  // if a client is already connected, check whether there is data to read
  if(tcp_connection_state == true)
  {
    if(!tcpClient.connected())
    {
      ESP_LOGI("WIFI", "Client disconnected!\n");
      tcp_connection_state = false;
      udpClient.stop();  // close the client connection
      tcpClient.stop();  // close the client connection
      tcp_state = DISCONNECTED;
      return false;
    }
    if (tcpClient.available())
    {
      // tcp_state = 2;
      size_t bytesRead = tcpClient.readBytesUntil('&', rec_buffer, sizeof(rec_buffer) - 1);
      rec_buffer[bytesRead] = '\0';  // append the string terminator
      String fullData = String(rec_buffer);  // convert rec_buffer to a String
      rec = WiFiServerManager::read_data(fullData);
      // printf("fullData %s\n", fullData.c_str());
      ESP_LOGI("Received from client", "%s", fullData.c_str());
      // send response data to the client
      // String response = "ESP32 received: " + clientData;
      // tcpClient.println(response);
      tcp_state = RECEIVEDATA;
      return true;
    } 
    else
    {
      tcp_state = CONNECTED;
      return true;
    }
  }
}

void WiFiServerManager::parameters_reset()
{
  wifi_connection_state = false;
  tcp_connection_state = false;
  udp_pairing_state = false;

  memset(incomingPacket, 0, sizeof(incomingPacket));    
}

