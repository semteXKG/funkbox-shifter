#include <stdint.h>
#include <WiFi.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "proto/message.pb.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#define PRIMARY 1
#if (PRIMARY)
    #define CONFIG_SSID "funkbox-car-prim"
    #define CONFIG_PWD "funkbox-car-prim"
#else
    #define CONFIG_SSID "funkbox-car-sec"
    #define CONFIG_PWD "funkbox-car-sec"
#endif

#define UDP_PORT 3333

#define MULTICAST_TTL 1
#define MULTICAST_IPV4_ADDR "232.10.11.12"

Proto_Mcu_Data persistent_state;
Proto_Mcu_Data incoming_state; 

void socketserver_send_lora(Proto_LoRa_Data message);
void socketserver_send(Proto_Message message);
void handle_proto(uint8_t* message, size_t length);
void handle_command(char* message);
Proto_Update_Data create_status_update();

//
//
// WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN
//
//

void wlan_setup() {
  Serial.print("[WIN] Starting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(CONFIG_SSID, CONFIG_PWD);
  int retryCnt = 0;
  
  do {
    delay(2000);
    Serial.print(".");
  } while (WiFi.status() != WL_CONNECTED || ++retryCnt == 10);

  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  Serial.printf("[WIN] Wlan connected\nIP: %s\n", WiFi.localIP().toString().c_str());
}


int sock;

/* Add a socket to the IPV4 multicast group */
static int socket_add_ipv4_multicast_group(esp_netif_t* interface, int sock, bool assign_source_if)
{
    struct ip_mreq imreq = { 0 };
    struct in_addr iaddr = { 0 };
    int err = 0;
    // Configure source interface

    esp_netif_ip_info_t ip_info = { 0 };
    err = esp_netif_get_ip_info(interface, &ip_info);
    if (err != ESP_OK) {
        Serial.printf("[WIN] Failed to get IP address info. Error 0x%x\n", err);
        goto err;
    }
    inet_addr_from_ip4addr(&iaddr, &ip_info.ip);

    // Configure multicast address to listen to
    err = inet_aton(MULTICAST_IPV4_ADDR, &imreq.imr_multiaddr.s_addr);
    if (err != 1) {
        Serial.printf("[WIN] Configured IPV4 multicast address '%s' is invalid.\n", MULTICAST_IPV4_ADDR);
        // Errors in the return value have to be negative
        err = -1;
        goto err;
    }
    Serial.printf("[WIN] Configured IPV4 Multicast address %s\n", inet_ntoa(imreq.imr_multiaddr.s_addr));
    if (!IP_MULTICAST(ntohl(imreq.imr_multiaddr.s_addr))) {
        Serial.printf("[WIN] Configured IPV4 multicast address '%s' is not a valid multicast address. This will probably not work.\n", MULTICAST_IPV4_ADDR);
    }

    if (assign_source_if) {
        // Assign the IPv4 multicast source interface, via its IP
        // (only necessary if this socket is IPV4 only)
        err = setsockopt(sock, IPPROTO_IP, IP_MULTICAST_IF, &iaddr,
                         sizeof(struct in_addr));
        if (err < 0) {
            Serial.printf("[WIN] Failed to set IP_MULTICAST_IF. Error %d\n", errno);
            goto err;
        }
    }

    err = setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                         &imreq, sizeof(struct ip_mreq));
    if (err < 0) {
        Serial.printf("[WIN] Failed to set IP_ADD_MEMBERSHIP. Error %d\n", errno);
        goto err;
    }

 err:
    return err;
}
static int create_multicast_ipv4_socket(esp_netif_t *interface)
{
  struct sockaddr_in saddr = {0};
  int sock = -1;
  int err = 0;

  sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (sock < 0)
  {
    Serial.printf("[WIN] Failed to create socket. Error %d\n", errno);
    return -1;
  }

  // Bind the socket to any address
  saddr.sin_family = PF_INET;
  saddr.sin_port = htons(UDP_PORT);
  saddr.sin_addr.s_addr = htonl(INADDR_ANY);
  err = bind(sock, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in));

  uint8_t ttl = MULTICAST_TTL;
  if (err < 0)
  {
    Serial.printf("[WIN] Failed to bind socket. Error %d\n", errno);
    goto err;
  }

  // Assign multicast TTL (set separately from normal interface TTL)
  setsockopt(sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(uint8_t));
  if (err < 0)
  {
    Serial.printf("[WIN] Failed to set IP_MULTICAST_TTL. Error %d\n", errno);
    goto err;
  }

  // this is also a listening socket, so add it to the multicast
  // group for listening...
  err = socket_add_ipv4_multicast_group(interface, sock, true);
  if (err < 0)
  {
    goto err;
  }

  // All set, socket is configured for sending and receiving
  return sock;

err:
  close(sock);
  return -1;
}

static void listen(void *pvParameters)
{
  esp_netif_t *interface = (esp_netif_t *)pvParameters;
  while (1)
  {

    sock = create_multicast_ipv4_socket(interface);
    if (sock < 0)
    {
      Serial.printf("[WIN] Failed to create IPv4 multicast socket\n");
    }

    if (sock < 0)
    {
      // Nothing to do!
      vTaskDelay(5 / portTICK_PERIOD_MS);
      continue;
    }

    // set destination multicast addresses for sending from these sockets
    struct sockaddr_in sdestv4 = {
        .sin_family = PF_INET,
        .sin_port = htons(UDP_PORT),
    };
    // We know this inet_aton will pass because we did it above already
    inet_aton(MULTICAST_IPV4_ADDR, &sdestv4.sin_addr.s_addr);

    // Loop waiting for UDP received, and sending UDP packets if we don't
    // see any.
    int err = 1;
    while (err > 0)
    {
      struct timeval tv = {
          .tv_sec = 2,
          .tv_usec = 0,
      };
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(sock, &rfds);

      int s = select(sock + 1, &rfds, NULL, NULL, &tv);
      if (s < 0)
      {
        Serial.printf("[WIN] Select failed: errno %d\n", errno);
        err = -1;
        break;
      }
      else if (s > 0)
      {
        if (FD_ISSET(sock, &rfds))
        {
          // Incoming datagram received
          uint8_t recvbuf[500];
          char raddr_name[32] = {0};
          struct sockaddr_storage raddr; // Large enough for both IPv4 or IPv6
          socklen_t socklen = sizeof(raddr);
          int len = recvfrom(sock, recvbuf, sizeof(recvbuf) - 1, 0,
                             (struct sockaddr *)&raddr, &socklen);
          if (len < 0)
          {
            Serial.printf("[WIN] multicast recvfrom failed: errno %d\n", errno);
            err = -1;
            break;
          }

          if (raddr.ss_family == PF_INET)
          {
            inet_ntoa_r(((struct sockaddr_in *)&raddr)->sin_addr,
                        raddr_name, sizeof(raddr_name) - 1);
          }
          //Serial.printf("[WIN] Rcv %dB from %s: \n", len, raddr_name);
          handle_proto(recvbuf, len);
        }
      }
    }
  }

  Serial.printf("Shutting down socket and restarting...");
  shutdown(sock, 0);
  close(sock);
}


void handle_proto(uint8_t* message, size_t length) {
  Proto_Message decoded_message = Proto_Message_init_zero;
  pb_istream_t stream = pb_istream_from_buffer(message, length);
  bool status = pb_decode(&stream, Proto_Message_fields, &decoded_message);
  if(!status) {
    Serial.printf("Could not decode message");
    return;
  }
  if(decoded_message.has_mcu_data) {
    persistent_state = decoded_message.mcu_data;
  }
}

Proto_Update_Data create_status_update() {
  Proto_Update_Data data = Proto_Update_Data_init_zero;
  data.has_gas_sensor = persistent_state.has_gas;
  data.has_oil_sensor = persistent_state.has_oil;
  data.has_water_sensor = persistent_state.has_water;
  data.has_stint_data = persistent_state.has_stint;
  data.has_lap_data = persistent_state.has_lap_data;

  data.has_gps_data = persistent_state.has_gps;
  data.gps_data = persistent_state.gps;

  data.gas_sensor = persistent_state.gas;
  data.water_sensor = persistent_state.water;
  data.oil_sensor = persistent_state.oil;
  data.stint_data = persistent_state.stint;
  data.lap_data = persistent_state.lap_data;

  return data;
}

void socketserver_start() {
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  xTaskCreate(&listen, "listen_task", 8192, netif, 5, NULL);
}


//
//
// MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN
//
//

void print_status() {
    Serial.printf("\n\n----------------------------------------------------------\n");
    Serial.printf("|--------------------------------------------------------|\n");
    #if PRIMARY
        Serial.printf("|----PRIMARY---PRIMARY---PRIMARY---PRIMARY---PRIMARY-----|\n");
    #else
        Serial.printf("|-----SECONDARY---SECONDARY---SECONDARY---SECONDARTY-----|\n");
    #endif
    Serial.printf("|--------------------------------------------------------|\n");
    Serial.printf("----------------------------------------------------------\n");
}

SET_LOOP_TASK_STACK_SIZE(1024 * 16);

void setup()
{
  wlan_setup();
  socketserver_start();
  print_status();
}

void loop() 
{
}
