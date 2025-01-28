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
#include "FastLED.h"

char WLAN_SSID_RES[50] = {};
char WLAN_PWD_RES[50] = {};

#define UDP_PORT 3333

#define MULTICAST_TTL 1
#define MULTICAST_IPV4_ADDR "232.10.11.12"

#define NUM_LEDS 10
#define DATA_PIN 12
CRGB leds[NUM_LEDS];
CRGB led_colors[] = { CRGB::Blue, CRGB::Green, CRGB::Green, CRGB::Yellow, CRGB::Yellow, CRGB::Yellow, CRGB::Red, CRGB::Red, CRGB::Red, CRGB::Red};
int rpm_on[] = {500, 2000, 3000, 3500, 4000, 4500, 5000, 5300, 5500, 6000};

Proto_Mcu_Data persistent_state;
Proto_Mcu_Data incoming_state; 

void handle_proto(uint8_t* message, size_t length);
Proto_Update_Data create_status_update();

//
//
// WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN
//
//
void extract_credentials() {
  strcpy(WLAN_SSID_RES, CONFIG_SSID);
  strcpy(WLAN_PWD_RES, CONFIG_PWD);

  #if PRIMARY
    strcat(WLAN_SSID_RES, "prim");
    strcat(WLAN_PWD_RES, "prim");
  #else
    strcat(WLAN_SSID_RES, "sec");
    strcat(WLAN_PWD_RES, "sec");
  #endif
}

void wlan_setup() {
  Serial.print("[WIN] Starting");
  extract_credentials();
  Serial.printf("WLAN User %s, PW %s \n", WLAN_SSID_RES, WLAN_PWD_RES);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WLAN_SSID_RES, WLAN_PWD_RES);
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
    //handle_data_update(decoded_message.mcu_data);
    persistent_state = decoded_message.mcu_data;
  }
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

SET_LOOP_TASK_STACK_SIZE(1024 * 16);

int create_checksum(Proto_Mcu_Data data) {
  int event_chksum = 0;
  int command_chksum = 0; 
  for (int i = 0; i < data.events_count; i++) {
    event_chksum += data.events[i].id;
  }

  for (int i = 0; i < data.incoming_commands_count; i++) {
    command_chksum += data.incoming_commands->id;
  }

  return data.lap_data.lap_no + event_chksum + command_chksum;
}

long new_event_since = -1;
long old_amount = -2;
boolean should_show_new_event(int amount) {
  if (old_amount == -2) {
    old_amount = amount;
    return false;
  }

  if (old_amount != amount) {
    old_amount = amount;
    new_event_since = esp_timer_get_time();
  }

  if(new_event_since == -1) {
    return false;
  }

  if(new_event_since + 3000000 < esp_timer_get_time()) {
    new_event_since = -1;
    return false;
  }
  return true;
}

long flashing_since = -1;
boolean should_show_color(uint32_t rpm, int32_t red_flash) {
  if (rpm < red_flash) {
    flashing_since = -1;
    return true;
  }

  if (flashing_since == -1) {
    flashing_since = esp_timer_get_time();
  }

  long delta = esp_timer_get_time() - flashing_since;
  Serial.printf("Delta is: %ld\n", delta);
  return (((delta / (1000*250)) % 2) == 0);
}

void left_to_right(Proto_Mcu_Data data) {
  Proto_Shiftlight_Config config = data.shiftlight_config;
  bool show = should_show_color(data.odb2.rpm, config.rpm_red_flash);
  for (int i = 0; i < NUM_LEDS; i++) {
    if(data.odb2.rpm > config.rpm_limits[i]) {
      leds[i] = show ? led_colors[i] : CRGB::Black;
    } else {
      leds[i] = CRGB::Black;
    }
  }

  if (should_show_new_event(create_checksum(data))) {
      leds[0] = CRGB::White;
      leds[1] = CRGB::White;
      leds[8] = CRGB::White;
      leds[9] = CRGB::White;
  }
}

void both_sides(Proto_Mcu_Data data) {
  Proto_Shiftlight_Config config = data.shiftlight_config;
  bool show = should_show_color(data.odb2.rpm, config.rpm_red_flash);
  for (int i = 0; i < NUM_LEDS / 2; i++) {
    if(data.odb2.rpm > config.rpm_limits[i*2]) {
      leds[i] = show ? led_colors[i*2] : CRGB::Black;
      leds[NUM_LEDS-1-i] = show ? led_colors[i*2] : CRGB::Black;
    } else {
      leds[i] = CRGB::Black;
      leds[NUM_LEDS-1-i] = CRGB::Black;
    }
  }
    
  if (should_show_new_event(create_checksum(data))) {
      leds[0] = CRGB::White;
      leds[1] = CRGB::White;
      leds[8] = CRGB::White;
      leds[9] = CRGB::White;
  }
}

void update_leds(Proto_Mcu_Data new_data) {
  Proto_Shiftlight_Config config = new_data.shiftlight_config;
  FastLED.setBrightness(config.brightness);
  switch (config.mode) {
    case Shiftlight_Mode_LEFT_RIGHT:
      left_to_right(new_data);
    break;
    case Shiftlight_Mode_BOTH_SIDES:
      both_sides(new_data);
    break;
    default:
    break;
  }
  
  FastLED.show();
}

void fastled_setup() {
  FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(100);
}

void update_leds(void* pv) {
  while(true) {
    update_leds(persistent_state);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.printf("Starting up\n");
  fastled_setup(); 
  wlan_setup();
  socketserver_start();
  xTaskCreate(&update_leds, "update LEDS", 8192, NULL, 5, NULL);
}

void loop() 
{
  
}
