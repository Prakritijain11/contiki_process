/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup cc26xx-web-demo
 * @{
 *
 * \file
 *   Main module for the CC26XX web demo. Activates on-device resources,
 *   takes sensor readings periodically and caches them for all other modules
 *   to use.
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "contiki-net.h"
#include "rest-engine.h"
#include "board-peripherals.h"
#include "lib/sensors.h"
#include "lib/list.h"
#include "sys/process.h"
#include "net/ipv6/sicslowpan.h"
#include "button-sensor.h"
#include "batmon-sensor.h"
#include "httpd-simple.h"
#include "cc26xx-web-demo.h"
#include "mqtt-client.h"
#include "coap-server.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ti-lib.h"
/*---------------------------------------------------------------------------*/
PROCESS_NAME(cetic_6lbr_client_process);
PROCESS(cc26xx_web_demo_process, "CC26XX Web Demo");
/*---------------------------------------------------------------------------*/
/*
 * Update sensor readings in a staggered fashion every SENSOR_READING_PERIOD
 * ticks + a random interval between 0 and SENSOR_READING_RANDOM ticks
 */
#define SENSOR_READING_PERIOD (CLOCK_SECOND * 20)
#define SENSOR_READING_RANDOM (CLOCK_SECOND << 4)

//struct ctimer batmon_timer;

#if 0
#if CC26XX_WEB_DEMO_CONF_SENSOR_MULTIVARIABLE
struct ctimer multivariable_timer;
#endif

#if CC26XX_WEB_DEMO_CONF_SENSOR_CARMEN
struct ctimer carmen_timer;
#endif
#endif
/*---------------------------------------------------------------------------*/
/* Provide visible feedback via LEDS while searching for a network */
#define NO_NET_LED_DURATION        (CC26XX_WEB_DEMO_NET_CONNECT_PERIODIC >> 1)

static struct etimer et;
//static struct ctimer ct;
static struct etimer MQTT_shutdown_timer;
/*---------------------------------------------------------------------------*/
/* Parent RSSI functionality */
#if CC26XX_WEB_DEMO_READ_PARENT_RSSI
static struct uip_icmp6_echo_reply_notification echo_reply_notification;
static struct etimer echo_request_timer;
int def_rt_rssi = 0;
#endif
/*---------------------------------------------------------------------------*/
process_event_t cc26xx_web_demo_publish_event;
process_event_t cc26xx_web_demo_config_loaded_event;
process_event_t cc26xx_web_demo_load_config_defaults;
/*---------------------------------------------------------------------------*/
/* Saved settings on flash: store, offset, magic */
#define CONFIG_FLASH_OFFSET        0
#define CONFIG_MAGIC      0xCC265002

cc26xx_web_demo_config_t cc26xx_web_demo_config;
/*---------------------------------------------------------------------------*/
/* A cache of sensor values. Updated periodically or upon key press */
LIST(sensor_list);
/*---------------------------------------------------------------------------*/
/* The objects representing sensors used in this demo */
#define DEMO_SENSOR(name, type, descr, xml_element, form_field, units) \
  cc26xx_web_demo_sensor_reading_t name##_reading = \
  { NULL, 0, 0, descr, xml_element, form_field, units, type, 1, 1 }


/*---------------------------------------------------------------------------*/
void MQTT_ShutdownTimerOn( void )
{
//  etimer_set(&MQTT_shutdown_timer, (CLOCK_SECOND * 30)); // Approximately 1 minute
  etimer_set(&MQTT_shutdown_timer, (4)); 
}

/*---------------------------------------------------------------------------*/
void MQTT_ShutdownTimerOff( void )
{
  etimer_stop(&MQTT_shutdown_timer);
}

/*---------------------------------------------------------------------------*/
static void
publish_led_off(void *d)
{
  leds_off(CC26XX_WEB_DEMO_STATUS_LED);
}


/*---------------------------------------------------------------------------*/
/* Don't start everything here, we need to dictate order of initialisation */
AUTOSTART_PROCESSES(&cc26xx_web_demo_process);
/*---------------------------------------------------------------------------*/
int
cc26xx_web_demo_ipaddr_sprintf(char *buf, uint8_t buf_len,
                               const uip_ipaddr_t *addr)
{
  uint16_t a;
  uint8_t len = 0;
  int i, f;
  for(i = 0, f = 0; i < sizeof(uip_ipaddr_t); i += 2) {
    a = (addr->u8[i] << 8) + addr->u8[i + 1];
    if(a == 0 && f >= 0) {
      if(f++ == 0) {
        len += snprintf(&buf[len], buf_len - len, "::");
      }
    } else {
      if(f > 0) {
        f = -1;
      } else if(i > 0) {
        len += snprintf(&buf[len], buf_len - len, ":");
      }
      len += snprintf(&buf[len], buf_len - len, "%x", a);
    }
  }

  return len;
}

/*---------------------------------------------------------------------------*/
void
cc26xx_web_demo_restore_defaults(void)
{
  cc26xx_web_demo_sensor_reading_t *reading = NULL;

  leds_on(LEDS_ALL);

  for(reading = list_head(sensor_list);
      reading != NULL;
      reading = list_item_next(reading)) {
    reading->publish = 1;
  }

#if CC26XX_WEB_DEMO_MQTT_CLIENT
  process_post_synch(&mqtt_client_process,
                     cc26xx_web_demo_load_config_defaults, NULL);
#endif

#if CC26XX_WEB_DEMO_NET_UART
  process_post_synch(&net_uart_process, cc26xx_web_demo_load_config_defaults,
                     NULL);
#endif

  leds_off(LEDS_ALL);
}
/*---------------------------------------------------------------------------*/
static int
defaults_post_handler(char *key, int key_len, char *val, int val_len)
{
  if(key_len != strlen("defaults") ||
     strncasecmp(key, "defaults", strlen("defaults")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  cc26xx_web_demo_restore_defaults();

  return HTTPD_SIMPLE_POST_HANDLER_OK;
}

/*---------------------------------------------------------------------------*/
#if CC26XX_WEB_DEMO_READ_PARENT_RSSI
static int
ping_interval_post_handler(char *key, int key_len, char *val, int val_len)
{
  int rv = 0;

  if(key_len != strlen("ping_interval") ||
     strncasecmp(key, "ping_interval", strlen("ping_interval")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  rv = atoi(val);

  if(rv < CC26XX_WEB_DEMO_RSSI_MEASURE_INTERVAL_MIN ||
     rv > CC26XX_WEB_DEMO_RSSI_MEASURE_INTERVAL_MAX) {
    return HTTPD_SIMPLE_POST_HANDLER_ERROR;
  }

  cc26xx_web_demo_config.def_rt_ping_interval = rv * CLOCK_SECOND;

  return HTTPD_SIMPLE_POST_HANDLER_OK;
}
#endif
/*---------------------------------------------------------------------------*/
//HTTPD_SIMPLE_POST_HANDLER(sensor, sensor_readings_handler);
HTTPD_SIMPLE_POST_HANDLER(defaults, defaults_post_handler);

#if CC26XX_WEB_DEMO_READ_PARENT_RSSI
HTTPD_SIMPLE_POST_HANDLER(ping_interval, ping_interval_post_handler);
/*---------------------------------------------------------------------------*/
static void
echo_reply_handler(uip_ipaddr_t *source, uint8_t ttl, uint8_t *data,
                   uint16_t datalen)
{
  if(uip_ip6addr_cmp(source, uip_ds6_defrt_choose())) {
    def_rt_rssi = sicslowpan_get_last_rssi();
  }
}
/*---------------------------------------------------------------------------*/
static void
ping_parent(void)
{
  if(uip_ds6_get_global(ADDR_PREFERRED) == NULL) {
    return;
  }

  uip_icmp6_send(uip_ds6_defrt_choose(), ICMP6_ECHO_REQUEST, 0,
                 CC26XX_WEB_DEMO_ECHO_REQ_PAYLOAD_LEN);
}
#endif

/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
const cc26xx_web_demo_sensor_reading_t *
cc26xx_web_demo_sensor_first()
{
  return list_head(sensor_list);
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc26xx_web_demo_process, ev, data)
{
  PROCESS_BEGIN();

  printf("CC26XX Web Demo Process\n");


  cc26xx_web_demo_publish_event = process_alloc_event();
  cc26xx_web_demo_config_loaded_event = process_alloc_event();
  cc26xx_web_demo_load_config_defaults = process_alloc_event();

  /* Start all other (enabled) processes first */
  //process_start(&httpd_simple_process, NULL);

/*#if CC26XX_WEB_DEMO_COAP_SERVER
  process_start(&coap_server_process, NULL);
#endif*///

/*#if CC26XX_WEB_DEMO_6LBR_CLIENT
  process_start(&cetic_6lbr_client_process, NULL);
#endif*/

#if CC26XX_WEB_DEMO_MQTT_CLIENT
  process_start(&mqtt_client_process, NULL);
#endif

/*#if CC26XX_WEB_DEMO_NET_UART
  process_start(&net_uart_process, NULL);
#endif*/

  /*
   * Now that processes have set their own config default values, set our
   * own defaults and restore saved config from flash...
   */
  cc26xx_web_demo_config.sensors_bitmap = 0xFFFFFFFF; /* all on by default */
  cc26xx_web_demo_config.def_rt_ping_interval =  CC26XX_WEB_DEMO_DEFAULT_RSSI_MEAS_INTERVAL;

  /*
   * Notify all other processes (basically the ones in this demo) that the
   * configuration has been loaded from flash, in case they care
   */
  process_post(PROCESS_BROADCAST, cc26xx_web_demo_config_loaded_event, NULL);

  //httpd_simple_register_post_handler(&sensor_handler);
  httpd_simple_register_post_handler(&defaults_handler);

#if CC26XX_WEB_DEMO_READ_PARENT_RSSI
  httpd_simple_register_post_handler(&ping_interval_handler);

  def_rt_rssi = 0x8000000;
  uip_icmp6_echo_reply_callback_add(&echo_reply_notification,
                                    echo_reply_handler);
  etimer_set(&echo_request_timer, CC26XX_WEB_DEMO_NET_CONNECT_PERIODIC);
#endif

  etimer_set(&et, CC26XX_WEB_DEMO_NET_CONNECT_PERIODIC);
  /*
   * Update all sensor readings on a configurable sensors_event
   * (e.g a button press / or reed trigger)
   */
  while(1) {

#if CC26XX_WEB_DEMO_SHUTDOWN_MODE
//    if(ev == PROCESS_EVENT_TIMER && etimer_expired(&MQTT_shutdown_timer) ) {

    if(true == mqtt_GetMsgPublished()) {
//      mqtt_ClearMsgPublished();
      // Shut down the board here
          lpm_shutdown( BOARD_IOID_RTC_CLK, IOC_IOPULL_UP, IOC_WAKE_ON_LOW );

    }
//    }

#endif
//    if(ev == PROCESS_EVENT_TIMER && etimer_expired(&et)) {
//      if(uip_ds6_get_global(ADDR_PREFERRED) == NULL) {
//        leds_on(CC26XX_WEB_DEMO_STATUS_LED);
//        ctimer_set(&ct, NO_NET_LED_DURATION, publish_led_off, NULL);
//        etimer_set(&et, CC26XX_WEB_DEMO_NET_CONNECT_PERIODIC);
//      }
//    }

#if CC26XX_WEB_DEMO_READ_PARENT_RSSI
    if(ev == PROCESS_EVENT_TIMER && etimer_expired(&echo_request_timer)) {
      if(uip_ds6_get_global(ADDR_PREFERRED) == NULL) {
        etimer_set(&echo_request_timer, CC26XX_WEB_DEMO_NET_CONNECT_PERIODIC);

      } else {
        ping_parent();
        etimer_set(&echo_request_timer, cc26xx_web_demo_config.def_rt_ping_interval);
      }
    }
#endif
    PROCESS_YIELD();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 */
