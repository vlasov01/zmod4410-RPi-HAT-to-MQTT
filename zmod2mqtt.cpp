/**
 * @file   zmod2mqtt.cpp
 * @brief  publishing ZMOD4410 Indoor Air Quality Raspberry Pi Hat data to MQTT broker
 * @version 0.1.0
 * @author Sergey Vlasov
 */

#include "zmodhat.h"
#include <mosquitto.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

struct mosquitto *mosq = NULL;
char *topic = NULL;
void mqtt_setup() {

  char *host = (char *)"mqtt";
  // char *host = (char*)"localhost";
  // char *host = "broker.hivemq.com";
  int port = 1883;
  int keepalive = 60;
  bool clean_session = true;
  topic = (char *)"office/sensor1";

  mosquitto_lib_init();
  mosq = mosquitto_new(NULL, clean_session, NULL);
  if (!mosq) {
    fprintf(stderr, "Error: Out of memory.\n");
    exit(1);
  }

  if (mosquitto_connect(mosq, host, port, keepalive)) {
    fprintf(stderr, "Unable to connect.\n");
    exit(1);
  }
  int loop = mosquitto_loop_start(mosq);
  if (loop != MOSQ_ERR_SUCCESS) {
    fprintf(stderr, "Unable to start loop: %i\n", loop);
    exit(1);
  }
}

int mqtt_send(char *msg) {
  return mosquitto_publish(mosq, NULL, topic, strlen(msg), msg, 0, 0);
}

int main(int argc, char *argv[]) {
  mqtt_setup();

  i2c_openport();

  if (zmodhat_init()) {
    fprintf(stderr, "ZMOD: INIT ERR");
  }
  i2c_closeport();

  char *buf = (char *)malloc(96);
  while (1) {
    read_zmod_hat(buf);
    //fprintf(stdout, "%s\n", buf);

    int snd = mqtt_send(buf);
    if (snd != 0)
      fprintf(stderr, "mqtt_send error=%i for payload %s\n", snd, buf);
    usleep(10000000); // 10 seconds
  }
}
