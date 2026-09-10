// PDS193: nonblocking RS485 turnaround trial. Pump-only CAN.
#include <Arduino.h>
#include "driver/twai.h"
#include "esp_task_wdt.h"
#include "pb1.h"

namespace {
pb1::Receiver receiver;
pb1::Parser parser;
bool installed=false,can_running=false,feedback_seen=false,tx_fault=false;
uint8_t feedback[8]={};
uint32_t feedback_at=0,last_tx=0,last_log=0,recovery_at=0;
uint32_t rx_count=0,tx_errors=0,tx_failed=0,rx_loss=0,reply_errors=0;
uint8_t queued_raw=0;
// 20 ms = about 38 character times at 19200 8N1. This is a diagnostic
// turnaround margin, NOT a proven timing requirement of either transceiver.
constexpr uint32_t REPLY_GUARD_MS=20;
uint8_t pending_request[20]={};
bool reply_pending=false;
uint32_t pending_at=0,last_rs485_byte=0;
uint32_t replies_queued=0,replies_dropped=0;

bool fresh(uint32_t now){return feedback_seen && uint32_t(now-feedback_at)<=500;}
bool alarm(){return (feedback[0]&2)||feedback[7]!=0;}
bool healthy(uint32_t now){return can_running && !tx_fault && fresh(now) && !alarm();}

bool send_can(uint32_t id,uint8_t raw) {
  twai_message_t m={};
  m.identifier=id;m.data_length_code=8;m.ss=1;m.data[0]=raw;
  // Single-shot, no unbounded retry; status is checked on subsequent iterations.
  return twai_transmit(&m,0)==ESP_OK;
}

void poll_can(uint32_t now) {
  if(!installed)return;
  uint32_t alerts=0;
  twai_read_alerts(&alerts,0);
  if(alerts&TWAI_ALERT_TX_FAILED){
    ++tx_failed;tx_fault=true;receiver.disarm();
  }
  if(alerts&TWAI_ALERT_RX_QUEUE_FULL){
    ++rx_loss;feedback_seen=false;receiver.disarm();twai_clear_receive_queue();
  }
  twai_status_info_t st={};
  can_running=twai_get_status_info(&st)==ESP_OK && st.state==TWAI_STATE_RUNNING;
  if(st.state==TWAI_STATE_BUS_OFF) {
    receiver.disarm();feedback_seen=false;tx_fault=true;
    if(uint32_t(now-recovery_at)>=1000){
      recovery_at=now;twai_initiate_recovery();
    }
  } else if(st.state==TWAI_STATE_STOPPED) {
    // Bus-off recovery ends stopped. Clear all old requests before restart.
    receiver.disarm();feedback_seen=false;tx_fault=true;
    if(uint32_t(now-recovery_at)>=1000){
      recovery_at=now;twai_clear_transmit_queue();twai_clear_receive_queue();twai_start();
    }
  }
  twai_message_t m={};
  // Bounded RX work. Drop a backlog instead of treating queued old data as fresh.
  if(st.msgs_to_rx>16){
    ++rx_loss;feedback_seen=false;receiver.disarm();twai_clear_receive_queue();return;
  }
  for(unsigned i=0;i<16 && twai_receive(&m,0)==ESP_OK;++i) {
    if(!m.extd && !m.rtr && m.identifier==0x4e4 && m.data_length_code==8) {
      memcpy(feedback,m.data,8);feedback_at=millis();feedback_seen=true;++rx_count;
    }
  }
  // Recover TX readiness only after a successful zero transmission and fresh RX.
  if((alerts&TWAI_ALERT_TX_SUCCESS) && queued_raw==0 && fresh(now) &&
     !(alerts&TWAI_ALERT_TX_FAILED))tx_fault=false;
}

void command_can(uint32_t now) {
  uint8_t demand=receiver.output(now,healthy(now));
  // Send zero immediately on a local fault/expiry, otherwise maintain 100 ms.
  if(!can_running)return;
  if(uint32_t(now-last_tx)<100 && !(queued_raw && !demand))return;
  last_tx=now;
  if(!demand && queued_raw)twai_clear_transmit_queue();
  bool a=send_can(0x4de,demand);
  bool b=send_can(0x523,0); // Purge OFF, all eight bytes zero.
  if(a && b)queued_raw=demand;
  else {
    ++tx_errors;tx_fault=true;receiver.disarm();
    twai_clear_transmit_queue();queued_raw=0;
    // A failed enqueue is NOT proof that zero reached the pump.
  }
}

void poll_rs485(uint32_t now) {
  // If this loop was delayed, discard accumulated frames, never renew a lease
  // from a stale UART backlog. Next normal 250 ms poll will be handled.
  static uint32_t last_loop=0;
  if((last_loop && uint32_t(now-last_loop)>200) || Serial2.available()>80) {
    receiver.disarm();parser.used=0;
    if(reply_pending){reply_pending=false;++replies_dropped;}
    for(unsigned i=0;i<256 && Serial2.available();++i)Serial2.read();
    last_loop=now;return;
  }
  last_loop=now;
  // No delay()/flush(): CAN and watchdog continue on every loop iteration.
  // Also require silence after the last received byte, not merely packet decode.
  if(reply_pending && uint32_t(now-pending_at)>100) {
    reply_pending=false;++replies_dropped;
  }
  if(reply_pending && uint32_t(now-pending_at)>=REPLY_GUARD_MS &&
     uint32_t(now-last_rs485_byte)>=REPLY_GUARD_MS && !Serial2.available()) {
    uint8_t response[20];
    // Re-evaluate readiness when sending; never cache old healthy flags.
    receiver.output(now,healthy(now));
    receiver.reply(pending_request,now,can_running && !tx_fault,fresh(now),
                   feedback_seen && alarm(),
                   feedback_seen?uint32_t(now-feedback_at):65535,feedback,response);
    if(Serial2.availableForWrite()>=20) {
      if(Serial2.write(response,20)==20)++replies_queued;
      else {++reply_errors;receiver.disarm();}
    } else {++reply_errors;receiver.disarm();}
    reply_pending=false;
  }
  uint8_t request[20];
  for(unsigned budget=0;budget<80 && Serial2.available();++budget) {
    int v=Serial2.read();if(v<0)break;
    last_rs485_byte=millis();
    if(!parser.feed(uint8_t(v),last_rs485_byte,request))continue;
    uint32_t t=millis();
    if(!receiver.accept(request,t))continue;
    // Healthy feedback plus a ZERO handshake is required to arm.
    receiver.output(t,healthy(t));
    if(reply_pending)++replies_dropped;
    memcpy(pending_request,request,20);
    pending_at=t;reply_pending=true;
    // One response per loop prevents UART bursts from delaying CAN service.
    break;
  }
}

void report(uint32_t now) {
  if(uint32_t(now-last_log)<1000)return;
  last_log=now;
  if(!Serial)return;
  char b[200];
  int n=snprintf(b,sizeof(b),
    "PB1 link=%u armed=%u CAN=%u fresh=%u want=%u queued=%u B1=%u A=%.1f alarm=%02X/%02X RX=%lu ok=%lu reject=%lu crc=%lu timeout=%lu txerr=%lu fail=%lu replyerr=%lu\n",
    receiver.linked(now),receiver.armed,can_running,fresh(now),receiver.wanted,queued_raw,
    feedback[1],fresh(now)?feedback[5]/10.0:-1.0,feedback[0],feedback[7],
    (unsigned long)rx_count,(unsigned long)receiver.accepted,(unsigned long)receiver.rejected,
    (unsigned long)parser.bad,(unsigned long)receiver.timeouts,(unsigned long)tx_errors,
    (unsigned long)tx_failed,(unsigned long)reply_errors);
  if(n>0 && size_t(n)<sizeof(b) && Serial.availableForWrite()>=n)Serial.write((uint8_t*)b,size_t(n));
}

void report_reply(uint32_t now) {
  static uint32_t last=0;
  // Offset from the longer PB1 log to avoid competing for USB buffer space.
  if(uint32_t(now-last)<1000 || uint32_t(now-last_log)<100)return;
  last=now;
  if(!Serial)return;
  char b[112];
  int n=snprintf(b,sizeof(b),"PB1TX guard=20ms queued=%lu dropped=%lu errors=%lu pending=%u\n",
                (unsigned long)replies_queued,(unsigned long)replies_dropped,
                (unsigned long)reply_errors,reply_pending);
  if(n>0 && size_t(n)<sizeof(b) && Serial.availableForWrite()>=n)
    Serial.write((const uint8_t*)b,size_t(n));
}
}

void setup() {
  Serial.setTxBufferSize(512);Serial.begin(115200);
  // T-CAN485 original board, NOT T-2CAN. Pins follow its Battery-Emulator HAL.
  pinMode(16,OUTPUT);digitalWrite(16,HIGH); // Peripheral 5 V rail
  pinMode(23,OUTPUT);digitalWrite(23,LOW);  // CAN normal/high-speed
  pinMode(17,OUTPUT);digitalWrite(17,HIGH); // MAX13487 /RE: vendor automatic mode
  pinMode(19,OUTPUT);digitalWrite(19,HIGH); // MAX13487 /SHDN: enable transceiver
  Serial2.setRxBufferSize(256);
  Serial2.begin(19200,SERIAL_8N1,21,22);
  twai_general_config_t g=TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_27,GPIO_NUM_26,TWAI_MODE_NORMAL);
  g.tx_queue_len=2;g.rx_queue_len=32;
  g.alerts_enabled=TWAI_ALERT_TX_FAILED|TWAI_ALERT_TX_SUCCESS|TWAI_ALERT_RX_QUEUE_FULL|
                   TWAI_ALERT_BUS_OFF|TWAI_ALERT_BUS_RECOVERED;
  twai_timing_config_t t=TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f=TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t e=twai_driver_install(&g,&t,&f);
  installed=e==ESP_OK;
  if(installed)e=twai_start();
  can_running=installed && e==ESP_OK;
  last_tx=millis()-100;
  // Arduino-ESP32 2.x / IDF4 watchdog: reboot if loop stalls for 3 seconds.
  // A reboot is NOT a physical pump-stop mechanism.
  esp_task_wdt_init(3,true);esp_task_wdt_add(nullptr);
  Serial.printf("PDS193 T-CAN485 PB1 19200 8N1; reply guard 20ms; init=%s; boot ZERO\n",esp_err_to_name(e));
}

void loop() {
  uint32_t now=millis();
  static uint32_t last_loop=0;
  static bool have_loop=false;
  if(have_loop && uint32_t(now-last_loop)>200)receiver.disarm();
  last_loop=now;have_loop=true;
  receiver.tick(now);
  poll_can(now);
  command_can(millis());
  poll_rs485(millis());
  command_can(millis());
  report(millis());
  report_reply(millis());
  esp_task_wdt_reset();
  delay(1);
}
