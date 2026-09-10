#pragma once
#include <stdint.h>
#include <string.h>

namespace pb1 {
constexpr uint32_t WATCHDOG_MS=750;
constexpr uint8_t MAX_RAW=153;
inline uint16_t crc(const uint8_t* p,unsigned n) {
  uint16_t c=0xffff;
  while(n--){c^=*p++;for(unsigned i=0;i<8;++i)c=c&1?(c>>1)^0xa001:c>>1;}
  return c;
}
inline uint16_t get16(const uint8_t* p){return uint16_t(p[0])|(uint16_t(p[1])<<8);}
inline uint32_t get32(const uint8_t* p){
  return uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24);
}
inline void put16(uint8_t* p,uint16_t v){p[0]=uint8_t(v);p[1]=uint8_t(v>>8);}
inline void put32(uint8_t* p,uint32_t v){for(unsigned i=0;i<4;++i)p[i]=uint8_t(v>>(i*8));}
inline bool request_valid(const uint8_t* p) {
  if(p[0]!=0xa5||p[1]!=0x5a||p[2]!=1||p[3]!=1||crc(p,18)!=get16(p+18))return false;
  if(!get32(p+4)||!get16(p+8)||p[10]>MAX_RAW||(p[11]&0xf8))return false;
  for(unsigned i=14;i<18;++i)if(p[i])return false;
  int16_t t=int16_t(get16(p+12));
  if(p[11]&1){if(t < -400 || t > 1000)return false;}
  else if(t!=INT16_MIN)return false;
  return true;
}

struct Parser {
  uint8_t bytes[20]={};
  unsigned used=0;
  uint32_t last=0,bad=0;
  bool feed(uint8_t v,uint32_t now,uint8_t* out) {
    if(used && uint32_t(now-last)>50)used=0;
    last=now;bytes[used++]=v;
    if(used<20)return false;
    if(request_valid(bytes)){memcpy(out,bytes,20);used=0;return true;}
    ++bad;memmove(bytes,bytes+1,19);used=19;return false;
  }
};

struct Receiver {
  uint32_t nonce=0,last=0,accepted=0,rejected=0,timeouts=0;
  uint32_t retired[8]={};
  unsigned retire_index=0;
  uint16_t sequence=0;
  uint8_t wanted=0;
  bool seen=false,armed=false,expired=false;
  void disarm(){armed=false;wanted=0;}
  bool linked(uint32_t now)const{return seen && uint32_t(now-last)<WATCHDOG_MS;}
  void tick(uint32_t now) {
    if(seen && !linked(now)){disarm();if(!expired){++timeouts;expired=true;}}
  }
  bool accept(const uint8_t* p,uint32_t now) {
    tick(now);
    if(!request_valid(p)){++rejected;return false;}
    uint32_t incoming=get32(p+4);
    uint16_t seq=get16(p+8);
    if(incoming!=nonce) {
      // A new sender session must begin with zero, never a retained demand.
      if(p[10]){++rejected;return false;}
      for(auto n:retired)if(n==incoming){++rejected;return false;}
      if(nonce)retired[retire_index++%8]=nonce;
      nonce=incoming;sequence=0;disarm();
    }
    // The sender regenerates its nonce before sequence wrap.
    if(seq<=sequence){++rejected;return false;}
    sequence=seq;last=now;seen=true;expired=false;++accepted;
    if(!p[10])armed=true;
    wanted=armed?p[10]:0;
    return true;
  }
  uint8_t output(uint32_t now,bool healthy) {
    tick(now);
    if(!healthy)disarm();
    return healthy && armed && linked(now)?wanted:0;
  }
  void reply(const uint8_t* request,uint32_t now,bool running,bool fresh,
             bool alarm,uint32_t age,const uint8_t* feedback,uint8_t* out)const {
    memset(out,0,20);out[0]=0xa5;out[1]=0x5a;out[2]=1;out[3]=2;
    put32(out+4,get32(request+4));put16(out+8,get16(request+8));
    out[10]=request[10]; // Echo request, NOT speed or locally applied output.
    out[11]=(running && armed && linked(now)?1:0)|(fresh?2:0)|(alarm?4:0);
    put16(out+12,uint16_t(age>65535?65535:age));
    out[14]=feedback[0];out[15]=feedback[1];out[16]=feedback[5];out[17]=feedback[7];
    put16(out+18,crc(out,18));
  }
};
}
