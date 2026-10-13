#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
struct StreamSink { enum Channel { CHANNEL_TYPE_USB,CHANNEL_TYPE_UART4,CHANNEL_TYPE_UART5 }; Channel channelType; };
struct Received { unsigned channel; std::string text; };
static std::vector<Received> received;
static unsigned checks=0;
#define CHECK(x) do { ++checks; if(!(x)) {fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);exit(1);} } while(0)
void OnUsbAsciiCmd(const char* text,size_t n,StreamSink& s) { received.push_back({unsigned(s.channelType),std::string(text,n)}); }
void OnUart4AsciiCmd(const char* text,size_t n,StreamSink& s) { OnUsbAsciiCmd(text,n,s); }
void OnUart5AsciiCmd(const char* text,size_t n,StreamSink& s) { OnUsbAsciiCmd(text,n,s); }
#include "ascii_processor_under_test.inc"
static void Send(StreamSink& s,const std::string& value) { ASCII_protocol_parse_stream(reinterpret_cast<const uint8_t*>(value.data()),value.size(),s); }
int main() {
    StreamSink usb{StreamSink::CHANNEL_TYPE_USB},uart4{StreamSink::CHANNEL_TYPE_UART4},uart5{StreamSink::CHANNEL_TYPE_UART5};
    const std::string record="~S"+std::string(184,'a');
    Send(usb,record.substr(0,64)); Send(uart4,"#GET"); Send(usb,record.substr(64,64));
    Send(uart5,"!STOP\n"); Send(uart4,"JPOS\n");Send(usb,record.substr(128)+"\n");
    CHECK(received.size()==3);CHECK(received[0].channel==2 && received[0].text=="!STOP");
    CHECK(received[1].channel==1 && received[1].text=="#GETJPOS");CHECK(received[2].channel==0 && received[2].text==record);
    received.clear();Send(usb,std::string(400,'x'));Send(uart4,"!STOP\n");Send(usb,"\n#STREAM\n");
    CHECK(received.size()==2);CHECK(received[0].channel==1 && received[0].text=="!STOP");CHECK(received[1].text=="#STREAM");
    for(unsigned chunk=1;chunk<=64;++chunk) {
        received.clear();
        for(size_t at=0;at<record.size();at+=chunk) Send(usb,record.substr(at,chunk));
        Send(usb,"\n");CHECK(received.size()==1);CHECK(received[0].text==record);
    }
    printf("core_stream_parser: %u checks passed; actual production parser, no hardware\n",checks);
}
