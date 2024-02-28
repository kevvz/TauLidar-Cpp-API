
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <time.h>
#include <stdbool.h>
#include <stropts.h>
#include <poll.h>


#include <errno.h>

#define BAUDRATE B4000000
#define DEVICE  "/dev/ttyACM0"






class TauCam {

   public:
     TauCam();
     int GetId();
     
     void SendCommand();
     
     void SetMode();
     void SetModulationFrequency();
     void SetHdr();
     void SetIntegrationTime3d(uint16_t index,uint16_t time);
     void ParseData();
     void GrabRawDistanceData();
     
     
     uint16_t getUint16_LittleEndian();
     
   private:
   
     uint8_t write_buffer[14];
     char read_buffer[24000];

     char cam_buffer[24000];
     
     int fd;
     struct pollfd fds[1];
     int ret,res;
     
     int size = 0;
     void ClearWriteBuffer();
     
   //  void F(uint16_t value1);
     uint32_t CRC_Uint32(uint32_t crc, uint8_t data);
     uint32_t CS_Uint32();
     void CalculateChecksumUint32();
     void setUint32_LittleEndian(uint32_t crc);
     
     void setUint16_LittleEndian(uint16_t crc,int idx);
   


};


