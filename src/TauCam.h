
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
#include <vector>
#include <cmath>
#include <errno.h>

#define BAUDRATE B4000000
#define DEVICE  "/dev/ttyACM0"
#define PI 3.141565358979323846
#define THETA_A 1.3962634016
#define ALPHA_A 0.87266462599
#define THETA_B 0.52359877559
#define ALPHA_B 6.02138591923 





class TauCam {

   public:
     TauCam();
     int GetId();
     
     void SendCommand();
     
     void SetMode();
     void SetModulationFrequency();
     void SetHdr();
     void SetIntegrationTime3d(uint16_t index,uint16_t time);
     int ParseData();
     void GrabRawDistanceData();
     void FillDepth();
     void GetDistanceAmplitudeData();
     std::vector<std::vector<float>*> GetPointCloud();
     
     uint16_t getUint16_LittleEndian();
     
     float depth_buffer[9600];
     
   private:
   
     uint8_t write_buffer[14];
     char read_buffer[24000];

     char cam_buffer[24000];
     
     
     
     int fd;
     struct pollfd fds[1];
     int ret,res;
     
     int size = 0;
     void ClearWriteBuffer();
     
   
     uint32_t CRC_Uint32(uint32_t crc, uint8_t data);
     uint32_t CS_Uint32();
     void CalculateChecksumUint32();
     void setUint32_LittleEndian(uint32_t crc);
     
     void setUint16_LittleEndian(uint16_t crc,int idx);
   


};



