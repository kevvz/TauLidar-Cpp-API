#include "TauCam.h"


int set_attributes(int fd, int speed, int parity) {
    
    struct termios tty;
    
    memset(&tty,0, sizeof tty);
    
    if (tcgetattr(fd,&tty )!= 0) {
	printf("error %d from tcgetattr from set_attributes\n",errno);
	return -1;
    } 
    
    cfsetospeed(&tty,speed);
    cfsetispeed(&tty,speed);
    
    tty.c_cflag = ((tty.c_cflag & ~CSIZE) | CS8);
    
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag =0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    
    tty.c_iflag &= ~(IXON| IXOFF | IXANY);
    
    tty.c_iflag &= ~(INPCK | ISTRIP);
    tty.c_cflag |= (CLOCAL | CREAD);
    
    tty.c_cflag &= ~(PARENB | PARODD);
  
    tty.c_cflag |= parity;
    
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    if (tcsetattr (fd,TCSANOW,&tty) != 0)
      printf("error from tgetattr from set_blocking\n");
    return 0;
}

void set_blocking (int fd, int should_block) {
    struct termios tty;
    memset(&tty,0,sizeof tty);
    
    if (tcgetattr(fd,&tty) != 0 ) {
	printf("error %d from tgetattr from set_blocking\n",errno);
	return;
    }
    
    tty.c_cc[VMIN] = should_block ? 1: 0;
    tty.c_cc[VTIME] = 5;
    
    if (tcsetattr (fd,TCSANOW, &tty) != 0)
     printf("error %d setting term attributes\n",errno);
}

void TauCam::ClearWriteBuffer() {
    for (int i =0; i < 14; i++) 
      write_buffer[i] = 0;
	
}

int TauCam::ParseData() {// bit one is the data start bit, next three bits are the data size, last four bits are th checksum. For PC data, there is a 80 long header 
	uint16_t data_size = getUint16_LittleEndian();
	printf("%d\n",data_size);
	//printf("%d\n",size);
	
	if (size != data_size) {
	    printf("Bad data, not saving\n");
	    return -1;
	}
	    //Calculate checksum?
	    
	return 1;
}


void TauCam::GrabRawDistanceData() {
	size = 19280;
	ClearWriteBuffer();
	write_buffer[1] = 0x20;
	write_buffer[2] = 0x01;
//	if (SendCommand() == -1) {
//	    printf("error\n");
	    
//	}
        SendCommand();
	
}

std::vector< std::vector<float>* > TauCam::GetPointCloud() {
    GrabRawDistanceData();
    std::vector<std::vector<float>* > r;
    int x = 0;
    int y = 0;
    int inval = 0;
    for(int i =0; i <  9600;i++) {
	uint16_t temp = ((cam_buffer[2*i+1+8+80] << 8) | cam_buffer[2*i+8+80]) & 0x3FFF;
	if (temp > 16000) {
	    inval++;
	    continue;
	}
	else {
	    x = i % 160;
	    y = i / 60;
	    
	    float gamma_a  = ALPHA_A + THETA_A * (x/160.0);
	    float gamma_b = ALPHA_B + THETA_B * (y/60.0);
            
	    float Z = abs(0.001*temp*sin(gamma_a));
	    Z = abs(Z*cos(gamma_b));
	    float X = Z/ tan(gamma_a);
	    float Y = -1 * Z * tan(gamma_b);
                
	    std::vector<float>*t = new std::vector<float>{X,Y,Z}; 
	    r.push_back(t);
                
	}
	
	
    }
    printf("%d\n",inval);
    return r;
    
}

void TauCam::FillDepth() {
    GrabRawDistanceData();
    int x = 0;
    int y = 0;
    int inval = 0;
    for(int i =0; i <  9600;i++) {
	uint16_t temp = ((cam_buffer[2*i+1+8+80] << 8) | cam_buffer[2*i+8+80]) & 0x3FFF;
	if (temp > 16000) {
	    depth_buffer[i] = -1.0;
	    continue;
	}
	else {
	    x = i % 160;
	    y = i / 60;
	    
	    float gamma_a  = ALPHA_A + THETA_A * (x/160.0);
	    float gamma_b = ALPHA_B + THETA_B * (y/60.0);
            
	    float Z = abs(0.001*temp*sin(gamma_a));
	    Z = abs(Z*cos(gamma_b));
	    depth_buffer[i] = Z;
	    
                
	    
                
	}
	
	
    }
    
    
}

void TauCam::GetDistanceAmplitudeData() {
    
    
}

uint32_t TauCam::CRC_Uint32(uint32_t crc, uint8_t data) {
    crc = crc ^ data;
    for (int i =0; i < 32; i++ ) {
	if (crc & 0x80000000) {
	    crc = (crc << 1)^0x04C11DB7;
	}
	else {
	    crc = crc <<1 ;
	}
    }
    return crc;
}

uint32_t TauCam::CS_Uint32() {
    u_int32_t crc = 0xFFFFFFFF;
    for (int i = 0; i < 10; i++) {
	crc = CRC_Uint32(crc,write_buffer[i]);
    }
    crc = crc ^ 0x00000000;
    
    return crc;
}

void TauCam::CalculateChecksumUint32() {
    
    setUint32_LittleEndian(CS_Uint32());
    
}

void TauCam::setUint32_LittleEndian(uint32_t crc) {
	
	write_buffer[10] = (crc) & 0xFF;
	write_buffer[11] = (crc>>8) & 0xFF;
	write_buffer[12] = (crc>>16) & 0xFF;
	write_buffer[13] = (crc>>24) & 0xFF;
	
}

void TauCam::setUint16_LittleEndian(uint16_t crc,int idx)	{
	write_buffer[idx] = crc & 0xFF;
	write_buffer[1+idx] = (crc >> 8) & 0xFF;
}

uint16_t TauCam::getUint16_LittleEndian()	{
	uint16_t result;
	result = (cam_buffer[3] << 8) | cam_buffer[2];
	
	
	return result;
}


TauCam::TauCam() {
	for (int i =0; i < 14; i++) 
      write_buffer[i] = 0;
	fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
	set_attributes(fd,B4000000,0);
	set_blocking(fd,0);
	    if (fd == 0) {
	perror(DEVICE);
	printf("Failed to open DEVICE \"dev/ttyAMCO\"\n");
	
    }
    fds[0].fd = fd;
    fds[0].events = POLLRDNORM;
    
    sleep(1);
    tcflush(fd,TCIOFLUSH);
   for (int i = 0; i< 24000;i++) {
       cam_buffer[i] = 0;
   }
   for (int i = 0; i< 24000;i++) {
       read_buffer[i] = 0;
   }
   //  GetId();
   //  SetIntegrationTime3d(0,1000);
    // SetMode();
  //   SetModulationFrequency();
   // SetHdr();
 
   // F(1);
 

    
}

int TauCam::GetId() {
    
    size = 4;
    
    ClearWriteBuffer();

    write_buffer[1] = 0x47;
    
    SendCommand();
	
	    

    
	
        return 1;
}

void TauCam::SetMode() { //NOT USEFUL
    size = 0;
    ClearWriteBuffer();
    write_buffer[1] = 0x04;
    write_buffer[2] = 0x00;
    
    SendCommand();
    
}

void TauCam::SetModulationFrequency() {
    size = 0;
    ClearWriteBuffer();
    write_buffer[1] = 0x05;
    write_buffer[2] = 1 & 0xFF;
    SendCommand();
    
}

void TauCam::SetHdr() { //NOT USEFUL
    size =0;
    ClearWriteBuffer();
    write_buffer[1]  = 0x13;
    write_buffer[2] = 0x01;
    SendCommand();
}
void TauCam::SendCommand() { //Send whats in the write buffer to the Lidar.
	
	write_buffer[0] = 0xF5;
	CalculateChecksumUint32();
	write(fd,write_buffer,14);
	usleep(5000);
       // sleep(5);
	//if (size == 0 )
	  
	ret = poll(fds,1,1000);
	int remaining_bits = size + 8;
	int count = 0;
	int h = 0;
	if (ret>0) {
	    
	    while (remaining_bits > 0){
		if (fds[0].revents & POLLRDNORM) {
		res = read(fd,read_buffer,2048);
		usleep(5000);
		//sleep(1);
		if (res == -1){
		    printf("error%d, return early\n",errno);
		    return;
		}
		  
	
		//printf("%d\n",res);
	
		remaining_bits = remaining_bits - res;
	
	        h += res;
		//printf("total bits: %d\n",h);
		
		read_buffer[res] = 0;
		for (int i =0; i < res; i++) {
		    
		cam_buffer[count] = read_buffer[i];
		count++;
		}
		
		//for(int i = 0; i < 320; i++)
		//  printf("%d\n",cam_buffer[i]);
		//sleep(10);

    }
	    }
	  //  read(fd,read_buffer,2048);
	}
	ParseData();
	
}


void TauCam::SetIntegrationTime3d(uint16_t index,uint16_t time) {
    
        size = 0;
	ClearWriteBuffer();
	write_buffer[1] = 0x00;
	write_buffer[3] = time & 0xFF;
	write_buffer[4] = (time >>8) & 0xFF;
	//setUint16_LittleEndian(time,3);
	SendCommand();
	
	
}


