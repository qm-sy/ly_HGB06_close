#include "modbus.h"
#include  "STC15W4Kxx.H"
#include "crc.h"

#define FUNCTION_READ_COILS_1                 1     
#define FUNCTION_READ_DISCREATE_INPUT_2       2     
#define FUNCTION_READ_HOLDING_REGISTERS_3     3     
#define FUNCTION_READ_INPUT_REGISTERS_4       4     
#define FUNCTION_WRITE_SINGLE_COIL_5          5     
#define FUNCTION_WRITE_SINGLE_REGISTER_6      6     
#define FUNCTION_WRITE_MULTIPLE_COILS_F       0xf   
#define FUNCTION_WRITE_MULTIPLE_REGISTERS_10  0x10  
#define FUNCTION_READ_WRITE_MULTIPLE_REGISTERS_17  0x17  


#define uchar  unsigned char
#define uint   unsigned int
#define ulong  unsigned long

extern  uchar   xdata  tm_set1;
extern  uchar   xdata  tm_set2;
extern  uchar   xdata  tm_set3;
extern  uchar   xdata  tm_set4;

extern int   idata  temp_set1;
extern int   idata  temp_set2;
extern int   idata  temp_set3;
extern int   idata  temp_set4;

extern int     idata  temp1;
extern int     idata  temp2;
extern int     idata  temp3;
extern int     idata  temp4;

extern  uchar idata  fan_num;
extern  uchar idata  tb_num;
extern  uchar idata  hot_num;
extern  uchar idata  mode_num;
extern  uchar   hot_power;
extern  uchar   fan_power;
extern  uchar  idata hot_in_data;


extern bit hot_power_bit;
extern bit mode_num_bit;

unsigned char idata dev_address = 1;   			    
unsigned char idata dev_broadcast_address = 0;  

void send_buffer(unsigned char *buf,int len);
void  save_mode_wr(void);

unsigned char xdata send_modbus_buffer[256];
int xdata send_modbus_buf_count = 0;
unsigned char xdata localbuf[256];
char xdata msg[64];




void send_bad_msg(unsigned char address,unsigned char function,unsigned char errorCode)
{
	unsigned char return_msg[5];
	unsigned short crc = 0;
	
	return_msg[0] = address;
	return_msg[1] = 0x80 + function;
	return_msg[2] = errorCode;
	crc = crc16(return_msg,3);
	return_msg[3] = crc;
	return_msg[4] = crc >> 8;
	send_buffer(return_msg,5);  
}

int broadcast_process(unsigned char *buf,int len)
{	 
	buf = buf;
	len = len;
	
	return 1;
}

 

  


int function_READ_COILS_1(unsigned char *buf) 
{
	int ret = 0;
	unsigned char out_count = 0;       
	unsigned short start_address = 0;  
	unsigned short count = 0;          
	unsigned short crc  = 0;
	int send_total_count = 0;
	
	
	
	start_address  = buf[2] << 8;
	start_address += buf[3];
	count  = buf[4] << 8;
	count += buf[5];
	//sprintf(msg,"addr=%u,count=%u\r\n\0",start_address,count);
	//debug_out(msg);
	
	//数量是否有效，如果无效 则发送异常码 3
	if (count < 1 || count > 0x07D0) {
		send_bad_msg(buf[0],buf[1],3);   
		ret = 1;
    return ret;		
	}
	
	
	
	
	out_count = count / 8;
	if (count % 8 > 0) out_count++;
	send_modbus_buf_count = 0;
	send_modbus_buffer[0] = buf[0];   
	send_modbus_buffer[1] = buf[1];   
	send_modbus_buffer[2] = out_count; 
	while (out_count--) {
	  send_modbus_buffer[3+send_modbus_buf_count] = P5; 
		send_modbus_buf_count++;
  }		
	send_modbus_buffer[2] = send_modbus_buf_count;	 
	send_total_count = send_modbus_buf_count + 5;    
	crc = crc16(send_modbus_buffer,send_total_count-2);  
	send_modbus_buffer[send_total_count-2] = crc; 
	send_modbus_buffer[send_total_count-1] = crc >> 8;
	send_buffer(send_modbus_buffer,send_total_count);  
	
	//debug_out("function_READ_COILS_1\r\n\0");
	return ret;
}


int function_READ_DISCREATE_INPUT_2(unsigned char *buf)
{
  int ret = 0;
	unsigned char out_count = 0;       
	unsigned short start_address = 0;  
	unsigned short count = 0;          
	unsigned short crc  = 0;
	int send_total_count = 0;
	
	
	start_address  = buf[2] << 8;
	start_address += buf[3];
	count  = buf[4] << 8;
	count += buf[5];
	//sprintf(msg,"addr=%u,count=%u\r\n\0",start_address,count);
	//debug_out(msg);
	
	//数量是否有效，如果无效 则发送异常码 3
	if (count < 1 || count > 0x07D0) {
		send_bad_msg(buf[0],buf[1],3);   //发送 异常码 3 
		ret = 1;
    return ret;		
	}
	

	out_count = count / 8;
	if (count % 8 > 0) out_count++;
	send_modbus_buf_count = 0;
	send_modbus_buffer[0] = buf[0];   
	send_modbus_buffer[1] = buf[1];   
	send_modbus_buffer[2] = out_count;  
	while (out_count--) {
	  send_modbus_buffer[3+send_modbus_buf_count] = P5; 
		send_modbus_buf_count++;
  }		
	send_modbus_buffer[2] = send_modbus_buf_count;	 
	send_total_count = send_modbus_buf_count + 5;    
	crc = crc16(send_modbus_buffer,send_total_count-2);  
	send_modbus_buffer[send_total_count-2] = crc; 
	send_modbus_buffer[send_total_count-1] = crc >> 8;
	send_buffer(send_modbus_buffer,send_total_count);  
	
	//debug_out("function_READ_DISCREATE_INPUT_2\r\n\0");
	return ret;
}


int function_READ_HOLDING_REGISTERS_3(unsigned char *buf)
{
  int ret = 0;
	unsigned char out_count = 0;       
	unsigned short start_address = 0;  
	unsigned short count = 0;          
	unsigned short crc  = 0;
	int send_total_count = 0;
	
	//输入的起始地址和输出的数量
	start_address  = buf[2] << 8;
	start_address += buf[3];
	count  = buf[4] << 8;
	count += buf[5];
	count-=4;
	//sprintf(msg,"addr=0x%x,count=%x\r\n\0",start_address,count);
	//debug_out(msg);
	//sprintf(msg,"addr=%u,count=%u\r\n\0",start_address,count);
	//debug_out(msg);
	
	//数量是否有效，如果无效 则发送异常码 3
	if (count < 1 || count > 0x07D) {
		send_bad_msg(buf[0],buf[1],3);   
		ret = 1;
    return ret;		
	}
	
	
	
	
	out_count = count / 8;
	if (count % 8 > 0) out_count++;
	send_modbus_buf_count = 0;
	send_modbus_buffer[0] = buf[0];   
	send_modbus_buffer[1] = buf[1];   
	send_modbus_buffer[2] = out_count;  
    if(start_address==40001)
	 {
	  while (count--) {
	  //send_modbus_buffer[3+send_modbus_buf_count] =hot_in_data; send_modbus_buf_count++; 	
	  //send_modbus_buffer[3+send_modbus_buf_count] =fan_num; send_modbus_buf_count++; 
	  send_modbus_buffer[3+send_modbus_buf_count] =mode_num; send_modbus_buf_count++; 
	  send_modbus_buffer[3+send_modbus_buf_count] =hot_power; send_modbus_buf_count++; 

	  send_modbus_buffer[3+send_modbus_buf_count] =temp1; send_modbus_buf_count++; 
	  send_modbus_buffer[3+send_modbus_buf_count] =temp2; send_modbus_buf_count++; 
	  send_modbus_buffer[3+send_modbus_buf_count] =temp3; send_modbus_buf_count++; 
	  send_modbus_buffer[3+send_modbus_buf_count] =temp4; send_modbus_buf_count++; 

	  send_modbus_buffer[3+send_modbus_buf_count] =tm_set1; send_modbus_buf_count++; 
	  send_modbus_buffer[3+send_modbus_buf_count] =tm_set2; send_modbus_buf_count++; 
	  send_modbus_buffer[3+send_modbus_buf_count] =tm_set3; send_modbus_buf_count++; 
	  send_modbus_buffer[3+send_modbus_buf_count] =tm_set4; send_modbus_buf_count++; 
	  } 
	  send_modbus_buffer[2] = send_modbus_buf_count;	
	  send_total_count = send_modbus_buf_count + 5;   
	  crc = crc16(send_modbus_buffer,send_total_count-2);  
	  send_modbus_buffer[send_total_count-2] = crc; 
	  send_modbus_buffer[send_total_count-1] = crc >> 8;
	  send_buffer(send_modbus_buffer,send_total_count);  
	  ret = 2;
	 }

	else { send_bad_msg(buf[0],buf[1],2); ret = 1;}  
	return ret;
}


int function_READ_INPUT_REGISTERS_4(unsigned char *buf)
{
	int ret = 0;
	unsigned char out_count = 0;       
	unsigned short start_address = 0;  
	unsigned short count = 0;          
	unsigned short crc  = 0;
	int send_total_count = 0;
	
	
	start_address  = buf[2] << 8;
	start_address += buf[3];
	count  = buf[4] << 8;
	count += buf[5];
	count-=4;
	//sprintf(msg,"addr=0x%x,count=%x\r\n\0",start_address,count);
	//debug_out(msg);
	//sprintf(msg,"addr=%u,count=%u\r\n\0",start_address,count);
	//debug_out(msg);
	
	//数量是否有效，如果无效 则发送异常码 3
	if (count < 1 || count > 0x07D) {
		send_bad_msg(buf[0],buf[1],3);   //发送 异常码 3 
		ret = 1;
    return ret;		
	}
	
	//地址是否ok,地址+输出数量是否ok,否则发送 异常码 2
	
	//读取输入寄存器是否ok,否则发送 异常码 4
	
	send_modbus_buf_count = 0;
	send_modbus_buffer[0] = buf[0];   
	send_modbus_buffer[1] = buf[1];  
	send_modbus_buffer[2] = out_count;  
	if(start_address==40001)
	 {															
	  while (count--) {
	  //send_modbus_buffer[3+send_modbus_buf_count] =hot_in_data; send_modbus_buf_count++; 	
	  //send_modbus_buffer[3+send_modbus_buf_count] =fan_num; send_modbus_buf_count++; 
	  send_modbus_buffer[3+send_modbus_buf_count] =mode_num; send_modbus_buf_count++; 
	  send_modbus_buffer[3+send_modbus_buf_count] =hot_power; send_modbus_buf_count++; 
	  

	  send_modbus_buffer[3+send_modbus_buf_count] =temp1; send_modbus_buf_count++; 
	  send_modbus_buffer[3+send_modbus_buf_count] =temp2; send_modbus_buf_count++; 
	  send_modbus_buffer[3+send_modbus_buf_count] =temp3; send_modbus_buf_count++; 
	  send_modbus_buffer[3+send_modbus_buf_count] =temp4; send_modbus_buf_count++; 

	  send_modbus_buffer[3+send_modbus_buf_count] =tm_set1; send_modbus_buf_count++; 
	  send_modbus_buffer[3+send_modbus_buf_count] =tm_set2; send_modbus_buf_count++; 
	  send_modbus_buffer[3+send_modbus_buf_count] =tm_set3; send_modbus_buf_count++; 
	  send_modbus_buffer[3+send_modbus_buf_count] =tm_set4; send_modbus_buf_count++; 
	  } 
	  send_modbus_buffer[2] = send_modbus_buf_count;	
	  send_total_count = send_modbus_buf_count + 5;   
	  crc = crc16(send_modbus_buffer,send_total_count-2);  
	  send_modbus_buffer[send_total_count-2] = crc; 
	  send_modbus_buffer[send_total_count-1] = crc >> 8;
	  send_buffer(send_modbus_buffer,send_total_count);  
	  ret = 2;
	 } 
	

	else { send_bad_msg(buf[0],buf[1],2); ret = 1;}  
	return ret;
}


int function_WRITE_SINGLE_COIL_5(unsigned char *buf,int len)
{
	int ret = 0;
	unsigned short start_address = 0;  
	unsigned short value = 0;          
	
	
	start_address  = buf[2] << 8;
	start_address += buf[3];
	value  = buf[4] << 8;
	value += buf[5];
	//sprintf(msg,"addr=0x%x,value=%x\r\n\0",start_address,value);
	//debug_out(msg);
	//sprintf(msg,"addr=%u,value=%u\r\n\0",start_address,value);
	//debug_out(msg);
	
	//数量是否有效，如果无效 则发送异常码 3
	if (value != 0x0 && value != 0xFF00) {
		send_bad_msg(buf[0],buf[1],3);   //发送 异常码 3 
		ret = 1;
    return ret;		
	}
      //这里可以添加执行的数据或命令

	//if((start_address==40001)&&(value==0))	    P27=0;
	//if((start_address==40001)&&(value==0xff00))	P27=1;

	
	//地址是否ok, 否则发送 异常码 2
	
	//写线圈是否ok,否则发送 异常码 4
	
	
		send_buffer(buf,len);             
	 	
	
	
	return ret;
}


int function_WRITE_SINGLE_REGISTER_6(unsigned char *buf,int len)
{
  int ret = 0;
	unsigned short start_address = 0;  
	unsigned short value = 0;          
	
	
	
	start_address  = buf[2] << 8;
	start_address += buf[3];
	value  = buf[4] << 8;
	value += buf[5];
	
	
	
	if (value < 0x0 || value > 0xFFFF) {
		send_bad_msg(buf[0],buf[1],3);    
		ret = 1;
    return ret;		
	}
	if(start_address==42005)	 
	  {
	   if((buf[5]>0)&&(buf[5]<5)) {
	     mode_num=buf[5];
		 mode_num_bit=1;
	     send_buffer(buf,len);            
		 ret = 2;
		}
	  }  

	else if(start_address==42009)  
	   {
		
	     hot_power=buf[5];
		 hot_power_bit=1;
	     send_buffer(buf,len);            

		 ret = 2;
	   }

	 
	      
	 else { send_bad_msg(buf[0],buf[1],2); ret = 1;}  
	
	
	return ret;
}










int function_WRITE_MULTIPLE_COILS_F(unsigned char *buf)
{
  int ret = 0;
	unsigned short crc = 0;
	unsigned short start_address = 0;  
	unsigned short count = 0;          
	unsigned char localbufLength = 0;
	unsigned char index = 0;
	
	
	
	start_address  = buf[2] << 8;
	start_address += buf[3];
	count  = buf[4] << 8;
	count += buf[5];
	//sprintf(msg,"addr=0x%x,count=%x\r\n\0",start_address,count);
	//debug_out(msg);
	//sprintf(msg,"addr=%u,count=%u\r\n\0",start_address,count);
	//debug_out(msg);
	
	//数量是否有效，如果无效 则发送异常码 3
	if (count < 0x1 || count > 0x7B0) {
		send_bad_msg(buf[0],buf[1],3);    
		ret = 1;
    return ret;		
	}
	
	
	
	localbufLength = buf[6];
	for (index = 0; index < localbufLength; index++) {
    localbuf[index] = buf[7+index];		
	}
	

		crc = crc16(buf,6);
		buf[6] = crc;
		buf[7] = crc >> 8;
		send_buffer(buf,8);              
	
	
	
	return ret;
}







//写多个寄存器,功能码 0x10
int function_WRITE_MULTIPLE_REGISTERS_10(unsigned char *buf)
{
  int ret = 0;
	unsigned short crc = 0;
	unsigned short start_address = 0;  //起始地址
	unsigned short count = 0;          //输出量	
	unsigned char localbufLength = 0;
	unsigned char index = 0;
	
	

	start_address  = buf[2] << 8;
	start_address += buf[3];
	count  = buf[4] << 8;
	count += buf[5];
	

	
	
	if (count < 0x1 || count > 0x7B) {
		send_bad_msg(buf[0],buf[1],3);   
		ret = 1;
    return ret;		
	}

     
    if(start_address!=41000) {send_bad_msg(buf[0],buf[1],2);   
	   ret = 1;
       return ret;
	  }
	
	
	
	   tm_set1=buf[7];
	   tm_set2=buf[8];
	   tm_set3=buf[9];
	   tm_set4=buf[10];

	    crc = crc16(buf,6);
		buf[6] = crc;
		buf[7] = crc >> 8;
		send_buffer(buf,8);           

	   temp_set1=(int)tm_set1;
       temp_set2=(int)tm_set2;
       temp_set3=(int)tm_set3;
	   temp_set4=(int)tm_set4;
	   save_mode_wr(); 
		 
		ret = 2;  
	
	return ret;
}







int function_READ_WRITE_MULTIPLE_REGISTERS_17(unsigned char *buf)
{
  int ret = 1;
	unsigned short crc = 0;
	unsigned short read_start_address = 0;  
	unsigned short write_start_address = 0;  
	unsigned short readCount = 0;         
	unsigned char writebytes = 0;
	unsigned short writeCount = 0;         	
	unsigned char localbufLength = 0;
	unsigned char index = 0;
	int send_total_count = 0;
	
	
	
	read_start_address  = buf[2] << 8;
	read_start_address += buf[3];
	readCount  = buf[4] << 8;
	readCount += buf[5];
	
	write_start_address  = buf[6] << 8;
	write_start_address += buf[7];
	writeCount  = buf[8] << 8;
	writeCount += buf[9];
	
	writebytes = buf[10];
	 
	
	if (readCount < 0x1 || readCount > 0x7D) {
		send_bad_msg(buf[0],buf[1],3);   
		ret = 1;
    return ret;		
	}
	if (writeCount < 0x1 || writeCount > 0x79) {
		send_bad_msg(buf[0],buf[1],3);   
		ret = 1;
    return ret;		
	}
	
	
	
	localbufLength = writebytes;
	for (index = 0; index < localbufLength; index++) {
    localbuf[index] = buf[11+index];		
	}
	

		send_modbus_buf_count = 0;
	  send_modbus_buffer[0] = buf[0];   
	  send_modbus_buffer[1] = buf[1];   
	  send_modbus_buffer[2] = readCount;  
	  while (readCount--) {
	    send_modbus_buffer[3+send_modbus_buf_count] = P0; 
		  send_modbus_buf_count++;
		  send_modbus_buffer[3+send_modbus_buf_count] = P0; 
		  send_modbus_buf_count++;
     }		
	  send_modbus_buffer[2] = send_modbus_buf_count;	
	  send_total_count = send_modbus_buf_count + 5;   
	  crc = crc16(send_modbus_buffer,send_total_count-2);  
	  send_modbus_buffer[send_total_count-2] = crc; 
	  send_modbus_buffer[send_total_count-1] = crc >> 8;
	  send_buffer(send_modbus_buffer,send_total_count); 		
	
	
	return ret;
}




int not_support_function_code(unsigned char address,unsigned char function)
{
	int ret = 0;	
	send_bad_msg(address,function,1);
	
	
	return ret;
}




int parse_recv_buffer(unsigned char *buf,int len)
{	
	int ret = 0;
	unsigned short temp_crc = 0;
	unsigned short temp_crc2 = 0;
	
	if (len < 4)     return 0;         
	if (buf == NULL) return 0;
							   
	if (buf[0] != dev_address && buf[0] != dev_broadcast_address) {  
		
		ret = 1;
		return ret;
	}
	temp_crc  = buf[len-1] << 8;
	temp_crc += buf[len-2];
	temp_crc2 = crc16(buf,len-2);   
	
	if (temp_crc != temp_crc2) { 
		
		ret = 1;
		return ret;
	}
	
	if (buf[0] == 0) {   
		return broadcast_process(buf,len);
	   }
	
	
	switch (buf[1]) {
		case FUNCTION_READ_COILS_1                 : ret = function_READ_COILS_1(buf); break;   			  
		case FUNCTION_READ_DISCREATE_INPUT_2       : ret = function_READ_DISCREATE_INPUT_2(buf); break;       
		case FUNCTION_READ_HOLDING_REGISTERS_3     : ret = function_READ_HOLDING_REGISTERS_3(buf); break;     
		case FUNCTION_READ_INPUT_REGISTERS_4       : ret = function_READ_INPUT_REGISTERS_4(buf); break;       
		case FUNCTION_WRITE_SINGLE_COIL_5          : ret = function_WRITE_SINGLE_COIL_5(buf,len); break;          
		case FUNCTION_WRITE_SINGLE_REGISTER_6      : ret = function_WRITE_SINGLE_REGISTER_6(buf,len); break;      
		case FUNCTION_WRITE_MULTIPLE_COILS_F       : ret = function_WRITE_MULTIPLE_COILS_F(buf); break;       
		case FUNCTION_WRITE_MULTIPLE_REGISTERS_10  : ret = function_WRITE_MULTIPLE_REGISTERS_10(buf); break;  
		case FUNCTION_READ_WRITE_MULTIPLE_REGISTERS_17  : ret = function_READ_WRITE_MULTIPLE_REGISTERS_17(buf); break; 	
		default: ret = not_support_function_code(buf[0],buf[1]);        
	}	
	
	return ret;
}
