#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include <linux/fb.h>
#include <sys/mman.h>


#include "1024.h"
#include "ascii.h"

typedef unsigned           char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;

#define FILE	"/dev/s3c2410_serial0"			

#define PWM_IOCTL_SET_FREQ		1
#define PWM_IOCTL_STOP			0


#define uchar unsigned char 
#define uint unsigned int
#define ulong unsigned long

#define WHITE		0xffffffff			// test ok
#define BLACK		0x00000000
#define RED			0xffff0000
#define GREEN		0xff00ff00			// test ok
#define BLUE		0xff0000ff

/*************************指纹*************************/ 
int ID_num=0;
int len;
int fd;//指纹fd
unsigned char  ch; //finger mode trans
int nread,i,nwrite;
char buff0[6]="hello\n";
char buff[100];
unsigned char head[6]={0xEF,0x01,0xFF,0xFF,0xFF,0xFF};
fd_set rd;
int res;
unsigned char rx_buf[20];
unsigned char use_buf[20];
struct sigaction saio;

uint32_t AS608_Addr = 0xFFFFFFFF; 

/*************************LCD_SCREEN*************************/
struct fb_fix_screeninfo finfo = {0};
struct fb_var_screeninfo vinfo = {0};
struct input_event ev;
int x,y;
int fd1 = -1,fd2 = -1, fd3 = -1,fd4= -1;
int ret1 = -1, ret2 = -1;
unsigned int *pfb = NULL;

/*************************系统参数*************************/
unsigned int data=0;
int fd_pipe[2] = {0}; //pipe 通道
int nBytes = 0;
unsigned char uart_connect[8] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x07,'\0'};

unsigned char cmd[100] = {
0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13,
0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D,
0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31,
0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B,
0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45,
0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F,
0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x62, 0x63
};

void draw_WHITE(void);
void LCD_write(void);
void signal_handler_IO (int status);
void draw_ascii_ok(unsigned int x, unsigned int y, unsigned int color, unsigned char *str);
/*************************系统参数*************************/
int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;
    if  ( tcgetattr( fd,&oldtio)  !=  0) 
    { 
        perror("SetupSerial 1");
        return -1;
    }
    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag  |=  CLOCAL | CREAD; 
    newtio.c_cflag &= ~CSIZE; 

    switch( nBits )
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch( nEvent )
    {
    case 'O':                     
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':                     
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':                    
        newtio.c_cflag &= ~PARENB;
        break;
    }

switch( nSpeed )
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if( nStop == 1 )
    {
        newtio.c_cflag &=  ~CSTOPB;
    }
    else if ( nStop == 2 )
    {
        newtio.c_cflag |=  CSTOPB;
    }
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd,TCIFLUSH);
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}
    
int open_port(int fd,int comport)
{
    char *dev[]={"/dev/s3c2410_serial0","/dev/s3c2410_serial2"};
    long  vdisable;
    if (comport==1)
    {    fd = open( "/dev/s3c2410_serial0", O_RDWR|O_NOCTTY|O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else 
        {
            printf("open s3c2410_serial0 .....\n");
        }
    }
    else if(comport==2)
    {    fd = open( "/dev/s3c2410_serial2", O_RDWR|O_NOCTTY|O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else 
        {
            printf("open s3c2410_serial2 .....\n");
        }    
    }
    
    if(fcntl(fd, F_SETFL, 0)<0)
    {
        printf("fcntl failed!\n");
    }
    else
    {
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
    }
    if(isatty(STDIN_FILENO)==0)
    {
        printf("standard input is not a terminal device\n");
    }
    else
    {
        printf("isatty success!\n");
    }
    printf("open->fd =%d\n",fd);
    return fd;
}

static void trans_Head_address(void)
{ 
  /*包头*/
  nwrite = write(fd, &head[0], 1);
  nwrite = write(fd, &head[1], 1);
  
  /*指纹模块地址*/	
  nwrite = write(fd, &head[2], 1);
  nwrite = write(fd, &head[3], 1);
  nwrite = write(fd, &head[4], 1);
  nwrite = write(fd, &head[5], 1);

}



unsigned int PS_GetImage()
{
	trans_Head_address();
	//包标识
	nwrite = write(fd, &cmd[1], 1);
	//包长度
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[3], 1);
	//指令码
	nwrite = write(fd, &cmd[1], 1);
	//校验合
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[5], 1);
	
	sleep(1);
	
	if(use_buf[9] == 0x00)
	{
		memset (use_buf,0,sizeof(use_buf));
		return 1;
	}
	
	memset (use_buf,0,sizeof(use_buf));
	return 0;
}

unsigned int PS_GenChar(unsigned int b)
{
	trans_Head_address();
	//包标识
	nwrite = write(fd, &cmd[1], 1);
	//包长度
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[4], 1);
	//指令码
	nwrite = write(fd, &cmd[2], 1);
	//BufferID
	nwrite = write(fd, &cmd[b], 1);
	//校验合
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[7+b], 1);
	
	sleep(1);
	
	if(use_buf[9] == 0x00)
	{
		memset (use_buf,0,sizeof(use_buf));
		return 2;
	}
	
	return 0;
}

unsigned int PS_Match()
{
	trans_Head_address();
	//包标识
	nwrite = write(fd, &cmd[1], 1);
	//包长度
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[3], 1);
	//指令码
	nwrite = write(fd, &cmd[3], 1);
	//BufferID
	//nwrite = write(fd, &cmd[b], 1);
	//校验合
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[7], 1);
	
	sleep(1);
	
	if(use_buf[9] == 0x00)
	{
		memset (use_buf,0,sizeof(use_buf));
		return 3;
	}
	
	return 0;
}




unsigned int PS_RegModel()
{
	trans_Head_address();
	//包标识
	nwrite = write(fd, &cmd[1], 1);
	//包长度
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[3], 1);
	//指令码
	nwrite = write(fd, &cmd[5], 1);
	//BufferID
	//nwrite = write(fd, &cmd[b], 1);
	//校验合
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[9], 1);
	
	sleep(1);
	
	if(use_buf[10] == 0x00)
	{
		memset (use_buf,0,sizeof(use_buf));
		return 4;
	}
	
	return 0;
}


unsigned int PS_StoreChar(unsigned int b,unsigned int ID)
{
	trans_Head_address();
	//包标识
	nwrite = write(fd, &cmd[1], 1);
	//包长度
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[6], 1);
	//指令码
	nwrite = write(fd, &cmd[6], 1);
	//BufferID
	nwrite = write(fd, &cmd[b], 1);
	//PageID
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[ID], 1);
	//校验合
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[13+b+ID], 1);
	
	sleep(1);
	
	if(use_buf[9] == 0x00)
	{
		memset (use_buf,0,sizeof(use_buf));
		return 5;
	}
	
	return 0;
}



unsigned int PS_HighSpeedSearch(uint StartPage,uint PageNum)
{
	trans_Head_address();
	//包标识
	nwrite = write(fd, &cmd[1], 1);
	//包长度
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[8], 1);
	//指令码
	nwrite = write(fd, &cmd[27], 1);
	//BufferID
	nwrite = write(fd, &cmd[1], 1);
	//StartPage
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[StartPage], 1);
	//PageNum
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[PageNum], 1);
	//校验合
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[37+PageNum], 1);
	
	sleep(1);
	
	if(use_buf[9] == 0x00)
	{
		memset (use_buf,0,sizeof(use_buf));
		return 6;
	}
	
	else if(use_buf[9] == 0x09)
	{
		memset (use_buf,0,sizeof(use_buf));
		return 9;
	}
	return 0;
}


unsigned int PS_DeletChar(uint PageID)
{
	trans_Head_address();
	//包标识
	nwrite = write(fd, &cmd[1], 1);
	//包长度
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[7], 1);
	//指令码
	nwrite = write(fd, &cmd[12], 1);
	//PageID
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[PageID], 1);
	//delete N
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[10], 1);
	//校验合
	nwrite = write(fd, &cmd[0], 1);
	nwrite = write(fd, &cmd[30+PageID], 1);
	
	sleep(1);
	
	if(use_buf[9] == 0x00)
	{
		memset (use_buf,0,sizeof(use_buf));
		return 7;
	}
	
	return 0;
}





/*****************************************录指纹*****************************************/

int Add_FR(unsigned int ID)
{
	int i;
	for(i=0;i<20;i++)
	{
		data = PS_GetImage();
		if(data == 1)
		{
			data = PS_GenChar(1);
			if(data == 2)
			{
				data = PS_HighSpeedSearch(0,10);
				if(data == 9)
				{
					data = PS_GetImage();
					if(data == 1)
					{
						data = PS_GenChar(2);
						if(data == 2)
						{
							data = PS_Match();
							if(data == 3)
							{
								data = PS_RegModel();
								if(data == 4)
								{
									data = PS_StoreChar(1,ID);
									printf("ENTRY SUCCESS\n");
									draw_ascii_ok(185,550, WHITE, "ENTRY SUCCESS");
									sleep(1);
									ID_num++;
									return 0;
								}
							}
						}
					}			
				}
				else
				{
					printf("Have Exist \n");
					draw_ascii_ok(195,550, WHITE, "Have Exist");
					sleep(1);
					return 0;
				}
			}
		}
	}
	
	printf("NO Finger\n");
	draw_ascii_ok(195,550, WHITE, "NO FINGER");
	sleep(2);
	return 0;
}



int press_FR(void)
{
	int i;
	
	for(i=0;i<20;i++)
	{
		data = PS_GetImage();
		if(data == 1)
		{
			data = PS_GenChar(1);
			if(data == 2)
			{
				data = PS_HighSpeedSearch(0,10);
				if(data == 6)
				{
					printf("Search SUCCESS\n");
					draw_ascii_ok(460,550, WHITE, "Search SUCCESS");
					write(fd2, "1", 1);
					ioctl(fd4, PWM_IOCTL_SET_FREQ, 10000);
					sleep(1);
					ioctl(fd4, PWM_IOCTL_STOP);
					sleep(1);
					write(fd2, "0", 1);
					return 0;
				}
				else
				{
					printf("Search ERROR\n");
					draw_ascii_ok(460,550, WHITE, "Search ERROR");
					sleep(2);
					return 0;
				}
			}	
		}
	}
	draw_ascii_ok(470,550, WHITE, "NO FINGER");
	sleep(2);
	
	printf("NO Finger\n");
	return 0;
}

int Del_FR(unsigned int ID)
{
	data = PS_DeletChar(ID);
	if(data == 7)
	{
		printf("PS_DeletChar SUCCESS\n");
		draw_ascii_ok(730,550, WHITE, "Delete SUCCESS");
		sleep(1);
		return 0;
	}
	else
	{
		printf("PS_DeletChar ERROR\n");
		return 3;
	}
}

int Init(void)
{
	
	fd1 = open("/dev/input/event2", O_RDONLY);
	if (fd1 < 0)
	{
		perror("open");
		return -1;
	}
	fd2 = open("/sys/devices/platform/x210-led/led1", O_RDWR);
	if (fd2 < 0)
	{
		printf("open %s error.\n", "/sys/devices/platform/x210-led/led1");
		return -1;
	}
	fd3 = open("/dev/fb0", O_RDWR);
	
	if (fd3 < 0)
	{
		perror("open");
		return -1;
	}
	
	
	ret2 = ioctl(fd3, FBIOGET_FSCREENINFO, &finfo);
	if (ret2 < 0)
	{
		perror("ioctl");
		return -1;
	}
	ret2 = ioctl(fd3, FBIOGET_VSCREENINFO, &vinfo);
	if (ret2 < 0)
	{
		perror("ioctl");
		return -1;
	}
	
	unsigned long len = vinfo.xres_virtual * vinfo.yres_virtual * vinfo.bits_per_pixel / 8;
	printf("len = %ld\n", len);
	pfb = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd3, 0);
	if (NULL == pfb)
	{
		perror("mmap");
		return -1;
	}
	
	
}


void uart_init(void)
{
	if((fd=open_port(fd,1))<0)
    {
        perror("open_port error");
        return;
    }
	
	saio.sa_handler = signal_handler_IO;
	sigemptyset (&saio.sa_mask);	
	saio.sa_flags = 0;
	saio.sa_restorer = NULL;
	sigaction (SIGIO, &saio, NULL);

	
	fcntl (fd, F_SETOWN, getpid ());
	
	fcntl (fd, F_SETFL, FASYNC);
	
    if((i=set_opt(fd,115200,8,'N',1))<0)
    {
        perror("set_opt error");
        return;
    }
}





void Z_pipe_read() //zhiwen read
{
	close(fd_pipe[1]);
    nBytes = read(fd_pipe[0], &ch, 1);
}
void F_pipe_write() //LCD write
{
	close(fd_pipe[0]);
    nBytes = write(fd_pipe[1], &ch, 1);
}

void F_pipe_read() //LCD read
{
	close(fd_pipe[0]);
    nBytes = read(fd_pipe[1], &ch, 1);
}
void Z_pipe_write() //zhiwen write
{
	close(fd_pipe[1]);
    nBytes = write(fd_pipe[0], &ch, 1);
}

		
static inline void lcd_draw_pixel(unsigned int x, unsigned int y, unsigned int color)
{
	*(pfb + 1024 * y + x) = color;
}


static void lcd_draw_background1(int ROW, int COL,int color)
{
	int i, j;
	
	for (j=542; j<542+ROW; j++)
	{
		for (i=160; i<160+COL; i++)
		{
			lcd_draw_pixel(i, j, color);
		}
	}
}

static void lcd_draw_background2(int ROW, int COL,int color)
{
	int i, j;
	
	for (j=542; j<542+ROW; j++)
	{
		for (i=435; i<435+COL; i++)
		{
			lcd_draw_pixel(i, j, color);
		}
	}
}

static void lcd_draw_background3(int ROW, int COL,int color)
{
	int i, j;
	
	for (j=542; j<542+ROW; j++)
	{
		for (i=711; i<711+COL; i++)
		{
			lcd_draw_pixel(i, j, color);
		}
	}
}




void draw_circular(unsigned int centerX, unsigned int centerY, unsigned int radius, unsigned int color)
{
	int x,y ;
	int tempX,tempY;;
    int SquareOfR = radius*radius;

	for(y=0; y<600; y++)
	{
		for(x=0; x<1024; x++)
		{
			if(y<=centerY && x<=centerX)
			{
				tempY=centerY-y;
				tempX=centerX-x;                        
			}
			else if(y<=centerY&& x>=centerX)
			{
				tempY=centerY-y;
				tempX=x-centerX;                        
			}
			else if(y>=centerY&& x<=centerX)
			{
				tempY=y-centerY;
				tempX=centerX-x;                        
			}
			else
			{
				tempY = y-centerY;
				tempX = x-centerX;
			}
			if ((tempY*tempY+tempX*tempX)<=SquareOfR)
				lcd_draw_pixel(x, y, color);
		}
	}
}

void lcd_draw_picture(const unsigned char *pData)
{
	unsigned int x, y, color, p = 0;
	
	for (y=0; y<600; y++)
	{
		for (x=0; x<1024; x++)
		{
			// 在这里将坐标点(x, y)的那个像素填充上相应的颜色值即可
			color = (pData[p+0] << 0) | (pData[p+1] << 8) | (pData[p+2] << 16);
			lcd_draw_pixel(x, y, color);
			p += 3;
		}
	}
}





// 写字
// 写字的左上角坐标(x, y)，字的颜色是color，字的字模信息存储在data中
static void show_8_161(unsigned int x, unsigned int y, unsigned int color, unsigned char *data)  
{  
// count记录当前正在绘制的像素的次序
    int i, j, p,count = 0;  
	  
	  for(p=0;p<3;p++)
	  {
	  
		for (j=y; j<(y+43); j++)  //纵向刷新
		{  
			for (i=x; i<(x+8); i++)  //横向刷新
			{  
				if (data[count/8] & (1<<(count%8))) 
				{
					lcd_draw_pixel(i, j, color);
				}				
				count++;  
			}  
				
		}
		x=x+8;
	  }
} 

// 写字符串
// 字符串起始坐标左上角为(x, y)，字符串文字颜色是color,字符串内容为str
void draw_ascii_ok1(unsigned int x, unsigned int y, unsigned int color, unsigned char *str)
{
	int i;  
	unsigned char *ch;
    for (i=0; str[i]!='\0'; i++)  
    {  
		ch = (unsigned char *)ascii_16_16[(unsigned char)str[i]-0x30];
        show_8_161(x, y, color, ch); 
		
        x += 16;
		if (x >= 600)
		{
			x -= 600;			// 回车
			y += 43;			// 换行
		}
    }  
}

// 写字
// 写字的左上角坐标(x, y)，字的颜色是color，字的字模信息存储在data中
static void show_8_16(unsigned int x, unsigned int y, unsigned int color, unsigned char *data)  
{  
// count记录当前正在绘制的像素的次序
    int i, j, count = 0;  
	  
    for (j=y; j<(y+16); j++)  
    {  
        for (i=x; i<(x+8); i++)  
        {  
            if (i<1024 && j<600)  
            {  
			// 在坐标(i, j)这个像素处判断是0还是1，如果是1写color；如果是0直接跳过
            	if (data[count/8] & (1<<(count%8)))   
					lcd_draw_pixel(i, j, color);
            }  
            count++;  
        }  
    }  
} 

// 写字符串
// 字符串起始坐标左上角为(x, y)，字符串文字颜色是color,字符串内容为str
void draw_ascii_ok(unsigned int x, unsigned int y, unsigned int color, unsigned char *str)
{
	int i;  
	unsigned char *ch;
    for (i=0; str[i]!='\0'; i++)  
    {  
		ch = (unsigned char *)ascii_8_16[(unsigned char)str[i]-0x20];
        show_8_16(x, y, color, ch); 
		
        x += 8;
		if (x >= 1024)
		{
			x -= 1024;			// 回车
			y += 16;			// 换行
		}
    }  
}


		

void finger(void)
{
    uart_init();
	ch = 0;
	while(1)
	{

		switch(ch)
		{
			case 0:Z_pipe_read();						break;
			case 1:ch = Add_FR(ID_num);	draw_WHITE();	break;
			case 2:ch = press_FR();		draw_WHITE();	break;
			case 3:ch = Del_FR(0);		draw_WHITE(); 	break;
		}
	}
    close(fd);
	close(fd4);
    return;
}

void draw_WHITE(void)
{
	lcd_draw_background1(24, 150,WHITE);
	lcd_draw_background2(24, 150,WHITE);
	lcd_draw_background3(24, 150,WHITE);
}
void Press_Screen()
{
	while(1)
	{
		memset(&ev, 0, sizeof(struct input_event));
		ret1 = read(fd1, &ev, sizeof(struct input_event));
		if (ret1 != sizeof(struct input_event))
		{
			perror("read");
			close(fd1);
		}
		
		if(ev.type==3)
		{
			if(ev.code==0 || ev.code==1)
			{
				if(ev.code==0)
				{
					x=ev.value;
				}
				else if(ev.code==1)
				{
					y=ev.value;
					printf("x=%d,y=%d\n",x,y );
					
/*					
					if(x>=0&&x<=512&&y>=0&&y<=600)
					{
						write(fd2, "1", 1);
					}
					else
					{
						write(fd2, "0", 1);
					}
				
*/				
					if(y>=460&&y<=540)
					{
						if(x>=130&&x<=300)
						{
							ch = 1;
							lcd_draw_background1(24, 150,BLUE);
							lcd_draw_background2(24, 150,WHITE);
							lcd_draw_background3(24, 150,WHITE);
						}
						else if(x>=420&&x<=580)
						{
							ch = 2;
							lcd_draw_background2(24, 150,BLUE);
							lcd_draw_background1(24, 150,WHITE);
							lcd_draw_background3(24, 150,WHITE);
						}
						else if(x>=700&&x<=900)
						{
							ch = 3;
							lcd_draw_background3(24, 150,BLUE);
							lcd_draw_background1(24, 150,WHITE);
							lcd_draw_background2(24, 150,WHITE);
						}
					}
					else
					{
						ch = 0;
					}
					
					F_pipe_write();
					printf("Press_Screen=%d\n",ch);
					LCD_write();
					
				}
			}
			
		}
	}
	close(fd1);
	close(fd2);
	close(fd4);
}


void signal_handler_IO (int status)
{
	int i;
	res = read  (fd, rx_buf, sizeof(rx_buf));
	if(res != 0)
	{
		printf ("read=%d,", res);
		for (i = 0;i < res;i++)
		{
			use_buf[i]=rx_buf[i];
			printf ("%02X",use_buf[i]);
		}
		printf ("\n");
	}
	memset (rx_buf,0,sizeof(rx_buf));
	
}

unsigned char c;

void LCD_write(void)
{
	draw_circular(800, 275, 45, BLACK);
	ch =ch + 0x30;
	draw_ascii_ok1(790, 255, RED, &ch);
	ch =ch - 0x30;
}
void operation_LCD(void)
{
	lcd_draw_picture(gImage_1024);
	usleep(100000);
	//draw_ascii_ok(512, 300, RED, &c);
	draw_circular(800, 275, 45, BLACK);
	while(1)
	{
		Press_Screen();
	}
}


int BUZZER_INIT(void)
{
	fd4 = open("/dev/buzzer", O_RDWR);
	if (fd4 < 0)
	{
		perror("open");
		return -1;
	}
}
int main(void)
{
	pid_t p1 = -1;
	
	pipe(fd_pipe);
	p1 = fork();		
	
	Init();
	
	
	
	
	if (p1 == 0)
	{
		BUZZER_INIT();
		finger();
	}
	
	if (p1 > 0)
	{
		
		operation_LCD();
	}
	
	if (p1 < 0)
	{
		printf("fork error\n");
	}
	

	return 0;
}