#include "AD5933.h"
#include "delay.h"
#define Delay_IIC 1
#define AD5933 1
#define AD5933_MCLK 16.776 //=536870912/MCLK;
//#define	AD5933_MCLK_USE_OUT	1	//0内部时钟  1外部时钟
#define AD5933_Correction 101615461.47044108 //10k

double R_Correction[8] = { 841204.98046875, 8412049.8046875, 101615461.47044108, 1008900806.64062, 2019625082.03125, 5054057539.0625, 10066827753.9062, 19650779687.5};
extern uint8_t range;
void Ini_I2c(void) //初始化I2C
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10
			| GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	return;
}
/* 档位选择 0-7 测试8档, 档位选择 8-14 测试7档 */
void RangeSelect(uint8_t index) {
	switch (index) {
	case 0:
		/* Y0, IY2 */
		S0(0);
		S1(0);
		S2(0);
		S3(0);
		S4(1);
		S5(0);
		break;
	case 1:
		/* Y1, IY2 */
		S0(1);
		S1(0);
		S2(0);
		S3(0);
		S4(1);
		S5(0);
		break;
	case 2:
		/* Y2, IY2 */
		S0(0);
		S1(1);
		S2(0);
		S3(0);
		S4(1);
		S5(0);
		break;
	case 3:
		/* Y3, IY2 */
		S0(1);
		S1(1);
		S2(0);
		S3(0);
		S4(1);
		S5(0);
		break;
	case 4:
		/* Y4, IY2 */
		S0(0);
		S1(0);
		S2(1);
		S3(0);
		S4(1);
		S5(0);
		break;
	case 5:
		/* Y5, IY2 */
		S0(1);
		S1(0);
		S2(1);
		S3(0);
		S4(1);
		S5(0);
		break;
	case 6:
		/* Y6, IY2 */
		S0(0);
		S1(1);
		S2(1);
		S3(0);
		S4(1);
		S5(0);
		break;
	case 7:
		/* Y7, IY2 */
		S0(1);
		S1(1);
		S2(1);
		S3(0);
		S4(1);
		S5(0);
		break;
	case 8:
		/* Y1, IY1 1K校准 */
		S0(1);
		S1(0);
		S2(0);
		S3(1);
		S4(0);
		S5(0);
		break;
	case 9:
		/* Y2, IY0 10K校准 */
		S0(0);
		S1(1);
		S2(0);
		S3(0);
		S4(0);
		S5(0);
		break;
	case 10:
		/* Y3, IY3 */
		S0(1);
		S1(1);
		S2(0);
		S3(1);
		S4(1);
		S5(0);
		break;
	case 11:
		/* Y4, IY4 */
		S0(0);
		S1(0);
		S2(1);
		S3(0);
		S4(0);
		S5(1);
		break;
	case 12:
		/* Y5, IY5 */
		S0(1);
		S1(0);
		S2(1);
		S3(1);
		S4(0);
		S5(1);
		break;
	case 13:
		/* Y6, IY6 */
		S0(0);
		S1(1);
		S2(1);
		S3(0);
		S4(1);
		S5(1);
		break;
	case 14:
		/* Y7, IY7 */
		S0(1);
		S1(1);
		S2(1);
		S3(1);
		S4(1);
		S5(1);
		break;
	default:
		break;
	}
}
void NOPS(void) {
	delay_us(20);
}
void SDA_1(void) {
//    SDA_OUT();		//将SDA设置为输出模式
	SDA(1); //SDA管脚输出为高电平
	NOPS();
	return;
}

void SDA_0(void) {
//    SDA_OUT();		//将SDA设置为输出模式
	SDA(0); //SDA管脚输出为低电平
	NOPS();
	return;
}

void SCL_1(void) {
//将SCL设置为输出模式
	SCL(1); //SCL管脚输出为高电平
	NOPS();
	return;
}

void SCL_0(void) {
//将SCL设置为输出模式
	SCL(0); //SCL管脚输出为低电平
	NOPS();
	return;
}

void GetACK(void) {
	uint8_t ucErrTime = 0;

	SDA_IN(); //SDA设置为输入
	SDA(1);
	delay_us(Delay_IIC * 1);
	SCL(1);
	delay_us(Delay_IIC * 1);
	while (READ_SDA) {
		ucErrTime++;
		if (ucErrTime > 250) {
			STOP();
			return;
		}
	}
	SCL(0); //时钟输出0
}

void SendNACK(void) {
	SCL(0);SDA_OUT();
	SDA(1);
	delay_us(Delay_IIC * 2);
	SCL(1);
	delay_us(Delay_IIC * 2);
	SCL(0);
}

void START(void) // 启动数据总线
{
	SDA_OUT(); //sda线输出
	SDA(1);
	SCL(1);
	delay_us(Delay_IIC * 4);
	SDA(0); //START:when CLK is high,DATA change form high to low
	delay_us(Delay_IIC * 4);
	SCL(0); //钳住I2C总线，准备发送或接收数据
}

void STOP(void) {
	SDA_OUT(); //sda线输出
	SCL(0);
	SDA(0); //STOP:when CLK is high DATA change form low to high
	delay_us(Delay_IIC * 4);
	SCL(1);
	SDA(1); //发送I2C总线结束信号
	delay_us(Delay_IIC * 4);
}

void SendByte(uint8_t txd) // 发送一个字节数据子函数
{
	uint8_t t;
	SDA_OUT();
	SCL(0); //拉低时钟开始数据传输
	for (t = 0; t < 8; t++) {
		SDA((txd & 0x80) >> 7);
		txd <<= 1;
		delay_us(Delay_IIC * 2); //对TEA5767这三个延时都是必须的
		SCL(1);
		delay_us(Delay_IIC * 2);
		SCL(0);
		delay_us(Delay_IIC * 2);
	}
}

uint8_t ReadByte(void) //读一个字节数据
{
	unsigned char i, receive = 0;
	SDA_IN(); //SDA设置为输入
	for (i = 0; i < 8; i++) {
		SCL(0);
		delay_us(Delay_IIC * 2);
		SCL(1);
		receive <<= 1;
		if (READ_SDA)
			receive++;
		delay_us(Delay_IIC * 1);
	}
	SendNACK();
	return receive;
}

void Write_Byte(char nAddr, uint nValue) //nAddr中写入字节nValue
{
	int nTemp = 0x1A; // AD5933的默认地址&写控制位（低）
	START();
	SendByte(nTemp); // 发送地址
	GetACK();

	SendByte(nAddr); // 发送地址字节
	GetACK();

	SendByte(nValue); // 发送数据字节
	GetACK();

	STOP(); // 停止总线
	return;
}

void SetPointer(char nAddr) //   设置地址指针
{
	int nTemp = 0x1A; // AD5933的默认地址&写控制位（低）

	START();
	SendByte(nTemp); // 发送地址
	GetACK();        // 等待 ACK

	SendByte(0xB0); // 发送指针命令1101 0000
	GetACK();

	SendByte(nAddr); // 发送地址指针
	GetACK();

	STOP(); // 停止总线
	return;
}

int Rece_Byte(char nAddr) //读取nAddr中的字节到返回值
{
	int nTemp;
	SetPointer(nAddr); //地址指针指向nAddr
	nTemp = 0x1B;      // AD5933的默认地址&读控制位（高）
	START();

	SendByte(nTemp); // 发送地址
	GetACK();

	nTemp = ReadByte(); //读一个字节数据
	SendNACK();         //发送NO_ACK

	STOP(); // 停止总线
	return nTemp;
}

uint16_t AD5933_Tempter(void) {

// unsigned char Status;  //保存状态
	unsigned int Tm; //保存实部，虚部，温度

//   //复位AD5933
//   Write_Byte(0x80,0XB1);
//   Write_Byte(0x81,0X00);

//启动温度测量
	Write_Byte(0x80, 0x93);

//等待转换完成
//   do
//    {
//
//     Status=Rece_Byte(0x8F);
//
//}while(!(Status & 0x01));
//读出温度，保存在Tm中
	Tm = Rece_Byte(0x92);
	Tm <<= 8;
	Tm += Rece_Byte(0x93);
	Tm <<= 2;

	return Tm;
}

//float Scale_imp (uint SValue[3],uint IValue[3],uint NValue[2],uint CValue[2],float ki,int Ps);
float resistance[200];
float rads[200];
int AD5933_Dat_Re[200];
int AD5933_Dat_Im[200];

void Maopao_Paixu(float *dat, uint16_t leng) {
	uint16_t i, j;
	float buf;
	for (j = 0; j < leng - 1; j++)
		for (i = 0; i < leng - j - 1; i++)
			if (dat[i] > dat[i + 1]) {
				buf = dat[i];
				dat[i] = dat[i + 1];
				dat[i + 1] = buf;
			}
}

float Get_resistance(uint16_t num) {
	uint16_t i;
	float navle;
	Maopao_Paixu(resistance, num);
	navle = resistance[0];
	for (i = num / 2 - num / 4; i < num / 2 + num / 4; i++) {
		navle = (navle + resistance[i]) / 2;
	}

	return (navle * R_Correction[range]);
}
void Fre_To_Hex(float fre, uint8_t *buf) {
	uint32_t dat;
	dat = (536870912 / (double) (AD5933_MCLK * 1000000)) * fre;
	buf[0] = dat >> 16;
	buf[1] = dat >> 8;
	buf[2] = dat;
}
/*Fre_Begin起始频率，Fre_UP频率增量，UP_Num增量数，OUTPUT_Vatage输出电压，Gain增益系数，SWeep_Rep扫频为1重复为0*/

/*
 Fre_Begin 		开始频率 （HZ）
 Fre_UP				步进频率（HZ）
 UP_Num				步进次数
 OUTPUT_Vatage	输出电压
 AD5933_OUTPUT_2V
 AD5933_OUTPUT_1V
 AD5933_OUTPUT_400mV
 AD5933_OUTPUT_200mV

 Gain					PGA增益
 AD5933_Gain_1
 AD5933_Gain_5
 SWeep_Rep			扫描模式
 AD5933_Fre_UP 	递增频率
 AD5933_Fre_Rep	重复频率
 */

//AD5933_Sweep(30000,200,200,AD5933_OUTPUT_2V,AD5933_Gain_1,AD5933_Fre_UP);
float AD5933_Sweep(float Fre_Begin, float Fre_UP, uint16_t UP_Num,
		uint16_t OUTPUT_Vatage, uint16_t Gain, uint16_t SWeep_Rep) {
	uint8_t SValue[3], IValue[3], NValue[2], CValue[2];
	uint16_t buf = 0;
	Fre_To_Hex(Fre_Begin, SValue);
	Fre_To_Hex(Fre_UP, IValue);
	NValue[0] = UP_Num >> 8;
	NValue[1] = UP_Num;
#ifdef AD5933_MCLK_USE_OUT
  buf = OUTPUT_Vatage | Gain | SWeep_Rep | AD5933_OUT_MCLK;
#else
	buf = OUTPUT_Vatage | Gain | SWeep_Rep | AD5933_IN_MCLK;
#endif
	CValue[0] = buf >> 8;
	CValue[1] = buf;

	Scale_imp(SValue, IValue, NValue, CValue);
	return 0;
}
/*SValue[3]起始频率，IValue[3]频率增量，NValue[2]增量数，CValue[2]控制字，ki增益系数，Ps扫频为1重复为0*/

float Scale_imp(uint8_t *SValue, uint8_t *IValue, uint8_t *NValue,
		uint8_t *CValue) {
	int i, j, AddrTemp;
	float Gain = ((~CValue[0]) & 0x01) ? 5 : 1;
	uint8_t SWeep_Rep = ((CValue[0] & 0xF0) == (AD5933_Fre_UP >> 8)) ? 1 : 0;
	uint8_t Mode = CValue[0] & 0x0f;
	long ReadTemp, realArr[3], imageArr[3];
	float magnitude;
//                uint start_f[3]={0X33,0X26,0X17};
//                uint inc_f[3]={0,0,0X21};
//                uint num_f[2]={0,0XC8};
//                uint control[2]={0XB1,0X00};
//								CValue[0]=Mode|AD5933_Standby;
	j = 0;
	Ini_I2c(); //初始化I2C

	AddrTemp = 0X82; //初始化起始频率寄存器
	for (i = 0; i < 3; i++) {
		Write_Byte(AddrTemp, SValue[i]);
		AddrTemp++;
	}
	AddrTemp = 0X85; //初始化频率增量寄存器
	for (i = 0; i < 3; i++) {
		Write_Byte(AddrTemp, IValue[i]);
		AddrTemp++;
	}
	AddrTemp = 0X88; //初始化频率点数寄存器
	for (i = 0; i < 2; i++) {
		Write_Byte(AddrTemp, NValue[i]);
		AddrTemp++;
	}
//初始化控制寄存器，1011 0001 0000 0000待机模式，2V，一倍放大，内部时钟
	AddrTemp = 0X80;
//                for(i = 0;i <2;i++)
	{
		Write_Byte(AddrTemp, (Mode | (AD5933_Standby >> 8)));
		AddrTemp++;
		Write_Byte(AddrTemp, CValue[1]);
		AddrTemp++;
	}

	Write_Byte(0x80, (Mode | (AD5933_SYS_Init >> 8))); //控制寄存器写入初始化频率扫描命令
	HAL_Delay(10);
	Write_Byte(0X80, (Mode | (AD5933_Begin_Fre_Scan >> 8))); //控制寄存器写入开始频率扫描命令
	while (1) {
		while (1) {
			ReadTemp = Rece_Byte(0x8F); //读取状态寄存器检查DFT是否完成
										//							ReadTemp=ReadTemp&0x07;
			if (ReadTemp & 0x02)
				break;
		}
		realArr[0] = Rece_Byte(0x94);
		realArr[1] = Rece_Byte(0x95);
		realArr[2] = realArr[0] * 0x100 + realArr[1];

		imageArr[0] = Rece_Byte(0x96);
		imageArr[1] = Rece_Byte(0x97);
		imageArr[2] = imageArr[0] * 0x100 + imageArr[1];

		rads[j] = atan2(imageArr[2], realArr[2]) - 0.00143485062;

		if (realArr[2] >= 0x8000) //计算实部的原码(除符号位外，取反加一)
				{
			realArr[2] ^= 0xFFFF;
			realArr[2] ^= 0x8000;
			realArr[2] += 1;
			realArr[2] ^= 0x8000;
		}
		if (imageArr[2] >= 0x8000) //计算虚部的原码(除符号位外，取反加一)
				{
			imageArr[2] ^= 0xFFFF;
			imageArr[2] ^= 0x8000;
			imageArr[2] += 1;
			imageArr[2] ^= 0x8000;
		}
		AD5933_Dat_Re[j] = realArr[2];
		AD5933_Dat_Im[j] = imageArr[2];
		magnitude = sqrt(realArr[2] * realArr[2] + imageArr[2] * imageArr[2]); //模值计算
		resistance[j++] = 1 / (magnitude * Gain);                         //阻抗计算
																		  //								printf("%s%f%s", "magnitude = ",magnitude,"\r\n");
		ReadTemp = Rece_Byte(0x8F);                          //读取状态寄存器检查频率扫描是否完成
		if (ReadTemp & 0x04)
			break;
		if (SWeep_Rep == 1)
			Write_Byte(0X80, CValue[0]); //控制寄存器写入增加频率（跳到下一个频率点)的命令
		else
			Write_Byte(0X80, CValue[0]); //控制寄存器写入重复当前频率点扫描
	}
	Write_Byte(0X80, 0XA1); //进入掉电模式
	return magnitude;
}

float DA5933_Get_Rs(void) {
	float __attribute__ ((unused)) Rs, re, im;

	AD5933_Sweep(10000, 1, 8, AD5933_OUTPUT_2V, AD5933_Gain_1, AD5933_Fre_UP);
	Rs = Get_resistance(8);
	re = Rs * cos(rads[0]);
	im = Rs * sin(rads[0]);
	return Rs;
}
float DA5933_Dat_Cap(float Fre) {
	float pp;
//	float dat=3.1415926*2*Fre;
	float dat = 1;
	pp = 1000000 * 23.9999992 / (AD5933_Dat_Re[0] * dat);
	return pp;
}

float DA5933_Get_Cap(float Fre) {
//	float Cap,dat;
//
//	AD5933_Sweep(30000,1,40,AD5933_OUTPUT_2V,AD5933_Gain_1,AD5933_Fre_UP);
//	Cap=DA5933_Get_Rs();
//	dat=1/(Cap*30000*2*3.1415926/100000000000000);
//	DA5933_Dat_Cap(30000);
//
//	return Cap;

	float __attribute__ ((unused)) Rs, re, im, cap;

//	AD5933_Sweep(30000,200,2,AD5933_OUTPUT_2V,AD5933_Gain_1,AD5933_Fre_Rep);
	AD5933_Sweep(100000, 1, 20, AD5933_OUTPUT_2V, AD5933_Gain_1, AD5933_Fre_UP);
	Rs = Get_resistance(20);
//	re=Rs*cos(rads[0]);
	im = Rs * sin(rads[0]);
	cap = 0.9442 / (2 * 3.1415926 * im / 10000000);
	return cap;
}
float DA5933_Get_L(float Fre) {
	float L;
	float __attribute__ ((unused)) Rs, re, im;

	AD5933_Sweep(100000, 1, 20, AD5933_OUTPUT_2V, AD5933_Gain_1, AD5933_Fre_UP);
	Rs = Get_resistance(20);
//	re=Rs*cos(rads[0]);
	im = Rs * sin(rads[0]);
	L = im * 1000 / (2 * 3.1415926);
	return L;
}
//---------------------------------------------------------------------------------------------------------------------
// 函数原形：void display(unsigned int re,unsigned int im)
// 功能描述：显示函数。
// 参数说明：unsigned int re,unsigned int im ,实部和虚部
// 返回值：无
//---------------------------------------------------------------------------------------------------------------------

/*
 void display(signed int re,signed int im)
 {
 unsigned char a[11],y=0,x=0,i=0;
 signed int  revalu=0,imvalu=0;
 float  valu=0;
 double xishu=0,regist;
 long  int ll;

 revalu=re;
 imvalu=im;
 if(revalu&0x8000)           //把带符号的换算成无符号整数
 {
 revalu=(revalu-0x10000)*(-1);
 x=1;
 }
 if(imvalu&0x8000)           //把带符号的换算成无符号整数
 {
 imvalu=(imvalu-0x10000)*(-1);
 y=1;
 }
 valu=sqrt(imvalu*imvalu+revalu*revalu);  //计算幅值
 switch(j)
 {
 case 0:
 if((valu<13000)&&(valu>8000))
 {
 xishu=(1/(96.86810));            //计算系数
 xishu=(1/(xishu*valu))*100000+0.5;   //计算阻值
 ll=xishu-250;
 i=1;
 }
 break;
 case 1:
 if((valu<13000)&&(valu>1350))
 {
 xishu=(1/(13.33726));            //计算系数
 xishu=(1/(xishu*valu))*1000000+0.5;   //计算阻值
 ll=xishu-100;
 i=1;
 }
 break;
 case 3:
 if((valu<12000)&&(valu>1050))
 {
 xishu=(1/(10.398994));            //计算系数
 xishu=(1/(xishu*valu))*10000000+0.5;   //计算阻值
 ll=xishu;
 i=1;
 }
 break;
 case 4:
 if((valu<12000)&&(valu>1000))
 {
 xishu=(1/(96.736596));            //计算系数
 xishu=(1/(xishu*valu))*10000000+0.5;   //计算阻值
 ll=xishu;
 i=1;
 }
 break;
 case 5:
 if((valu<2130)&&(valu>1060))
 {
 xishu=(1/(21.006));            //计算系数
 xishu=(1/(xishu*valu))*100000000+0.5;   //计算阻值
 ll=xishu;
 i=1;
 }
 break;
 case 6:
 if((valu<4800)&&(valu>800))
 {
 xishu=(1/(96.527914));            //计算系数
 xishu=(1/(xishu*valu))*100000000+0.5;   //计算阻值
 ll=xishu;
 i=1;
 }
 break;
 default:
 break;
 }
 if(i==1)                //满足条件则显示阻值，相位
 {
 a[0]=ll/10000000+48;
 a[1]=ll%10000000/1000000+48;
 a[2]=ll%1000000/100000+48;
 a[3]=ll%100000/10000+48;
 a[4]=ll%10000/1000+48;
 a[5]='.';
 a[6]=ll%1000/100+48;
 a[7]=ll%100/10+48;
 a[8]=ll%100%10+48;
 a[9]='K';
 a[10]=0;
 LCD_WriteStr(4,2,a);

 regist=ll;
 if((y==0)&&(x==0))
 {
 //计算并显示  atan2(float y, float x);  求y/x（弧度表示）的反正切值
 xishu=atan2(imvalu,revalu)*180/3.14+0.5;
 }
 if((y==1)&&(x==0))
 {
 //计算并显示  atan2(float y, float x);  求y/x（弧度表示）的反正切值
 xishu=atan2((imvalu*(-1)),revalu)*180/3.14-0.5;
 xishu=xishu+180;
 }
 if((y==1)&&(x==1))
 {
 //计算并显示  atan2(float y, float x);  求y/x（弧度表示）的反正切值
 xishu=atan2(imvalu,revalu)*180/3.14+0.5;
 xishu=xishu+180;
 }
 if((y==0)&&(x==1))
 {
 xishu=atan2(imvalu,(revalu*(-1)))*180/3.14-0.5;          //计算并显示  atan2(float y, float x);  求y/x（弧度表示）的反正切值
 xishu=xishu+360;
 }
 if((ll<1000)&&(ll>400))                //相位误差修正
 ll=xishu-89.5;
 if((ll<10000)&&(ll>=1000))
 ll=xishu-(-0.0012237*regist+91.46)+0.5;
 if((ll<100000)&&(ll>=10000))
 ll=xishu-(-0.0001515*regist+91.016)+0.5;
 if((ll<1000000)&&(ll>=100000))
 ll=xishu-(-0.0000179*regist+91.806)+0.5;
 if((ll<=2000000)&&(ll>=1000000))
 ll=xishu-(-0.000009675*regist+93.789)+0.5;
 if((ll<12000000)&&(ll>2000000))
 ll=xishu-(-0.00000256*regist+95.444)+0.5;
 if(ll<0)                     //判断正负
 {
 ll=ll*(-1);
 a[0]='-';
 a[1]=ll/100+48;
 a[2]=ll%100/10+48;
 a[3]=ll%100%10+48;
 a[4]=0;
 LCD_WriteStr(4,3,a);
 }
 else
 {
 a[0]=' ';
 a[1]=ll/100+48;
 a[2]=ll%100/10+48;
 a[3]=ll%100%10+48;
 a[4]=0;
 LCD_WriteStr(4,3,a);
 }
 }
 }

 */
