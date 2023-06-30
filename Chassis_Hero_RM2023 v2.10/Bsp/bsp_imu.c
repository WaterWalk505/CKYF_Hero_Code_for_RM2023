/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_imu.c
 * @brief      mpu6500 module driver, configurate MPU6500 and Read the Accelerator
 *             and Gyrometer data using SPI interface      
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#include "bsp_imu.h"
#include "ist8310_reg.h" 
#include "stm32f4xx_hal.h"
#include <math.h>
#include "mpu6500_reg.h"
#include "spi.h"



/*************MPU的SPI和片选****************************/
#define MPU_HSPI hspi5
#define MPU_NSS_LOW HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

#define Kp 2.0f  // 比例增益控制加速度计/磁力计的收敛速度
#define Ki 0.01f    //积分增益控制陀螺仪偏置的收敛速度                      

volatile float        q0 = 1.0f;
volatile float        q1 = 0.0f;
volatile float        q2 = 0.0f;
volatile float        q3 = 0.0f;
volatile float        exInt, eyInt, ezInt;                   /* error integral */
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;  
volatile uint32_t     last_update, now_update;               /* Sampling cycle count, ubit ms */
static uint8_t        tx, rx;
static uint8_t        tx_buff[14] = { 0xff };








/***********************************[1] 寄存器读写操作函数*********************************************/
/**[1.1]					
  * brief					向寄存器写一个字节的数据
	* param[in]			reg:要写入的寄存器的地址
  * param[in]			data:要写入寄存器的数据
	* postscript					  		*/
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
{
		/*(1) 片选线拉低，选中设备 */
    MPU_NSS_LOW;
	
		/*(2) 发送要写入的寄存器地址 */
    tx = reg & 0x7F;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
	
		/*(3) 发送要写入寄存器的数据 */
    tx = data;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
	
		/*(4) 片选线拉高，取消选中设备 */
    MPU_NSS_HIGH;
	
    return 0;
}

/**[1.2]					
  * brief					从寄存器读一个字节的数据
	* param[in]			reg:要读出的寄存器的地址
  * retval				读出的数据
	* usage								*/
uint8_t mpu_read_byte(uint8_t const reg)
{
		/*(1) 片选线拉低，选中设备 */
    MPU_NSS_LOW;
	
		/*(2) 发送要读取的寄存器地址 */
    tx = reg | 0x80;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
		
		/*(3) 接收寄存器传回来的数据存到rx */
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
	
		/*(4) 片选线拉高，取消选中设备 */
    MPU_NSS_HIGH;
	
    return rx;
}

/**[1.3]					
  * brief					从寄存器读多个字节的数据
	* param[in]			regAddr:要读出的寄存器的地址
	* param[out]		*pData:读出的数据存到这里
	* param[in]			要读取的数据长度
	* postscript		为什么要单独开一个数组tx_buff[]？？？是不是因为SPI传输方式是用同一个移位寄存器同时传tx和rx，数据长度不一样所以提前开一个大数组							*/
uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len)
{
		/*(1) 片选线拉低，选中设备 */
    MPU_NSS_LOW;
	
		/*(2) 发送要读取的寄存器地址 */
    tx         = regAddr | 0x80;
    tx_buff[0] = tx;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
	
		/*(3) 接收寄存器传回来的数据存到rx */
    HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
	
		/*(4) 片选线拉高，取消选中设备 */
    MPU_NSS_HIGH;
	
    return 0;
}

/***********************************[2] IST8310读写操作函数*********************************************/
/**[2.1]					
  * brief					通过MPU6500的I2C主机写入IST8310寄存器一个字节
	* param[in]			addr:IST8310寄存器的写入地址
  * param[in]			data:要写入的数据
	* usage		      ist8310_init() 							*/
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
    /*(1)	首先关闭从设备1 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(2);
	
		/*(2)	向从设备1的地址寄存器写入地址信息 */
    mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
    MPU_DELAY(2);
	
		/*(3)	向从设备1的啥寄存器写入数据信息 */
    mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
    MPU_DELAY(2);
	
    /*(4) 再打开从设备1 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
	
    /*(5) 等待更长时间以确保数据从从设备1传输 */
    MPU_DELAY(10);
}

/**[2.2]					
  * brief					通过MPU6500的I2C主机读取IST8310寄存器一个字节
	* param[in]			addr:IST8310寄存器的读出地址
  * retval				读出的数据		*/
static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
    uint8_t retval;
	
		/*(1)	向从设备4的地址寄存器写入地址信息 */
    mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
    MPU_DELAY(10);
	
		/*(2)	向从设备4的控制寄存器写入指令（是要开启从设备4吗） */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
    MPU_DELAY(10);
	
		/*(3)	从从设备4的xx寄存器读取数据 */
    retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
	
    /*(4) 读取完毕后关闭从设备4 */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);
	
		/*(5)	返回读取的数据*/
    return retval;
}

/***********************************[3] IST8310和MPU6500初始化函数*********************************************/
/**[3.1]					
  * brief					初始化MPU6500 I2C从0以进行I2C读取。
	* param[in]			device_address：从设备地址，Address[6:0]
	* param[in]			reg_base_addr:
	* param[in]			data_num:			
	* postscript		先找资料搞明白I2C的传输原理和MPU6500的这些寄存器是干嘛的*/
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    /*(1)	配置IST8310的设备地址(使用slave1，自动发送单测量模式)*/
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    MPU_DELAY(2);

    /*(2) 使用slave0，自动读取数据 */
    mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    MPU_DELAY(2);

    /*(3)	每八个mpu6500内部采样一个i2c主读取 */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
    MPU_DELAY(2);
	
		/*(4) 启用从机0和从机1访问延迟 */
    mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    MPU_DELAY(2);
		
		/*(5) 启用从机1自动传输 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
		
		/*(6) 等待6ms（16次内部平均设置的最小等待时间） */
    MPU_DELAY(6); 
		
    /*(7) 启用从机0，读取data_num个字节 */
    mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    MPU_DELAY(2);
}


/**[3.2]					
  * brief					初始化IST8310
	* usage 			  mpu_device_init() 			*/
uint8_t ist8310_init()
{
	  /*(1) 启用iic主模式 */
    mpu_write_byte(MPU6500_USER_CTRL, 0x30);
    MPU_DELAY(10);
	  /*(2) 启用iic 400khz */
    mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d); 
    MPU_DELAY(10);

    /*(3)为ist写入而打开从设备1，为ist读取而打开从设备4 */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);  
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    MPU_DELAY(10);

    /*(4) IST8310_R_CONFB 0x01 = device rst */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    MPU_DELAY(10);
    if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
        return 1;

		/*(5) 软复位(soft reset) */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01); 
    MPU_DELAY(10);

		/*(6) 配置为访问寄存器的就绪模式(config as ready mode to access register) */
    ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00); 
    if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
        return 2;
    MPU_DELAY(10);

		/*(7) 正常状态，无int(normal state, no int) */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
        return 3;
    MPU_DELAY(10);
		
    /*(8) 配置低噪声模式，x，y，z轴16时间1平均值*/
    ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
    if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
        return 4;
    MPU_DELAY(10);

    /*(9) 设置/重置脉冲持续时间设置，正常模式 */
    ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
    if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
        return 5;
    MPU_DELAY(10);

    /*(10) 关闭从设备1和从设备4 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);

    /*(11) 配置并打开从设备0*/
    mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    MPU_DELAY(100);
    return 0;
}

/***********************************[4] 读取IST8310和MPU6500数据*********************************************/
/**[4.1]					
  * brief					获取IST8310的数据
	* param[in]			buff：保存IST8310数据的缓冲区
	* usage  				mpu_get_data() 		*/
void ist8310_get_data(uint8_t* buff)
{
    mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6); 
}


/**[4.2]		
	* brief  获取imu的数据
  * usage  main() 
	*/
uint8_t               mpu_buff[14];                          /* 保存imu原始数据的缓冲区 */
mpu_data_t            mpu_data;
uint8_t               ist_buff[6];                           /* 保存IST8310原始数据的缓冲区 */
imu_t                 imu={0};
void mpu_get_data()
{
	  /**(1) 读取加速度计、温度计、陀螺仪的数据 **/
		/*(1.1) 读取原始数据*/
    mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);//MPU6500_ACCEL_XOUT_H是加速度计数据寄存器的首地址，往后14个字节包括了加速度计、温度计、陀螺仪数据寄存器的数据
	  /*(1.2) 加速度计数据拼接*/
    mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];
    mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
    mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];
		/*(1.3) 温度计数据拼接*/
    mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];
		/*(1.4) 陀螺仪数据拼接*/
    mpu_data.gx = ((mpu_buff[8]  << 8 | mpu_buff[9])  - mpu_data.gx_offset);
    mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
    mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);

	
	
		/**(2) 读取磁力计的数据 **/
 	  /*(2.1) 读取磁力计原始数据*/
    ist8310_get_data(ist_buff);
		/*(2.2) 把ist_buff的原始数据复制到mpu_data结构体成员变量，和加速度计、温度计、陀螺仪的数据一块存放*/
		memcpy(&mpu_data.mx, ist_buff, 6);//6字节的长度是把mx,my,mz都复制过去了



		/**(3)	把mpu_data的数据处理一下存到imu的结构体里**/
		/*(3.1)	把三个方向加速度数据存到imu*/
    memcpy(&imu.ax, &mpu_data.ax, 6 * sizeof(int16_t));
		/*(3.2)	把温度原始数据处理成实际温度存到imu*/
    imu.temp = 21 + mpu_data.temp / 333.87f;
		/*(3.3)	把角速度原始数据处理成实际角速度数据（单位：弧度每秒）存到imu*/
	  imu.wx   = mpu_data.gx / 16.384f / 57.3f; // 2000dps(degree per second吧，这个2000是什么意思) -> rad/s 
    imu.wy   = mpu_data.gy / 16.384f / 57.3f; 
    imu.wz   = mpu_data.gz / 16.384f / 57.3f;
}


/**[4.3]	
	* brief  			获取MPU6500的偏移数据
  * postscript	读取300次数据取平均值，就是偏置量(offset) */
void mpu_offset_call(void)
{
	/*(1)	读取300次数据加和*/
	int i;
	for (i=0; i<300;i++)
	{
		mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

		mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
		mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
		mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];
	
		mpu_data.gx_offset += mpu_buff[8]  << 8 | mpu_buff[9];
		mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
		mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];

		MPU_DELAY(5);
	}
	/*(2)	除以300就是平均值*/
	mpu_data.ax_offset=mpu_data.ax_offset / 300;
	mpu_data.ay_offset=mpu_data.ay_offset / 300;
	mpu_data.az_offset=mpu_data.az_offset / 300;
	
	mpu_data.gx_offset=mpu_data.gx_offset / 300;
	mpu_data.gy_offset=mpu_data.gy_offset / 300;
	mpu_data.gz_offset=mpu_data.gz_offset / 300;
}


/***********************************[5] 写入数据设置IST8310和MPU6500测量范围*********************************************/
/**[5.1]		
	* brief  设置imu6500陀螺仪测量范围
  * param  fsr: range(0,±250dps;1,±500dps;2,±1000dps;3,±2000dps)
	* retval 
  * usage  mpu_device_init() */
uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}


/**[5.2]	
	* brief  设置imu 6050/6500加速度测量范围
  * param  fsr: range(0,±2g;1,±4g;2,±8g;3,±16g)
	* retval 
  * usage  mpu_device_init() */
uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3); 
}


/***********************************[6] 初始化两个传感器和四元数*********************************************/
/**[6.1]	
	* brief  		初始化imu mpu6500和磁强计ist3810
  * usage  	  main() */
uint8_t id;
uint8_t mpu_device_init(void)
{
	MPU_DELAY(100);
	
	/*(1)	读取设备ID*/
	id = mpu_read_byte(MPU6500_WHO_AM_I);
	
	/*(2)	MPU6500设备初始化*/
	/*(2.1)	设置寄存器地址和指令数组*/
	uint8_t MPU6500_Init_Data[10][2] = 
	{
			{ MPU6500_PWR_MGMT_1, 0x80 },     /* 重置设备 */ 
			{ MPU6500_PWR_MGMT_1, 0x03 },     /* Clock Source - Gyro-Z */ 
			{ MPU6500_PWR_MGMT_2, 0x00 },     /* 使能陀螺仪和加速度计 */ 
			{ MPU6500_CONFIG, 0x04 },         /* LPF 41Hz */ 
			{ MPU6500_GYRO_CONFIG, 0x18 },    /* +-2000dps */ 
			{ MPU6500_ACCEL_CONFIG, 0x10 },   /* +-8G */ 
			{ MPU6500_ACCEL_CONFIG_2, 0x02 }, /* 使能低通滤波器(LPF,LowPassFilter)  设置加速度计低通滤波 */ 
			{ MPU6500_USER_CTRL, 0x20 },    /* Enable AUX */ 
	};
	/*(2.2)	挨个初始化*/
	uint8_t i = 0;
	for (i = 0; i < 10; i++)
	{
		mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		MPU_DELAY(1);
	}
	
	/*(3)	设置MPU6500陀螺仪和加速度计的测量范围*/
	mpu_set_gyro_fsr(3); 		
	mpu_set_accel_fsr(2);
	
	/*(4)	初始化IST8310*/
	ist8310_init();
	
	/*(5)	获取MPU6500的偏移数据*/
	mpu_offset_call();
	
	return 0;
}


/**[6.2]	
	* brief  			初始化四元数
  * postscript  为什么这么初始化？？？为什么把hz注释了？ */
#define BOARD_DOWN (1)   
void init_quaternion(void)
{
	int16_t hx, hy;//hz;
	
	hx = imu.mx;
	hy = imu.my;
	//hz = imu.mz;
	
	#ifdef BOARD_DOWN
	/*(1)	如果hx负的，hy是负的*/
	if (hx < 0 && hy < 0) 
	{
		/*(1.1)	如果hx比hy大*/
		if (fabs(hx / hy) >= 1)
		{
			q0 = -0.005;
			q1 = -0.199;
			q2 = 0.979;
			q3 = -0.0089;
		}
		/*(1.2)	如果hx比hy小*/
		else
		{
			q0 = -0.008;
			q1 = -0.555;
			q2 = 0.83;
			q3 = -0.002;
		}
		
	}
	/*(2)	如果hx负的，hy是正的*/
	else if (hx < 0 && hy > 0)
	{
		/*(2.1)	如果hx比hy大*/
		if (fabs(hx / hy)>=1)   
		{
			q0 = 0.005;
			q1 = -0.199;
			q2 = -0.978;
			q3 = 0.012;
		}
		/*(2.2)	如果hx比hy小*/
		else
		{
			q0 = 0.005;
			q1 = -0.553;
			q2 = -0.83;
			q3 = -0.0023;
		}
		
	}
	/*(3)	如果hx正的，hy是正的*/
	else if (hx > 0 && hy > 0)
	{
		/*(3.1)	如果hx比hy大*/
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0012;
			q1 = -0.978;
			q2 = -0.199;
			q3 = -0.005;
		}
		/*(3.2)	如果hx比hy小*/
		else
		{
			q0 = 0.0023;
			q1 = -0.83;
			q2 = -0.553;
			q3 = 0.0023;
		}
		
	}
	/*(4)	如果hx正的，hy是负的*/
	else if (hx > 0 && hy < 0)
	{
		/*(4.1)	如果hx比hy大*/
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0025;
			q1 = 0.978;
			q2 = -0.199;
			q3 = 0.008;			
		}
		/*(4.2)	如果hx比hy小*/
		else
		{
			q0 = 0.0025;
			q1 = 0.83;
			q2 = -0.56;
			q3 = 0.0045;
		}		
	}
	#else
		if (hx < 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.195;
			q1 = -0.015;
			q2 = 0.0043;
			q3 = 0.979;
		}
		else
		{
			q0 = 0.555;
			q1 = -0.015;
			q2 = 0.006;
			q3 = 0.829;
		}
		
	}
	else if (hx < 0 && hy > 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.193;
			q1 = -0.009;
			q2 = -0.006;
			q3 = 0.979;
		}
		else
		{
			q0 = -0.552;
			q1 = -0.0048;
			q2 = -0.0115;
			q3 = 0.8313;
		}
		
	}
	else if (hx > 0 && hy > 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.9785;
			q1 = 0.008;
			q2 = -0.02;
			q3 = 0.195;
		}
		else
		{
			q0 = -0.9828;
			q1 = 0.002;
			q2 = -0.0167;
			q3 = 0.5557;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.979;
			q1 = 0.0116;
			q2 = -0.0167;
			q3 = -0.195;			
		}
		else
		{
			q0 = -0.83;
			q1 = 0.014;
			q2 = -0.012;
			q3 = -0.556;
		}		
	}
	#endif
}


/***********************************[7] 数据更新*********************************************/
/**[7.1]	
	* brief  			更新imu AHRS
  * usage  			main() 
	*	postscript	Mahony算法姿态解算，比C板那个貌似简洁不少*/
#define IST8310
void imu_ahrs_update(void) 
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez, halfT;
	float tempq0,tempq1,tempq2,tempq3;

	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;   
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;   
	
	gx = imu.wx;
	gy = imu.wy;
	gz = imu.wz;
	ax = imu.ax;
	ay = imu.ay;
	az = imu.az;
	mx = imu.mx;
	my = imu.my;
	mz = imu.mz;

	/*(1)	获取时间刻度并计算时间差的一半*/
	now_update  = HAL_GetTick(); //ms
	halfT       = ((float)(now_update - last_update) / 2000.0f);
	last_update = now_update;
	
	/* (2)	归一化加速度*/
	norm = inv_sqrt(ax*ax + ay*ay + az*az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	
	/* (3)	归一化磁力计数据（如果用到磁力计的话）*/
	#ifdef IST8310
		norm = inv_sqrt(mx*mx + my*my + mz*mz);          
		mx = mx * norm;
		my = my * norm;
		mz = mz * norm; 
	#else
		mx = 0;
		my = 0;
		mz = 0;		
	#endif
	/* (4)	计算磁通量的参考方向 */
	hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
	hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
	hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz; 
	
	/* (5)	重力和通量的估计方向 (v and w) */
	vx = 2.0f*(q1q3 - q0q2);
	vy = 2.0f*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
	wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
	wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
	
	/* (6)	叉乘计算误差*/
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);//误差是场的参考方向和传感器测量的方向之间的叉积之和
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

	/* (7)	PI */
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;	
		ezInt = ezInt + ez * Ki * halfT;
		
		gx = gx + Kp*ex + exInt;
		gy = gy + Kp*ey + eyInt;
		gz = gz + Kp*ez + ezInt;
	}
	
	/* (8)	更新四元数（先把更新值放到临时变量） */
	tempq0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;//因为原始的q0还要参与后面q1,q2,q3的值的更新计算，所以都先放到临时变量tempq0里面
	tempq1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
	tempq2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
	tempq3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;  

	/* (9)	归一化四元数 */
	norm = inv_sqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
	q0 = tempq0 * norm;
	q1 = tempq1 * norm;
	q2 = tempq2 * norm;
	q3 = tempq3 * norm;
}

/**[7.2]	
	* brief  更新imu姿态
  * usage  main()*/
void imu_attitude_update(void)
{
	/* yaw    -pi----pi */
	imu.yaw = -atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1)* 57.3; 
	/* pitch  -pi/2----pi/2 */
	imu.pit = -asin(-2*q1*q3 + 2*q0*q2)* 57.3;   
	/* roll   -pi----pi  */	
	imu.rol =  atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)* 57.3;
}

/**[7.3]					
  * brief					快速平方根取倒数
	* param[in]			x:被平方根取倒数的数
	* retval				平方根取倒数后的结果
  * usage  				call in     imu_ahrs_update() function*/
float inv_sqrt(float x) 
{
	float halfx = 0.5f * x;
	float y     = x;
	long  i     = *(long*)&y;
	
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	
	return y;
}
