#include "include.h"
#include "common.h"
volatile int16 MPU6050_ACCEL_XOUT_DATA=0;
volatile int16 MPU6050_ACCEL_YOUT_DATA=0;
volatile int16 MPU6050_ACCEL_ZOUT_DATA=0;
volatile int16 MPU6050_TEMP_OUT_DATA=0;
volatile int16 MPU6050_GYRO_XOUT_DATA=0;
volatile int16 MPU6050_GYRO_YOUT_DATA=0;
volatile int16 MPU6050_GYRO_ZOUT_DATA=0;
volatile int var[7];    //所发送波形的数量
volatile int16 ax=0,ay=0,az=0,angular_x=0,angular_y=0,angular_z=0,temp=0;  //陀螺仪自用
void MPU6050_Init(I2Cn_e i2cn)
{
  i2c_init(i2cn,400*1000);
  i2c_write_reg(i2cn, MPU6050_ADDRESS,MPU6050_PWR_MGMT_1, 0x00);
  i2c_write_reg(i2cn, MPU6050_ADDRESS,MPU6050_PWR_MGMT_2, 0x00);
  i2c_write_reg(i2cn, MPU6050_ADDRESS,MPU6050_CONFIG, 0x06);//低通滤波器带宽5Hz
  i2c_write_reg(i2cn, MPU6050_ADDRESS,MPU6050_GYRO_CONFIG, 0x08);//不自检   ±500 °/s
  i2c_write_reg(i2cn, MPU6050_ADDRESS,MPU6050_ACCEL_CONFIG, 0x08);//不自检  ±4g
} 

//----------------------------------------------------------------------
/*!
 *  @brief      读取数据
 *  @param      I2Cn_e        I2C模块(I2C0、I2C1)
 *  @param      Addr         从机寄存器地址
 *  @param      Data        数据
 *  @since      v1.0
 *  Sample usage:       MPU6050_GetDoubleData(MPU6050_I2C_Moudle_1 , MPU6050_GYRO_YOUT)；
 */
int16 MPU6050_GetDoubleData(I2Cn_e i2cn ,uint8 Addr)
{
  uint16 data=0x0000;
  data=i2c_read_reg(i2cn,MPU6050_ADDRESS, Addr);
  data=(uint16)((data<<8)&0xff00);
  data+=i2c_read_reg(i2cn,MPU6050_ADDRESS, Addr+1);
  
  return (int16)data;//合成数据，为有符号整形数
  
}
void mpu6050_data()
{
  

  ax=MPU6050_GetDoubleData(MPU6050_I2C_Moudle_2,MPU6050_ACCEL_XOUT);//读X轴加速度
  var[0] = ax;
  printf("X轴加速度为%d\n",ax);
  DELAY_MS(100);
                
  ay=MPU6050_GetDoubleData(MPU6050_I2C_Moudle_2,MPU6050_ACCEL_ZOUT);//读Y轴加速度
  var[1] = ay;
  printf("Y轴加速度为%d\n",ay);
  DELAY_MS(100);
                
  az=MPU6050_GetDoubleData(MPU6050_I2C_Moudle_2,MPU6050_ACCEL_ZOUT);//读Z轴加速度
  var[2] = az;
  printf("Z轴加速度为%d\n",az);
  DELAY_MS(100);
 angular_x=MPU6050_GetDoubleData(MPU6050_I2C_Moudle_2,MPU6050_GYRO_XOUT);//读X轴角速度
  var[3] = angular_x;
  printf("X轴角速度为%d\n",angular_x);
  DELAY_MS(100);
                
 angular_y=MPU6050_GetDoubleData(MPU6050_I2C_Moudle_2,MPU6050_GYRO_YOUT);//读Y轴角速度
  var[4] = angular_y;
  printf("Y轴角速度为%d\n",angular_y);
  DELAY_MS(100);
                
angular_z=MPU6050_GetDoubleData(MPU6050_I2C_Moudle_2,MPU6050_GYRO_ZOUT);//读Z轴角速度
  var[5] = angular_z;
  printf("Z轴角速度为%d\n",angular_z);
  DELAY_MS(100);
                
 temp=MPU6050_GetDoubleData(MPU6050_I2C_Moudle_2,MPU6050_TEMP_OUT);//读温度
 var[6] = temp;
  printf("温度为%d\n",temp);
  DELAY_MS(100);
}
void systemInit()
{  

  MPU6050_Init(MPU6050_I2C_Moudle_2);
  DELAY_MS(3000);
}