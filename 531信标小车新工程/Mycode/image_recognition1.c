#include "common.h"
#include "include.h"
#include "math.h"

extern uint8 imgbuff[CAMERA_SIZE]; 
uint8 img[CAMERA_W*CAMERA_H];                           //由于鹰眼摄像头是一字节8个像素，因而需要解压为 1字节1个像素，方便处理
                    //假如需要二维数组，只需要改成 uint8 img[CAMERA_H][CAMERA_W];
                    //imgbuff是采集的缓冲区，img是解压后的缓冲区。imgbuff用于采集图像，img用于图像处理.
uint8 img_buf[60][80]; 
uint32 Lookingforabeacon()
{
      int i=0,j=0,k=0;
      int mid_1=0,mid_2=0;
      camera_get_img();                                   //摄像头获取图像
      img_extract((uint8 *)img_buf,(uint8 *)imgbuff, CAMERA_W*CAMERA_H/8);        //解压为灰度图像，方便发送到上位机显
      for(i=5;i<40;i++)
        { 
          for(j=0;j<79;j++)
            {
              if(img_buf[i][j]!=img_buf[i][j+1])
                {
                  // printf("找到跳变点：%d\r\n",j);
                  k++;
                  mid_1+=j;
                  }             
            }
        }
      mid_2=mid_1/k;
      //printf("中线的横坐标为：%d\r\n",mid_2);
      return(mid_2);

}