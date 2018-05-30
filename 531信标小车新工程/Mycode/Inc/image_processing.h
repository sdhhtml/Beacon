#ifndef _IMAGE_PROCESSING_H_
#define _IMAGE_PROCESSING_H_

#define CON_NUM 5

typedef struct
{
    int start;
    int end;
    int num;
} Nect;

typedef struct 
{
    int row_st[60];
    int row_end[60];
    int row_mid[60];
    Nect nnect[CON_NUM];
} Record;



void Image_Processing(uint8 *img ,int row ,int col);

#endif