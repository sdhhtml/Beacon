#include "common.h"
#include "Image_processing.h"

#define WHITE 255
#define BLACK 0


void Image_Processing(uint8 *img ,int row ,int col)
{
    int i,j;
    Record loc;
    i = j = 0;
    
    for (i = 0;i < row;i++)
    {
        loc.row_st[i] = 0;
        loc.row_mid[i] = 0;
        loc.row_end[i] = 0;
    }
    
    for (i = 0; i < CON_NUM; i++)
    {
        loc.nnect[i].num = 0;
        loc.nnect[i].start = 0;
        loc.nnect[i].end = 0;
    }

    for (i = 0;i < row;i++)
    {
        const uint8 *rowdata = &img[i * col];
        
        //if (rowdata[0] == WHITE)
        //{
          //  loc.row_st[i] = 0;
        //}

        for (j = 1;j < col; j++)
        {
            if (rowdata[j - 1] == BLACK && rowdata[j] == WHITE)
            {
                loc.row_st[i] = j;
            }
            else if (rowdata[j - 1] == WHITE && rowdata[j] == BLACK)
            {
                loc.row_end[i] = j;
            }
        }
        loc.row_mid[i] = (loc.row_st[i] + loc.row_end[i])/2;
    }//get all 

    int count = 0;
    char FLAG_CONNECT_START = 0;
    char FLAG_CONNECT_END = 0;
    int connect_num = 0;
    int midx[60] = {0};//每一锟斤拷锟叫帮拷色锟脚碉拷锟斤拷值 锟斤拷锟斤拷锟斤拷一锟斤拷锟叫帮拷色锟斤拷锟斤拷 锟斤拷锟铰硷拷锟斤拷锟斤拷锟斤拷锟揭伙拷锟� 锟斤拷锟斤拷为0 
    int midy[60] = {0};//每一锟叫帮拷色锟脚碉拷锟斤拷锟斤拷值 锟斤拷锟斤拷色锟斤拷锟截碉拷锟斤拷位y锟斤拷锟斤拷 锟斤拷锟饺拷锟斤拷锟轿�0
    
    for (i = 1;i < row;i++)//锟劫次憋拷锟斤拷图锟斤拷 锟矫碉拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
    {
        if (loc.row_mid[i - 1] != 0 && loc.row_mid[i] != 0 && connect_num <= CON_NUM)
        {
            FLAG_CONNECT_START = 1;
            count++;
            midx[i - 1] = i - 1;
            midy[i - 1] = loc.row_mid[i - 1];
        }
        else if (FLAG_CONNECT_START == 1)
        {
            FLAG_CONNECT_END = 1;
            loc.nnect[connect_num].num = count;
            loc.nnect[connect_num].start = i - count;
            loc.nnect[connect_num].end = i;
        }
        
        if (FLAG_CONNECT_START == 1 && FLAG_CONNECT_END == 1)
        {
            connect_num++;
            count = 0;
            FLAG_CONNECT_START = FLAG_CONNECT_END = 0;
        }
        
        if (connect_num > CON_NUM)
            break;
    }
    
    int max_x = 0;
    int min_x = 0;
    int max_count = 0;
    int x, y;
    x = y = 0;
    int max_y = 0;
    
    for (i = 0; i < CON_NUM; i++)
    {
         if (loc.nnect[i].num > max_count)
         {
             max_count = loc.nnect[i].num;
             min_x = loc.nnect[i].start;
             max_x = loc.nnect[i].end;
         }
    }
    
    for (i = 1; i < row; i++)
    {   
        /*if (midx[i - 1] == 0 && midx[i] != 0)
        {
            min_x = i;
        }
        if (midx[i - 1] != 0 && midx[i] == 0)
        {
            max_x = i;
        }*/
        
        if (max_y < midy[i])
        {
            max_y = midy[i];
        }
    }
    
    x = (int)(min_x + max_x) / 2;
    y = max_y;
    
    for (i = 0; i < row; i++)
    {
        //printf("%d\t%d\t%d\t%d\n",i,midy[i],loc.row_st[i],loc.row_end[i]);
    }
    
    //printf("%d\t%d\n\n\n", x, y);

    img[col * x + y] = BLACK;
    
}