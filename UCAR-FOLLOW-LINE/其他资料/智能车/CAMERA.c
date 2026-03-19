#include "CAMERA.h"

uint8* Image_Use[LCDH][LCDW];      //�����洢ѹ��֮��Ҷ�ͼ��Ķ�ά����
uint8 Pixle[LCDH][LCDW];          //ͼ����ʱ���������Ķ�ֵ��ͼ������
//uint8 All_Sobel_Image[LCDH][LCDW];
uint8 Threshold;                                //ͨ����򷨼�������Ķ�ֵ����ֵ
uint16 threshold1,threshold2,threshold3,block_yuzhi=60;
uint16 yuzhi1,yuzhi2,yuzhi3;
uint16 Ramp_cancel,circle_stop,block_num,duan_num;
int ImageScanInterval=5;                        //ɨ�ߵķ�Χ
static uint8* PicTemp;                          //һ�����浥��ͼ���ָ�����
static int IntervalLow = 0, IntervalHigh = 0;   //ɨ������������ޱ���
static int Ysite = 0, Xsite = 0;                //Ysite����ͼ����У�Xsite����ͼ����С�
static int BottomBorderRight = 79,              //59�е��ұ߽�
BottomBorderLeft = 0,                           //59�е���߽�
BottomCenter = 0;                               //59�е��е�
uint8 ExtenLFlag = 0;                           //������Ƿ���Ҫ���ߵı�־����
uint8 ExtenRFlag = 0;                           //�ұ����Ƿ���Ҫ���ߵı�־����
uint8 Ring_Help_Flag = 0;                       //����������־
int Right_RingsFlag_Point1_Ysite, Right_RingsFlag_Point2_Ysite; //��Բ���жϵ�����������
int Left_RingsFlag_Point1_Ysite, Left_RingsFlag_Point2_Ysite;   //��Բ���жϵ�����������
int Point_Xsite,Point_Ysite;                   //�յ��������
int Repair_Point_Xsite,Repair_Point_Ysite;     //���ߵ��������
int Point_Ysite1;                               //�жϴ�СԲ��ʱ�õ�������
int Black;                                      //�жϴ�СԲ��ʱ�õĺڵ�����
int Less_Big_Small_Num;                         //�жϴ�СԲ��ʱ�õĶ���
int left_difference_num;                        //ʮ����������׼����39����ĺͣ�40-20�У�
int right_difference_num;                       //ʮ���ұ������׼����39����ĺͣ�40-20�У�
uint8 Garage_Location_Flag = 0;                 //�жϿ�Ĵ���
float Big_Small_Help_Gradient;               //��СԲ���ĸ����ж�б��
static int ytemp = 0;                           //����е���ʱ����
static int TFSite = 0, left_FTSite = 0,right_FTSite = 0;              //���߼���б�ʵ�ʱ����Ҫ�õĴ���еı�����
static float DetR = 0, DetL = 0;                //��Ų���б�ʵı���
ImageDealDatatypedef ImageDeal[60];             //��¼������Ϣ�Ľṹ������
ImageDealDatatypedef ImageDeal1[80];             //��¼������Ϣ�Ľṹ������
ImageStatustypedef ImageStatus;                 //ͼ�����ĵ�ȫ�ֱ���
ImageFlagtypedef ImageFlag;
uint64 Gray_Value=0;
float Mh = MT9V03X_H;
float Lh = LCDH;
float Mw = MT9V03X_W;
float Lw = LCDW;

float variance, variance_acc;                   //�ж�ֱ������ķ���
int variance_limit_long,variance_limit_short;   //��ֱ������ֱ���ķ����޶�ֵ
#define Middle_Center 39                        //��Ļ����

//int j=5;
uint8 Half_Road_Wide[60] =                      //ֱ���������
{  4, 5, 5, 6, 6, 6, 7, 7, 8, 8,
   9, 9,10,10,10,11,12,12,13,13,
  13,14,14,15,15,16,16,17,17,17,
  18,18,19,19,20,20,20,21,21,22,
  23,23,23,24,24,25,25,25,26,26,
  27,28,28,28,29,30,31,31,31,32,
};
uint8 Half_Bend_Wide[60] =                      //����������
{   33,33,33,33,33,33,33,33,33,33,
    33,33,32,32,30,30,29,29,28,27,
    28,27,27,26,26,25,25,24,24,23,
    22,21,21,22,22,22,23,24,24,24,
    25,25,25,26,26,26,27,27,28,28,
    28,29,29,30,30,31,31,32,32,33,
};
/*uint8 Half_Bend_Wide[60] =                      //����������+1
{   33,33,33,33,33,33,33,33,33,33,
    33,33,32,32,30,30,29,29,28,27,
    29,28,28,27,27,26,26,25,25,24,
    23,22,22,23,23,23,24,25,25,25,
    26,26,26,27,27,27,28,28,29,29,
    29,30,30,31,31,32,32,33,33,34,
};*/
/*uint8 Half_Bend_Wide[60] =                      //����������-1
{   33,33,33,33,33,33,33,33,33,33,
    33,33,32,32,30,30,29,29,28,27,
    27,26,26,25,25,24,24,23,23,22,
    21,20,20,21,21,21,22,23,23,23,
    25,25,25,26,26,26,27,27,28,28,
    28,29,29,30,30,31,31,32,32,33,
};*/
/*****************ֱ���ж�******************/
float Straight_Judge(uint8 dir, uint8 start, uint8 end)     //���ؽ��С��1��Ϊֱ��
{
    int i;
    float S = 0, Sum = 0, Err = 0, k = 0;
    switch (dir)
    {
    case 1:k = (float)(ImageDeal[start].LeftBorder - ImageDeal[end].LeftBorder) / (start - end);
        for (i = 0; i < end - start; i++)
        {
            Err = (ImageDeal[start].LeftBorder + k * i - ImageDeal[i + start].LeftBorder) * (ImageDeal[start].LeftBorder + k * i - ImageDeal[i + start].LeftBorder);
            Sum += Err;
        }
        S = Sum / (end - start);
        break;
    case 2:k = (float)(ImageDeal[start].RightBorder - ImageDeal[end].RightBorder) / (start - end);
        for (i = 0; i < end - start; i++)
        {
            Err = (ImageDeal[start].RightBorder + k * i - ImageDeal[i + start].RightBorder) * (ImageDeal[start].RightBorder + k * i - ImageDeal[i + start].RightBorder);
            Sum += Err;
        }
        S = Sum / (end - start);
        break;
    }
    return S;
}

void Straight_long_judge(void)     //���ؽ��С��1��Ϊֱ��
{
    if( ImageFlag.Bend_Road ||  ImageFlag.Zebra_Flag || ImageFlag.Out_Road == 1 || ImageFlag.RoadBlock_Flag == 1
            || ImageFlag.image_element_rings == 1  || ImageFlag.image_element_rings == 2 ) return;
    if((Straight_Judge(1,10,50) < 1) && (Straight_Judge(2,10,50) < 1) && ImageStatus.OFFLine < 3 && ImageStatus.Miss_Left_lines < 2 && ImageStatus.Miss_Right_lines < 2)
    {
        ImageFlag.straight_long = 1;
        //Stop=1;
//        Statu = Straight_long;
        //gpio_set_level(B0, 1);
    }
}

void Straight_long_handle(void)     //���ؽ��С��1��Ϊֱ��
{

    if(ImageFlag.straight_long)
    {
        if((Straight_Judge(1,10,50) > 1) || (Straight_Judge(2,10,50) > 1) || (ImageStatus.OFFLine >= 3)||ImageStatus.Miss_Left_lines >= 2||ImageStatus.Miss_Right_lines>=2)
        {
            ImageFlag.straight_long = 0;
            //gpio_set_level(B0, 0);
        }
    }
}
//���ڱ������ֱ�����
float sum1=0,sum2=0;
void Straight_xie_judge(void)
{
    float S = 0, Sum = 0, Err = 0 , midd_k=0 ;
    int i;
    if (ImageFlag.Zebra_Flag != 0 || ImageFlag.image_element_rings != 0 || ImageFlag.Ramp == 1  )
        return;

    ImageFlag.straight_xie = 0;

    midd_k = (float)(ImageDeal[55].Center - ImageDeal[ImageStatus.OFFLine + 1].Center) / (55 - ImageStatus.OFFLine - 1);
    for (i = 0 ; i < 55 - ImageStatus.OFFLine - 1; i++)
    {
        Err = (ImageDeal[ImageStatus.OFFLine + 1].Center + midd_k * i - ImageDeal[i + ImageStatus.OFFLine + 1].Center) * (ImageDeal[ImageStatus.OFFLine + 1].Center + midd_k * i - ImageDeal[i + ImageStatus.OFFLine + 1].Center);
        Sum += Err;
    }
    S = Sum / (55 - ImageStatus.OFFLine - 1);
//    ips200_show_float(160,200,midd_k,3,2);
//    ips200_show_float(160,240,S,3,2);
    if (S < 1 && ImageStatus.OFFLine < 10 && (ImageStatus.Miss_Left_lines > 30 || ImageStatus.Miss_Right_lines > 30))
    {
        ImageFlag.straight_xie = 1;
//        Statu = Straight_xie;
        //gpio_set_level(B0, 1);
    }

}


/*****************��������ж�******************/
uint32 break_road(uint8 line_start)//�������
{
    short i,j;
    int bai=0;
    for(i=58;i>line_start;i--)
    {
        for(j=5;j<74;j++)
        {
            if(Pixle[i][j]==1)
            {
                bai++;
            }
        }
    }
    return bai;
}
uint32 white_point(uint8 line_end,uint8 line_start) //�׵�����
{
    short i,j;
    int bai=0;
    for(i=line_end;i>line_start;i--)
    {
        for(j=29;j<49;j++)
        {
            if(Pixle[i][j]==1)
            {
                bai++;
            }
        }
    }
    return bai;
}
uint32 black_point(uint8 line_end,uint8 line_start) //�ڵ�����
{
    short i,j;
    int hei=0;
    for(i=line_end;i>line_start;i--)
    {
        for(j=29;j<49;j++)
        {
            if(Pixle[i][j]==0)
            {
                hei++;
            }
        }
    }
    return hei;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Image_CompressInit
//  @brief          ԭʼ�Ҷ�ͼ��ѹ������ ѹ����ʼ��
//  @brief          ���þ��ǽ�ԭʼ�ߴ�ĻҶ�ͼ��ѹ����������Ҫ�Ĵ�С���������ǰ�ԭʼ80��170�еĻҶ�ͼ��ѹ����60��80�еĻҶ�ͼ��
//  @brief          ΪʲôҪѹ������Ϊ�Ҳ���Ҫ��ô�����Ϣ��60*80ͼ����չʾ����Ϣԭ�����Ѿ��㹻��ɱ��������ˣ���Ȼ����Ը����Լ��������޸ġ�
//  @parameter      void
//  @return         void
//  @time           2022��2��18��
//  @Author         
//  Sample usage:   Image_CompressInit();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Image_CompressInit(void)
{
  int i, j, row, line;
  const float div_h = Mh / Lh, div_w = Mw / Lw;         //����ԭʼ��ͼ��ߴ��������Ҫ��ͼ��ߴ�ȷ����ѹ��������
  for (i = 0; i < LCDH; i++)                            //����ͼ���ÿһ�У��ӵ����е���59�С�
  {
    row = i * div_h + 0.5;
    for (j = 0; j < LCDW; j++)                          //����ͼ���ÿһ�У��ӵ����е���79�С�
    {
      line = j * div_w + 0.5;
      Image_Use[i][j] = &mt9v03x_image[row][line];       //mt9v03x_image����������ԭʼ�Ҷ�ͼ��Image_Use����洢������֮��Ҫ��ȥ������ͼ�񣬵���Ȼ�ǻҶ�ͼ��Ŷ��ֻ��ѹ����һ�¶��ѡ�
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           get_Threshold  //ָ��
//  @brief          �Ż�֮��ĵĴ�򷨡���򷨾���һ���ܹ����һ��ͼ����ѵ��Ǹ��ָ���ֵ��һ���㷨��
//  @brief          ����������ǿ������ʵ�ڲ��������ֱ�������ã�ʲô�����������޸ģ�ֻҪû�й���Ӱ�죬��ô������������ֵ��һ�����Եõ�һ��Ч���������Ķ�ֵ��ͼ��
//  @parameter      image  ԭʼ�ĻҶ�ͼ������
//  @parameter      clo    ͼ��Ŀ���ͼ����У�
//  @parameter      row    ͼ��ĸߣ�ͼ����У�
//  @return         uint8
//  @time           2022��2��17��
//  @Author         
//  Sample usage:   Threshold = Threshold_deal(Image_Use[0], 80, 60); �Ѵ��60��80�еĶ�άͼ������Image_Use��������������ͼ�����ֵ�����������ֵ����Threshold��
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
uint8 get_Threshold(uint8* image[][LCDW],uint16 col, uint16 row)
{
  #define GrayScale 256
  uint16 width = col;
  uint16 height = row;
  int pixelCount[GrayScale];
  float pixelPro[GrayScale];
  int i, j, pixelSum = width * height;
  uint8 threshold = 0;
  for (i = 0; i < GrayScale; i++)                    //�Ȱ�pixelCount��pixelPro��������Ԫ��ȫ����ֵΪ0
  {
    pixelCount[i] = 0;
    pixelPro[i] = 0;
  }

  uint32 gray_sum = 0;
  /**************************************ͳ��ÿ���Ҷ�ֵ(0-255)������ͼ���г��ֵĴ���**************************************/
  for (i = 0; i < height; i += 1)                   //����ͼ���ÿһ�У��ӵ����е���59�С�
  {
    for (j = 0; j < width; j += 1)                  //����ͼ���ÿһ�У��ӵ����е���79�С�
    {
      pixelCount[*image[i][j]]++;       //����ǰ�����ص������ֵ���Ҷ�ֵ����Ϊ����������±ꡣ
      gray_sum += *image[i][j];         //���������Ҷ�ͼ��ĻҶ�ֵ�ܺ͡�
    }
  }
  /**************************************ͳ��ÿ���Ҷ�ֵ(0-255)������ͼ���г��ֵĴ���**************************************/



  /**************************************����ÿ������ֵ���Ҷ�ֵ���������Ҷ�ͼ������ռ�ı���*************************************************/
  for (i = 0; i < GrayScale; i++)
  {
      pixelPro[i] = (float)pixelCount[i] / pixelSum;
  }
  /**************************************����ÿ������ֵ���Ҷ�ֵ���������Ҷ�ͼ������ռ�ı���**************************************************/



  /**************************************��ʼ��������ͼ��ĻҶ�ֵ��0-255������һ��Ҳ�Ǵ�����������һ��***************************/
  /*******************Ϊʲô˵�������⣿��Ϊ��Ҳ�ǲ����⣡�������������һ����ѧ���⣬���������Ϊ��ѧ��ʽ��***************************/
  float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
  w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
  for (j = 0; j < GrayScale; j++)
  {
    w0 += pixelPro[j];                          //�����������ÿ���Ҷ�ֵ�����ص���ռ�ı���֮�ͣ����������ֵı�����
    u0tmp += j * pixelPro[j];

    w1 = 1 - w0;
    u1tmp = gray_sum / pixelSum - u0tmp;

    u0 = u0tmp / w0;                            //����ƽ���Ҷ�
    u1 = u1tmp / w1;                            //ǰ��ƽ���Ҷ�
    u = u0tmp + u1tmp;                          //ȫ��ƽ���Ҷ�
    deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
    if (deltaTmp > deltaMax)
    {
      deltaMax = deltaTmp;
      threshold = j;
    }
    if (deltaTmp < deltaMax)
    {
      break;
    }
  }
  /**************************************��ʼ��������ͼ��ĻҶ�ֵ��0-255������һ��Ҳ�Ǵ�����������һ��***************************/
  /*******************Ϊʲô˵�������⣿��Ϊ��Ҳ�ǲ����⣡�������������һ����ѧ���⣬���������Ϊ��ѧ��ʽ��***************************/

  return threshold;                             //��������ô���д������������ֵ��return��ȥ��
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_BinaryImage
//  @brief          �Ҷ�ͼ���ֵ������
//  @brief          ����˼·���ǣ��ȵ���Get_Threshold���������õ���ֵ��Ȼ�����ԭʼ�Ҷ�ͼ���ÿһ�����ص㣬��ÿһ�����ص�ĻҶ�ֵ������ֵ�ƽϡ�
//  @brief          ������ֵ����Ͱ����Ǹ����ص��ֵ��ֵΪ1����Ϊ�׵㣩������͸�ֵΪ0����Ϊ�ڵ㣩����Ȼ����԰������ֵ��������ֻҪ���Լ����1��0˭������˭�����׾��С�
//  @brief          ������ǰ���ᵽ��60*80�������Ǿ�Ӧ��������ʲô��˼�˰ɣ��������ص��һ����80�����ص㣬һ��60�У�Ҳ����ѹ�����ÿһ��ͼ����4800�����ص㡣
//  @parameter      void
//  @return         void
//  @time           2022��2��17��
//  @Author         
//  Sample usage:   Get_BinaryImage();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Get_BinaryImage(void)
{
    if(ImageFlag.Out_Road == 1)
    {
        if(duan_num==0)
        {
            Threshold  = block_yuzhi ;
        }
        else if(duan_num==1)
        {
            Threshold  = threshold1 ;
        }
        else if(duan_num==2)
        {
            Threshold  = threshold2 ;
        }
        else if(duan_num==3)
        {
            Threshold  = threshold3 ;
        }
    }
    else if(ImageFlag.RoadBlock_Flag == 1)
    {
        if(block_num==1)
        {
            Threshold  = yuzhi1 ;
        }
        else if(block_num==2)
        {
            Threshold  = yuzhi2 ;
        }
        else if(block_num==3)
        {
            Threshold  = yuzhi3 ;
        }
    }
    else {
        Threshold = get_Threshold(Image_Use, LCDW, LCDH);      //������һ���������ã�ͨ���ú������Լ����һ��Ч���ܲ����Ķ�ֵ����ֵ��
    }
  uint8 i, j = 0;
  for (i = 0; i < LCDH; i++)                                //������ά�����ÿһ��
  {
    for (j = 0; j < LCDW; j++)                              //������ά�����ÿһ��
    {
      if (*Image_Use[i][j] > Threshold)                      //��������ĻҶ�ֵ������ֵThreshold
          Pixle[i][j] = 1;                                  //��ô������ص�ͼ�Ϊ�׵�
      else                                                  //��������ĻҶ�ֵС����ֵThreshold
          Pixle[i][j] = 0;                                  //��ô������ص�ͼ�Ϊ�ڵ�
    }
  }
}


//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_Border_And_SideType
//  @brief          �õ����ߺͱ��ߵ����ͣ�������������߷�Ϊ���������ͣ�T���͡�W���ͺ�H���͡��ֱ�����������ߡ��ޱ߱��ߺʹ�������ߡ�
//  @brief          ������һ����Ҫ�뿴���ҵ��߼���ǰ����Ҫ�㶮T��W��H�������͵ı��ߵ�����ʲô���ӵġ�
//  @parameter      p        ָ�򴫽��������һ��ָ�������
//  @parameter      type     ֻ����L������R���ֱ����ɨ����ߺ�ɨ�ұ��ߡ�
//  @parameter      L        ɨ����������� ��Ҳ���Ǵ���һ�п�ʼɨ��
//  @parameter      H        ɨ����������� ��Ҳ����һֱɨ����һ�С�
//  @parameter      Q        ��һ���ṹ��ָ��������Լ�����ȥ��������ṹ������ĳ�Ա��
//  @time           2022��2��20��
//  @Author        
//  Sample usage:   Get_SideType_And_Border(PicTemp, 'R', IntervalLow, IntervalHigh,&JumpPoint[1]);
//  Sample usage:   ��PicTemp(PicTemp�Ǹ�ָ�룬ָ��һ������)��IntervalLow�п�ʼɨ��ɨ��IntervalHigh�У�Ȼ��ѵõ��ı������ڵ��кͱ������ͼ�¼��JumpPoint�ṹ���С�
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Get_Border_And_SideType(uint8* p,uint8 type,int L,int H,JumpPointtypedef* Q)
{
  int i = 0;
  if (type == 'L')                              //���Type��L(Left),��ɨ������ߡ�
  {
    for (i = H; i >= L; i--)                    //��������ɨ
    {
      if (*(p + i) == 1 && *(p + i - 1) != 1)   //����кڰ�����    1�ǰ� 0�Ǻ�
      {
        Q->point = i;                           //�ǾͰ�����м�¼������Ϊ�����
        Q->type = 'T';                          //���Ұ���һ�е������������䣬�������ͼ�ΪT��������������
        break;                                  //�ҵ��˾�����ѭ��������
      }
      else if (i == (L + 1))                    //Ҫ��ɨ�����û�ҵ�
      {
        if (*(p + (L + H) / 2) != 0)            //����ɨ��������м��ǰ����ص�
        {
          Q->point = (L + H) / 2;               //��ô����Ϊ��һ�е�������Ǵ�����ɨ��������е㡣
          Q->type = 'W';                        //���Ұ���һ�е����Ƿ��������䣬�������ͼ�ΪW�����ޱ��С�
          break;                                //����ѭ��������
        }
        else                                    //Ҫ��ɨ�����û�ҵ�������ɨ��������м��Ǻ����ص�
        {
          Q->point = H;                         //��ô����Ϊ��һ�е�������Ǵ�����ɨ��������������ޡ�
          Q->type = 'H';                        //����Ҳ����һ�е����Ƿ��������䣬�����������ͼ�ΪH�����������С�
          break;                                //����ѭ��������
        }
      }
    }
  }
  else if (type == 'R')                         //���Type��R(Right),��ɨ���ұ��ߡ�
  {
    for (i = L; i <= H; i++)                    //��������ɨ
    {
      if (*(p + i) == 1 && *(p + i + 1) != 1)   //����кڰ�����    1�ǰ� 0�Ǻ�
      {
        Q->point = i;                           //�ǾͰ�����м�¼������Ϊ�ұ���
        Q->type = 'T';                          //���Ұ���һ�е������������䣬�������ͼ�ΪT��������������
        break;                                  //�ҵ��˾�����ѭ��������
      }
      else if (i == (H - 1))                    //Ҫ��ɨ�����û�ҵ�
      {
        if (*(p + (L + H) / 2) != 0)            //����ɨ��������м��ǰ����ص�
        {
          Q->point = (L + H) / 2;               //��ô����Ϊ��һ�е��ұ����Ǵ�����ɨ��������е㡣
          Q->type = 'W';                        //���Ұ���һ�е����Ƿ��������䣬�������ͼ�ΪW�����ޱ��С�
          break;
        }
        else                                    //Ҫ��ɨ�����û�ҵ�������ɨ��������м��Ǻ����ص�
        {
          Q->point = L;                         //��ô����Ϊ��һ�е��ұ����Ǵ�����ɨ��������������ޡ�
          Q->type = 'H';                        //����Ҳ����һ�е����Ƿ��������䣬�����������ͼ�ΪH�����������С�
          break;                                //����ѭ��������
        }
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_BaseLine
//  @brief          �ñ����ķ����õ�ͼ����������У�59-55�У��ı��ߺ�������Ϣ�������б��ߺ�������Ϣ��׼ȷ�ȷǳ�����Ҫ��ֱ��Ӱ�쵽����ͼ��Ĵ��������
//  @brief          Get_BaseLine����������Get_AllLine�����Ļ�����ǰ�ᣬGet_AllLine��������Get_BaseLineΪ�����ġ�������Ӧ��Ҳ�ܿ����԰ɣ�һ���еõ������ߣ�һ���еõ������ߡ�
//  @brief          Get_BaseLine������Get_AllLine��������һ��Ҳ����������Ż�֮����ѱ����㷨��
//  @parameter      void
//  @time           2022��2��21��
//  @Author        
//  Sample usage:   Get_BaseLine();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Get_BaseLine(void)
{
    /**************************************��������ͼ������У�59�У����ұ��ߴӶ�ȷ�����ߵĹ��� ********************************************************************/
    /****************************************************Begin*****************************************************************************/

    PicTemp = Pixle[59];                                                //��PicTemp���ָ�����ָ��ͼ�������Pixle[59]
    for (Xsite = ImageSensorMid; Xsite < 79; Xsite++)                   //����39�������У��������п�ʼһ��һ�е����ұ������ұ���
    {
      if (*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite + 1) == 0)       //������������������ڵ㣬˵û�ҵ��˱��ߡ�
      {
        BottomBorderRight = Xsite;                                      //����һ�м�¼������Ϊ��һ�е��ұ���
        break;                                                          //����ѭ��
      }
      else if (Xsite == 78)                                             //����ҵ��˵�58�ж���û���ֺڵ㣬˵����һ�еı��������⡣
      {
        BottomBorderRight = 79;                                         //����������Ĵ������ǣ�ֱ�Ӽ���ͼ�����ұߵ���һ�У���79�У�������һ�е��ұ��ߡ�
        break;                                                          //����ѭ��
      }
    }

    for (Xsite = ImageSensorMid; Xsite > 0; Xsite--)                    //����39�������У��������п�ʼһ��һ�е���������������
    {
      if (*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite - 1) == 0)       //������������������ڵ㣬˵û�ҵ��˱��ߡ�
      {
        BottomBorderLeft = Xsite;                                       //����һ�м�¼������Ϊ��һ�е������
        break;                                                          //����ѭ��
      }
      else if (Xsite == 1)                                              //����ҵ��˵�1�ж���û���ֺڵ㣬˵����һ�еı��������⡣
      {
        BottomBorderLeft = 0;                                           //����������Ĵ������ǣ�ֱ�Ӽ���ͼ������ߵ���һ�У���0�У�������һ�е�����ߡ�
        break;                                                          //����ѭ��
      }
    }

    BottomCenter =(BottomBorderLeft + BottomBorderRight) / 2;           //�������ұ߽�������59�е�����
    ImageDeal[59].LeftBorder = BottomBorderLeft;                        //�ѵ�59�е���߽�洢�����飬ע�⿴ImageDeal������ֵ��±꣬�ǲ������ö�Ӧ59��
    ImageDeal[59].RightBorder = BottomBorderRight;                      //�ѵ�59�е��ұ߽�洢�����飬ע�⿴ImageDeal������ֵ��±꣬�ǲ������ö�Ӧ59��
    ImageDeal[59].Center = BottomCenter;                                //�ѵ�59�е����ߴ洢�����飬    ע�⿴ImageDeal������ֵ��±꣬�ǲ������ö�Ӧ59��
    ImageDeal[59].Wide = BottomBorderRight - BottomBorderLeft;          //�ѵ�59�е��������ȴ洢���飬ע�⿴ImageDeal������ֵ��±꣬�ǲ������ö�Ӧ59��
    ImageDeal[59].IsLeftFind = 'T';                                     //��¼��59�е����������ΪT���������ҵ�����ߡ�
    ImageDeal[59].IsRightFind = 'T';                                    //��¼��59�е��ұ�������ΪT���������ҵ��ұ��ߡ�

    /****************************************************End*******************************************************************************/
    /**************************************��������ͼ������У�59�У����ұ��ߴӶ�ȷ�����ߵĹ��� ********************************************************************/



    /**************************************�ڵ�59�������Ѿ�ȷ���������ȷ��58-54���������ߵĹ��� ******************************************/
    /****************************************************Begin*****************************************************************************/
    /*
         * ���漸�еĵ����߹����ҾͲ���׸���ˣ������ҵ�ע�Ͱѵ�59�е����߹�������ã�
         * ��ô58��54�е����߾���ȫû���⣬��һģһ�����߼��͹��̡�
     */
    for (Ysite = 58; Ysite > 54; Ysite--)
    {
        PicTemp = Pixle[Ysite];
        for(Xsite = ImageDeal[Ysite + 1].Center; Xsite < 79;Xsite++)
        {
          if(*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite + 1) == 0)
          {
            ImageDeal[Ysite].RightBorder = Xsite;
            break;
          }
          else if (Xsite == 78)
          {
            ImageDeal[Ysite].RightBorder = 79;
            break;
          }
        }

        for (Xsite = ImageDeal[Ysite + 1].Center; Xsite > 0;Xsite--)
        {
          if (*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite - 1) == 0)
          {
            ImageDeal[Ysite].LeftBorder = Xsite;
            break;
          }
          else if (Xsite == 1)
          {
            ImageDeal[Ysite].LeftBorder = 0;
            break;
          }
        }

        ImageDeal[Ysite].IsLeftFind  = 'T';
        ImageDeal[Ysite].IsRightFind = 'T';
        ImageDeal[Ysite].Center =(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2;
        ImageDeal[Ysite].Wide   = ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
    }

    /****************************************************End*****************************************************************************/
    /**************************************�ڵ�59�������Ѿ�ȷ���������ȷ��58-54���������ߵĹ��� ****************************************/
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_AllLine
//  @brief          ��Get_BaseLine�Ļ����ϣ���Բ����������������һЩ����Ĵ����㷨�õ�ʣ���еı��ߺ�������Ϣ��
//  @brief          �������Ӧ����ĿǰΪֹ�Ҵ����������������һ�����ˣ�һ��Ҫ������ʱ�侲������ȥ��������ʱ����������Ҫ���Ǹ���ֵ���ڰ�ͼ��
//  @brief          �������һ�����Ŷ�ֵ���ڰ�ͼ��һ����˼���Ҵ�����߼��Ļ����ܶ�ط���������������ˣ���Ҫ�ⶢ���Ǹ�����һֱ������������û�õģ��мɣ�
//  @brief          �ද��˼������������������ǿ϶�Ҳ���Եġ�������̻�ܿ�������㶼���������֮����ĳ������Ѿ�������ֱ��������ˡ�
//  @parameter      void
//  @time           2023��2��21��
//  @Author         
//  Sample usage:   Get_AllLine();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Get_AllLine(void)
{
  uint8 L_Found_T  = 'F';    //ȷ���ޱ�б�ʵĻ�׼�б����Ƿ��ҵ��ı�־
  uint8 Get_L_line = 'F';    //�ҵ���һ֡ͼ��Ļ�׼��б�ʣ�Ϊʲô����Ҫ��ΪF����������Ĵ����֪���ˡ�
  uint8 R_Found_T  = 'F';    //ȷ���ޱ�б�ʵĻ�׼�б����Ƿ��ҵ��ı�־
  uint8 Get_R_line = 'F';    //�ҵ���һ֡ͼ��Ļ�׼��б�ʣ�Ϊʲô����Ҫ��ΪF����������Ĵ����֪���ˡ�
  float D_L = 0;             //������ӳ��ߵ�б��
  float D_R = 0;             //�ұ����ӳ��ߵ�б��
  int ytemp_W_L;             //��ס�״��󶪱���
  int ytemp_W_R;             //��ס�״��Ҷ�����
  ExtenRFlag = 0;            //��־λ��0
  ExtenLFlag = 0;            //��־λ��0
  ImageStatus.OFFLine=2;     //����ṹ���Ա��֮���������︳ֵ������Ϊ��ImageStatus�ṹ������ĳ�Ա̫���ˣ�������ʱ��ֻ�õ���OFFLine�������������õ��������ĸ�ֵ��
  ImageStatus.Miss_Right_lines = 0;
  ImageStatus.WhiteLine = 0;
  ImageStatus.Miss_Left_lines = 0;
  for (Ysite = 54 ; Ysite > ImageStatus.OFFLine; Ysite--)                            //ǰ5����Get_BaseLine()���Ѿ��������ˣ����ڴ�55�д������Լ��趨�Ĳ�������OFFLine��
  {                                                                                  //��Ϊ̫ǰ���ͼ��ɿ��Բ��㣬����OFFLine�����ú��б�Ҫ��û��Ҫһֱ����ɨ����0�С�
    PicTemp = Pixle[Ysite];
    JumpPointtypedef JumpPoint[2];                                                   // JumpPoint[0]��������ߣ�JumpPoint[1]�����ұ��ߡ�

  /******************************ɨ�豾�е��ұ���******************************/
    IntervalLow  = ImageDeal[Ysite + 1].RightBorder  - ImageScanInterval;               //����һ�е��ұ��߼Ӽ�Interval��Ӧ���п�ʼɨ�豾�У�Intervalһ��ȡ5����Ȼ��Ϊ�˱���������԰����ֵ�ĵĴ�һ�㡣
    IntervalHigh = ImageDeal[Ysite + 1].RightBorder + ImageScanInterval;              //���������ֻ��Ҫ�����б�������5�Ļ����ϣ����10�е�������䣩ȥɨ�ߣ�һ������ҵ����еı����ˣ��������ֵ��ʵ����̫��
    LimitL(IntervalLow);                                                             //������ǶԴ���GetJumpPointFromDet()������ɨ���������һ���޷�������
    LimitH(IntervalHigh);                                                            //������һ�еı����ǵ�2�У�����2-5=-3��-3�ǲ��Ǿ�û��ʵ�������ˣ���ô����-3���أ�
    Get_Border_And_SideType(PicTemp, 'R', IntervalLow, IntervalHigh,&JumpPoint[1]);  //ɨ���õ�һ���Ӻ������Լ�����ȥ�������߼���
  /******************************ɨ�豾�е��ұ���******************************/

  /******************************ɨ�豾�е������******************************/
    IntervalLow =ImageDeal[Ysite + 1].LeftBorder  -ImageScanInterval;                //����һ�е�����߼Ӽ�Interval��Ӧ���п�ʼɨ�豾�У�Intervalһ��ȡ5����Ȼ��Ϊ�˱���������԰����ֵ�ĵĴ�һ�㡣
    IntervalHigh =ImageDeal[Ysite + 1].LeftBorder +ImageScanInterval;                //���������ֻ��Ҫ�����б�������5�Ļ����ϣ����10�е�������䣩ȥɨ�ߣ�һ������ҵ����еı����ˣ��������ֵ��ʵ����̫��
    LimitL(IntervalLow);                                                             //������ǶԴ���GetJumpPointFromDet()������ɨ���������һ���޷�������
    LimitH(IntervalHigh);                                                            //������һ�еı����ǵ�2�У�����2-5=-3��-3�ǲ��Ǿ�û��ʵ�������ˣ���ô����-3���أ�
    Get_Border_And_SideType(PicTemp, 'L', IntervalLow, IntervalHigh,&JumpPoint[0]);  //ɨ���õ�һ���Ӻ������Լ�����ȥ�������߼���
  /******************************ɨ�豾�е������******************************/

  /*
       ����Ĵ���Ҫ���뿴�����������ҵ����ڸ���ħ�Ļ���
        ����ذ�GetJumpPointFromDet()����������߼�������
        ������������������棬��T������W������H��������־����ʲô��
        һ��Ҫ�㶮!!!��Ȼ�Ļ������鲻Ҫ���¿��ˣ���Ҫ��ĥ�Լ�!!!
  */
    if (JumpPoint[0].type =='W')                                                     //������е���������ڲ��������䣬����10���㶼�ǰ׵㡣
    {
      ImageDeal[Ysite].LeftBorder =ImageDeal[Ysite + 1].LeftBorder;                  //��ô���е�����߾Ͳ�����һ�еı��ߡ�
    }
    else                                                                             //������е����������T������H���
    {
      ImageDeal[Ysite].LeftBorder = JumpPoint[0].point;                              //��ôɨ�赽�ı����Ƕ��٣��Ҿͼ�¼�����Ƕ��١�
    }

    if (JumpPoint[1].type == 'W')                                                    //������е��ұ������ڲ��������䣬����10���㶼�ǰ׵㡣
    {
      ImageDeal[Ysite].RightBorder =ImageDeal[Ysite + 1].RightBorder;                //��ô���е��ұ��߾Ͳ�����һ�еı��ߡ�
    }
    else                                                                             //������е��ұ�������T������H���
    {
      ImageDeal[Ysite].RightBorder = JumpPoint[1].point;                             //��ôɨ�赽�ı����Ƕ��٣��Ҿͼ�¼�����Ƕ��١�
    }

    ImageDeal[Ysite].IsLeftFind =JumpPoint[0].type;                                  //��¼�����ҵ�����������ͣ���T����W������H��������ͺ��������õģ���Ϊ�һ�Ҫ��һ��������
    ImageDeal[Ysite].IsRightFind = JumpPoint[1].type;                                //��¼�����ҵ����ұ������ͣ���T����W������H��������ͺ��������õģ���Ϊ�һ�Ҫ��һ��������


  /*
        ����Ϳ�ʼ��W��H���͵ı��߷ֱ���д����� ΪʲôҪ������
        ����㿴����GetJumpPointFromDet�����߼���������T W H�������ͷֱ��Ӧʲô�����
        �����Ӧ��֪��W��H���͵ı��߶����ڷ��������ͣ������ǲ���Ҫ������
        ��һ���ֵĴ���˼·��Ҫ�Լ�������ʱ��úõ�ȥ��ĥ������ע������û������˵����ġ���
        ʵ���벻ͨ�������Ұɣ�
  */

    /************************************����ȷ��������(��H��)�ı߽�*************************************/

    if (( ImageDeal[Ysite].IsLeftFind == 'H' || ImageDeal[Ysite].IsRightFind == 'H'))
    {
      /**************************��������ߵĴ�����***************************/
      if (ImageDeal[Ysite].IsLeftFind == 'H')
      {
        for (Xsite = (ImageDeal[Ysite].LeftBorder + 1);Xsite <= (ImageDeal[Ysite].RightBorder - 1);Xsite++)                                                           //���ұ���֮������ɨ��
        {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite + 1) != 0))
          {
            ImageDeal[Ysite].LeftBorder =Xsite;
            ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          }
          else if (*(PicTemp + Xsite) != 0)
            break;
          else if (Xsite ==(ImageDeal[Ysite].RightBorder - 1))
          {
            ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          }
        }
      }
      /**************************��������ߵĴ�����***************************/


      /**************************�����ұ��ߵĴ�����***************************/
      if (ImageDeal[Ysite].IsRightFind == 'H')
      {
        for (Xsite = (ImageDeal[Ysite].RightBorder - 1);Xsite >= (ImageDeal[Ysite].LeftBorder + 1); Xsite--)
        {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite - 1) != 0))
          {
            ImageDeal[Ysite].RightBorder =Xsite;
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          }
          else if (*(PicTemp + Xsite) != 0)
            break;
          else if (Xsite == (ImageDeal[Ysite].LeftBorder + 1))
          {
            ImageDeal[Ysite].RightBorder = Xsite;
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          }
         }
       }
     }
    /**************************�����ұ��ߵĴ�����***************************/

  /*****************************����ȷ��������ı߽�******************************/



 /************************************����ȷ���ޱ��У���W�ࣩ�ı߽�****************************************************************/
    int ysite = 0;
    uint8 L_found_point = 0;
    uint8 R_found_point = 0;
    /**************************��������ߵ��ޱ���***************************/
    if (ImageDeal[Ysite].IsRightFind == 'W'&&Ysite > 10&&Ysite < 50)
    {
      if (Get_R_line == 'F')
      {
        Get_R_line = 'T';
        ytemp_W_R = Ysite + 2;
        for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++)
        {
          if (ImageDeal[ysite].IsRightFind =='T')
          {
              R_found_point++;
          }
        }
        if (R_found_point >8)
        {
          D_R = ((float)(ImageDeal[Ysite + R_found_point].RightBorder - ImageDeal[Ysite + 3].RightBorder)) /((float)(R_found_point - 3));
          if (D_R > 0)
          {
            R_Found_T ='T';
          }
          else
          {
            R_Found_T = 'F';
            if (D_R < 0)
            {
                ExtenRFlag = 'F';
            }
          }
        }
      }
      if (R_Found_T == 'T')
      {
        ImageDeal[Ysite].RightBorder =ImageDeal[ytemp_W_R].RightBorder -D_R * (ytemp_W_R - Ysite);  //����ҵ��� ��ô�Ի�׼�����ӳ���
      }
      LimitL(ImageDeal[Ysite].RightBorder);  //�޷�
      LimitH(ImageDeal[Ysite].RightBorder);  //�޷�
    }
    /**************************��������ߵ��ޱ���***************************/


    /**************************�����ұ��ߵ��ޱ���***************************/
    if (ImageDeal[Ysite].IsLeftFind == 'W' && Ysite > 10 && Ysite < 50 )
    {
      if (Get_L_line == 'F')
      {
        Get_L_line = 'T';
        ytemp_W_L = Ysite + 2;
        for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++)
        {
          if (ImageDeal[ysite].IsLeftFind == 'T')
            {
              L_found_point++;
            }
        }
        if (L_found_point > 8)              //�ҵ���׼б�ʱ�  ���ӳ�������ȷ���ޱ�
        {
          D_L = ((float)(ImageDeal[Ysite + 3].LeftBorder -ImageDeal[Ysite + L_found_point].LeftBorder)) /((float)(L_found_point - 3));
          if (D_L > 0)
          {
            L_Found_T = 'T';
          }
          else
          {
            L_Found_T = 'F';
            if (D_L < 0)
            {
                ExtenLFlag = 'F';
            }
          }
        }
      }

      if (L_Found_T == 'T')
      {
          ImageDeal[Ysite].LeftBorder =ImageDeal[ytemp_W_L].LeftBorder + D_L * (ytemp_W_L - Ysite);
      }

      LimitL(ImageDeal[Ysite].LeftBorder);  //�޷�
      LimitH(ImageDeal[Ysite].LeftBorder);  //�޷�
    }

    /**************************�����ұ��ߵ��ޱ���***************************/
    /************************************����ȷ���ޱ��У���W�ࣩ�ı߽�****************************************************************/


    /************************************��������֮��������һЩ������������*************************************************/
      if (ImageDeal[Ysite].IsLeftFind == 'W'&&ImageDeal[Ysite].IsRightFind == 'W')
      {
          ImageStatus.WhiteLine++;  //Ҫ�����Ҷ��ޱߣ�������+1
      }
     if (ImageDeal[Ysite].IsLeftFind == 'W'&&Ysite<55)
     {
          ImageStatus.Miss_Left_lines++;
     }
     if (ImageDeal[Ysite].IsRightFind == 'W'&&Ysite<55)
     {
          ImageStatus.Miss_Right_lines++;
     }

      LimitL(ImageDeal[Ysite].LeftBorder);   //�޷�
      LimitH(ImageDeal[Ysite].LeftBorder);   //�޷�
      LimitL(ImageDeal[Ysite].RightBorder);  //�޷�
      LimitH(ImageDeal[Ysite].RightBorder);  //�޷�

      ImageDeal[Ysite].Wide =ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
      ImageDeal[Ysite].Center =(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;

      if (ImageDeal[Ysite].Wide <= 7)
      {
          ImageStatus.OFFLine = Ysite + 1;
          break;
      }
      else if (ImageDeal[Ysite].RightBorder <= 10||ImageDeal[Ysite].LeftBorder >= 70)
      {
          ImageStatus.OFFLine = Ysite + 1;
          break;
      }
      /************************************��������֮��������һЩ������������*************************************************/
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_ExtensionLine
//  @brief          ������������þ��ǲ��ߣ�
//  @brief          ʮ��·�����������˵����ֱ�ж԰ɣ����������������ͷɨ�ߵ�ʱ���ǲ��ǻ����ɨ�������ߵ��������Ϊ�Ǽ��ж��ǰ�ɫ����Ҳ����ڰ�����㡣
//  @brief          ���԰���������ѱ����㷨��������ǲ�������������㷨�����Ļ���������Щ�е����ұ߽��ǲ��ǾͲ����ˣ���Ӧ�������ǲ���Ҳ�����ˣ������ܱ�֤С����ֱ���
//  @brief          ��Ȼ��֤���ˣ��������ʱ��С�����ܾͻ��������������ߣ�ֱ����ת������ת�ˣ��ǲ���Υ�����������ˣ����ǲ��Ǿͼ��ˣ�����˵�����Ƿǳ���Ҫ��һ����
//  @parameter      void
//  @time           2023��2��21��
//  @Author         
//  Sample usage:   Get_ExtensionLine();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Get_ExtensionLine(void)
{
    //ImageStatus.OFFLine=5;                                                  //����ṹ���Ա��֮���������︳ֵ������Ϊ��ImageStatus�ṹ������ĳ�Ա̫���ˣ�������ʱ��ֻ�õ���OFFLine�������������õ��������ĸ�ֵ��
    /************************************����ߵĲ��ߴ���*************************************************/
    if (ImageStatus.WhiteLine >= 8)                                       //��������е���������8
        TFSite = 55;                                                      //�Ǿ͸�TFSite��ֵΪ55����������Ǵ����㲹��б�ʵ�һ��������
    left_FTSite=0;
    right_FTSite=0;
    if (ExtenLFlag != 'F')                                                //���ExtenLFlag��־��������F���ǾͿ�ʼ���в��߲�����
        for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4);Ysite--)        //�ӵ�54��ʼ����ɨ��һֱɨ���������漸�С�
        {
            PicTemp = Pixle[Ysite];
            if (ImageDeal[Ysite].IsLeftFind =='W')                            //������е������������W���ͣ�Ҳ�����ޱ������͡�
            {
                if (ImageDeal[Ysite + 1].LeftBorder >= 70)                      //�����߽絽�˵�70���ұ�ȥ�ˣ��Ǵ���ʾ��Ǽ��������˵���Ѿ�����ˡ�
                {
                  ImageStatus.OFFLine = Ysite + 1;                              //���������õĴ����������ǲ�������ֱ������ѭ����
                  break;
                }
                while (Ysite >= (ImageStatus.OFFLine + 4))                      //�����߽��������Ǿͽ���whileѭ�����ţ�ֱ������ѭ������������
                {
                    Ysite--;                                                      //��������
                    if (ImageDeal[Ysite].IsLeftFind == 'T'
                      &&ImageDeal[Ysite - 1].IsLeftFind == 'T'
                      &&ImageDeal[Ysite - 2].IsLeftFind == 'T'
                      &&ImageDeal[Ysite - 2].LeftBorder > 0
                      &&ImageDeal[Ysite - 2].LeftBorder <70
                      )                                                         //���ɨ�����ޱ��е������������ж�����������
                    {
                      left_FTSite = Ysite - 2;                                         //�ǾͰ�ɨ������һ�е��������д���FTsite����
                    break;                                                      //����whileѭ��
                    }
                }
                DetL =((float)(ImageDeal[left_FTSite].LeftBorder -ImageDeal[TFSite].LeftBorder)) /((float)(left_FTSite - TFSite));  //��������ߵĲ���б��
                if (left_FTSite > ImageStatus.OFFLine)                              //���FTSite�洢����һ����ͼ�񶥱�OFFline������
                    for (ytemp = TFSite; ytemp >= left_FTSite; ytemp--)               //��ô�Ҿʹӵ�һ��ɨ������߽������ڶ��е�λ�ÿ�ʼ����һֱ���ߣ�����FTSite�С�
                    {
                    ImageDeal[ytemp].LeftBorder =(int)(DetL * ((float)(ytemp - TFSite))) +ImageDeal[TFSite].LeftBorder;     //������Ǿ���Ĳ��߲�����
                    }
            }
            else                                                              //ע�⿴������else���ĸ�if��һ�ԣ�������߼���ϵ��
                TFSite = Ysite + 2;                                             //����ΪʲôҪYsite+2����û����ע�����潲������Լ����ɡ�
        }
    /************************************����ߵĲ��ߴ���*************************************************/


    /************************************�ұ��ߵĲ��ߴ�����������ߴ���˼·һģһ����ע����*************************************************/
    if (ImageStatus.WhiteLine >= 8)
    TFSite = 55;
    if (ExtenRFlag != 'F')
    for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4);Ysite--)
    {
      PicTemp = Pixle[Ysite];  //�浱ǰ��
      if (ImageDeal[Ysite].IsRightFind =='W')
      {
        if (ImageDeal[Ysite + 1].RightBorder <= 10)
        {
          ImageStatus.OFFLine =Ysite + 1;
          break;
        }
        while (Ysite >= (ImageStatus.OFFLine + 4))
        {
          Ysite--;
          if (ImageDeal[Ysite].IsRightFind == 'T'
              &&ImageDeal[Ysite - 1].IsRightFind == 'T'
              &&ImageDeal[Ysite - 2].IsRightFind == 'T'
              &&ImageDeal[Ysite - 2].RightBorder < 79
              &&ImageDeal[Ysite - 2].RightBorder > 10
              )
          {
              right_FTSite = Ysite - 2;
            break;
          }
        }

        DetR =((float)(ImageDeal[right_FTSite].RightBorder -ImageDeal[TFSite].RightBorder)) /((float)(right_FTSite - TFSite));
        if (right_FTSite > ImageStatus.OFFLine)
          for (ytemp = TFSite; ytemp >= right_FTSite;ytemp--)
          {
            ImageDeal[ytemp].RightBorder =(int)(DetR * ((float)(ytemp - TFSite))) +ImageDeal[TFSite].RightBorder;
          }
      }
      else
        TFSite =Ysite +2;
    }
      /************************************�ұ��ߵĲ��ߴ�����������ߴ���˼·һģһ����ע����*************************************************/



}

/*�Ͻ��������ַ���ɨ�ߣ���Ϊ����Բ�����ж�Ԫ�صĵڶ�����*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Bottom_Line_OTSU
//  @brief          ��ȡ�ײ����ұ���
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        �����ͼ������
//  @param          Row                                     ͼ���Ysite
//  @param          Col                                     ͼ���Xsite
//  @return         Bottonline                              �ױ���ѡ��
//  @time           2022��10��9��
//  @Author         ������
//  Sample usage:   Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
//--------------------------------------------------------------------------------------------------------------------------------------------

void Search_Bottom_Line_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{
  
    //Ѱ����߽߱�
    for (int Xsite = Col / 2-2; Xsite > 1; Xsite--)
    {
        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite - 1] == 0)
        {
            ImageDeal[Bottonline].LeftBoundary = Xsite;//��ȡ�ױ������
            break;
        }
    }
    for (int Xsite = Col / 2+2; Xsite < LCDW-1; Xsite++)
    {       
        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite + 1] == 0)
        {
            ImageDeal[Bottonline].RightBoundary = Xsite;//��ȡ�ױ��ұ���
            break;
        }
    }
   

}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Left_and_Right_Lines
//  @brief          ͨ��sobel��ȡ���ұ���
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        �����ͼ������
//  @param          Row                                     ͼ���Ysite
//  @param          Col                                     ͼ���Xsite
//  @param          Bottonline                              �ױ���ѡ��
//  @return         ��
//  @time           2022��10��7��
//  @Author         ������
//  Sample usage:   Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);
//--------------------------------------------------------------------------------------------------------------------------------------------

void Search_Left_and_Right_Lines(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{
    //����С�˵ĵ�ǰ����״̬λ��Ϊ �� �� �� �� һ��Ҫ�� �ϣ����Ϊ��ɫ ���ϱ�Ϊ��ɫ �£��ұ�Ϊɫ  �ң������к�ɫ
/*  ǰ�������壺
                *   0
                * 3   1
                *   2
*/
/*Ѱ�����������*/
    uint8 Left_Rule[2][8] = {
                                  {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},  (x,y )
                                  {-1,-1,1,-1,1,1,-1,1} //{-1,-1},{1,-1},{1,1},{-1,1}
    };
    /*Ѱ�����������*/
    int Right_Rule[2][8] = {
                              {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},
                              {1,-1,1,1,-1,1,-1,-1} //{1,-1},{1,1},{-1,1},{-1,-1}
    };
      int num=0;
    uint8 Left_Ysite = Bottonline;
    uint8 Left_Xsite = ImageDeal[Bottonline].LeftBoundary;
    uint8 Left_Rirection = 0;//��߷���
    uint8 Pixel_Left_Ysite = Bottonline;
    uint8 Pixel_Left_Xsite = 0;

    uint8 Right_Ysite = Bottonline;
    uint8 Right_Xsite = ImageDeal[Bottonline].RightBoundary;
    uint8 Right_Rirection = 0;//�ұ߷���
    uint8 Pixel_Right_Ysite = Bottonline;
    uint8 Pixel_Right_Xsite = 0;
    uint8 Ysite = Bottonline;
    ImageStatus.OFFLineBoundary = 5;
    while (1)
    {    
            num++;
            if(num>400)
            {
                 ImageStatus.OFFLineBoundary = Ysite;
                break;
            }
        if (Ysite >= Pixel_Left_Ysite && Ysite >= Pixel_Right_Ysite)
        {           
            if (Ysite < ImageStatus.OFFLineBoundary)
            {
                ImageStatus.OFFLineBoundary = Ysite;
                break;
            }
            else
            {
                Ysite--;
            }
        }
        /*********���Ѳ��*******/
        if ((Pixel_Left_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)//�ұ�ɨ��
        {
            /*����ǰ������*/
            Pixel_Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
            Pixel_Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];

            if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)//ǰ���Ǻ�ɫ
            {
                //˳ʱ����ת90
                if (Left_Rirection == 3)
                    Left_Rirection = 0;
                else
                    Left_Rirection++;
            }
            else//ǰ���ǰ�ɫ
            {
                /*������ǰ������*/
                Pixel_Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                Pixel_Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];

                if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)//��ǰ��Ϊ��ɫ
                {
                    //���򲻱�  Left_Rirection  
                    Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
                    Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
                    if (ImageDeal[Left_Ysite].LeftBoundary_First == 0)
                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
                    ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
                }
                else//��ǰ��Ϊ��ɫ
                {
                    // �������ı� Left_Rirection  ��ʱ��90��
                    Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                    Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
                    if (ImageDeal[Left_Ysite].LeftBoundary_First == 0 )
                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
                    ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
                    if (Left_Rirection == 0)
                        Left_Rirection = 3;
                    else
                        Left_Rirection--;
                }

            }
        }
        /*********�ұ�Ѳ��*******/
        if ((Pixel_Right_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)//�ұ�ɨ��
        {
            /*����ǰ������*/
            Pixel_Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
            Pixel_Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];

            if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)//ǰ���Ǻ�ɫ
            {
                //��ʱ����ת90
                if (Right_Rirection == 0)
                    Right_Rirection = 3;
                else
                    Right_Rirection--;
            }
            else//ǰ���ǰ�ɫ
            {
                /*������ǰ������*/
                Pixel_Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                Pixel_Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];

                if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)//��ǰ��Ϊ��ɫ
                {
                    //���򲻱�  Right_Rirection  
                    Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
                    if (ImageDeal[Right_Ysite].RightBoundary_First == 79 )
                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
                }
                else//��ǰ��Ϊ��ɫ
                {
                    // �������ı� Right_Rirection  ��ʱ��90��
                    Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];
                    if (ImageDeal[Right_Ysite].RightBoundary_First == 79)
                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
                    if (Right_Rirection == 3)
                        Right_Rirection = 0;
                    else
                        Right_Rirection++;
                }

            }
        }

        if (abs(Pixel_Right_Xsite - Pixel_Left_Xsite) < 3)//Ysite<80��Ϊ�˷��ڵײ��ǰ�����ɨ�����  3 && Ysite < 30
        {
          
            ImageStatus.OFFLineBoundary = Ysite;
            break;
        }

    }
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Border_OTSU
//  @brief          ͨ��OTSU��ȡ���� ����Ϣ
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        �����ͼ������
//  @param          Row                                     ͼ���Ysite
//  @param          Col                                     ͼ���Xsite
//  @param          Bottonline                              �ױ���ѡ��
//  @return         ��
//  @time           2022��10��7��
//  @Author         ������
//  Sample usage:   Search_Border_OTSU(mt9v03x_image, IMAGE_ROW, IMAGE_COL, IMAGE_ROW-8);
//--------------------------------------------------------------------------------------------------------------------------------------------

void Search_Border_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{
    ImageStatus.WhiteLine_L = 0;
    ImageStatus.WhiteLine_R = 0;
    //ImageStatus.OFFLine = 1;
    /*�����±߽紦��*/
    for (int Xsite = 0; Xsite < LCDW; Xsite++)
    {
        imageInput[0][Xsite] = 0;
        imageInput[Bottonline + 1][Xsite] = 0;
    }
    /*�����ұ߽紦��*/
    for (int Ysite = 0; Ysite < LCDH; Ysite++)
    {
            ImageDeal[Ysite].LeftBoundary_First = 0;
            ImageDeal[Ysite].RightBoundary_First = 79;

            imageInput[Ysite][0] = 0;
            imageInput[Ysite][LCDW - 1] = 0;
    }
    /********��ȡ�ײ�����*********/
    Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
    /********��ȡ���ұ���*********/
    Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);



    for (int Ysite = Bottonline; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
    {
        if (ImageDeal[Ysite].LeftBoundary < 3)
        {
            ImageStatus.WhiteLine_L++;
        }
        if (ImageDeal[Ysite].RightBoundary > LCDW - 3)
        {
            ImageStatus.WhiteLine_R++;
        }
    }
}


//--------------------------------------------------------------
//  @name           Element_Judgment_Left_Rings()
//  @brief          ����ͼ���жϵ��Ӻ����������ж���Բ������.
//  @parameter      void
//  @time           
//  @Author         MRCHEN
//  Sample usage:   Element_Judgment_Left_Rings();
//--------------------------------------------------------------
void Element_Judgment_Left_Rings()
{
    if (   ImageStatus.Miss_Right_lines > 3 || ImageStatus.Miss_Left_lines < 13
        || ImageStatus.OFFLine > 5 || Straight_Judge(2, 5, 55) > 1
        || ImageFlag.image_element_rings == 2 || ImageFlag.Out_Road == 1 || ImageFlag.RoadBlock_Flag == 1
        || ImageDeal[52].IsLeftFind == 'W'
        || ImageDeal[53].IsLeftFind == 'W'
        || ImageDeal[54].IsLeftFind == 'W'
        || ImageDeal[55].IsLeftFind == 'W'
        || ImageDeal[56].IsLeftFind == 'W'
        || ImageDeal[57].IsLeftFind == 'W'
        || ImageDeal[58].IsLeftFind == 'W')
        return;
    int ring_ysite = 25;
    uint8 Left_Less_Num = 0;
    Left_RingsFlag_Point1_Ysite = 0;
    Left_RingsFlag_Point2_Ysite = 0;
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].LeftBoundary_First - ImageDeal[Ysite - 1].LeftBoundary_First > 4)
        {
            Left_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite + 1].LeftBoundary - ImageDeal[Ysite].LeftBoundary > 4)
        {
            Left_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = Left_RingsFlag_Point1_Ysite; Ysite > Left_RingsFlag_Point1_Ysite - 11; Ysite--)
    {
        if (ImageDeal[Ysite].IsLeftFind == 'W')
            Left_Less_Num++;
    }
    for (int Ysite = Left_RingsFlag_Point1_Ysite; Ysite > ImageStatus.OFFLine; Ysite--)
    {
//        if (ImageDeal[Ysite + 3].LeftBoundary_First < ImageDeal[Ysite].LeftBoundary_First
//            && ImageDeal[Ysite + 2].LeftBoundary_First < ImageDeal[Ysite].LeftBoundary_First
//            && ImageDeal[Ysite].LeftBoundary_First > ImageDeal[Ysite - 1].LeftBoundary_First
//            && ImageDeal[Ysite].LeftBoundary_First > ImageDeal[Ysite - 1].LeftBoundary_First
//            )
        if (   ImageDeal[Ysite + 6].LeftBorder < ImageDeal[Ysite+3].LeftBorder
            && ImageDeal[Ysite + 5].LeftBorder < ImageDeal[Ysite+3].LeftBorder
            && ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 2].LeftBorder
            && ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 1].LeftBorder
            )
        {
            Ring_Help_Flag = 1;
            break;
        }
    }
    if(Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 0 && Left_Less_Num>7)
    {
        if(ImageStatus.Miss_Left_lines > 13)
            Ring_Help_Flag = 1;
    }
    if (Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 1 && Left_Less_Num>7)
    {
        ImageFlag.image_element_rings = 1;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small=1;
        Front_Wait_After_Enter_Ring_Flag = 0;
        gpio_set_level(B0, 1);
    }
    Ring_Help_Flag = 0;
}


//--------------------------------------------------------------
//  @name           Element_Judgment_Right_Rings()
//  @brief          ����ͼ���жϵ��Ӻ����������ж���Բ������.
//  @parameter      void
//  @time           
//  @Author         MRCHEN
//  Sample usage:   Element_Judgment_Right_Rings();
//--------------------------------------------------------------
void Element_Judgment_Right_Rings()
{
    if (   ImageStatus.Miss_Left_lines > 3 || ImageStatus.Miss_Right_lines < 15
        || ImageStatus.OFFLine > 5 || Straight_Judge(1, 5, 55) > 1
        || ImageFlag.image_element_rings == 1 || ImageFlag.Out_Road == 1 || ImageFlag.RoadBlock_Flag == 1
        || ImageDeal[52].IsRightFind == 'W'
        || ImageDeal[53].IsRightFind == 'W'
        || ImageDeal[54].IsRightFind == 'W'
        || ImageDeal[55].IsRightFind == 'W'
        || ImageDeal[56].IsRightFind == 'W'
        || ImageDeal[57].IsRightFind == 'W'
        || ImageDeal[58].IsRightFind == 'W')
        return;

    int ring_ysite = 25;
    uint8 Right_Less_Num = 0;
    Right_RingsFlag_Point1_Ysite = 0;
    Right_RingsFlag_Point2_Ysite = 0;
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite - 1].RightBoundary_First - ImageDeal[Ysite].RightBoundary_First > 4)
        {
            Right_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary > 4)
        {
            Right_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = Right_RingsFlag_Point1_Ysite; Ysite > Right_RingsFlag_Point1_Ysite - 11; Ysite--)
    {
        if (ImageDeal[Ysite].IsRightFind == 'W')
            Right_Less_Num++;
    }
    //ips114_show_int(60,40, Right_Less_Num,3);
    for (int Ysite = Right_RingsFlag_Point1_Ysite; Ysite > ImageStatus.OFFLine; Ysite--)
    {
//        if (ImageDeal[Ysite + 3].RightBoundary_First > ImageDeal[Ysite].RightBoundary_First
//            && ImageDeal[Ysite + 2].RightBoundary_First > ImageDeal[Ysite].RightBoundary_First
//            && ImageDeal[Ysite].RightBoundary_First < ImageDeal[Ysite - 1].RightBoundary_First
//            && ImageDeal[Ysite].RightBoundary_First < ImageDeal[Ysite - 2].RightBoundary_First
//           )
        if (   ImageDeal[Ysite + 6].RightBorder > ImageDeal[Ysite + 3].RightBorder
            && ImageDeal[Ysite + 5].RightBorder > ImageDeal[Ysite + 3].RightBorder
            && ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 2].RightBorder
            && ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 1].RightBorder
           )
        {
            Ring_Help_Flag = 1;
            break;
        }
    }
    if(Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 0 && Right_Less_Num > 7)
    {
        if(ImageStatus.Miss_Right_lines>13)
            Ring_Help_Flag = 1;
    }
    if (Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 1 && Right_Less_Num > 7)
    {
        //Stop=1;
        ImageFlag.image_element_rings = 2;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small=1;
        Front_Wait_After_Enter_Ring_Flag = 0;
        gpio_set_level(B0, 1);
    }

        //ips200_show_uint(100,220,ImageFlag.image_element_rings,3);
    Ring_Help_Flag = 0;
}



//--------------------------------------------------------------
//  @name           Element_Judgment_Ramp()
//  @brief          ����ͼ���жϵ��Ӻ����������ж��µ�
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Judgment_Ramp();
//--------------------------------------------------------------
void Element_Judgment_Ramp()
{
    if (ImageStatus.WhiteLine >= 3 || Ramp_cancel)
        return;
    int i = 0;
    if (ImageStatus.OFFLine <= 5)
    {
        for (Ysite = ImageStatus.OFFLine + 1; Ysite < 7; Ysite++)
        {
            if ( ImageDeal[Ysite].Wide > 18
                        && (ImageDeal[Ysite].IsRightFind == 'T'
                        && ImageDeal[Ysite].IsLeftFind == 'T')
                        && ImageDeal[Ysite].LeftBorder < 40
                        && ImageDeal[Ysite].RightBorder > 40 && Pixle[Ysite][ImageDeal[Ysite].Center] == 1
                        && Pixle[Ysite][ImageDeal[Ysite].Center-2] == 1
                        && Pixle[Ysite][ImageDeal[Ysite].Center+2] == 1
                        && dl1b_distance_mm < 700
                        && ImageStatus.Miss_Left_lines <7
                        && ImageStatus.Miss_Right_lines < 7
                )
                     i++;
//            ips114_show_int(20,0, i,3);
            if (i >= 3)
            {
//                Stop=1;
                ImageFlag.Ramp = 1;
                BeeOn;
                Statu = Ramp;
                i = 0;
                break;
            }
        }
    }

}


//--------------------------------------------------------------
//  @name           Element_Judgment_OutRoad()
//  @brief          ����ͼ���жϵ��Ӻ����������ж϶�·����.
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Judgment_Bend();
//--------------------------------------------------------------
float S = 0;
int Start=0,End=0,a=0;
int Out_Black_flag = 0;
void Element_Judgment_OutRoad()
{
    if(dl1b_distance_mm < 1150 || ImageFlag.RoadBlock_Flag == 1 || ImageFlag.Out_Road)   return;
    int Right_Num=0,Left_Num=0;
    if(ImageStatus.OFFLine > 20)
    {
        for(int Ysite = ImageStatus.OFFLine + 1;Ysite < ImageStatus.OFFLine + 11;Ysite++)
        {
            if(ImageDeal[Ysite].IsLeftFind == 'T')
                Left_Num++;
            if(ImageDeal[Ysite].IsRightFind == 'T')
                Right_Num++;
        }
    }
//    ips114_show_int(20,0,Left_Num,5);
//    ips114_show_int(20,20,Right_Num,5);
    if(Left_Num > 7 && Right_Num > 7 && !ImageFlag.Out_Road)
    {
        ImageFlag.Out_Road = 1;
        duan_num++;
        BeeOn;
    }
//    if(duan_num==1)
//        Stop=1;
//
//    for(int Xsite = 0;Xsite < 80;Xsite++)
//        ImageDeal1[Xsite].TopBorder = 59;
//    Start=0,End=0;
//    for(int Xsite = 5;Xsite < 75;Xsite++)
//    {
//        for(int Ysite = 50;Ysite > 15;Ysite--)
//        {
//            if(Pixle[Ysite][Xsite] == 0)
//                break;
//            if(Pixle[Ysite][Xsite] == 1 && Pixle[Ysite-1][Xsite] == 0)
//            {
//                ImageDeal1[Xsite].TopBorder = Ysite;
//                break;
//            }
//        }
//    }
//    for(int Xsite = 5;Xsite < 75;Xsite++)
//    {
//        if(ImageDeal1[Xsite].TopBorder != 79)
//        {
//            Start = Xsite;
//            break;
//        }
//    }
//    for(int Xsite = 74;Xsite >= 5;Xsite--)
//    {
//        if(ImageDeal1[Xsite].TopBorder != 79)
//        {
//            End = Xsite;
//            break;
//        }
//    }
//    for(int Xsite = Start+1;Xsite <= End;Xsite++)
//    {
//        if(ImageDeal1[Xsite].TopBorder < ImageDeal1[Xsite-1].TopBorder)
//        {
//            a = Xsite;
//        }
//    }
//    End=a;
//    if(End<39)
//    {
//        Start=End;
//        End=End+20;
//    }
//
//    else
//        Start=End-20;
//    S=0;
//    float  Sum = 0, Err = 0, k = 0;
//    k = (float)(ImageDeal1[Start].TopBorder - ImageDeal1[End].TopBorder) / (Start - End);
//    for (int i = 0; i < 20; i++)
//    {
//        Err = (ImageDeal1[Start].TopBorder + k * i - ImageDeal1[i + Start].TopBorder) * (ImageDeal1[Start].TopBorder + k * i - ImageDeal1[i + Start].TopBorder);
//        Sum += Err;
//    }
//    S = Sum / 20;
//    if(S<1)
//    {
//        ImageFlag.Out_Road=1;
//        gpio_set_level(B0, 1);
//    }
}
//--------------------------------------------------------------
//  @name           Element_Judgment_RoadBlock()
//  @brief          ����ͼ���жϵ��Ӻ����������ж�·������.
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Judgment_RoadBlock();
//--------------------------------------------------------------
uint8 Auto_Through_RoadBlock = 0;
int RoadBlock_length = 0;
uint8 RoadBlock_Flag = 0;
uint8 RoadBlock_Thruough_Flag = 2;
uint8 Regular_RoadBlock = 0;
uint8 RoadBlock_Regular_Way[8][3] = {{0,0,0},{1,0,0},{0,1,0},{1,1,0},{0,0,1},{1,0,1},{0,1,1},{1,1,1}};
uint8 RoadBlock_Thruough_Flag_Record = 0;
uint8 Bend_Thruough_RoadBlock_Flag = 0;
uint8 ICM20602_Clear_Flag = 0;
int RoadBlock_Length_Compensate = 0;
void Element_Judgment_RoadBlock()
{
//    if(ImageFlag.Ramp /*|| ImageStatus.OFFLineBoundary < 20 || dl1a_distance_mm > 1100*/)   return;
//    if(ImageStatus.OFFLine >= 10  && ImageStatus.OFFLine < 50 && dl1b_distance_mm < 1100
//    && Straight_Judge(1, ImageStatus.OFFLine+8, ImageStatus.OFFLine+25) < 1
//    && Straight_Judge(2, ImageStatus.OFFLine+8, ImageStatus.OFFLine+25) < 1
//    && ImageStatus.Miss_Left_lines< 3
//    && ImageStatus.Miss_Right_lines< 3
//    )
//    {
//        ImageFlag.RoadBlock_Flag = 1;
//        //Stop = 1;
//        BeeOn;
//        Statu = RoadBlock;
//        Steer_Flag=1; //��������
//        Angle_block = Yaw_Now;//������һ�δ��
//        RoadBlock_Flag++;
//    }

    if(ImageFlag.Ramp || ImageStatus.OFFLine < 10 || dl1b_distance_mm > 1100)   return;
    int Right_Num=0,Left_Num=0;
    for(int Ysite = ImageStatus.OFFLine + 1;Ysite < ImageStatus.OFFLine + 11;Ysite++)
    {
        //if((ImageDeal[Ysite].IsLeftFind))
        if(ImageDeal[Ysite].IsLeftFind == 'T')
            Left_Num++;
        if(ImageDeal[Ysite].IsRightFind == 'T')
            Right_Num++;
    }
    if(Left_Num > 7 && Right_Num > 7)
    {
        ImageFlag.RoadBlock_Flag = 1;
        block_num++;
        //Stop = 1;
        Statu = RoadBlock;
        if( !Regular_RoadBlock )
            Auto_RoadBlock_Through();
        else
        {
            if( RoadBlock_Regular_Way[Regular_RoadBlock][block_num-1] )
            {
                if(Parameter_Justment_Flag == 1)
                   { RoadBlock_Thruough_Flag = 1; Auto_Through_RoadBlock = 0; }
                if(Parameter_Justment_Flag == 2)
                   { RoadBlock_Thruough_Flag = 2; Auto_Through_RoadBlock = 0; }
                if( ImageStatus.Det_True > 2 || ImageStatus.Det_True < -2 )
                   { RoadBlock_Length_Compensate = 267; BeeOn; }
                else
                    RoadBlock_Length_Compensate = 0;
            }
            else
                Auto_RoadBlock_Through();
        }
        Steer_Flag=1; //��������
        ICM20602_Clear_Flag++;
        RoadBlock_Flag++;
    }
}

void Auto_RoadBlock_Through()
{
    if( ImageStatus.Det_True >= 0 )
    {
        RoadBlock_Thruough_Flag = 1;  //��
        if( ImageStatus.Det_True > 3 )
         { Auto_Through_RoadBlock = 1; BeeOn; }
        else
            Auto_Through_RoadBlock = 0;
    }
    else
    {
        RoadBlock_Thruough_Flag = 2;   //��
        if( ImageStatus.Det_True < -3 )
         { Auto_Through_RoadBlock = 1; BeeOn; }
        else
            Auto_Through_RoadBlock = 0;
    }
}
//--------------------------------------------------------------
//  @name           Element_Handle_RoadBlock()
//  @brief          ����ͼ�������Ӻ�������������·������.
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Handle_RoadBlock();
//--------------------------------------------------------------

void Element_Handle_RoadBlock()
{
    if( ICM20602_Clear_Flag == 2 )
    {
        if ( RoadBlock_Thruough_Flag == 1 )
        {//(�����޸ĵ�һ�������Ǵ��)
            if ( ((( Yaw_Now-Angle_block > 25 && !Auto_Through_RoadBlock ) || ( Yaw_Now-Angle_block > 20 && Auto_Through_RoadBlock )) &&
                          RoadBlock_Flag == 1 && !Bend_Thruough_RoadBlock_Flag ) ||
               ( ( Yaw_Now-Angle_block > 45 && RoadBlock_Flag == 1 &&  Bend_Thruough_RoadBlock_Flag ) ) )
            { Steer_Flag = 0; RoadBlock_Flag++; /*Stop = 1;*/ } // ·�ϵ�һ�δ�ǣ����������ж�  25
            if ( RoadBlock_Flag == 2 )
            { // ·�ϵڶ��Σ��ñ���������
                RoadBlock_length = (Element_encoder1+Element_encoder2)/2;
                if ( ( (( RoadBlock_length > (1123+RoadBlock_Length_Compensate) && !Auto_Through_RoadBlock ) || ( RoadBlock_length > 987 && Auto_Through_RoadBlock ))
                        && !Bend_Thruough_RoadBlock_Flag ) || (RoadBlock_length > 678 && Bend_Thruough_RoadBlock_Flag) )
                { RoadBlock_Flag++; Steer_Flag = 1;RoadBlock_length = 0; /*Stop = 1;*/ }//  �����޸ı���������  3500 1970
                if(RoadBlock_Flag == 3 && !block_flag)   //�����ǵڶ��δ��
                {
                    Angle_block = Yaw_Now;
                    block_flag = 1;
                }
            }
            if( (((( Angle_block-Yaw_Now > 45 && !Auto_Through_RoadBlock ) || ( Angle_block-Yaw_Now > 40 && Auto_Through_RoadBlock )) &&
                    RoadBlock_Flag == 3 && !Bend_Thruough_RoadBlock_Flag)  ||
                 (Angle_block-Yaw_Now > 45 && RoadBlock_Flag == 3 &&  Bend_Thruough_RoadBlock_Flag)) || RoadBlock_Flag > 3 )  //�޸ĵ����������Ǵ��  50
            {
                //Stop = 1;
                RoadBlock_Flag++;
                RoadBlock_Flag = RoadBlock_Flag > 10 ? 10 : RoadBlock_Flag;
                if(RoadBlock_Flag == 4)
                {
                    Bend_Thruough_RoadBlock_Flag = 0;
                    Auto_Through_RoadBlock = 0;
                    return;
                }
                Gray_Value=0;
                for(Ysite = 55;Ysite>45;Ysite--)
                    for(Xsite=35;Xsite<65;Xsite++)
                        Gray_Value=Gray_Value+Pixle[Ysite][Xsite];
                if(Gray_Value>270)//���300
                {
                    //Stop = 1;
                    ImageFlag.RoadBlock_Flag = 0;
                    BeeOff;
                    RoadBlock_Flag = 0;block_flag = 0;Angle_block=0;Steer_Flag=0;
                    Element_encoder1 = 0;Element_encoder2 = 0; RoadBlock_Length_Compensate = 0;
                    Through_RoadBlock_Flag_Delay = 0;ICM20602_Clear_Flag = 0;
                    RoadBlock_Thruough_Flag = RoadBlock_Thruough_Flag_Record;
                }
            }
        }
        else if ( RoadBlock_Thruough_Flag == 2 )
        {//(�����޸ĵ�һ�������Ǵ��)
            if ( ((( Angle_block-Yaw_Now > 25 && !Auto_Through_RoadBlock ) || ( Angle_block-Yaw_Now > 20 && Auto_Through_RoadBlock )) &&
                                  RoadBlock_Flag == 1 && !Bend_Thruough_RoadBlock_Flag ) ||
                       ( ( Angle_block-Yaw_Now > 45 && RoadBlock_Flag == 1 &&  Bend_Thruough_RoadBlock_Flag ) ) )
            { Steer_Flag = 0; RoadBlock_Flag++; /*Stop = 1;*/ } // ·�ϵ�һ�δ�ǣ����������ж�
            if ( RoadBlock_Flag == 2 )
            { // ·�ϵڶ��Σ��ñ���������
                RoadBlock_length = (Element_encoder1+Element_encoder2)/2;
                if ( ( (( RoadBlock_length > (1423+RoadBlock_Length_Compensate) && !Auto_Through_RoadBlock ) || ( RoadBlock_length > 987 && Auto_Through_RoadBlock ))
                                    && !Bend_Thruough_RoadBlock_Flag ) || (RoadBlock_length > 789 && Bend_Thruough_RoadBlock_Flag) )
                { RoadBlock_Flag++; Steer_Flag = 1;RoadBlock_length = 0; /*Stop = 1;*/ }//  �����޸ı���������  3500 1970
                if(RoadBlock_Flag == 3 && !block_flag)   //�����ǵڶ��δ��
                {
                    Angle_block = Yaw_Now;
                    block_flag = 1;
                }
            }
            if( (((( Yaw_Now-Angle_block > 45 && !Auto_Through_RoadBlock ) || ( Yaw_Now-Angle_block > 40 && Auto_Through_RoadBlock )) &&
                            RoadBlock_Flag == 3 && !Bend_Thruough_RoadBlock_Flag)  ||
                         (Yaw_Now-Angle_block > 45 && RoadBlock_Flag == 3 &&  Bend_Thruough_RoadBlock_Flag)) || RoadBlock_Flag > 3 )  //�޸ĵ����������Ǵ��  50
            {
                //Stop = 1;
                RoadBlock_Flag++;
                RoadBlock_Flag = RoadBlock_Flag > 10 ? 10 : RoadBlock_Flag;
                if(RoadBlock_Flag == 4)
                {
                    Bend_Thruough_RoadBlock_Flag = 0;
                    Auto_Through_RoadBlock = 0;
                    return;
                }
                Gray_Value=0;
                for(Ysite = 55;Ysite>45;Ysite--)
                    for(Xsite=15;Xsite<45;Xsite++)
                        Gray_Value=Gray_Value+Pixle[Ysite][Xsite];
                if(Gray_Value>270)//���400
                {
                    //Stop = 1;
                    ImageFlag.RoadBlock_Flag = 0;
                    BeeOff;
                    RoadBlock_Flag = 0;block_flag = 0;Angle_block=0;Steer_Flag=0;
                    Element_encoder1 = 0;Element_encoder2 = 0;
                    Through_RoadBlock_Flag_Delay = 0;ICM20602_Clear_Flag = 0;
                    RoadBlock_Thruough_Flag = RoadBlock_Thruough_Flag_Record;
                    RoadBlock_Length_Compensate = 0;
                }
            }
        }
    }
}

//--------------------------------------------------------------
//  @name           Element_Judgment_Zebra()
//  @brief          ����ͼ���жϵ��Ӻ����������жϰ�����
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Judgment_Zebra();
//--------------------------------------------------------------
void Element_Judgment_Zebra()//�������ж�
{
    if(ImageFlag.Zebra_Flag || ImageFlag.image_element_rings == 1 || ImageFlag.image_element_rings == 2
            || ImageFlag.Out_Road == 1 || ImageFlag.RoadBlock_Flag == 1) return;
    int NUM = 0, net = 0;
    if(ImageStatus.OFFLineBoundary<20)
    {
        for (int Ysite = 20; Ysite < 33; Ysite++)
        {
            net = 0;
            for (int Xsite =ImageDeal[Ysite].LeftBoundary + 2; Xsite < ImageDeal[Ysite].RightBoundary - 2; Xsite++)
            {
                if (Pixle[Ysite][Xsite] == 0 && Pixle[Ysite][Xsite + 1] == 1)
                {
                    net++;
                    if (net > 4)
                        NUM++;
                }
            }
        }
    }

    if (NUM >= 4 && ImageFlag.Zebra_Flag == 0)
    {
        if(ImageStatus.Miss_Left_lines > (ImageStatus.Miss_Right_lines + 3))//�󳵿�
        {
            ImageFlag.Zebra_Flag = 1;
            Garage_Location_Flag++;
            gpio_set_level(B0, 1);
        }
        if((ImageStatus.Miss_Left_lines + 3)<ImageStatus.Miss_Right_lines)//�ҳ���
        {
            ImageFlag.Zebra_Flag = 2;
            Garage_Location_Flag++;
            gpio_set_level(B0, 1);
        }
    }

}


//--------------------------------------------------------------
//  @name           Element_Handle_Zebra()
//  @brief          ����ͼ�������Ӻ�������������������
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Handle_Zebra();
//--------------------------------------------------------------
int Zebra_length = 0;
void Element_Handle_Zebra()//�����ߴ���
{
    Zebra_length=((Element_encoder1+Element_encoder2)/2);

    if(ImageFlag.Zebra_Flag == 1)//�󳵿�
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
        {
             ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBoundary  - Half_Road_Wide[Ysite];
        }
    }
    else if(ImageFlag.Zebra_Flag == 2)//�ҳ���
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
        {
             ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBoundary  + Half_Road_Wide[Ysite];
        }
    }
    if(Garage_Location_Flag < Garage_num)
    {
        if(Zebra_length > Zebra_num)
        {
            ImageFlag.Zebra_Flag = 0;
            gpio_set_level(B0, 0);
            Element_encoder1 = 0;
            Element_encoder2 = 0;
            Zebra_length = 0;
        }
    }
    else if(Garage_Location_Flag == Garage_num)
    {
        if(Zebra_length > Garage_length)
        {
            Stop=1;
            //ImageFlag.Zebra_Flag = 0;
            Element_encoder1 = 0;
            Element_encoder2 = 0;
            Zebra_length = 0;
            gpio_set_level(B0, 0);
        }
    }
}

//--------------------------------------------------------------
//  @name           Element_Handle_OutRoad()
//  @brief          ����ͼ�������Ӻ��������������µ�
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Handle_OutRoad();
//--------------------------------------------------------------
void Element_Handle_OutRoad()//��·����
{
    Gray_Value=0;
    for(int Ysite = 35;Ysite < 55;Ysite++)
    {
        for(int Xsite = 30;Xsite < 50;Xsite++)
        {
            Gray_Value=Gray_Value+Pixle[Ysite][Xsite];
        }
    }
    if(Gray_Value > 360 && ImageStatus.OFFLine < 20)//���400
    {
        //Stop=1;
        ImageFlag.Out_Road=0;
        BeeOff;
    }
}

//--------------------------------------------------------------
//  @name           Element_Handle_Ramp()
//  @brief          ����ͼ�������Ӻ��������������µ�
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Handle_Ramp();
//--------------------------------------------------------------
uint Ramp_length = 0;
void Element_Handle_Ramp()//�µ�����
{
     Ramp_length = ((Element_encoder1+Element_encoder2)/2);

     if( Ramp_length > Ramp_num )//    170���µ�
     {
         //Stop=1;
         ImageFlag.Ramp = 0;
         BeeOff;
         Element_encoder1 = 0;
         Element_encoder2 = 0;
         Ramp_length = 0;
     }
}


//--------------------------------------------------------------
//  @name           Element_Handle_Left_Rings()
//  @brief          ����ͼ�������Ӻ���������������Բ������.
//  @parameter      void
//  @time           
//  @Author         MRCHEN
//  Sample usage:   Element_Handle_Left_Rings();
//-------------------------------------------------------------
void Element_Handle_Left_Rings()
{   
    /****************��СԲ���ж�*****************/
//    if(ImageFlag.ring_big_small == 0)
//    {
//        Black = 0;
//        for (int Ysite = 30; Ysite > 0; Ysite--)
//        {
//            if(Pixle[Ysite][2]==0)
//            {
//                Black++;
//            }
//            if(Ysite>2)
//            {
//                if(Pixle[Ysite-1][2]==0 && Pixle[Ysite][2]==1 && Pixle[Ysite-1][2]==1)
//                    break;
//            }
//        }
//        if(Black > 12) //��Բ��
//            ImageFlag.ring_big_small = 1;
//        else                        //СԲ��
//            {ImageFlag.ring_big_small = 2;
//             BeeOn;}
//    }

    /***************************************�ж�**************************************/
    int num = 0;
    for (int Ysite = 55; Ysite > 30; Ysite--)
    {
//        if(ImageDeal[Ysite].LeftBoundary_First < 3)
//        {
//            num++;
//        }
        if(ImageDeal[Ysite].IsLeftFind == 'W')
            num++;
        if(    ImageDeal[Ysite+3].IsLeftFind == 'W' && ImageDeal[Ysite+2].IsLeftFind == 'W'
            && ImageDeal[Ysite+1].IsLeftFind == 'W' && ImageDeal[Ysite].IsLeftFind == 'T')
            break;
    }
//    ips114_show_int(180,20,num,5);
        //׼������
    if (ImageFlag.image_element_rings_flag == 1 && num>15)
    {
        ImageFlag.image_element_rings_flag = 2;
    }
    if (ImageFlag.image_element_rings_flag == 2 && num<10)
    {

        ImageFlag.image_element_rings_flag = 5;
    }
//    if(ImageFlag.image_element_rings_flag == 3 && num<3)//
//    {
//        ImageFlag.image_element_rings_flag = 4;
//    }
        //�յ�Բ��
    if(ImageFlag.image_element_rings_flag == 4)
    {
        //Stop= 1;
        Point_Ysite = 0;
        Point_Xsite = 0;
        for (int Ysite = 20; Ysite > ImageStatus.OFFLine + 2; Ysite--)
        {
            if(ImageDeal[Ysite].IsLeftFind == 'W' && ImageDeal[Ysite-1].IsLeftFind == 'T')
            {
                Point_Ysite = Ysite;
                Point_Xsite = 0;
                break;
            }
        }
        if(Point_Ysite > 6 && ImageFlag.ring_big_small == 1)
        {
            ImageFlag.image_element_rings_flag=5;
        }
        if(Point_Ysite > 11 && ImageFlag.ring_big_small == 2)
        {
            ImageFlag.image_element_rings_flag=5;
        }
    }
        //����
    if(ImageFlag.image_element_rings_flag == 5 && ImageStatus.Miss_Right_lines>15)
    {
        ImageFlag.image_element_rings_flag = 6;
    }
        //����СԲ��
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.Miss_Right_lines<7)
    {
        //Stop = 1;
        ImageFlag.image_element_rings_flag = 7;
    }
        //���� ��Բ���ж�
    if (ImageFlag.ring_big_small == 1 && ImageFlag.image_element_rings_flag == 7)
    {
        Point_Ysite = 0;
        Point_Xsite = 0;
        for (int Ysite = 45; Ysite > ImageStatus.OFFLine + 7; Ysite--)
        {
            if (    ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 2].RightBorder
                 && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 2].RightBorder
                 && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 1].RightBorder
                 && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 1].RightBorder
                 && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 4].RightBorder
                 && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 4].RightBorder
                 && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 6].RightBorder
                 && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 6].RightBorder
                 && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 5].RightBorder
                 && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 5].RightBorder
               )
            {
                Point_Xsite = ImageDeal[Ysite].RightBorder;
                Point_Ysite = Ysite;
                break;
            }
        }
        if (Point_Ysite > 22)
        {
            ImageFlag.image_element_rings_flag = 8;
            //Stop = 1;
        }
    }
        //���� СԲ���ж�
    if (ImageFlag.image_element_rings_flag == 7 && ImageFlag.ring_big_small == 2)
    {
        Point_Ysite = 0;
        Point_Xsite = 0;
        for (int Ysite = 50; Ysite > ImageStatus.OFFLineBoundary + 3; Ysite--)
        {
            if (    ImageDeal[Ysite].RightBoundary < ImageDeal[Ysite + 2].RightBoundary
                 && ImageDeal[Ysite].RightBoundary < ImageDeal[Ysite - 2].RightBoundary
               )
            {
                Point_Xsite = ImageDeal[Ysite].RightBoundary;
                Point_Ysite = Ysite;
                break;
            }
        }
        if (Point_Ysite > 20)
          ImageFlag.image_element_rings_flag = 8;
    }
        //������
    if (ImageFlag.image_element_rings_flag == 8)
    {
        if (    Straight_Judge(2, ImageStatus.OFFLine+15, 50) < 1
             && ImageStatus.Miss_Right_lines < 8
             && ImageStatus.OFFLine < 7)    //�ұ�Ϊֱ���ҽ�ֹ�У�ǰհֵ����С

            ImageFlag.image_element_rings_flag = 9;
    }
        //����Բ������
//    if (ImageFlag.image_element_rings_flag == 9 )
//    {
//        int num=0;
//        for (int Ysite = 58; Ysite > 30; Ysite--)
//        {
//            if(ImageDeal[Ysite].LeftBoundary_First < 3 )
//            {
//                num++;
//            }
//        }
//        if(num > 8)
//        {
//            ImageFlag.image_element_rings_flag = 10;
//        }
//    }

    if (ImageFlag.image_element_rings_flag == 9)
    {
        int num=0;
        for (int Ysite = 40; Ysite > 10; Ysite--)
        {
            if(ImageDeal[Ysite].IsLeftFind == 'W' )
                num++;
        }
        if(num < 5)
        {
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
            Front_Wait_After_Enter_Ring_Count++;
            gpio_set_level(B0, 0);
        }
    }
    


    /***************************************����**************************************/
        //׼������  �������
    if (   ImageFlag.image_element_rings_flag == 1
        || ImageFlag.image_element_rings_flag == 2
        || ImageFlag.image_element_rings_flag == 3
        || ImageFlag.image_element_rings_flag == 4)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
        }
    }
        //����  ����
    if  ( ImageFlag.image_element_rings_flag == 5
        ||ImageFlag.image_element_rings_flag == 6
        )
    {
        int  flag_Xsite_1=0;
        int flag_Ysite_1=0;
        float Slope_Rings=0;
        for(Ysite=55;Ysite>ImageStatus.OFFLine;Ysite--)//���满��
        {
            for(Xsite=ImageDeal[Ysite].LeftBorder + 1;Xsite<ImageDeal[Ysite].RightBorder - 1;Xsite++)
            {
                if(  Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0)
                 {
                   flag_Ysite_1 = Ysite;
                   flag_Xsite_1 = Xsite;
                   Slope_Rings=(float)(79-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                   break;
                 }
            }
            if(flag_Ysite_1 != 0)
            {
                break;
            }
        }
        if(flag_Ysite_1 == 0)
        {
            for(Ysite=ImageStatus.OFFLine+1;Ysite<30;Ysite++)
            {
                if(ImageDeal[Ysite].IsLeftFind=='T'&&ImageDeal[Ysite+1].IsLeftFind=='T'&&ImageDeal[Ysite+2].IsLeftFind=='W'
                    &&abs(ImageDeal[Ysite].LeftBorder-ImageDeal[Ysite+2].LeftBorder)>10
                  )
                {
                    flag_Ysite_1=Ysite;
                    flag_Xsite_1=ImageDeal[flag_Ysite_1].LeftBorder;
                    ImageStatus.OFFLine=Ysite;
                    Slope_Rings=(float)(79-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                    break;
                }

            }
        }
        //����
        if(flag_Ysite_1 != 0)
        {
            for(Ysite=flag_Ysite_1;Ysite<60;Ysite++)
            {
                ImageDeal[Ysite].RightBorder=flag_Xsite_1+Slope_Rings*(Ysite-flag_Ysite_1);
                //if(ImageFlag.ring_big_small==1)//��Բ���������
                    ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2;
                //else//СԲ�������
                //    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
//                if(ImageDeal[Ysite].Center<0)
//                    ImageDeal[Ysite].Center = 0;
            }
            ImageDeal[flag_Ysite_1].RightBorder=flag_Xsite_1;
            for(Ysite=flag_Ysite_1-1;Ysite>10;Ysite--) //A���Ϸ�����ɨ��
            {
                for(Xsite=ImageDeal[Ysite+1].RightBorder-10;Xsite<ImageDeal[Ysite+1].RightBorder+2;Xsite++)
                {
                    if(Pixle[Ysite][Xsite]==1 && Pixle[Ysite][Xsite+1]==0)
                    {
                        ImageDeal[Ysite].RightBorder=Xsite;
                        //if(ImageFlag.ring_big_small==1)//��Բ���������
                            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2;
                        //else//СԲ�������
                        //    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
                        //if(ImageDeal[Ysite].Center<0)
                        //    ImageDeal[Ysite].Center = 0;
                        //ImageDeal[Ysite].Wide=ImageDeal[Ysite].RightBorder-ImageDeal[Ysite].LeftBorder;
                        break;
                    }
                }

                if(ImageDeal[Ysite].Wide>8 &&ImageDeal[Ysite].RightBorder< ImageDeal[Ysite+2].RightBorder)
                {
                    continue;
                }
                else
                {
                    ImageStatus.OFFLine=Ysite+2;
                    break;
                }
            }
        }
    }
        //���� С���������� �󻷲���
    if (ImageFlag.image_element_rings_flag == 7)
    {
//        for (int Ysite = 57; Ysite > ImageStatus.OFFLine+1; Ysite--)
//        {
//            if(ImageFlag.ring_big_small==2)
//                ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
//            if(ImageDeal[Ysite].Center<=0)
//            {
//                ImageDeal[Ysite].Center = 0;
//                ImageStatus.OFFLine=Ysite-1;
//                break;
//            }
//        }
    }
        //��Բ������ ����
    if (ImageFlag.image_element_rings_flag == 8 && ImageFlag.ring_big_small == 1)    //��Բ��
    {
        Repair_Point_Xsite = 20;
        Repair_Point_Ysite = 0;
        for (int Ysite = 40; Ysite > 5; Ysite--)
        {
            if (Pixle[Ysite][23] == 1 && Pixle[Ysite-1][23] == 0)//28
            {
                Repair_Point_Xsite = 23;
                Repair_Point_Ysite = Ysite-1;
                ImageStatus.OFFLine = Ysite + 1;  //��ֹ�����¹滮
                break;
            }
        }
        for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         //����
        {
            ImageDeal[Ysite].RightBorder = (ImageDeal[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].RightBorder;
            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
        }
    }
        //СԲ������ ����
    if (ImageFlag.image_element_rings_flag == 8 && ImageFlag.ring_big_small == 2)    //СԲ��
    {
        Repair_Point_Xsite = 0;
        Repair_Point_Ysite = 0;
        for (int Ysite = 55; Ysite > 5; Ysite--)
        {
            if (Pixle[Ysite][15] == 1 && Pixle[Ysite-1][15] == 0)
            {
                Repair_Point_Xsite = 15;
                Repair_Point_Ysite = Ysite-1;
                ImageStatus.OFFLine = Ysite + 1;  //��ֹ�����¹滮
                break;
            }
        }
        for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         //����
        {
            ImageDeal[Ysite].RightBorder = (ImageDeal[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].RightBorder;
            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
        }
    }
        //�ѳ��� �������
    if (ImageFlag.image_element_rings_flag == 9 || ImageFlag.image_element_rings_flag == 10)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
        }
    }
}

//--------------------------------------------------------------
//  @name           Element_Handle_Right_Rings()
//  @brief          ����ͼ�������Ӻ���������������Բ������.
//  @parameter      void
//  @time           
//  @Author         MRCHEN
//  Sample usage:   Element_Handle_Right_Rings();
//-------------------------------------------------------------
void Element_Handle_Right_Rings()
{
    /****************��СԲ���ж�*****************/
//    if(ImageFlag.ring_big_small == 0)
//    {
//        Less_Big_Small_Num = 0;
//        Black=0;
//        for (int Ysite = 30; Ysite > 0; Ysite--)
//        {
//            if(Pixle[Ysite][77]==0)
//            {
//                Black++;
//            }
//            if(Ysite>2)
//            {
//                if(Pixle[Ysite-1][77]==0 && Pixle[Ysite][77]==1 && Pixle[Ysite-1][77]==1)
//                    break;
//            }
//        }
//        if(Black > 12) //��Բ��
//            ImageFlag.ring_big_small = 1;
//        else          //СԲ��
//            {ImageFlag.ring_big_small = 2;
//             BeeOn;}
//    }
    /****************�ж�*****************/
    int num =0 ;
    for (int Ysite = 55; Ysite > 30; Ysite--)
    {
//        if(ImageDeal[Ysite].RightBoundary_First > 76)
//        {
//            num++;
//        }
        if(ImageDeal[Ysite].IsRightFind == 'W')
            num++;
        if(    ImageDeal[Ysite+3].IsRightFind == 'W' && ImageDeal[Ysite+2].IsRightFind == 'W'
            && ImageDeal[Ysite+1].IsRightFind == 'W' && ImageDeal[Ysite].IsRightFind == 'T')
            break;
    }
        //׼������
    if (ImageFlag.image_element_rings_flag == 1 && num>15)
    {
        ImageFlag.image_element_rings_flag = 2;
    }
    if (ImageFlag.image_element_rings_flag == 2 && num<10)
    {
        ImageFlag.image_element_rings_flag = 5;
        //Stop = 1;
    }
        //�յ�Բ��
    if (ImageFlag.image_element_rings_flag == 4)
    {
        Point_Ysite = 0;
        Point_Xsite = 0;
        for (int Ysite = 30; Ysite > ImageStatus.OFFLine + 2; Ysite--)
        {
            if(ImageDeal[Ysite].IsRightFind == 'W' && ImageDeal[Ysite-1].IsRightFind == 'T')
            {
                Point_Ysite = Ysite;
                Point_Xsite = 0;
                break;
            }
        }
        if(Point_Ysite > 6 && ImageFlag.ring_big_small == 1)
        {
         //   Stop =1;
            ImageFlag.image_element_rings_flag=5;
        }
        if(Point_Ysite > 11 && ImageFlag.ring_big_small == 2)
        {
         //   Stop =1;
            ImageFlag.image_element_rings_flag=5;
        }

    }
        //����
    if(ImageFlag.image_element_rings_flag == 5 && ImageStatus.Miss_Left_lines>15)
    {
      //  Stop = 1;
        ImageFlag.image_element_rings_flag = 6;
    }
        //����СԲ��
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.Miss_Left_lines<7)
    {
        ImageFlag.image_element_rings_flag = 7;
       // Stop=1;


        //����Ŀǰֱ�Ӹ��ƴ�Բ�� ��СԲ���ж�������



//        ImageFlag.ring_big_small = 1 ;
    }
        //���� ��Բ���ж�
    if (ImageFlag.ring_big_small == 1 && ImageFlag.image_element_rings_flag == 7)
    {
        Point_Xsite = 0;
        Point_Ysite = 0;
        for (int Ysite = 45; Ysite > ImageStatus.OFFLine + 7; Ysite--)
        {
            if (    ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 2].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 2].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 1].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 1].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 4].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 4].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 5].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 5].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 6].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 6].LeftBorder
                )

            {
                        Point_Xsite = ImageDeal[Ysite].LeftBorder;
                        Point_Ysite = Ysite;
                        break;
            }
        }
        if (Point_Ysite > 22)
        {
            ImageFlag.image_element_rings_flag = 8;
            //Stop = 1;
        }
    }
        //���� СԲ���ж�
    if (ImageFlag.ring_big_small == 2 && ImageFlag.image_element_rings_flag == 7)
    {
        Point_Xsite = 0;
        Point_Ysite = 0;
        for (int Ysite = 50; Ysite > ImageStatus.OFFLineBoundary+3; Ysite--)
        {
            if (  ImageDeal[Ysite].LeftBoundary > ImageDeal[Ysite + 2].LeftBoundary
               && ImageDeal[Ysite].LeftBoundary > ImageDeal[Ysite - 2].LeftBoundary
              )

            {
                      Point_Xsite = ImageDeal[Ysite].LeftBoundary;
                      Point_Ysite = Ysite;
                      break;
            }
        }
        if (Point_Ysite > 20)
        {
            ImageFlag.image_element_rings_flag = 8;
        }
    }
        //������
    if (ImageFlag.image_element_rings_flag == 8)
    {
         if (   Straight_Judge(1, ImageStatus.OFFLine+15, 50) < 1
             && ImageStatus.Miss_Left_lines < 8
             && ImageStatus.OFFLine < 7)    //�ұ�Ϊֱ���ҽ�ֹ�У�ǰհֵ����С
            {ImageFlag.image_element_rings_flag = 9;

            }
    }

    //����Բ������
    if (ImageFlag.image_element_rings_flag == 9)
    {
        int num=0;
        for (int Ysite = 50; Ysite > 10; Ysite--)
        {
            if(ImageDeal[Ysite].IsRightFind == 'W' )
                num++;
        }
        if(num < 5)
        {
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
            Front_Wait_After_Enter_Ring_Count++;
            gpio_set_level(B0, 0);
        }
    }
    /***************************************����**************************************/
         //׼������  �������
    if (   ImageFlag.image_element_rings_flag == 1
        || ImageFlag.image_element_rings_flag == 2
        || ImageFlag.image_element_rings_flag == 3
        || ImageFlag.image_element_rings_flag == 4)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }

        //����  ����
    if (   ImageFlag.image_element_rings_flag == 5
        || ImageFlag.image_element_rings_flag == 6
       )
    {
        int flag_Xsite_1=0;
        int  flag_Ysite_1=0;
        float Slope_Right_Rings = 0;
        for(Ysite=55;Ysite>ImageStatus.OFFLine;Ysite--)
        {
            for(Xsite=ImageDeal[Ysite].LeftBorder + 1;Xsite<ImageDeal[Ysite].RightBorder - 1;Xsite++)
            {
                if(Pixle[Ysite][Xsite]==1 && Pixle[Ysite][Xsite+1]==0)
                {
                    flag_Ysite_1=Ysite;
                    flag_Xsite_1=Xsite;
                    Slope_Right_Rings=(float)(0-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                    break;
                }
            }
            if(flag_Ysite_1!=0)
            {
              break;
            }
        }
        if(flag_Ysite_1==0)
        {
        for(Ysite=ImageStatus.OFFLine+1;Ysite<30;Ysite++)
        {
         if(ImageDeal[Ysite].IsRightFind=='T'&&ImageDeal[Ysite+1].IsRightFind=='T'&&ImageDeal[Ysite+2].IsRightFind=='W'
               &&abs(ImageDeal[Ysite].RightBorder-ImageDeal[Ysite+2].RightBorder)>10
         )
         {
             flag_Ysite_1=Ysite;
             flag_Xsite_1=ImageDeal[flag_Ysite_1].RightBorder;
             ImageStatus.OFFLine=Ysite;
             Slope_Right_Rings=(float)(0-flag_Xsite_1)/(float)(59-flag_Ysite_1);
             break;
         }

        }

        }
        //����
        if(flag_Ysite_1!=0)
        {
            for(Ysite=flag_Ysite_1;Ysite<60;Ysite++)
            {
                ImageDeal[Ysite].LeftBorder=flag_Xsite_1+Slope_Right_Rings*(Ysite-flag_Ysite_1);
                //if(ImageFlag.ring_big_small==2)//СԲ���Ӱ��
                //    ImageDeal[Ysite].Center=ImageDeal[Ysite].LeftBorder+Half_Bend_Wide[Ysite];//���
//              else//��Բ�����Ӱ��
                    ImageDeal[Ysite].Center=(ImageDeal[Ysite].LeftBorder+ImageDeal[Ysite].RightBorder)/2;//���
                //if(ImageDeal[Ysite].Center>79)
                //    ImageDeal[Ysite].Center=79;
            }
            ImageDeal[flag_Ysite_1].LeftBorder=flag_Xsite_1;
            for(Ysite=flag_Ysite_1-1;Ysite>10;Ysite--) //A���Ϸ�����ɨ��
            {
                for(Xsite=ImageDeal[Ysite+1].LeftBorder+8;Xsite>ImageDeal[Ysite+1].LeftBorder-4;Xsite--)
                {
                    if(Pixle[Ysite][Xsite]==1 && Pixle[Ysite][Xsite-1]==0)
                    {
                     ImageDeal[Ysite].LeftBorder=Xsite;
                     ImageDeal[Ysite].Wide=ImageDeal[Ysite].RightBorder-ImageDeal[Ysite].LeftBorder;
                  //   if(ImageFlag.ring_big_small==2)//СԲ���Ӱ��
                  //       ImageDeal[Ysite].Center=ImageDeal[Ysite].LeftBorder+Half_Bend_Wide[Ysite];//���
                   //  else//��Բ�����Ӱ��
                         ImageDeal[Ysite].Center=(ImageDeal[Ysite].LeftBorder+ImageDeal[Ysite].RightBorder)/2;//���
                   //  if(ImageDeal[Ysite].Center>79)
                   //      ImageDeal[Ysite].Center=79;
                     break;
                    }
                }
                if(ImageDeal[Ysite].Wide>8 && ImageDeal[Ysite].LeftBorder>  ImageDeal[Ysite+2].LeftBorder)
                {
                    continue;
                }
                else
                {
                    ImageStatus.OFFLine=Ysite+2;
                    break;
                }
            }
        }


    }
        //���ڲ�����
    if (ImageFlag.image_element_rings_flag == 7)
    {
//        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
//        {
//            if(ImageFlag.ring_big_small==2)
//                ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Bend_Wide[Ysite];
//            if(ImageDeal[Ysite].Center >= 79)
//            {
//                ImageDeal[Ysite].Center = 79;
//                ImageStatus.OFFLine=Ysite-1;
//                break;
//            }
//        }
    }

        //��Բ������ ����
    if (ImageFlag.image_element_rings_flag == 8 && ImageFlag.ring_big_small == 1)  //��Բ��
    {
        Repair_Point_Xsite = 60;
        Repair_Point_Ysite = 0;
        for (int Ysite = 50; Ysite > 5; Ysite--)
        {
            if (Pixle[Ysite][57] == 1 && Pixle[Ysite-1][57] == 0)
            {
                Repair_Point_Xsite = 57;
                Repair_Point_Ysite = Ysite-1;
                ImageStatus.OFFLine = Ysite + 1;  //��ֹ�����¹滮
                        //  ips200_show_uint(200,200,Repair_Point_Ysite,2);
                break;
            }
        }
        for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         //����
        {
            ImageDeal[Ysite].LeftBorder = (ImageDeal[58].LeftBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].LeftBorder;
            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
        }
    }
        //СԲ������ ����
    if (ImageFlag.image_element_rings_flag == 8 && ImageFlag.ring_big_small == 2)  //СԲ��
    {
        Repair_Point_Xsite = 79;
        Repair_Point_Ysite = 0;
        for (int Ysite = 40; Ysite > 5; Ysite--)
        {
            if (Pixle[Ysite][58] == 1 && Pixle[Ysite-1][58] == 0)
            {
                Repair_Point_Xsite = 58;
                Repair_Point_Ysite = Ysite-1;
                ImageStatus.OFFLine = Ysite + 1;  //��ֹ�����¹滮
                        //  ips200_show_uint(200,200,Repair_Point_Ysite,2);
                break;
            }
        }
        for (int Ysite = 55; Ysite > Repair_Point_Ysite-3; Ysite--)         //����
        {
            ImageDeal[Ysite].LeftBorder = (ImageDeal[58].LeftBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].LeftBorder;
            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
        }
    }
        //�ѳ��� �������
    if (ImageFlag.image_element_rings_flag == 9 || ImageFlag.image_element_rings_flag == 10)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }
}

//--------------------------------------------------------------
//  @name           Element_Judgment_Bend()
//  @brief          ����ͼ���жϵ��Ӻ����������ж������������.
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Judgment_Bend();
//--------------------------------------------------------------
int Miss_Left_Num = 0 ;
int Miss_Right_Num = 0;
void Element_Judgment_Bend()
{
    if(ImageFlag.image_element_rings != 0 || ImageStatus.OFFLine < 14 || ImageFlag.Zebra_Flag
            || ImageFlag.image_element_rings == 1 || ImageFlag.Out_Road == 1 || ImageFlag.RoadBlock_Flag == 1
            || ImageFlag.image_element_rings == 2)
        return;
        //ips114_show_int(20,20,ImageStatus.Miss_Right_lines,3);
        //ips114_show_int(20,40,ImageStatus.Miss_Left_lines,3);
        //ips114_show_int(20,80,ImageStatus.OFFLine,3);

//        if(ImageStatus.WhiteLine_R > 20 && ImageStatus.WhiteLine_L < 5)
//        {
//            ImageFlag.Bend_Road = 1;
//            gpio_set_level(B0, 1);
////            Statu = Bend;
//        }
//
//        if(ImageStatus.WhiteLine_L > 20 && ImageStatus.WhiteLine_R < 5)
//        {
//            ImageFlag.Bend_Road = 2;
//            gpio_set_level(B0, 1);
////            Statu = Bend;
//        }
    if(ImageDeal[ImageStatus.OFFLine+1].LeftBorder > 30
            && ImageStatus.Miss_Left_lines < 4
            && ImageStatus.Miss_Right_lines > 8
            && Straight_Judge(1, ImageStatus.OFFLine+2, 58) > 1)
    {

        ImageFlag.Bend_Road = 1;
        BeeOn;
    }
    if(ImageDeal[ImageStatus.OFFLine+1].RightBorder < 50
            && ImageStatus.Miss_Right_lines < 4
            && ImageStatus.Miss_Left_lines > 8
            && Straight_Judge(2, ImageStatus.OFFLine+2, 58) > 1)
    {

        ImageFlag.Bend_Road = 2;
        BeeOn;
    }
}

//--------------------------------------------------------------
//  @name           Element_Handle_Bend()
//  @brief          ����ͼ�������Ӻ��������������������
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Handle_Bend();
//--------------------------------------------------------------
void Element_Handle_Bend()
{
    if(ImageStatus.OFFLine<10) { ImageFlag.Bend_Road = 0; BeeOff; }
    if(ImageFlag.Bend_Road==1)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Bend_Wide[Ysite];
            if(ImageDeal[Ysite].Center >= 79)
                ImageDeal[Ysite].Center = 79;
        }
    }
    if(ImageFlag.Bend_Road==2)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
            if(ImageDeal[Ysite].Center < 0)
                ImageDeal[Ysite].Center = 0;
        }
    }

//    int Miss_Left_Num = 0 ;
//    int Miss_Right_Num = 0;
//    Miss_Left_Num=0;
//    Miss_Right_Num=0;
//    int Right_Num=0,Left_Num=0;
//    if(ImageStatus.OFFLineBoundary > 20)
//    {
//        for(int Ysite = ImageStatus.OFFLineBoundary + 1;Ysite < ImageStatus.OFFLineBoundary + 11;Ysite++)
//        {
//            if(ImageDeal[Ysite].LeftBoundary > 2)
//                Left_Num++;
//            if(ImageDeal[Ysite].RightBoundary < 77)
//                Right_Num++;
//        }
//    }
//    if(ImageFlag.Bend_Road == 1)
//    {
//        for (int Ysite = 58; Ysite > 30; Ysite--)
//        {
//            if(ImageDeal[Ysite].RightBoundary_First == 78 )
//            {
//                Miss_Right_Num++;
//            }
//        }
//        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
//        {
//            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Bend_Wide[Ysite];
//            if(ImageDeal[Ysite].Center>79)
//            {
//                ImageDeal[Ysite].Center = 79;
//            }
//        }
//    }
//    if(ImageFlag.Bend_Road == 2)
//    {
//        for (int Ysite = 58; Ysite > 30; Ysite--)
//        {
//            if(ImageDeal[Ysite].LeftBoundary_First == 1)
//            {
//                Miss_Left_Num++;
//            }
//        }
//        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
//        {
//            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
//            if(ImageDeal[Ysite].Center < 0)
//            {
//                ImageDeal[Ysite].Center = 0;
//            }
//
//        }
//    }
//    if((ImageStatus.OFFLine < 12
//      &&( (Miss_Right_Num < 20 && ImageFlag.Bend_Road == 1) ||( Miss_Left_Num < 20 && ImageFlag.Bend_Road == 2)))
//      ||(Right_Num>3 && Left_Num>3)
//       )
//    {
//        ImageFlag.Bend_Road = 0;
//        gpio_set_level(C13, 0);
//    }
}
//--------------------------------------------------------------
//  @name           Element_GetStraightLine_Bend()
//  @brief          ����ͼ�������Ӻ��������������������
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Handle_Bend();
//--------------------------------------------------------------
void Element_GetStraightLine_Bend()
{
    if(ImageStatus.OFFLine < 4 && ImageStatus.WhiteLine_R>10 && Straight_Judge(1, 20, 50)<1)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }
    if(ImageStatus.OFFLine < 4 && ImageStatus.WhiteLine_L>10 && Straight_Judge(2, 20, 50)<1)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
        }
    }
}
/*Ԫ���жϺ���*/
void Scan_Element()
{
    if (       ImageFlag.Out_Road == 0   && ImageFlag.RoadBlock_Flag == 0
            && ImageFlag.Zebra_Flag == 0 && ImageFlag.image_element_rings == 0
            && ImageFlag.Ramp == 0       && ImageFlag.Bend_Road == 0
            &&  ImageFlag.straight_long== 0  )
    {
        Statu = Normal;                     //
        Element_Judgment_RoadBlock();       //·��
        Element_Judgment_OutRoad();         //��·
        Element_Judgment_Left_Rings();      //��Բ��
        Element_Judgment_Right_Rings();     //��Բ��
        Element_Judgment_Zebra();           //������
        Element_Judgment_Bend();            //���
        Element_Judgment_Ramp();            //�µ�
        Straight_long_judge();              //��ֱ��
    }
    if(ImageFlag.Bend_Road)
    {
        Element_Judgment_OutRoad();         //��·
        if(ImageFlag.Out_Road)
            ImageFlag.Bend_Road=0;
    }
//    if(ImageFlag.Ramp)
//    {
//        Element_Judgment_OutRoad();         //��·
//        if(ImageFlag.Ramp)
//            ImageFlag.Bend_Road=0;
//    }
    if(ImageFlag.Bend_Road)
    {
        Element_Judgment_RoadBlock();         //·��
        if(ImageFlag.RoadBlock_Flag)
        {
            Bend_Thruough_RoadBlock_Flag = 1;
            if( ImageFlag.Bend_Road == 1 )
                RoadBlock_Thruough_Flag = 2;
            if( ImageFlag.Bend_Road == 2 )
                RoadBlock_Thruough_Flag = 1;
            ImageFlag.Bend_Road=0;
        }
    }
    /*if(ImageFlag.Out_Road)
    {
        Element_Judgment_Ramp();            //�µ�
    }*/
    if(ImageFlag.Bend_Road)
    {
        Element_Judgment_Zebra();           //������
        if(ImageFlag.Zebra_Flag)
            ImageFlag.Bend_Road=0;
    }
}

/*Ԫ�ش�������*/
void Element_Handle()
{
    //Element_GetStraightLine_Bend();//ֱ������

    if(ImageFlag.RoadBlock_Flag == 1)
        Element_Handle_RoadBlock();
    else if(ImageFlag.Out_Road !=0)
        Element_Handle_OutRoad();
    else if (ImageFlag.image_element_rings == 1)
        Element_Handle_Left_Rings();
    else if (ImageFlag.image_element_rings == 2)
        Element_Handle_Right_Rings();
    else if (ImageFlag.Zebra_Flag != 0 )
        Element_Handle_Zebra();
    else if (ImageFlag.Ramp != 0)
        Element_Handle_Ramp();
    else if(ImageFlag.straight_long)
        Straight_long_handle();
    else if(ImageFlag.Bend_Road !=0)
        Element_Handle_Bend();
    else if(ImageStatus.WhiteLine >= 8) //ʮ�ִ���
        Get_ExtensionLine();
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Flag_init
//  @brief          ��־λ��0
//  @parameter      void
//  @time           2023��2��19��
//  @Author
//  Sample usage:   Flag_init();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Flag_init(void)
{
    ImageFlag.Bend_Road = 0;
    ImageFlag.Garage_Location = 0;
    ImageFlag.Ramp = 0;
    ImageFlag.Zebra_Flag = 0;
    ImageFlag.image_element_rings = 0;
    ImageFlag.image_element_rings_flag = 0;
    ImageFlag.straight_xie = 0;
    ImageFlag.straight_long = 0;
    ImageFlag.ring_big_small = 0;
    ImageFlag.RoadBlock_Flag = 0;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Image_Process
//  @brief          ����ͼ��������������������������е�ͼ�����Ӻ���
//  @parameter      void
//  @time           2023��2��19��
//  @Author         
//  Sample usage:   Image_Process();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Image_Process(void)
{
    if (mt9v03x_finish_flag == 1)                         //���һ֡ͼ��ɼ����ˣ���ô�Ϳ��Զ��⸱ͼ����д����ˡ�
    {
        if( RoadBlock_Flag != 1 && RoadBlock_Flag != 2 && RoadBlock_Flag != 3 )
        {
            Image_CompressInit();     �ڳ�ʼ�����              //ͼ��ѹ������ԭʼ��80*188��ͼ��ѹ����60*80��,��Ϊ����Ҫ��ô�����Ϣ��60*80�ܴ����õĻ��Ѿ��㹻�ˡ�
            Get_BinaryImage();                                //ͼ���ֵ���������Ѳɼ�����ԭʼ�Ҷ�ͼ���ɶ�ֵ��ͼ��Ҳ���Ǳ�ɺڰ�ͼ��
            if(ImageFlag.RoadBlock_Flag == 0)
            {
                if(break_road(30)<200 && !ImageFlag.Out_Road)
                    Stop = 1;
                Get_BaseLine();                                   //�Ż�֮��������㷨���õ�һ��ͼ��Ļ������ߣ�Ҳ������������еı�����Ϣ����������������
                Get_AllLine();                                    //�Ż�֮��������㷨���õ�һ��ͼ���ȫ�����ߡ�
                if(!ImageFlag.Ramp && !ImageFlag.RoadBlock_Flag && !ImageFlag.Out_Road)
                    Search_Border_OTSU(Pixle, LCDH, LCDW, LCDH - 2);//58��λ����
                else
                    ImageStatus.OFFLineBoundary = 5;
                Scan_Element();
            }
        }

        Element_Handle();

        if(!ImageFlag.RoadBlock_Flag)
        {
            if( speed < 60 )
            {
                if( ImageStatus.OFFLine <= 22)
                {
                    ImageStatus.Det_True=(ImageDeal[23].Center //���㵱ǰ���
                                         +ImageDeal[24].Center
                                         +ImageDeal[25].Center)/3 - 39;
                }
                else
                    {ImageStatus.Det_True=ImageDeal[ImageStatus.OFFLine+1].Center-39;}

                if(ImageFlag.Bend_Road)//(0)
                {
                    if( ImageStatus.OFFLine <= 23)
                    {
                        ImageStatus.Det_True=((float)ImageDeal[24].Center * 0.22 //���㵱ǰ���
                                             +(float)ImageDeal[25].Center * 0.45
                                             +(float)ImageDeal[26].Center * 0.33) - 39;
                    }
                    else
                        {ImageStatus.Det_True=ImageDeal[ImageStatus.OFFLine+1].Center-39;}
                }
            }
            else if( speed < 65 )
            {
                if( ImageStatus.OFFLine <= 21)
                {
                    ImageStatus.Det_True=(ImageDeal[22].Center //���㵱ǰ���
                                         +ImageDeal[23].Center
                                         +ImageDeal[24].Center)/3 - 39;
                }
                else
                    {ImageStatus.Det_True=ImageDeal[ImageStatus.OFFLine+1].Center-39;}

                if(ImageFlag.Bend_Road)//(0)
                {
                    if( ImageStatus.OFFLine <= 22)
                    {
                        ImageStatus.Det_True=((float)ImageDeal[23].Center * 0.22 //���㵱ǰ���
                                             +(float)ImageDeal[24].Center * 0.45
                                             +(float)ImageDeal[25].Center * 0.33) - 39;
                    }
                    else
                        {ImageStatus.Det_True=ImageDeal[ImageStatus.OFFLine+1].Center-39;}
                }
            }
            else
            {
                if( ImageStatus.OFFLine <= 20)
                {
                    ImageStatus.Det_True=(ImageDeal[21].Center //���㵱ǰ���
                                         +ImageDeal[22].Center
                                         +ImageDeal[23].Center)/3 - 39;
                }
                else
                    {ImageStatus.Det_True=ImageDeal[ImageStatus.OFFLine+1].Center-39;}

                if(ImageFlag.Bend_Road)//(0)
                {
                    if( ImageStatus.OFFLine <= 21)
                    {
                        ImageStatus.Det_True=((float)ImageDeal[22].Center * 0.22 //���㵱ǰ���
                                             +(float)ImageDeal[23].Center * 0.45
                                             +(float)ImageDeal[24].Center * 0.33) - 39;
                    }
                    else
                        {ImageStatus.Det_True=ImageDeal[ImageStatus.OFFLine+1].Center-39;}
                }
            }
            if(ImageFlag.image_element_rings==1)////��
            {
                if( ImageStatus.OFFLine <= 23 )
                {
                    ImageStatus.Det_True=(ImageDeal[24].Center //���㵱ǰ���
                                         +ImageDeal[25].Center
                                         +ImageDeal[26].Center)/3 - 39;
                }
                else
                    {ImageStatus.Det_True=ImageDeal[ImageStatus.OFFLine+1].Center-39;}
            }
            if(ImageFlag.image_element_rings==2)////�һ�
            {
                if( ImageStatus.OFFLine <= 23)
                {
                    ImageStatus.Det_True=(ImageDeal[24].Center //���㵱ǰ���
                                         +ImageDeal[25].Center
                                         +ImageDeal[26].Center)/3 - 39;
                }
                else
                    {ImageStatus.Det_True=ImageDeal[ImageStatus.OFFLine+1].Center-39;}
            }
            if(ImageFlag.Ramp)
                ImageStatus.Det_True=ImageDeal[45].Center - 39;
        }
        else
            ImageStatus.Det_True = 0;


        mt9v03x_finish_flag = 0;
    }
}

