/*
 * image_interface.c
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "darknet_ros/image_interface.h"

static float get_pixel(image m, int x, int y, int c) //获取某个像素的值
{
  assert(x < m.w && y < m.h && c < m.c);
  return m.data[c * m.h * m.w + y * m.w + x];
}

image **load_alphabet_with_file(char *datafile) //加载字母表
{
  int i, j;
  const int nsize = 8;
  image **alphabets = calloc(nsize, sizeof(image));                    //分配字母表空间
  char *labels = "/labels/%d_%d.png";                                  //字母表的文件名格式
  char *files = (char *)malloc(1 + strlen(datafile) + strlen(labels)); //分配字母表文件名空间
  strcpy(files, datafile);
  strcat(files, labels);
  for (j = 0; j < nsize; ++j)
  {
    alphabets[j] = calloc(128, sizeof(image)); //分配字母表中每个字母的空间
    for (i = 32; i < 127; ++i)
    {
      char buff[256];
      sprintf(buff, files, i, j);
      alphabets[j][i] = load_image_color(buff, 0, 0); //加载字母表中每个字母的图片
    }
  }
  return alphabets;
}

#ifdef OPENCV                                //如果使用opencv，则使用opencv的图像加载函数
void generate_image(image p, IplImage *disp) //生成图像
{
  int x, y, k;
  if (p.c == 3)
    rgbgr_image(p); //将图像转换为BGR格式
  // normalize_image(copy);

  int step = disp->widthStep; //获取图像的一行的字节数
  for (y = 0; y < p.h; ++y)
  {
    for (x = 0; x < p.w; ++x)
    {
      for (k = 0; k < p.c; ++k)
      {
        disp->imageData[y * step + x * p.c + k] = (unsigned char)(get_pixel(p, x, y, k) * 255); //将图像的每个像素的值转换为unsigned char类型，并存储到图像的imageData中
      }
    }
  }
}
#endif
