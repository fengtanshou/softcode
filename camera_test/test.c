/*************************************************************************
  > File Name: /home/zhanhongluo/luoDoc/camera_test/test.c
  > Author: luozh05
  > Mail: luozh05@163.com 
  > Created Time: 2018年03月28日 星期三 11时07分03秒
 ************************************************************************/

#include<stdio.h>
#include<math.h>
#include<string.h>

int convert(int a,char *p)//a为字符串长度，p为需要转换字符串指针
{ 
    int i,j,sum;

    sum=0;
    for(i=0;i<a;i++)
    {  
        if(*(p+i)<='f'&&*(p+i)>='a') 
            j=(int)(*(p+i))-87; 
        else if(*(p+i)<='F'&&*(p+i)>='A') 
            j=(int)(*(p+i))-55;
        else   
            j=(int)(*(p+i))-48; 
        sum=sum+pow(16.0,a-i-1)*j;

    }
    return (sum);

} 
int  main()
{
    char str[512]="2b5";
    printf("%d\n",convert(3,str));
    return 0;
}
