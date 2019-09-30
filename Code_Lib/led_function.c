#include<stdio.h>
#include <stdlib.h>
#define MAX              16             //随机数最大值
#define MIN              1              //随机数最小值
#define creat_random() (rand() % (MAX + 1 - MIN)+ MIN)

typedef 	unsigned int	u16;
typedef 	unsigned char	u8;
u8 SEND_ID[] ="MJX";
#define aaaa(x)  sizeof(x)/sizeof(u8)
#if 0
unsigned int random()
{
  unsigned int value;
  value = rand() % (MAX + 1 - MIN)+ MIN;               //获取一个随机数(1-16)
  return value;
}
	u16 i =0;
	while(1){
		if(++i<200){
		printf("i= %d,a =%d\n",i,creat_random());
		
		}
	if(i == 200)i=201;
	}
#endif
#define HC595_IC_NUM_MAX 100
#define HC595_IC_NUM_MIN 9
#define mod(a,b)	a/b  //求模
#define rem(a,b)	a%b  //求余
#define odd(x)      rem(x,2)  //奇数判断
#define setbit(x,y) x|=(1<<y)         //将X的第Y位置1
#define clrbit(x,y) x&=~(1<<y)        //将X的第Y位清0
#define reversebit(x,y)  x^=(1<<y)    //将x的第Y位取反
u8 LED_DISPLAY_SPEED;
u8 LED_CLOCK;
u8  display_buffer[HC595_IC_NUM_MAX]={0};
u8 val_to_Lbit(u8 num)
{
	u8 val =0,i=0;
	for(i=0;i<num;i++){
		val |= 1<<i;
	}
	return val;
}
u8 val_to_Hbit(u8 num)
{
    u8 val = 0,i=0;
	for(i=0;i<=num;i++){
		val |=1<<(7-i);
	}
	return val;
}
//1<----------------------2(0-100)
void led_ctrl_left1(u8 num2,u8 num1)//num2>num1
{
	u16 num=0,i,a,b;
	num = num2-num1+1;
#if 0
	for(i=0;i< num * 8;i++)
	{
		a = num2 - mod(i,8);
		b = rem(i,8) ;
		display_buffer[a] = val_to_Hbit(b);//1-------------2

		printf("display_buffer[%d]=%x \n",a,display_buffer[a]);
	}
#endif
	if(i< num * 8)
	{
		a = num2 - mod(i,8);
		b = rem(i,8) ;
		display_buffer[a] = val_to_Hbit(b);//1-------------2
		printf("display_buffer[%d]=%x \n",a,display_buffer[a]);
	}
	else
		i=0;
	if(LED_CLOCK){
		i++;
	}	
}
void led_ctrl_left0(u8 num2,u8 num1)//num2>num1
{
	u16 num=0,i,a,b;
	num = num2-num1+1;
#if 0
	for(i=0;i< num * 8;i++)
	{
		a = num2 - mod(i,8);
		b = rem(i,8) ;
		display_buffer[a] = ~val_to_Hbit(b);//1-------------2

		printf("display_buffer[%d]=%x \n",a,display_buffer[a]);
	}
#endif
	if(i< num * 8)
	{
		a = num2 - mod(i,8);
		b = rem(i,8) ;
		display_buffer[a] = ~val_to_Hbit(b);//1-------------2
		printf("display_buffer[%d]=%x \n",a,display_buffer[a]);
	}
	else
		i=0;
	if(LED_CLOCK){
		i++;
	}
	
}
//1---------------------->2(0-100)
void led_ctrl_right(u8 num1,u8 num2)//num1<num2
{
	u16 num=0,i,a,b;
	num = num2-num1+1;
#if 0
	for(i=0;i< num * 8;i++)
	{
		a = mod(i,8)+num1;
		b = rem(i,8)+1;
		display_buffer[a] = val_to_Lbit(b);//1-------------2
		printf("display_buffer[%d]=%x \n",a,display_buffer[a]);
	}	
#endif
	if(i< num * 8)
	{
		a = mod(i,8)+num1;
		b = rem(i,8)+1;
		display_buffer[a] = val_to_Lbit(b);//1-------------2
		printf("display_buffer[%d]=%x \n",a,display_buffer[a]);
	}	
	else
		i = 0;
	if(LED_CLOCK){
		i++;
	}
}
/****************************
1------------>2<------------3

****************************/
#if 1
void LED1(u16 num)
{
	u16 i,k;
	u8 a,b;
	u8 flag = 0;
	
	if(odd(num))//奇数判断
	{
		k = mod((num-1),2);
		flag = 1;
		printf("this num is odd  number = %d\n",k);
	}
	for(i=0;i<(num*8/2);i++)
	{	
		a = mod(i,8);
		b = rem(i,8)+1;
		display_buffer[a] = val_to_Lbit(b);//1-------------2
		printf("display_buffer[%d]=%x ;",a,display_buffer[a]);
		a = (num-1) - a;
		b = rem(i,8);
		if(flag)
			display_buffer[k-1] |= val_to_Hbit(k);//奇数相同数组相或
		else
			display_buffer[a] = val_to_Hbit(b);//3-------------2	
		printf("display_buffer[%d]=%x\n",a,display_buffer[a]);
	}
}
/****************************
1<------------2------------>3

****************************/
void LED2(u16 num)
{
	u16 i,k;
	u8 a,b;
	u8 flag = 0;
	
	if(odd(num))//奇数判断
	{
		k = mod((num-1),2);
		flag = 1;
		printf("this num is odd  number = %d\n",k);
	}
	for(i=0;i<(num*8/2);i++)
	{	
		if(i<4&&flag){
			display_buffer[k] = val_to_Hbit(i);
			display_buffer[k] |= val_to_Lbit(i+1);
			printf("display_buffer[%d]=%x \n;",k,display_buffer[k]);
		}
		else{
		a = mod(i,8);
		b = rem(i,8);
		a=num/2-a-1;
		display_buffer[a] = val_to_Hbit(b);//1<-------------2
		printf("display_buffer[%d]=%x ;",a,display_buffer[a]);

		a = (num/2)+mod(i,8);
		b = rem(i,8)+1;
		display_buffer[a] = val_to_Lbit(b);//2------------->3
		printf("display_buffer[%d]=%x \n",a,display_buffer[a]);
		}

	}	
}
#endif
void main(void)
{
	u16 i=0;
	for(i=0;i <= HC595_IC_NUM_MAX;i++){
		display_buffer[i]=0xff;
	}
	for(i=0;i<=HC595_IC_NUM_MAX;i++){
		printf("display_buffer[%d]=%d\n",i,display_buffer[i]);
	}
	for(i=0;i<=8;i++){
	//	printf("val = %x\n",val_to_Lbit(i));
	}
	//for(i=0;i<=8;i++){
	//	printf("val = %x\n",val_to_Hbit(i));
	//}
	printf("\n");
	//LED2(10);
	for(i=0;i<16;i++){
		LED_CLOCK = 1;
		led_ctrl_left1(3,1);
		//led_ctrl_right(16,25);
		//sleep(1000);
		LED_CLOCK =0;	
	}
	printf("\n");
	for(i=0;i<16;i++){
		LED_CLOCK = 1;
		led_ctrl_left0(3,1);
		//led_ctrl_right(16,25);
		//sleep(1000);
		LED_CLOCK =0;	
	}
	printf("lengh = %ld\n",aaaa(SEND_ID));
}
