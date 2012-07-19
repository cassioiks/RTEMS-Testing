// integer trig functions by Karl Lager
// 76752,2243@compuserve.com

// The following code is freeware.  You may prune and modify it for
// your own use, but if you share the code please include the origional
// fastint.cpp file.

// 9/12/96
// a slight error in the fastasin and fastacos has been corrected.
// When averaging two numbers, they should shift the sum right, not
// left.  They should work now.
// The high precision trig functions are present, but have not all
// been thouroughly tested.  Caveat hacker.

// 4/9/2003
// Cleaned up, removed asm, and made .c by Matt Cross.

#include <math.h>
#include "fastint.h"

/**************************************************************/
//                 high precision trig

#if 0
long hsinof[1025]; // sin of 0-90 deg
int htanhashtable[1025];
#endif
//


int isinof[360],icosof[360]; //sin&cos <<14
int tanhashtable[129];


void init_trig(void)
{
  double t;int i;

  for (i = 0;i<=128;i++)
   {
     t= i;
     tanhashtable[i] = atan(t/128)*180/M_PI;
   }

#if 0
  for (i = 0;i<=1025;i++)
   {
     t= i;
     htanhashtable[i] = atan(t/1024)*32768/M_PI;
   }
#endif

  for (i=0;i<360;i++)
   {
     t=M_PI/180*i;
     isinof[i] = sin(t)*(1<<14);
     icosof[i] = cos(t)*(1<<14);
   }

#if 0
  for (i=0;i<1025;i++)
   {
     t =M_PI/2048L*i;
     hsinof[i] = sin(t)*65536L;
   }
#endif
}


int fastatan2(long y, long x)
{
  int t,d,a;
  long tan;

  if (x == 0 && y == 0) return 0;//bulletproof

  if (x >= 0)
    {
      if (y >= 0)
	{
	  /* X & Y are positive */
	  if (x>y)
	    {
	      tan = (y<<7)/x;
	      d = 1;
	      a = 0;
	    }
	  else
	    {
	      tan = (x<<7)/y;
	      d = 0;
	      a = 90;
	    }
	}
      else
	{
	  /* X positive, Y negative. */
	  if (x>-y)
	    {
	      tan = -(y<<7)/x;
	      d = 0;
	      a = 360;
	    }
	  else
	    {
	      tan = (x<<7)/-y;
	      d = 1;
	      a = 270;
	    }
	}
    }
  else
    {
      if (y >= 0)
	{
	  /* X negative, Y posative. */
	  if (-x>y)
	    {
	      tan = (y<<7)/-x;
	      d = 0;
	      a = 180;
	    }
	  else
	    {
	      tan = (-x<<7)/y;
	      d = 1;
	      a = 90;
	    }
	}
      else
	{
	  /* Both X & Y are negative. */
	  if (-x>-y)
	    {
	      tan = (y<<7)/x;
	      d = 1;
	      a = 180;
	    }
	  else
	    {
	      tan = (x<<7)/y;
	      d = 0;
	      a = 270;
	    }
	}
    }
  t = tanhashtable[(int)tan];
  if(d)
    t = a+t;
  else
    t = a-t;
  if (t==360)
    t=0;
  return t;
}


int fastasin(int x)
{
  int a,b,i;

  if (x<0)
    {
      a=270;
      b=359;
    }
  else
    {
      a=0;
      b=90;
    }

  while(a < b-1)      //binary search
    {
      i= (a+b)>>1;
      if (isinof[i]<x)
	a=i;
      else
	b=i;
    }
  if (x-isinof[a] > isinof[b]-x)
    return b;
  else
    return a;
}

int fastacos(int x) // 90-fastasin(x)
{
  int a,b,i;
  
  a=0;
 b=180;
  while(a < b-1)      //binary search
    {
      i= (a+b)>>1;
      if (icosof[i]>x)
	a=i;
      else
	b=i;
    }
  if (icosof[a]-x > x-icosof[b])
    return b; //interpolation would go here for
  else
    return a; //higher precision angles
}


unsigned sqrti(unsigned long x)//{float f=x;unsigned i=sqrt(f);return i;}
{
  unsigned long rt;
  long i;

  if (x==0)
    return 0;
  if (x<4)
    return 1;

  for(i=2;(x>>i)>0;i+=2)  // find top bit
    continue;

  rt = x>>(i>>1);
  if (rt == 0xFFFF)
    return (unsigned)rt;

  while (x >= rt*rt+(rt<<1)+1)
   {
     i = (x/rt-rt)>>1;          //(aý+bý)=aý+2ab+bý; cý-aý=2ab+bý
     rt += i;                                //   (b ~= cý-aý)/2a
     while(x<=rt*rt-(rt<<1)+1)
      {
	i = (rt-x/rt)>>1;
        rt -= i;
      }
   }

  if (rt*rt < x)
    {
      if ((x - rt*rt) > (rt*rt + (rt<<1) + 1 - x))
	rt++;
    }
  else
    {
      if
	((rt*rt - x) > (x - (rt*rt - (rt<<1) + 1)))
	rt--;
    }

  return (unsigned)rt;
}

#if 0
// high precision trig functions
// 64k = 1 rev  (65536 points around a circle)

long hsin(unsigned x) // 16.16 fixed point
{ int s;
  int index;
  long sin1;

  if (x > 32768)     // if x > 180 sign is negative
    { s = 1;
      x -=32768;
    }
    else s = 0;

  if (x > 16384)
    x = 32768 - x;   // x is now between 0 and 90
  // return hsinof[x>>4] for no interpolation
  index = x>>4;
  sin1 = hsinof[index];
  if ((x&15)>0)        // that's x%16
  sin1 += (hsinof[index+1]-sin1)*(x&15) >> 4;
  if (s == 0) return sin1;
  else return - sin1;

}

long hcos(unsigned x) // 16.16 fixed point
{ int s;
  int index;
  long sin1;

  x = 16384 - x;     // cos(x) = sin(90-x)

  if (x > 32768)
    { s = 1;
      x -=32768;
    }
    else s = 0;

  if (x > 16384)
    x = 32768 - x;   // x is now between 0 and 90ø (0-16k)
  index = x>>4;
  sin1 = hsinof[index];
  if ((x&15)>0)
  sin1 += (hsinof[index+1]-sin1)*(x&15) >> 4;
  if (s == 0) return sin1;
  else return -sin1;

}

unsigned hfastatan2(long y, long x)
{int d;
 unsigned t,a;
 unsigned f;
 long tan;
 if (x == 0 && y == 0) return 0;
 if (x >= 0)
 { if (y >= 0)
   { if (x>y)
     { tan = (y<<10)/x;
       f = (((y<<10)-tan*x)<<4)/x;
       d = 1; a = 0;
     }
     else
     { tan = (x<<10)/y;
       f = (((x<<10)-tan*y)<<4)/y;
       d = 0; a = 16384;}
   }
   else
   { if (x>-y)
     { tan = -(y<<10)/x;
       f = (((-y<<10)-tan*x)<<4)/x;
       d = 0; a = 0; //== 65536;
     }
     else
     { tan = (x<<10)/-y;
       f = (((x<<10)+tan*y)<<4)/-y;
       d = 1; a = 49152;
     }
   }
 }
 else
 { if (y >= 0)
   { if (-x>y)
     { tan = (y<<10)/-x;
       f = (((y<<10)+tan*x)<<4)/-x;
       d = 0; a = 32768;
     }
     else
     { tan = (-x<<10)/y;
       f = (((-x<<10)-tan*y)<<4)/y;
       d = 1; a = 16384;
     }
   }
   else
   { if (-x>-y)
     { tan = (y<<10)/x;
       f = (((y<<10)-tan*x)<<4)/x;
       d = 1; a = 32768;
     }
     else
     { tan = (x<<10)/y;
       f = (((x<<10)-tan*y)<<4)/y;
       d = 0; a = 49152;}
   }
 }
 t = tanhashtable[(int)tan];
                               // add fraction for interpolation
 if (f>0) t += (tanhashtable[(int)tan+1]-t)*f >> 4;
 if(d) t = a+t;
 else t = a-t;
// if (t==65536) t=0; // handled by wraparound
 return t;
 }


unsigned hfastasin(long x)   // x is in the range -64k to 64k
{ int a,b,i,s,result;
 if (x<0)
  {s = 1;x = -x;} //{a=270;b=359;}
 else
  s = 0;

  a=0;b=1024;
 while(a<b-1)      //binary search
 { i= (a+b)>>1;
   if (hsinof[i]<x)
     a=i;
   else
     b=i;
 }
 // if (x-isinof[a] > isinof[b]-x) return b;
                   // interpolate
 result = (a<<4) + ((x-hsinof[a])<<4) / (hsinof[b]-hsinof[a]);

 if (s==0) return result;
 else return -result;
}


unsigned hfastacos(long x)   // x is in the range -64k to 64k
{ int a,b,i,s,result;
 if (x<0)
  {s = 1;x = -x;} //{a=270;b=359;}
 else
  s = 0;

  a=0;b=1024;
 while(a<b-1)      //binary search
 { i= (a+b)>>1;
   if (hsinof[i]<x)
     a=i;
   else
     b=i;
 }
 // if (x-isinof[a] > isinof[b]-x) return b;
                   // interpolate
 result = (a<<4) + ((x-hsinof[a])<<4) / (hsinof[b]-hsinof[a]);

 if (s==0) result = 16384 - result;
 else result = 16384 + result;
 return result;
}


// And finally, a quick way to get integers out of the coprocessor..

const double d2i = 6755399441055744.0;
const float f2i = 12582912.0;
double tempdub;
float tempfl;
#define fl2int(x,i)asm{fld dword ptr x; fadd f2i; fstp tempfl; fwait; mov eax,dword ptr tempfl;sub eax,f2i;mov i,eax}
#define dub2int(x,i) asm{fld qword ptr x; fadd d2i; fstp tempdub; fwait; mov eax,dword ptr tempdub;mov i,eax}

long float2int(double x)
 { long i;
   dub2int(x,i);
   return i;
 }

#endif
