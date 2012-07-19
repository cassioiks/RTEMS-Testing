/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id: imu-1d-fixed.c,v 1.3 2003/03/30 03:28:18 profesor Exp $
 *
 * (c) Aaron Kahn <Aaron.Kahn@itt.com>
 * (c) Trammell Hudson <hudson@rotomotion.com>
 *
 * One dimensional IMU to compute pitch angle based on the readings
 * from an ADXL202 and a pitch gyro.
 * 
 *  plot [200:1000] \
 *	"kalman.log" using 1 title "Actual state" with lines, \
 *	"" using 2 title "Estimated state" with lines, \
 *	"" using 3 title "Error" with lines
 *
 *************
 *
 *  This file is part of the autopilot simulation package.
 *
 *  Autopilot is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Autopilot is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Autopilot; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

typedef long f16_16;

#define		pi		  205887	/* 3.14159 as a 16.16 fixed */
const f16_16	dt		= 2185;		/* 1/30 as a 16.16 fixed
						   (30 Hz sensor update) */
const f16_16	omega		= 32768;	/* 0.5 as a 16.16 fixed
						   (Max roll rate) */
const f16_16	Q		= 66;		/* 0.001 as a 16.16 fixed
						   (Noise weighting matrix) */
const f16_16	max_angle	= 34315;	/* 30*pi/180.0 as a 16.16 fixed
						   (Maximum pitch angle) */
static f16_16	theta		= 4547;		/* 4*pi/180 as a 16.16 fixed
						   (Our initial state estimate) */
static f16_16	R		= 65536;	/* 1 as a 16.16 fixed
						   (Measurement error weight) */
static f16_16	P		= 655360;	/* 1 as a 16.16 fixed
						   (Covariance matrix) */
#define add_f16_16(a,b) ((a) + (b))
#define sub_f16_16(a,b) ((a) - (b))

f16_16
mult_f16_16(f16_16 a, f16_16 b)
{
#if 1
  int a_frac;
  int a_int;
  int b_frac;
  int b_int;
  int sign = 1;

  if (a < 0) {
    sign = -sign;
    a = -a;
  }
  if (b < 0) {
    sign = -sign;
    b = -b;
  }
  a_frac = a & 0x0000ffff;
  a_int  = (a & 0xffff0000) / 65536;
  b_frac = b & 0x0000ffff;
  b_int  = (b & 0xffff0000) / 65536;

  return (((unsigned)(a_frac * b_frac) >> 16) +
	  (a_frac * b_int) +
	  (a_int * b_frac) +
	  ((a_int * b_int) << 16)) * sign;
#else
	int a_frac = a & 0x0000ffff;
	int a_int  = (int) (a & 0xffff0000) / 65536;
	int b_frac = b & 0x0000ffff;
	int b_int  = (int) (b & 0xffff0000) / 65536;

	return ((unsigned)(a_frac * b_frac) >> 16) +
		(a_frac * b_int) +
		(a_int * b_frac) +
		((a_int * b_int) << 16);
#endif
}

/* Loses some precision in the fractional part of the divide, but may
   be good enough. */
f16_16
div_f16_16(f16_16 a, f16_16 b)
{
	return (((a / b) << 16) +
		(((a % b) << 8) / (b >> 8)));
}

void
print_f16_16(f16_16 a)
{
	if (a < 0) {
		printf ("-");
		a = -a;
	}
	printf("%d.%04d", a / 65536, ((a % 65536) * 10000) / 65536);
}

double
double_from_f16_16(f16_16 a)
{
	return ((double)a / 65536);
}

f16_16
f16_16_from_double(double a)
{
	return ((f16_16)(a * 65536));
}

f16_16
kalman(
	f16_16			t,		/* Time */
	f16_16			q,		/* Pitching gyro */
	double			ax,		/* X acceleration */
	double			ay		/* Y acceleration */
)
{
	f16_16			Pdot;		/* Derivative of P */
	f16_16			E;		/* ? */
	f16_16			K;		/* ? */

	f16_16			theta_m;	/* Our state measurement */

						/* A = 0 */
	Pdot = Q;				/* Pdot = A*P + P*A' + Q */
	P = add_f16_16 (P, mult_f16_16 (Pdot, dt));

	/* Update our state estimate from the rate gyro */
	theta = add_f16_16 (theta, mult_f16_16 (q, dt));

	/* We only run the Kalman filter at a slower 10 Hz */
	if( (int)( t * 100 / 65536 ) % 10 != 0 )
		return theta;

	/* Compute our measured state from the accelerometers */
	theta_m = f16_16_from_double(atan2( -ay, ax ));

	E = add_f16_16 (P, R);				/* E = CPC' + R */
	K = div_f16_16 (P, E);				/* K = PC'inv(E) */

	/* Update the state */
	theta = add_f16_16 (theta, mult_f16_16 (K, sub_f16_16(theta_m, theta )));

	/* Covariance update */
	P = add_f16_16 (mult_f16_16 (P, mult_f16_16 (sub_f16_16 (1 * 65536, K),
						     sub_f16_16 (1 * 65536, K))),
			mult_f16_16 (R, mult_f16_16 (K, K)));

	return theta;
}

static inline double
noise( void )
{
	return 2.0 * drand48() - 1.0;
}


int main( void )
{
	f16_16 t;
	time_t now = time(0);

	f16_16 theta_degs, real_theta_degs;
	f16_16 q_degs, real_q_degs;

	srand48( now );

	for( t=0 ; t<(600 * 65536) ; t+=dt )
	{
		/*
		 * Compute our actual state as a function of time.
		 */
		f16_16 real_q = mult_f16_16 (max_angle,
					     mult_f16_16 (omega,
							  f16_16_from_double(cos(double_from_f16_16(mult_f16_16(omega,t))))));
		f16_16 real_theta = mult_f16_16(max_angle,
						f16_16_from_double (sin(double_from_f16_16(mult_f16_16(omega,t)))));

		/*
		 * Fake our sensor readings by adding a little bit of noise
		 * to the system.
		 */
		double ax =  cos( double_from_f16_16(real_theta) ) + noise() * 0.9;
		double ay = -sin( double_from_f16_16(real_theta) ) + noise() * 0.9;
		f16_16 q = add_f16_16 (real_q,
				       f16_16_from_double (noise() * 8 * 3.14159 / 180.0));

		/* test if the filter can handle a fixed offset (IE gyro drift) */
		q = add_f16_16(q, pi/1800);

		/*
		 * Compute our new estimated state with the Kalman filter
		 */
		theta = kalman( t, q, ax, ay );

		print_f16_16 (t);
		printf (": ");
		print_f16_16 (real_theta);
		printf(" ");
		print_f16_16 (theta);
		printf(" ");
		print_f16_16 (sub_f16_16 (real_theta, theta));
		printf("   ");

		theta_degs = div_f16_16(theta * 180, pi);
		real_theta_degs = div_f16_16(real_theta * 180, pi);

		print_f16_16 (real_theta_degs);
		printf(" ");
		print_f16_16 (theta_degs);
		printf(" ");
		print_f16_16 (sub_f16_16 (real_theta_degs, theta_degs));

		q_degs = div_f16_16(q * 180, pi);
		real_q_degs = div_f16_16(real_q * 180, pi);

		printf(" q:");
		print_f16_16 (q_degs);
		printf(" rq:");
		print_f16_16 (real_q_degs);

		printf("\n");

#if 0
		if (abs(real_theta - theta) > 65536) {
			printf ("Argh!  too much difference!\n");
			printf ("seed = %d\n", now);
			exit(1);
		}
#endif
	}

	return 0;
}
