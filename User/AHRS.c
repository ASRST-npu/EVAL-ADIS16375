#include "AHRS.h"
#include "main.h"
#include "math.h"
#include "stdio.h"

float g[3]={0,0,0};
float array_gx[12],array_gy[12],array_gz[12];
float array_ax[12],array_ay[12],array_az[12];
uint16_t i;
extern float a_x,a_y,a_z;
extern float g_x,g_y,g_z;
extern uint16_t IRQ_n;

extern int n;
extern int m;

float acc[3][3];
float gyro[3][3];
float Vx,Vy,Vz;
float X,Y,Z;
float v2[3]={1,2,2};
float v[3];
float q[4]={1,0,0,0};
int N_mean=2500;
float Q[4]={1,0,0,0},V[3]={0,0,0},P[3]={0,0,0},Tm=4.0/2460;
float Altitude[3];
float theta;
float psi;
float gama;
float p0,p1,p2,p3;

static float q0=1.0f,q1=0.0f, q2 = 0.0f, q3 = 0.0f;

void AHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
		float beta = 0.1,sampleFreq=410;
    float s0, s1, s2, s3;
	  float r11,r21,r31,r32,r33;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz)/57.3f;
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy)/57.3f;
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx)/57.3f;
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx)/57.3f;
    
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;   
        
        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;
        
        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }
    
    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 / sampleFreq;
    q1 += qDot2 / sampleFreq;
    q2 += qDot3 / sampleFreq;
    q3 += qDot4 / sampleFreq;
    
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
				
		r11 = 2* q0*q0 -1 +2*q1*q1;
    r21 = 2*(q1*q2    -  q0*q3);
    r31 = 2*(q1*q3    +  q0*q2);
    r32 = 2*(q2*q3    -  q0*q1);
    r33 = 2* q1*q1 -1 +2*q3*q3;

    gama = 57.3f*atan2(r32, r33 ); // rotate around 
    theta = -57.3f*atan(r31 / sqrt(1-r31*r31) );
    psi = 57.3f*atan2(r21, r11 );
}

float invSqrt(float x) 
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}



void jiaji (float acc[3][3])//
{
    //
    float pi=3.14;
    float sum_fx=0,sum_fy=0,sum_fz=0;
	  float fx_mean;
	  float fy_mean;
	  float fz_mean;
	  float gamaq;
	  float gama0;
	  float theta0;
	  float psi0;
	  float pp;
	  int i;
	  for (i=0;i<3;i++)
    {
        sum_fx=sum_fx+acc[i][0];
        sum_fy=sum_fy+acc[i][1];
        sum_fz=sum_fz+acc[i][2];
    }
    fx_mean=sum_fx/3;
    fy_mean=sum_fy/3;
    fz_mean=sum_fz/3;				
    //
    gamaq=-atan(-fx_mean/fz_mean);
    if (fz_mean>0)
    {
        gama0=gamaq;
    }
    else 
    {
        if (gamaq<0)
        {
            gama0=gamaq+pi;
        }
        else
        {
            gama0=gamaq-pi;
        }
    }
    //
    //theta0=fabs(atan(fy_mean/(sqrt(pow(fx_mean,2)+pow(fz_mean,2)))));
		theta0=fabs(atan(fy_mean/9.8));
    psi0=0;
    p0=cos(psi0/2)*cos(theta0/2)*cos(gama0/2)+sin(psi0/2)*sin(theta0/2)*sin(gama0/2);
    p1=cos(psi0/2)*cos(theta0/2)*sin(gama0/2)-sin(psi0/2)*sin(theta0/2)*cos(gama0/2);
    p2=cos(psi0/2)*sin(theta0/2)*cos(gama0/2)+sin(psi0/2)*cos(theta0/2)*sin(gama0/2);
    p3=sin(psi0/2)*cos(theta0/2)*cos(gama0/2)-cos(psi0/2)*sin(theta0/2)*sin(gama0/2);
		//
	  pp=sqrt(p0*p0+p1*p1+p2*p2+p3*p3);
		p0=p0/pp;
		p1=p1/pp;
		p2=p2/pp;
		p3=p3/pp;
}
void update_array()
{
	g_x=g_x+0.2881f;
	g_y=g_y+0.015f;
	g_z=-(g_z-0.0f);
	a_z=-a_z;
	array_gx[IRQ_n] = g_x;
	array_gy[IRQ_n] = g_y;
	array_gz[IRQ_n] = g_z;
  array_ax[IRQ_n] = a_x;
	array_ay[IRQ_n] = a_y;
	array_az[IRQ_n] = a_z;
}
void dataprocess()
{
	gyro[0][0] = (array_gx[0]+array_gx[1]+array_gx[2]+array_gx[3])/4/180*3.14;
	gyro[1][0] = (array_gx[4]+array_gx[5]+array_gx[6]+array_gx[7])/4/180*3.14;
	gyro[2][0] = (array_gx[8]+array_gx[9]+array_gx[10]+array_gx[11])/4/180*3.14;
	
	gyro[0][1] = (array_gy[0]+array_gy[1]+array_gy[2]+array_gy[3])/4/180*3.14;
	gyro[1][1] = (array_gy[4]+array_gy[5]+array_gy[6]+array_gy[7])/4/180*3.14;
	gyro[2][1] = (array_gy[8]+array_gy[9]+array_gy[10]+array_gy[11])/4/180*3.14;
	
	gyro[0][2] = (array_gz[0]+array_gz[1]+array_gz[2]+array_gz[3])/4/180*3.14;
	gyro[1][2] = (array_gz[4]+array_gz[5]+array_gz[6]+array_gz[7])/4/180*3.14;
	gyro[2][2] = (array_gz[8]+array_gz[9]+array_gz[10]+array_gz[11])/4/180*3.14;

	acc[0][0] = (array_ax[0]+array_ax[1]+array_ax[2]+array_ax[3])/4;
	acc[1][0] = (array_ax[4]+array_ax[5]+array_ax[6]+array_ax[7])/4;
	acc[2][0]= (array_ax[8]+array_ax[9]+array_ax[10]+array_ax[11])/4;
	
	acc[0][1] = (array_ay[0]+array_ay[1]+array_ay[2]+array_ay[3])/4;
	acc[1][1] = (array_ay[4]+array_ay[5]+array_ay[6]+array_ay[7])/4;
	acc[2][1]= (array_ay[8]+array_ay[9]+array_ay[10]+array_ay[11])/4;
	
	acc[0][2] = (array_az[0]+array_az[1]+array_az[2]+array_az[3])/4;
	acc[1][2]= (array_az[4]+array_az[5]+array_az[6]+array_az[7])/4;
	acc[2][2] = (array_az[8]+array_az[9]+array_az[10]+array_az[11])/4;

}
float _dtheta(int i,int j,float Tm)
{
	float dtheta;
	dtheta=gyro[i][j];
	return dtheta*Tm;
}
float _dv(int i,int j,float Tm)
{
	float dv;
	dv=acc[i][j];
	return dv*Tm*9.8;
}
float norm(float Phi[])
{
	float dphi=0;
	int i;
	for ( i=0;i<3;i++)
	{
		dphi=dphi+Phi[i]*Phi[i];
	}
	dphi=sqrt(dphi);
	return dphi;
}
void cross(float v1[],float v2[],float v[])
{
	//
	v[0]=v1[1]*v2[2]-v1[2]*v1[1];
	v[1]=v1[2]*v2[0]-v1[0]*v2[2];
	v[2]=v1[0]*v2[1]-v1[1]*v2[0];
}
void qmul(const float q0[],const float q1[],float q[])
{
	//
	q[0]=q0[0] * q1[0] - q0[1] * q1[1] - q0[2] * q1[2] - q0[3] * q1[3];
	q[1]=q0[0] * q1[1] + q0[1] * q1[0] + q0[2] * q1[3] - q0[3] * q1[2];
	q[2]=q0[0] * q1[2] + q0[2] * q1[0] + q0[3] * q1[1] - q0[1] * q1[3];
	q[3]=q0[0] * q1[3] + q0[3] * q1[0] + q0[1] * q1[2] - q0[2] * q1[1];
}
void cross4(const float q0[],float dVsfm[])
{
	float q1[4],q2[4],q0_[4];

	q1[0]=0;
	q1[1]=dVsfm[0];
	q1[2]=dVsfm[1];
	q1[3]=dVsfm[2];
	qmul(q0,q1,q2);
	q0_[0]=q0[0];
	q0_[1]=-q0[1];
	q0_[2]=-q0[2];
	q0_[3]=-q0[3];
	qmul(q2,q0_,q1);
	dVsfm[0]=q1[1];
	dVsfm[1]=q1[2];
	dVsfm[2]=q1[3];
}
void r2q(float Phi[],float q1[])
{
	float dphi;
	int i;
	dphi=norm(Phi);
	
	if (dphi<0.0001)
	{
		q1[0]=cos(dphi/2);
		q1[1]=Phi[0]/2;
		q1[2]=Phi[1]/2;
		q1[3]=Phi[2]/2;
	}
	else
	{
		q1[0]=cos(dphi/2);
		for( i=1;i<4;i++)
			q1[i]=Phi[i-1]/dphi*sin(dphi/2);
	}
}

void Update_INS(float Q[],float V[],float P[],float Tm,float epsilon[],float nabla[])
{
	//
	float Phi[3],theta[3][3],q0[4],q1[4],dv[3][3];	//
	int i,j;
	float dthetam[3]={0,0,0};
  float dVm[3]={0,0,0};
	float cross_dvdtheta[6][3],dVsclm[3],dVrot[3],dVsfm[3];
	for (i=0;i<4;i++)
	{
		q0[i]=Q[i];
	}
	for(i=0;i<3;i++)
	{	for(j=0;j<3;j++)
		{
			theta[i][j]=_dtheta(i,j,Tm)-epsilon[j];
			dv[i][j]=_dv(i,j,Tm)-nabla[j];
		}
	}

	Phi[0]=theta[0][0]+theta[1][0]+theta[2][0]+9.0/20*(theta[0][1]*theta[2][2]-theta[2][1]*theta[0][2])+27.0/40*(theta[1][1]*(theta[2][2]-theta[0][2])-theta[1][2]*(theta[2][1]-theta[0][1]));
	Phi[1]=theta[0][1]+theta[1][1]+theta[2][1]+9.0/20*(theta[0][2]*theta[2][0]-theta[0][0]*theta[2][2])+27.0/40*(theta[1][2]*(theta[2][0]-theta[0][0])-theta[1][0]*(theta[2][2]-theta[0][2]));
	Phi[2]=theta[0][2]+theta[1][2]+theta[2][2]+9.0/20*(theta[0][0]*theta[2][1]-theta[0][1]*theta[2][0])+27.0/40*(theta[1][0]*(theta[2][1]-theta[0][1])-theta[1][1]*(theta[2][0]-theta[0][0]));
	r2q(Phi,q1);
	qmul(q0,q1,Q);

	/*-------------------------------------????--------------------------------------------------*/
	cross(theta[0],dv[2],cross_dvdtheta[0]);
	cross(dv[0],theta[2],cross_dvdtheta[1]);
	cross(theta[0],dv[1],cross_dvdtheta[2]);
	cross(theta[1],dv[2],cross_dvdtheta[3]);
	cross(dv[0],theta[1],cross_dvdtheta[4]);
	cross(dv[1],theta[2],cross_dvdtheta[5]);
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
		{
			dthetam[i]=dthetam[i]+theta[j][i];
			dVm[i]=dVm[i]+dv[j][i];
		}
		cross(dthetam,dVm,dVrot);
		for (i=0;i<3;i++)
		{
			dVsclm[i]=9.0/20*(cross_dvdtheta[0][i]+cross_dvdtheta[1][i])+27.0/40*(cross_dvdtheta[2][i]+cross_dvdtheta[3][i]+cross_dvdtheta[4][i]+cross_dvdtheta[5][i]);
			dVsfm[i]=dVm[i]+0.5*dVrot[i]+dVsclm[i];
		}
		cross4(Q,dVsfm);
		for (i=0;i<3;i++)
		{
			V[i]=V[i]+dVsfm[i]+g[i]*Tm;
		}
		/*---------------------------????-----------------------------------*/
		for (i=0;i<3;i++)
		{
			P[i]=P[i]+V[i]*Tm;
		}

}
void q2Cbn(float Q[],float Cbn[][3])
{
	Cbn[0][0]=(1-2*(Q[2]*Q[2]+Q[3]*Q[3]));
	Cbn[0][1]=2*(Q[1]*Q[2]-Q[0]*Q[3]);
	Cbn[0][2]=2*(Q[1]*Q[3]+Q[0]*Q[2]);
	Cbn[1][0]=2*(Q[1]*Q[2]+Q[0]*Q[3]);
	Cbn[1][1]=1-2*(Q[1]*Q[1]+Q[3]*Q[3]);
	Cbn[1][2]=2*(Q[2]*Q[3]-Q[0]*Q[1]);
	Cbn[2][0]=2*(Q[1]*Q[3]-Q[0]*Q[2]);
	Cbn[2][1]=2*(Q[2]*Q[3]+Q[0]*Q[1]);
	Cbn[2][2]=1-2*(Q[1]*Q[1]+Q[2]*Q[2]);
}
void q2atitude(float Q[],float Atitude[])
{
	float Cbn[3][3];
	q2Cbn(Q,Cbn);
	Atitude[0]=asin(Cbn[2][1])*57.3;
	if (fabs(Cbn[1][1])<0.0001)									//??Phi???
	{
		if(Cbn[0][1]<0)
			Atitude[2]=90;
		else
			Atitude[2]=-90;
	}
	else
	{
		if(Cbn[1][1]<0)
		{
			if(Cbn[0][1]>0)
				Atitude[2]=atan(Cbn[0][1]/Cbn[1][1])*57.3+180;
			else
				Atitude[2]=atan(Cbn[0][1]/Cbn[1][1])*57.3-180;
		}
		else
			Atitude[2]=atan(Cbn[0][1]/Cbn[1][1])*57.3;
	}

	if(Cbn[2][2]<0)											//??Gamma???
	{
		if(atan(-Cbn[2][0]/Cbn[2][2])>0)
			Atitude[1]=atan(-Cbn[2][0]/Cbn[2][2])*57.3-180;
		else
			Atitude[1]=atan(-Cbn[2][0]/Cbn[2][2])*57.3+180;
	}
	  else
	{
		Atitude[1]=atan(-Cbn[2][0]/Cbn[2][2])*57.3;
	}
}

void chushiduizhun()
{
	 
		float QQ;
	  cross4(q,v2);
		dataprocess();
		if (n*12.0/2460<1)
		{
			jiaji(acc);
			Q[0]=0*Q[0]+1*p0;
			Q[1]=0*Q[1]+1*p1;
			Q[2]=0*Q[2]+1*p2;
			Q[3]=0*Q[3]+1*p3;
			QQ=pow(Q[0],2)+pow(Q[1],2)+pow(Q[2],2)+pow(Q[3],2);
			Q[0]=Q[0]/QQ;
			Q[1]=Q[1]/QQ;
			Q[2]=Q[2]/QQ;
			Q[3]=Q[3]/QQ;
		}	
} 
void zitaijiesuan()
{
	  float epsilon[3]={0,0,0},nabla[3]={0,0,0},T_alignment=0;	 
	  Update_INS(Q,V,P,Tm,epsilon,nabla);
		q2atitude(Q,Altitude);
	  theta=Altitude[0];
	  psi=Altitude[1];
	  gama=Altitude[2];
	  Vx=V[0];
	  Vy=V[1];
	  Vz=V[2];
		X=P[0];
		Y=P[1];
		Z=P[2];	
}


