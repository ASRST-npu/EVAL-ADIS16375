
#include <math.h>

using namespace std;

double g[3]={0,0,0};

double _dtheta(int i,int j,double Tm)
{
	double dtheta;
	dtheta=gyro[i][j];
	return dtheta*Tm/180*3.1415926;
}
double _dv(int i,int j,double Tm)
{
	double dv;
	dv=acc[i][j];
	return dv*Tm*9.8;
}
double norm(double Phi[])
{
	double dphi=0;
	for (int i=0;i<3;i++)
	{
		dphi=dphi+Phi[i]*Phi[i];
	}
	dphi=sqrt(dphi);
	return dphi;
}
void cross(double v1[],double v2[],double v[])
{
	//??v1??v2??,???v?
	v[0]=v1[1]*v2[2]-v1[2]*v1[1];
	v[1]=v1[2]*v2[0]-v1[0]*v2[2];
	v[2]=v1[0]*v2[1]-v1[1]*v2[0];
}
void qmul(const double q0[],const double q1[],double q[])
{
	//??q0???q1??,???q ?
	q[0]=q0[0] * q1[0] - q0[1] * q1[1] - q0[2] * q1[2] - q0[3] * q1[3];
	q[1]=q0[0] * q1[1] + q0[1] * q1[0] + q0[2] * q1[3] - q0[3] * q1[2];
	q[2]=q0[0] * q1[2] + q0[2] * q1[0] + q0[3] * q1[1] - q0[1] * q1[3];
	q[3]=q0[0] * q1[3] + q0[3] * q1[0] + q0[1] * q1[2] - q0[2] * q1[1];
}
void cross4(const double q0[],double dVsfm[])
{
	double q1[4],q2[4],q0_[4];

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
void r2q(double Phi[],double q1[])
{
	double dphi;
	dphi=norm(Phi);
	if (dphi<0.0001)
	{
		q1[0]=cos(dphi/2);
		q1[1]=Phi[0]/2;
		q1[2]=Phi[1]/2;ifstream
		q1[3]=Phi[2]/2;
	}
	else
	{
		q1[0]=cos(dphi/2);
		for(int i=1;i<4;i++)
			q1[i]=Phi[i-1]/dphi*sin(dphi/2);
	}
}
void Update_INS(double Q[],double V[],double P[],double Tm,double epsilon[],double nabla[])
{
	//
	double Phi[3],theta[3][3],q0[4],q1[4],dv[3][3];	//
	for (int i=0;i<4;i++)
	{
		q0[i]=Q[i];
	}
	for(int i=0;i<3;i++)
	{	for(int j=0;j<3;j++)
		{
			theta[i][j]=_dtheta(i,j,Tm/3)-epsilon[j];
			dv[i][j]=_dv(i,j,Tm/3)-nabla[j];
		}
	}

	Phi[0]=theta[0][0]+theta[1][0]+theta[2][0]+9.0/20*(theta[0][1]*theta[2][2]-theta[2][1]*theta[0][2])+27.0/40*(theta[1][1]*(theta[2][2]-theta[0][2])-theta[1][2]*(theta[2][1]-theta[0][1]));
	Phi[1]=theta[0][1]+theta[1][1]+theta[2][1]+9.0/20*(theta[0][2]*theta[2][0]-theta[0][0]*theta[2][2])+27.0/40*(theta[1][2]*(theta[2][0]-theta[0][0])-theta[1][0]*(theta[2][2]-theta[0][2]));
	Phi[2]=theta[0][2]+theta[1][2]+theta[2][2]+9.0/20*(theta[0][0]*theta[2][1]-theta[0][1]*theta[2][0])+27.0/40*(theta[1][0]*(theta[2][1]-theta[0][1])-theta[1][1]*(theta[2][0]-theta[0][0]));
	r2q(Phi,q1);
	qmul(q0,q1,Q);

	/*-------------------------------------????--------------------------------------------------*/

	double dthetam[3]={0,0,0},dVm[3]={0,0,0},cross_dvdtheta[6][3],dVsclm[3],dVrot[3],dVsfm[3];
	cross(theta[0],dv[2],cross_dvdtheta[0]);
	cross(dv[0],theta[2],cross_dvdtheta[1]);
	cross(theta[0],dv[1],cross_dvdtheta[2]);
	cross(theta[1],dv[2],cross_dvdtheta[3]);
	cross(dv[0],theta[1],cross_dvdtheta[4]);
	cross(dv[1],theta[2],cross_dvdtheta[5]);
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			dthetam[i]=dthetam[i]+theta[j][i];
			dVm[i]=dVm[i]+dv[j][i];
		}
		cross(dthetam,dVm,dVrot);
		for (int i=0;i<3;i++)
		{
			dVsclm[i]=9.0/20*(cross_dvdtheta[0][i]+cross_dvdtheta[1][i])+27.0/40*(cross_dvdtheta[2][i]+cross_dvdtheta[3][i]+cross_dvdtheta[4][i]+cross_dvdtheta[5][i]);
			dVsfm[i]=dVm[i]+0.5*dVrot[i]+dVsclm[i];
		}
		cross4(Q,dVsfm);
		for (int i=0;i<3;i++)
		{
			V[i]=V[i]+dVsfm[i]+g[i]*Tm;
		}
		/*---------------------------????-----------------------------------*/
		for (int i=0;i<3;i++)
		{
			P[i]=P[i]+V[i]*Tm;
		}

}
void q2Cbn(double Q[],double Cbn[][3])
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
void q2atitude(double Q[],double Atitude[])
{ifstream
	double Cbn[3][3];
	q2Cbn(Q,Cbn);
	Atitude[0]=asin(Cbn[2][1])*57.3;
	if (abs(Cbn[1][1])<0.0001)									//??Phi???
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
int main()
{

	double v2[3]={1,2,2},v[3],q[4]={1,0,0,0};      //???????????
	cross4(q,v2);
	int N_mean=2500;
	double Tn=0;                                                     //??????
	double Q[4]={1,0,0,0},V[3]={0,0,0},P[3]={0,0,0},Tm=0.004*3;
	double Altitude[3];
	double epsilon[3]={0,0,0},nabla[3]={0,0,0},T_alignment=0;
/*--------------????------------------------------------------*/
//	for (int i=0;i<N_mean;i++)
//	{
//		for (int j=0;j<3;j++)
//		{
//			epsilon[j]=epsilon[j]+_dtheta(Tm/3);
//			nabla[j]=nabla[j]+_dv(Tm/3);
//		}
//		T_alignment=T_alignment+Tm/3;
//	}
	for (int i=0;i<3;i++)
	{
		epsilon[i]=epsilon[i]/N_mean;
		nabla[i]=nabla[i]/N_mean;
	}
	/*---------------????--------------------*/

		Update_INS(Q,V,P,Tm,epsilon,nabla);
		q2atitude(Q,Altitude);
	  theta=Altitude[1];
	  psi=Altitude[2];
	  gama=Altitude[3];
	  Vx=V[1];
	  Vy=V[2];
	  Vz=V[3];
//		for (int i=0;i<4;i++)
//		{
//			if (i<3)
//			{
//				V_out<<setw(7)<<V[i]<<" ";
//				Q_out<<setw(7)<<Q[i]<<" ";
//				P_out<<setw(7)<<P[i]<<" ";
//			}
//			else
//				Q_out<<setw(4)<<Q[i];
//		}
//		V_out<<endl;
//		Q_out<<endl;
//		P_out<<endl;
//		for (int i=0;i<3;i++)
//		{
//			A_out<<setw(4)<<Altitude[i]<<" ";
//		}
//		A_out<<endl;

//	} while(Tn<2400);

	return 0;
}


