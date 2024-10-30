#include "AxisTran.h"
#include "proj.h"
using namespace Eigen;
#define  deg_to_rad 0.01745329252
#define  rad_to_deg (1/0.01745329252)

AxisTran::GuassLLH  _Guass_LLH;

AxisTran::GuassLLH::GuassLLH()
{
}



void AxisTran::Skew_symmetric(Eigen::Vector3d a, Eigen::Matrix<double, 3, 3> *A)
{
    //Skew_symmetric - Calculates skew-symmetric matrix
    //
    // Software for use with "Principles of GNSS, Inertial, and Multisensor
    // Integrated Navigation Systems," Second Edition.
    //
    // This function created 1/4/2012 by Paul Groves
    //
    // Inputs:
    //   a       3-element vector
    // Outputs:
    //   A       3x3matrix

    // Copyright 2012, Paul Groves
    // License: BSD; see license.txt for details

    // Begins

    (*A)<<  0.0, -a(2),  a(1),
          a(2),  0.0, -a(0),
         -a(1),  a(0),  0.0;

    // Ends
}

void AxisTran::Gravity_ECEF(Eigen::Vector3d r_eb_e, Eigen::Vector3d *g)
{
    //Gravitation_ECI - Calculates  acceleration due to gravity resolved about
    //ECEF-frame
    //
    // Software for use with "Principles of GNSS, Inertial, and Multisensor
    // Integrated Navigation Systems," Second Edition.
    //
    // This function created 1/4/2012 by Paul Groves
    //
    // Inputs:
    //   r_eb_e  Cartesian position of body frame w.r.t. ECEF frame, resolved
    //           about ECEF-frame axes (m)
    // Outputs:
    //   g       Acceleration due to gravity (m/s^2)

    // Copyright 2012, Paul Groves
    // License: BSD; see license.txt for details

    //Parameters
 #define   R_0  6378137.0 //WGS84 Equatorial radius in meters
 #define   mu  3.986004418E14  //WGS84 Earth gravitational constant (m^3 s^-2)
 #define   J_2  1.082627E-3  //WGS84 Earth's second gravitational constant
 #define   omega_ie  7.292115E-5  // Earth rotation rate (rad/s)

    // Begins

    // Calculate distance from center of the Earth
    double mag_r;
    mag_r = sqrt(r_eb_e.transpose() * r_eb_e);

    // If the input position is 0,0,0, produce a dummy output
    if (mag_r==0){
        (*g) << 0.0,0.0,0.0;

    // Calculate gravitational acceleration using (2.142)
    }else{
        double z_scale = 5.0 * pow((r_eb_e(2) / mag_r),2.0);
        Eigen::Vector3d tmp;
        tmp <<(1.0 - z_scale) * r_eb_e(0),(1.0 - z_scale) * r_eb_e(1),
                (3.0 - z_scale) * r_eb_e(2);
        Eigen::Vector3d gamma ;
        gamma = -mu / pow(mag_r,3.0) *(r_eb_e + 1.5 * J_2 * pow((R_0 / mag_r),2.0) *tmp);

        // Add centripetal acceleration using (2.133)
        g->block<2,1>(0,0) = gamma.block<2,1>(0,0) + pow(omega_ie,2.0) * r_eb_e.block<2,1>(0,0);
        (*g)(2) = gamma(2);

    } // if

    // Ends
}
//r_llh:纬度（rad），经度（rad），椭球高（m）
void AxisTran::pos_Guass_to_WGS84(Eigen::Vector3d r_guass,Eigen::Vector3d *r_llh,int zone)
{
    double longitude,latitude;

    (*r_llh)(2) = r_guass(2);
    _Guass_LLH.GaussProjInvCal(r_guass(0)+BASE_X,r_guass(1)+BASE_Y,&longitude,&latitude,zone);
     (*r_llh)(0) = latitude*deg_to_rad;
     (*r_llh)(1) = longitude*deg_to_rad;
}
//r_llh:纬度（rad），经度（rad），椭球高（m）
void AxisTran::pos_WGS84_to_Guass(Eigen::Vector3d r_llh,Eigen::Vector3d *r_guass)
{
    (*r_guass)(2) = r_llh(2);
    _Guass_LLH.GaussProjCal(r_llh(1)*rad_to_deg,r_llh(0)*rad_to_deg,&((*r_guass)(0)),&((*r_guass)(1)));
}


void AxisTran::GuassLLH::MeridianABCDE(double e2,double &A,double &B,double &C,
  double &D,double &E,double &F,double &G)
{
	A = 1+0.75*e2+45.0/64.0*e2*e2+175.0/256.0*e2*e2*e2+11025.0/16384.0*e2*e2*e2*e2;
    A += 43659.0/65536.0*pow(e2,5.0)+693693.0/1048576.0*pow(e2,6.0);
	B = 0.375*e2+15.0/32.0*e2*e2+525.0/1024.0*e2*e2*e2+2205.0/4096.0*e2*e2*e2*e2;
    B += 72765.0/131072.0*pow(e2,5.0)+297297.0/524288.0*pow(e2,6.0);
	C = 15.0/256.0*e2*e2+105.0/1024.0*e2*e2*e2+2205.0/16384.0*e2*e2*e2*e2;
    C += 10395.0/65536.0*pow(e2,5.0)+1486485.0/8388608.0*pow(e2,6.0);
	D = 35.0/3072.0*e2*e2*e2+105.0/4096.0*e2*e2*e2*e2;
    D += 10395.0/262144.0*pow(e2,5.0)+55055.0/1048576.0*pow(e2,6.0);
    E = 315.0/131072.0*pow(e2,4.0)+3465.0/524288.0*pow(e2,5.0)+99099.0/8388608.0*pow(e2,6.0);
    F = 639.0/1310720.0*pow(e2,5.0)+9009.0/5242880.0*pow(e2,6.0);
    G = 1001.0/8388608.0*pow(e2,6.0);
}

double AxisTran::GuassLLH::dMeridian_X(double a,double e2,double dLat)
{
	double A,B,C,D,E,F,G,X;

	MeridianABCDE(e2,A,B,C,D,E,F,G);
	X = a*(1-e2)*(A*dLat-B*sin(2*dLat)+C*sin(4*dLat)-D*sin(6*dLat)
		+E*sin(8*dLat)-F*sin(10*dLat)+G*sin(12*dLat));

	return X;
}

double AxisTran::GuassLLH::dMeridian_Bf(double a,double e2,double X)
{
	double A, B, C, D, E, F, G, B0, Bf;

	MeridianABCDE(e2,A,B,C,D,E,F,G);
	Bf=X/(a*(1-e2))/A;
	do{
		B0=Bf;
		Bf=X/a/(1-e2)+B*sin(2*Bf)-C*sin(4*Bf)+D*sin(6*Bf)
			-E*sin(8*Bf)+F*sin(10*Bf)-G*sin(12*Bf);
		Bf=Bf/A;
	}while(fabs(Bf-B0)>1e-13);

	return Bf;
}

void AxisTran::GuassLLH::Gaussxy(double a,double e2,double B,double L,double L0,
		 double &x,double &y)
{
	double N, X, l, tB, nB;

	L=(L<0.0)? L+2.0*AngleToRangle:L;
	l=L-L0*AngleToRangle/180;   
	X=dMeridian_X(a,e2,B); 
	N=a/sqrt(1-e2*sin(B)*sin(B));
	tB=tan(B);
	nB=sqrt(e2/(1.0-e2))*cos(B);

	x = X+N*sin(B)*cos(B)*l*l/2
        +N*sin(B)*pow(cos(B),3.0)*(5.0-tB*tB+9.0*nB*nB+4.0*pow(nB,4.0))*pow(l,4.0)/24.0
        +N*sin(B)*pow(cos(B),5.0)*(61.0-58.0*tB*tB+pow(tB,4.0))*pow(l,6.0)/720.0;
    y = N*cos(B)*l+N*pow(cos(B),3.0)*(1.0-tB*tB+nB*nB)*pow(l,3.0)/6.0
        +N*pow(cos(B),5.0)*(5.0-18.0*tB*tB+pow(tB,4.0)+14.0*nB*nB-58.0*tB*tB*nB*nB)
        *pow(l,5.0)/120.0;

}

void AxisTran::GuassLLH::GaussBL(double a,double e2,double x,double y,double L0,
             double &B,double &L)
{
	double t,n,b,l,M,N,Bf;

	Bf=dMeridian_Bf(a,e2,x); 
	L0=L0*AngleToRangle/180;
    M=a*(1-e2)/pow(sqrt(1-e2*sin(Bf)*sin(Bf)),3.0);
	N=a/sqrt(1-e2*sin(Bf)*sin(Bf));
	t=tan(Bf);
	n=sqrt(e2/(1.0-e2))*cos(Bf);

	b=-t*y*y/2/M/N
        +t*pow(y,4.0)/24/M/pow(N,3.0)*(5+3*t*t+n*n-9*t*t*n*n)
        -t*pow(y,6.0)/720/M/pow(N,5.0)*(61+90*t*t+45*pow(t,4.0));
	l=y/N/cos(Bf)
        -pow(y,3.0)/6/pow(N,3.0)/cos(Bf)*(1+2*t*t+n*n)
        +pow(y,5.0)/120/pow(N,5.0)/cos(Bf)*(5+28*t*t+24*pow(t,4.0)+6*n*n+8*t*t*n*n);

	B=Bf+b;
	L=L0+l;
	if(fabs(B*PSEC) < 5e-5)  B=fabs(B);
	if(fabs(L*PSEC) < 5e-5)  L=fabs(L);
}

int AxisTran::GuassLLH::BLtoXY(double B, double L, double L0, double& X, double& Y)
{
    B = B * Co;
    L = L * Co;

    double  a = a54;
    double  e2 = e254;

    if( CoordSysParam == 84 )
    {
        a = a84;  e2 = e284;
    }

    Gaussxy(a, e2, B, L, L0, X, Y);

    Y += 500000.0;

    return 1;
}

int AxisTran::GuassLLH::XYtoBL(double X, double Y, double L0, double& B, double& L)
{
    Y -= 500000.0;

    double  a = a54;
    double  e2 = e254;

    if( CoordSysParam == 84 )
    {
        a = a84;  e2 = e284;
    }

    GaussBL(a, e2, X, Y, L0, B, L);

    B = B / Co;
    L = L / Co;

    return 1;
}

double AxisTran::GuassLLH::NtoL(int N)
{
    int  l = (N <= 30) ? N*6-3 : (N-60)*6-3;
    return (double)l;
}

int AxisTran::GuassLLH::LtoN(double L)
{
    if(L < 0)  L += 360;
    return ((int)(L/6) + 1);
}

double AxisTran::GuassLLH::LOCM(double L)
{
    return NtoL(LtoN(L));
}

void AxisTran::GuassLLH::GaussProjCal(double longitude, double latitude, double *X, double *Y, int zone)
{
    int ProjNo;
    double X0, Y0, xval,yval;

	if (-1 == zone){
		ProjNo = LtoN(longitude);
    }else{
        ProjNo = zone;
    }
	BLtoXY(latitude, longitude , NtoL(ProjNo), yval,  xval);
	xval = xval-500000L;

	X0 = 1000000L*(ProjNo)+500000L; 
	Y0 = 0;
    xval = xval+X0; 
	yval = yval+Y0;
	
	xval -= BASE_X;
	yval -= BASE_Y;

    *X = xval;    *Y = yval;


}

void AxisTran::GuassLLH::GaussProjInvCal( double X, double Y, double *longitude, double *latitude, int zone )
{
    int ProjNo;
	double X0, Y0, xval, yval;
	
	if (-1 ==  zone)
        ProjNo = (int)(X/1000000L) ; 
    else
        ProjNo = zone;
	
    X0 = ProjNo*1000000L+500000L;
	Y0 = 0;
    xval = X-X0;
    yval = Y-Y0; 

	XYtoBL(yval, xval+500000L,  NtoL(ProjNo), *latitude, *longitude);

}

void AxisTran::GuassLLH::GaussXS2GaussGeneral(double xsX, double xsY, double& X, double& Y)
{
	int ProjNo;
	double xval, yval;
	double X0, Y0;


	xval = xsX + BASE_X; 
	yval = xsY + BASE_Y;

	double dd = (xval-500000L)/1000000L;
	ProjNo = (int)(dd+0.5);
	X0 = 1000000L*(ProjNo)+500000L;
	Y0 = 0;
    xval = xval-X0; 
	yval = yval-Y0;


	X = yval; 
	Y = xval+500000L;
}

void AxisTran::GuassLLH::GaussGeneral2GaussXS(double X, double Y, double& xsX, double& xsY)
{
	int ProjNo;
	double xval, yval;
	double X0, Y0;


	xval = Y-500000L;
	yval = X;

	double dd = (xval + BASE_X-500000L)/1000000L;
	ProjNo = (int)(dd+0.5);
	X0 = 1000000L*(ProjNo+1)+500000L;
	Y0 = 0;
	xval = xval+X0;
	yval = yval+Y0;

	xsX = xval - BASE_X;
    xsY = yval - BASE_Y;
}

bool AxisTran::GuassLLH::TransformWgs84To2000(double lon, double lat, double& dbX, double& dbY)
{
    // 定义 WGS84 和 CGCS2000 的坐标系
    PJ *proj_wgs84;
    PJ_COORD coord_wgs84, coord_cgcs2000;

    // 初始化 WGS84 和 CGCS2000 坐标系 114
    int l_v = (lon + 1.5)/3;
    l_v *= 3;
    std::string str1, str2;
    str1 = "+proj=longlat +datum=WGS84 +no_defs";
    str2 = "+proj=tmerc +lat_0=0 +lon_0=" + std::to_string(l_v) + " +k=1 +x_0=500000 +y_0=0 +ellps=GRS80 +units=m +no_defs";
    proj_wgs84 = proj_create_crs_to_crs(PJ_DEFAULT_CTX,
                                        str1.c_str(),
                                        str2.c_str(),
                                        NULL);
    if (proj_wgs84 == 0)
    {
        proj_destroy(proj_wgs84);
        return false;
    }

    {
        PJ* P_for_GIS = proj_normalize_for_visualization(PJ_DEFAULT_CTX, proj_wgs84);
        if( 0 == P_for_GIS )  {
            proj_destroy(proj_wgs84);
            return false;
        }
        proj_destroy(proj_wgs84);
        proj_wgs84 = P_for_GIS;
    }

    coord_wgs84.lpzt.lam = lon;
    coord_wgs84.lpzt.phi = lat;
    coord_wgs84.lpzt.z = 0.0;
    coord_wgs84.lpzt.t = HUGE_VAL;

    // 进行转换
    coord_cgcs2000 = proj_trans(proj_wgs84, PJ_FWD, coord_wgs84);

    // 输出结果

    // 清理资源
    proj_destroy(proj_wgs84);

    dbX =coord_cgcs2000.xy.x;
    dbY = coord_cgcs2000.xy.y;

    return true;
}
