#ifndef AXISTRAN_H
#define AXISTRAN_H

#include<eigen3/Eigen/Dense>
#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <math.h>
#define			BASE_X						(19695588)
#define			BASE_Y						(3125798)


namespace AxisTran
{

    void  Skew_symmetric(Eigen::Vector3d a,Eigen::Matrix<double,3,3> *A);
    void  Gravity_ECEF(Eigen::Vector3d r_eb_e,Eigen::Vector3d *g);
    void  pos_Guass_to_WGS84(Eigen::Vector3d r_guass,Eigen::Vector3d *r_llh,int zone);
    void  pos_WGS84_to_Guass(Eigen::Vector3d r_llh,Eigen::Vector3d *r_guass);
    void  pos_LP_to_WGS84(Eigen::Vector3d localpos,double localyaw,Eigen::Vector3d init_llh,double init_Heading,Eigen::Vector3d *llh);
    void  pos_WGS84_to_LP(Eigen::Vector3d llh,Eigen::Vector3d init_llh,double init_Heading,Eigen::Vector3d *localpos);

    //高斯坐标和经纬高之间的转换
    class GuassLLH
    {
    public:
        GuassLLH();
        //为了避免同一个站点范围内出现跨越带号而导致的高斯坐标跳变问题，
        //在高斯经纬度转换函数中加入一个带号参数zone。默认情况下，如果不出现跨带问题，就不用传这个参数。
        //如果在同一个站点范围内可能出现跨带现象，则可以将该站点大部分区域所属的带号传进去进行带号固定，
        //以避免可能出现的高斯坐标数据跳变现象。
        //经纬度转行深高斯
        void GaussProjCal(double longitude, double latitude, double *X, double *Y, int zone=-1);
        //行深高斯转经纬度
        void GaussProjInvCal( double X, double Y, double *longitude, double *latitude, int zone=-1);

        //以下两个函数用于公司与第三方单位进行沟通时进行坐标转换使用
        //如果第三方使用通用高斯坐标，可以通过这两个函数进行数据交流和检验
        //行深高斯转通用高斯
        void GaussXS2GaussGeneral(double xsX, double xsY, double& X, double& Y);
        //通用高斯转行深高斯
        void GaussGeneral2GaussXS(double X, double Y, double& xsX, double& xsY);

        // 输入经纬度  输出2000坐标
        bool TransformWgs84To2000(double lon, double lat,double& dbX, double& dbY);
    private:

        int  CoordSysParam = 54;

        #define  AngleToRangle     3.14159265358979323846
        #define  PSEC   206264.80624709635515647

        double   Co=AngleToRangle/180.0;

        double  	C0=6367558.49687;
        double  	C1=32005.7801;
        double  	C2=133.9213;
        double  	C3= 0.7032;

        double  	K0=1.57046064172E-7;
        double  	K1=5.051773759E-3;
        double  	K2=2.9837302E-5;
        double  	K3=2.38189E-7;

        double  	a54=6378245.00000000;
        double  	b54=6356863.01877305;
        double   f54=1/298.3;
        double  	e254=6.693421622966E-3;
        double  	e_254=6.738525414683E-3;

        double  	a84=6378137.00000000;
        double  	b84=6356752.31424518;
        double   f84=1/298.257223563;
        double  	e284=6.69437999E-3;
        double  	e_284=6.739496742E-3;

        void MeridianABCDE(double e2,double &A,double &B,double &C,
        double &D,double &E,double &F,double &G);
        double dMeridian_X(double a,double e2,double dLat);
        double dMeridian_Bf(double a,double e2,double X);
        void Gaussxy(double a,double e2,double B,double L,double L0,
                double &x,double &y);
        void GaussBL(double a,double e2,double x,double y,double L0,
                    double &B,double &L);
        int BLtoXY(double B, double L, double L0, double& X, double& Y);
        int XYtoBL(double X, double Y, double L0, double& B, double& L);
        double NtoL(int N);
        int LtoN(double L);
        double LOCM(double L);

    };
}

#endif // AXISTRAN_H
