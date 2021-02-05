#ifndef MATH_H
#define MATH_H

#define _USE_MATH_DEFINES
#include <math.h>       /* isnan, sqrt */

inline double lerp (double a, double b, double t)
{
    return a + t*(b - a);
}

struct S_Curve
{
    /**
     * @brief fx A parameterized sigmoid (s curve) logistic function.
     * From: https://datascience.oneoffcoder.com/s-curve.html
     * @param x is the input
     * @param L  is the curve’s maximum value
     * @param x0 is the midpoint of the sigmoid
     * @param k is the logistic growth rate or steepness of the curve
     * @return double computed [0..1] value
     */
    static double fx(double x, double L=1.0, double x0=0.5, double k=1.0)
    {
        double sn = 1.0/(1+ exp(-1.));
        double s0 = 1.0/(1+ exp(0.));
        return  (1.0/((1.0 + exp(double(-x))))  * (1.0/(sn-s0)) );

    }
    static double scurve1(double x, double x0=0.0, double xn=1.0)
    {
        // 𝑓(𝑥)=𝐿/(1+𝑒−𝑘(𝑥−𝑥0))
        //return L/(1+ exp(-k*(x-x0)));
        double s = 1.0/(1+ exp(-x));
        double s0 = 1.0/(1+ exp(0.));
        double sn = 1.0/(1+ exp(-1.));
       // return -fx(0.0) + (1.0/((1.0 + exp(double(-x))))  * (1.0/(sn-s0)) );
        return -fx(0.0) + fx(x);
    }

    static double scurve2(double x, double x0=0.0, double xn=1.0)
    {
        double t = M_PI/2.0 * x;
        return sin(t);
    }

    //https://stats.stackexchange.com/questions/214877/is-there-a-formula-for-an-s-shaped-curve-with-domain-and-range-0-1#289477
    static double scurve(double x, double x0=0.0, double xn=1.0)
    {
        x=1.0-x;
        double t = 1.0 /(  1.0 + pow(x/(1.-x), 3.0));
        return t;
    }
};


#endif // MATH_H
