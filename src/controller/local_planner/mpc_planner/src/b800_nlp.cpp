#include "math.h"
#include "robot_nlp.hpp"

#include <cassert>
#include <iostream>
#include <fstream>

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#define HIDE_ALL_LOG

using namespace Ipopt;

double wrapMax_(double x, double max)
{
    /* integer math: `(max + x % max) % max` */
    return fmod(max + fmod(x, max), max);
}
double wrapMinMax_(double x, double min, double max)
{
    return min + wrapMax_(x - min, max - min);
}
double wrapAngle_rad_(double yaw)
{
    return wrapMinMax_(yaw, -M_PI, M_PI);
}

/* Constructor. */
ROBOT_NLP::ROBOT_NLP()
{ }

ROBOT_NLP::~ROBOT_NLP()
{ }

bool ROBOT_NLP::get_nlp_info(
   Index&          n,
   Index&          m,
   Index&          nnz_jac_g,
   Index&          nnz_h_lag,
   IndexStyleEnum& index_style
)
{
   // define opt_var like this: 
   // [ x0 y0 phi0 v0 delta0 x1 y1 phi1 v1 delta1 ... 
   //   x4 y4 phi4 v4 delta4 ]

   n = N * NXU + NX;  

   // define constraints like this:
   // c1: x0-x_ref0=0 c2: y0-y_ref0=0 c3: phi0-phi_ref0=0
   // c4: xi+dxi*T_MPC-xi+1=0
   // c5: yi+dyi*T_MPC-yi+1=0
   // c6: phii+dphi*T_MPC-phii+1=0
   // ...
   m = N * NX + NX;

   nnz_jac_g = 14 * N + NX; // Nx for x0, (5+5+4) for every iteration in N 
   // 14 means 14 nonzero element in constraints jacobian

   nnz_h_lag = 7 * N +1;

   nnz_h_lag = 0; // quasi-newton


   index_style = C_STYLE;

   // std::cout<<"=====================get_nlp_info done!"<<std::endl;
   return true;
}

bool ROBOT_NLP::get_bounds_info(
   Index   n,
   Number* x_l,
   Number* x_u,
   Index   m,
   Number* g_l,
   Number* g_u
)
{
   // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
   // If desired, we could assert to make sure they are what we think they are.
   // assert(n == N * NXU + NX);
   // assert(m == N * NX + NX);

   for (size_t i = 0; i < NX; i++) {
      x_l[i] = -2.0e19;
      x_u[i] = +2.0e19;

      g_l[i] = g_u[i] = 0;
   } // set bounds for first 3 state variables: x0, y0, phi0

   for (size_t i = 0; i < N; i++) {
      // bounds for x_i, i=1,2,3,4,5
      x_l[i*NXU + NXU] = -2.0e19;
      x_u[i*NXU + NXU] = +2.0e19;
      // bounds for y_i, i=1,2,3,4,5
      x_l[i*NXU + NXU + 1] = -2.0e19;
      x_u[i*NXU + NXU + 1] = +2.0e19;
      // bounds for phi_i, i=1,2,3,4,5
      x_l[i*NXU + NXU + 2] = -2.0e19;
      x_u[i*NXU + NXU + 2] = +2.0e19;

      // bounds for v_i, i=0,1,2,3,4
      x_l[i*NXU + 3] = v_lower_bound;
      x_u[i*NXU + 3] = v_upper_bound;
      // bounds for delta_i, i=0,1,2,3,4
      x_l[i*NXU + 4] = delta_lower_bound;
      x_u[i*NXU + 4] = delta_upper_bound;
   }

   // constraints in every iteration in N 
   for (size_t k = NX; k < (N+1) * NX; k++) {
      g_l[k] = g_u[k] = 0.0;
   }

   // std::cout<<"=====================get_bounds_info done!"<<std::endl;
   return true;
}

bool ROBOT_NLP::get_starting_point(
   Index   n,
   bool    init_x,
   Number* x,
   bool    init_z,
   Number* z_L,
   Number* z_U,
   Index   m,
   bool    init_lambda,
   Number* lambda
)
{
   assert(init_x == true);
   assert(init_z == false);
   assert(init_lambda == false);

   for (size_t i = 0; i < N * NXU + NX; i++) {
      // x[i] = x_opt[i]; // set the initial values
      x[i] = 0.0; 
   }

   // x[0] = current_pos_for_mpc[0];
   // x[1] = current_pos_for_mpc[1];
   // x[2] = current_pos_for_mpc[2];
   // x[0] = 0.0;
   // x[1] = 0.0;
   // x[2] = 0.0;
   // std::cout<<"=====================get_starting_point done!"<<std::endl;
   return true;
}

bool ROBOT_NLP::eval_f(
   Index         n,
   const Number* x,
   bool          new_x,
   Number&       obj_value
)
{
   obj_value = 0.0; // initialize

   for (size_t i = 0; i < N; i++) {
      
      #ifdef LINEARIZATION
      // if linearization
      obj_value += Qx * pow(x[i*NXU + NXU] + current_pos_for_mpc[0] - mLocalPoints[i].x, 2) + Qy * pow(x[i*NXU + NXU + 1] + current_pos_for_mpc[1] - mLocalPoints[i].y, 2) 
                  + Qphi * pow(x[i*NXU + NXU + 2] + current_pos_for_mpc[2] - mLocalPoints[i].theta, 2) + Qdelta * pow(x[i*NXU + 4], 2);
      #else
      obj_value += Qx * pow(x[i*NXU + NXU] - mLocalPoints[i].x, 2) + Qy * pow(x[i*NXU + NXU + 1] - mLocalPoints[i].y, 2) 
                  + Qphi * pow(x[i*NXU + NXU + 2] - mLocalPoints[i].theta, 2) + Qdelta * pow(x[i*NXU + 4], 2);  
      
      // obj_value += Qx * pow(x[i*NXU + NXU] - mLocalPoints[i].x, 2) + Qy * pow(x[i*NXU + NXU + 1] - mLocalPoints[i].y, 2) 
      //             + Qphi * pow(x[i*NXU + NXU + 2] - mLocalPoints[i].theta, 2) + Qdelta * pow(x[i*NXU + 4], 2) + Qv * pow(x[i*NXU + 3], 2);  
      // for test
      // obj_value += Qx * pow(x[i*NXU + NXU] - ref_x[i], 2) + Qy * pow(x[i*NXU + NXU + 1] - ref_y[i], 2) 
      //             + Qphi * pow(x[i*NXU + NXU + 2] - ref_phi[i], 2) + Qdelta * pow(x[i*NXU + 4], 2);             

      #endif

   }  
   // std::cout<<"=====================eval_f done!"<<std::endl;
   return true;
}

bool ROBOT_NLP::eval_grad_f(
   Index         n,
   const Number* x,
   bool          new_x,
   Number*       grad_f
)
{
   // return the gradient of the objective function grad_{x} f(x)
   for (size_t i = 0; i < NX; i++) {
      grad_f[i] = 0.0;
   }

   for (size_t j = 0; j < N; j++) {
      #ifdef LINEARIZATION
      // if linearization
      grad_f[j*NXU + NXU] = 2 * Qx * (x[j*NXU + NXU] + current_pos_for_mpc[0] - mLocalPoints[j].x);
      
      grad_f[j*NXU + NXU + 1] = 2 * Qy * (x[j*NXU + NXU + 1] + current_pos_for_mpc[1] - mLocalPoints[j].y);
      
      grad_f[j*NXU + NXU + 2] = 2 * Qphi * (x[j*NXU + NXU + 2] + current_pos_for_mpc[2] - mLocalPoints[j].theta);

      #else

      // dJ/dx_i, i=1,2,3,4,5
      grad_f[j*NXU + NXU] = 2 * Qx * (x[j*NXU + NXU] - mLocalPoints[j].x);

      // dJ/dy_i, i=1,2,3,4,5
      grad_f[j*NXU + NXU + 1] = 2 * Qy * (x[j*NXU + NXU + 1] - mLocalPoints[j].y);
      
      // dJ/dphi_i, i=1,2,3,4,5
      grad_f[j*NXU + NXU + 2] = 2 * Qphi * (x[j*NXU + NXU + 2] - mLocalPoints[j].theta);
      
      //for test
      // grad_f[j*NXU + NXU] = 2 * Qx * (x[j*NXU + NXU] - ref_x[j]);
      // grad_f[j*NXU + NXU + 1] = 2 * Qy * (x[j*NXU + NXU + 1] - ref_y[j]);
      // grad_f[j*NXU + NXU + 2] = 2 * Qphi * (x[j*NXU + NXU + 2] - ref_phi[j]);
      #endif

      // dJ/dv_i, i=0,1,2,3,4
      grad_f[j*NXU + 3] = 0;
      // grad_f[j*NXU + 3] = 2 * Qv * x[j*NXU + 3];

      // dJ/ddelta_i, i=0,1,2,3,4
      grad_f[j*NXU + 4] = 2 * Qdelta * x[j*NXU + 4];
   }
   // std::cout<<"=====================eval_grad_f done!"<<std::endl;
   return true;
}

bool ROBOT_NLP::eval_g(
   Index         n,
   const Number* x,
   bool          new_x,
   Index         m,
   Number*       g
)
{
   // define constraints like this:
   // c1: x0-x_ref0=0 c2: y0-y_ref0=0 c3: phi0-phi_ref0=0
   // c4: xi+dxi*T_MPC-xi+1=0
   // c5: yi+dyi*T_MPC-yi+1=0
   // c6: phii+dphi*T_MPC-phii+1=0
   // ...

   // first 3 constraints, current_.. are given external
   // g[0] = current_pos_for_mpc[0];
   // g[1] = current_pos_for_mpc[1];
   // g[2] = current_pos_for_mpc[2];
   g[0] = 0.0;
   g[1] = 0.0;
   g[2] = 0.0;

   // other constraints 
   #ifdef LINEARIZATION
   // if linearization
   for (size_t i = 0; i < N; i++) { 
      float phi = x[i*NXU + 2] + current_pos_for_mpc[2]; // i = 0,1,2,3,4
      g[i*NX + NX] = x[i*NXU] - x[i*NXU + NXU] + T_MPC * ( (-x[i*NXU + 3]) * cos(x[i*NXU + 4]) * sin( phi + rho*x[i*NXU + 4]) * x[i*NXU + 2] / cos( rho*x[i*NXU + 4])
                                                            + cos(x[i*NXU + 4]) * cos( phi + rho*x[i*NXU + 4])*x[i*NXU + 3]/cos(rho*x[i*NXU + 4]) 
                                                            + x[i*NXU + 3] * x[i*NXU + 4] * ( (-sin(x[i*NXU + 4]))*cos(phi+rho*x[i*NXU + 4])*cos(rho*x[i*NXU + 4]) - rho*cos(x[i*NXU + 4])*sin(phi) ) / pow(cos(rho*x[i*NXU + 4]),2)  ); 
      g[i*NX + NX + 1] = x[i*NXU + 1] - x[i*NXU + NXU + 1] + T_MPC * ( x[i*NXU + 3] * cos(x[i*NXU + 4]) * cos( phi + rho*x[i*NXU + 4]) * x[i*NXU + 2] / cos( rho*x[i*NXU + 4])
                                                            + cos(x[i*NXU + 4]) * sin(phi + rho*x[i*NXU + 4])*x[i*NXU + 3]/cos(rho*x[i*NXU + 4]) 
                                                            + x[i*NXU + 3] * x[i*NXU + 4] * ( (-sin(x[i*NXU + 4]))*sin(phi+rho*x[i*NXU + 4])*cos(rho*x[i*NXU + 4]) + rho*cos(x[i*NXU + 4])*cos(phi) ) / pow(cos(rho*x[i*NXU + 4]),2)  );  
      g[i*NX + NX + 2] = x[i*NXU + 2] - x[i*NXU + NXU + 2] + T_MPC * ( sin(x[i*NXU + 4]) * x[i*NXU + 3] / ( cos(rho*x[i*NXU + 4]) * l )
                                                            + x[i*NXU + 3] * x[i*NXU + 4] * ( cos(x[i*NXU + 4]) * cos(rho*x[i*NXU + 4]) + rho * sin(x[i*NXU + 4]) * sin(rho*x[i*NXU + 4])  ) / (l * pow(cos(rho*x[i*NXU + 4]),2)) ); 
   }

   #else
   for (size_t i = 0; i < N; i++) {
      // constraints for x_i
      g[i*NX + NX] = x[i*NXU] - x[i*NXU + NXU] + T_MPC * x[i*NXU + 3] * cos(x[i*NXU + 4]) * cos(x[i*NXU + 2]+ rho*x[i*NXU + 4]) / cos(rho*x[i*NXU + 4]); 
      // constraints for y_i
      g[i*NX + NX + 1] = x[i*NXU + 1] - x[i*NXU + NXU + 1] + T_MPC * x[i*NXU + 3] * cos(x[i*NXU + 4]) * sin(x[i*NXU + 2] + rho*x[i*NXU + 4]) / cos(rho*x[i*NXU + 4]); 
      // constraints for phi_i
      g[i*NX + NX + 2] = x[i*NXU + 2] - x[i*NXU + NXU + 2] + T_MPC * x[i*NXU + 3] * sin(x[i*NXU + 4]) / (cos(rho*x[i*NXU + 4])*l); 
   }
   #endif



   // std::cout<<"=====================eval_g done!"<<std::endl;
   return true;
}

bool ROBOT_NLP::eval_jac_g(
   Index         n,
   const Number* x,
   bool          new_x,
   Index         m,
   Index         nele_jac,
   Index*        iRow,
   Index*        jCol,
   Number*       values
)
{
   if( values == NULL )
   {
      // return the structure of the jacobian of the constraints
      // for first block
      iRow[0] = 0;
      jCol[0] = 0;
      iRow[1] = 1;
      jCol[1] = 1;
      iRow[2] = 2;
      jCol[2] = 2;
      
      // for block in every iteration in N
      for (size_t i = 0; i < N; i++) {
         // first row in every block
         iRow[i*14 + NX] = i*NX + NX;
         jCol[i*14 + NX] = i*NXU;
         iRow[i*14 + NX + 1] = i*NX + NX;
         jCol[i*14 + NX + 1] = i*NXU + 2;
         iRow[i*14 + NX + 2] = i*NX + NX;
         jCol[i*14 + NX + 2] = i*NXU + 3;
         iRow[i*14 + NX + 3] = i*NX + NX;
         jCol[i*14 + NX + 3] = i*NXU + 4;
         iRow[i*14 + NX + 4] = i*NX + NX;
         jCol[i*14 + NX + 4] = i*NXU + 5;

         // second row in every block
         iRow[i*14 + NX + 5] = i*NX + NX +1;
         jCol[i*14 + NX + 5] = i*NXU + 1;
         iRow[i*14 + NX + 6] = i*NX + NX +1;
         jCol[i*14 + NX + 6] = i*NXU + 2;
         iRow[i*14 + NX + 7] = i*NX + NX +1;
         jCol[i*14 + NX + 7] = i*NXU + 3;
         iRow[i*14 + NX + 8] = i*NX + NX +1;
         jCol[i*14 + NX + 8] = i*NXU + 4;
         iRow[i*14 + NX + 9] = i*NX + NX +1;
         jCol[i*14 + NX + 9] = i*NXU + 6;

         // third row in every block
         iRow[i*14 + NX + 10] = i*NX + NX +2;
         jCol[i*14 + NX + 10] = i*NXU + 2;
         iRow[i*14 + NX + 11] = i*NX + NX +2;
         jCol[i*14 + NX + 11] = i*NXU + 3;
         iRow[i*14 + NX + 12] = i*NX + NX +2;
         jCol[i*14 + NX + 12] = i*NXU + 4;
         iRow[i*14 + NX + 13] = i*NX + NX +2;
         jCol[i*14 + NX + 13] = i*NXU + 7;
      }
   }
   else
   {
      // for first block
      values[0] = 1.0;
      values[1] = 1.0;
      values[2] = 1.0;

      // for block in every iteration in N

      #ifdef LINEARIZATION
      // if linearization
      for (size_t i = 0; i < N; i++) {
         //  phi = x[i*NXU + 2]; vf = x[i*NXU + 3]; delta = x[i*NXU + 4];
         float phi = x[i*NXU + 2] + current_pos_for_mpc[2];
         // first row in every block
         values[i*14 + NX] = 1.0;

         values[i*14 + NX + 1] = -T_MPC * x[i*NXU + 3] * cos(x[i*NXU + 4]) * sin(phi+rho*x[i*NXU + 4]) / cos(rho*x[i*NXU + 4]);

         values[i*14 + NX + 2] = T_MPC * ( x[i*NXU + 2] * (-cos(x[i*NXU + 4])) * sin(phi + rho*x[i*NXU + 4]) / cos(rho*x[i*NXU + 4])
                                             +  x[i*NXU + 4] * ((-sin(x[i*NXU + 4])) * cos(phi + rho*x[i*NXU + 4]) * cos(rho*x[i*NXU + 4]) - rho*cos(x[i*NXU + 4])*sin(phi) ) / pow(cos(rho*x[i*NXU + 4]),2)  
                                             +  cos(x[i*NXU + 4]) * cos(phi+rho*x[i*NXU + 4]) / cos(rho*x[i*NXU + 4]) );
   
         values[i*14 + NX + 3] = T_MPC * ( x[i*NXU + 2] * x[i*NXU + 3] * ( sin(x[i*NXU + 4]) * cos(rho*x[i*NXU + 4]) * sin(phi+rho*x[i*NXU + 4]) - rho*cos(x[i*NXU + 4])*cos(phi) ) / pow(cos(rho*x[i*NXU + 4]),2) 
                                             +  x[i*NXU + 3] * ( (-sin(x[i*NXU + 4])) * cos(phi+rho*x[i*NXU + 4]) * cos(rho*x[i*NXU + 4]) - rho * cos(x[i*NXU + 4]) * sin(phi) ) / pow(cos(rho*x[i*NXU + 4]),2) 
                                             +  x[i*NXU + 4] * x[i*NXU + 3] * ( (-cos(x[i*NXU + 4])*cos(phi+rho*x[i*NXU + 4])*cos(rho*x[i*NXU + 4])+rho*sin(x[i*NXU + 4])*(sin(phi+2*rho*x[i*NXU + 4])+sin(phi)))*pow(cos(rho*x[i*NXU + 4]),2)
                                                                + rho*sin(2*rho*x[i*NXU + 4])*( -sin(x[i*NXU + 4])*cos(phi+rho*x[i*NXU + 4])*cos(rho*x[i*NXU + 4])-rho*cos(x[i*NXU + 4])*sin(phi) ) ) / pow(cos(rho*x[i*NXU + 4]),4)
                                             +  x[i*NXU + 3] * ( (-sin(x[i*NXU + 4])) * cos(phi+rho*x[i*NXU + 4]) * cos(rho*x[i*NXU + 4]) - rho*cos(x[i*NXU + 4])*sin(phi) ) / pow(cos(rho*x[i*NXU + 4]),2) ) ;

         values[i*14 + NX + 4] = -1.0;
      
         // second row in every block
         values[i*14 + NX + 5] = 1.0;
  
         values[i*14 + NX + 6] = T_MPC * x[i*NXU + 3] * cos(x[i*NXU + 4]) * cos(phi+rho*x[i*NXU + 4]) / cos(rho*x[i*NXU + 4]);
     
         values[i*14 + NX + 7] = T_MPC * ( x[i*NXU + 2] * cos(x[i*NXU + 4]) * cos(phi + rho*x[i*NXU + 4]) / cos(rho*x[i*NXU + 4])
                                             +  x[i*NXU + 4] * ((-sin(x[i*NXU + 4])) * sin(phi + rho*x[i*NXU + 4]) * cos(rho*x[i*NXU + 4]) + rho*cos(x[i*NXU + 4])*cos(phi) ) / pow(cos(rho*x[i*NXU + 4]),2)  
                                             +  cos(x[i*NXU + 4]) * sin(phi+rho*x[i*NXU + 4]) / cos(rho*x[i*NXU + 4]) );
    
         values[i*14 + NX + 8] = T_MPC * ( x[i*NXU + 2] * (-x[i*NXU + 3]) * ( sin(x[i*NXU + 4]) * cos(rho*x[i*NXU + 4]) * cos(phi+rho*x[i*NXU + 4]) + rho*cos(x[i*NXU + 4])*sin(phi) ) / pow(cos(rho*x[i*NXU + 4]),2) 
                                             +  x[i*NXU + 3] * ( (-sin(x[i*NXU + 4])) * sin(phi+rho*x[i*NXU + 4]) * cos(rho*x[i*NXU + 4]) + rho * cos(x[i*NXU + 4]) * cos(phi) ) / pow(cos(rho*x[i*NXU + 4]),2) 
                                             +  x[i*NXU + 4] * x[i*NXU + 3] * ( (-cos(x[i*NXU + 4])*sin(phi+rho*x[i*NXU + 4])*cos(rho*x[i*NXU + 4])-rho*sin(x[i*NXU + 4])*(cos(phi+2*rho*x[i*NXU + 4])-cos(phi)))*pow(cos(rho*x[i*NXU + 4]),2)
                                                                + rho*sin(2*rho*x[i*NXU + 4])*( -sin(x[i*NXU + 4])*sin(phi+rho*x[i*NXU + 4])*cos(rho*x[i*NXU + 4])+rho*cos(x[i*NXU + 4])*cos(phi) ) ) / pow(cos(rho*x[i*NXU + 4]),4)
                                             +  x[i*NXU + 3] * ( (-sin(x[i*NXU + 4])) * sin(phi+rho*x[i*NXU + 4]) * cos(rho*x[i*NXU + 4]) + rho*cos(x[i*NXU + 4])*cos(phi) ) / pow(cos(rho*x[i*NXU + 4]),2) ) ;
         values[i*14 + NX + 9] = -1.0;
        
         // third row in every block
         values[i*14 + NX + 10] = 1.0;
      
         values[i*14 + NX + 11] = T_MPC * ( sin(x[i*NXU + 4]) / (cos(rho*x[i*NXU + 4])*l) + x[i*NXU + 4] * ( cos(x[i*NXU + 4])*cos(rho*x[i*NXU + 4]) + rho*sin(x[i*NXU + 4])*sin(rho*x[i*NXU + 4]) ) / (l*pow(cos(rho*x[i*NXU + 4]),2)) );
        
         values[i*14 + NX + 12] = T_MPC * ( 2 * x[i*NXU + 3] * ( cos(x[i*NXU + 4]) * cos(rho*x[i*NXU + 4]) + rho*sin(x[i*NXU + 4])*sin(rho*x[i*NXU + 4]) ) / ( l*pow(cos(rho*x[i*NXU + 4]),2) ) 
                                             + x[i*NXU + 3] * x[i*NXU + 4] * ( (pow(rho,2)-1) * sin(x[i*NXU + 4]) * pow(cos(rho*x[i*NXU + 4]),3) + rho * sin(2*rho*x[i*NXU + 4]) * ( cos(x[i*NXU + 4])*cos(rho*x[i*NXU + 4]) + rho*sin(x[i*NXU + 4])*sin(rho*x[i*NXU + 4]) ) ) / (l *pow(cos(rho*x[i*NXU + 4]),4) ) );
         values[i*14 + NX + 13] = -1.0;
      }
      // std::cout<<"=====================eval_jac_g done!"<<std::endl;

      #else
      for (size_t i = 0; i < N; i++) {
         //  phi = x[i*NXU + 2]; vf = x[i*NXU + 3]; delta = x[i*NXU + 4];

         // first row in every block
         values[i*14 + NX] = 1.0;

         values[i*14 + NX + 1] = -T_MPC * x[i*NXU + 3] * cos(x[i*NXU + 4]) * sin(x[i*NXU + 2]+rho*x[i*NXU + 4]) / cos(rho*x[i*NXU + 4]);

         values[i*14 + NX + 2] = T_MPC * cos(x[i*NXU + 4]) * cos(x[i*NXU + 2]+rho*x[i*NXU + 4]) / cos(rho*x[i*NXU + 4]);
   
         values[i*14 + NX + 3] = T_MPC * x[i*NXU + 3] *(-sin(x[i*NXU + 4])*cos(x[i*NXU + 2]+rho*x[i*NXU + 4])*cos(rho*x[i*NXU + 4]) - rho*cos(x[i*NXU + 4])*sin(x[i*NXU + 2])) / pow(cos(rho*x[i*NXU + 4]),2);
         
         values[i*14 + NX + 4] = -1.0;
      

         // second row in every block
         values[i*14 + NX + 5] = 1.0;
  
         values[i*14 + NX + 6] = T_MPC * x[i*NXU + 3] * cos(x[i*NXU + 4]) * cos(x[i*NXU + 2]+rho*x[i*NXU + 4]) / cos(rho*x[i*NXU + 4]);
     
         values[i*14 + NX + 7] = T_MPC * cos(x[i*NXU + 4]) * sin(x[i*NXU + 2]+rho*x[i*NXU + 4]) / cos(rho*x[i*NXU + 4]);
    
         values[i*14 + NX + 8] = T_MPC * x[i*NXU + 3] *(-sin(x[i*NXU + 4])*sin(x[i*NXU + 2]+rho*x[i*NXU + 4])*cos(rho*x[i*NXU + 4]) + rho*cos(x[i*NXU + 4])*cos(x[i*NXU + 2])) / pow(cos(rho*x[i*NXU + 4]),2);;
    
         values[i*14 + NX + 9] = -1.0;
        
         // third row in every block
         values[i*14 + NX + 10] = 1.0;
      
         values[i*14 + NX + 11] = T_MPC * sin(x[i*NXU + 4]) / (cos(rho*x[i*NXU + 4])*l);
        
         values[i*14 + NX + 12] = T_MPC * x[i*NXU + 3] * (cos(x[i*NXU + 4])*cos(rho*x[i*NXU + 4]) + rho*sin(x[i*NXU + 4])*sin(rho*x[i*NXU + 4])) / (l*pow(cos(rho*x[i*NXU + 4]),2));
       
         values[i*14 + NX + 13] = -1.0;
      }
      #endif

      
      
   }

   return true;
}

bool ROBOT_NLP::eval_h(
   Index         n,
   const Number* x,
   bool          new_x,
   Number        obj_factor,
   Index         m,
   const Number* lambda,
   bool          new_lambda,
   Index         nele_hess,
   Index*        iRow,
   Index*        jCol,
   Number*       values
)
{
   return true;
}

void ROBOT_NLP::finalize_solution(
   SolverReturn               status,
   Index                      n,
   const Number*              x,
   const Number*              z_L,
   const Number*              z_U,
   Index                      m,
   const Number*              g,
   const Number*              lambda,
   Number                     obj_value,
   const IpoptData*           ip_data,
   IpoptCalculatedQuantities* ip_cq
)
{
   for (int i = 0; i < n; i++ ) {
      // X_OPT[i] = x[i];
      X_OPT.push_back(x[i]);
   }
   // std::cout << v_lower_bound << " " << lr << " " << Qx << " " << NX << " " << X_OPT[0] << std::endl;
   // std::cout<<"================optimized solution"<<std::endl;
   #ifdef LINEARIZATION
   // if linearization
   // std::cout<<"x["<<0<<"]: "<<x[0] + current_pos_for_mpc[0] <<" ";
   // std::cout<<"y["<<0<<"]: "<<x[1] + current_pos_for_mpc[1] <<" ";
   // std::cout<<"phi["<<0<<"]: "<<x[2] + current_pos_for_mpc[2] <<std::endl;

   // for (size_t i =0; i < N; i++) {
   //    std::cout<<"v["<<i<<"]: "<<x[i*NXU+NX]<<" ";
   //    std::cout<<"delta["<<i<<"]: "<<x[i*NXU+NX + 1]<<" "<<x[i*NXU+NX + 1] * 180/M_PI <<" || ";
   //    std::cout<<"x["<<i+1<<"]: "<< x[i*NXU+NX + 2] + current_pos_for_mpc[0] <<" ";
   //    std::cout<<"y["<<i+1<<"]: "<< x[i*NXU+NX + 3] + current_pos_for_mpc[1] <<" ";
   //    std::cout<<"phi["<<i+1<<"]: "<< x[i*NXU+NX + 4] + current_pos_for_mpc[2] <<std::endl;
   // }
   size_t i = 0;
   std::cout<<"v["<<i<<"]: "<<x[i*NXU+NX]<<" ";
   std::cout<<"delta["<<i<<"]: "<<x[i*NXU+NX + 1]<<" "<<x[i*NXU+NX + 1] * 180/M_PI <<" ";
   std::cout<<" ||  x["<<i+1<<"]: "<<x[i*NXU+NX + 2] + current_pos_for_mpc[0] <<" ";
   std::cout<<"y["<<i+1<<"]: "<<x[i*NXU+NX + 3] + current_pos_for_mpc[1] <<" ";
   std::cout<<"phi["<<i+1<<"]: "<<x[i*NXU+NX + 4] + current_pos_for_mpc[2] <<std::endl;

   // std::ofstream mpc_traj_log;
   // mpc_traj_log.open("/tmp/log/mpc_traj_log.log", std::ios::app);
   // // ref_x ref_y ref_phi opt_x opt_y opt_phi robot_x robot_y robot_phi v delta
   // mpc_traj_log << mLocalPoints[i].x << " " << mLocalPoints[i].y << " " << mLocalPoints[i].theta << " " 
   //              << x[i*NXU+NX + 2] + current_pos_for_mpc[0] << " " << x[i*NXU+NX + 3] + current_pos_for_mpc[1] << " " << x[i*NXU+NX + 4] + current_pos_for_mpc[2] << " " 
   //              << current_pos_for_mpc[0] << " " << current_pos_for_mpc[1] << " " << current_pos_for_mpc[2] << " " 
   //              << x[i*NXU+NX] << " " << x[i*NXU+NX + 1] << std::endl;
   // mpc_traj_log.close();

   #else
   // std::cout<<"x["<<0<<"]: "<<x[0]<<" ";
   // std::cout<<"y["<<0<<"]: "<<x[1]<<" ";
   // std::cout<<"phi["<<0<<"]: "<<x[2]<<std::endl;
  
   // for (size_t i =0; i < N; i++) {
   //    std::cout<<"v["<<i<<"]: "<<x[i*NXU+NX]<<" ";
   //    std::cout<<"delta["<<i<<"]: "<<x[i*NXU+NX + 1]<<" "<<x[i*NXU+NX + 1] * 180/M_PI <<" ";
   //    std::cout<<"x["<<i+1<<"]: "<<x[i*NXU+NX + 2]<<" ";
   //    std::cout<<"y["<<i+1<<"]: "<<x[i*NXU+NX + 3]<<" ";
   //    std::cout<<"phi["<<i+1<<"]: "<<x[i*NXU+NX + 4]<<std::endl;
   // }

   //zwz
   
   // float opt_x_global = current_pos_for_mpc[0] + x[i*NXU+NX + 2] * cos(current_pos_for_mpc[2]) - x[i*NXU+NX + 3] * sin(current_pos_for_mpc[2]);
   // float opt_y_global = current_pos_for_mpc[1] + x[i*NXU+NX + 3] * cos(current_pos_for_mpc[2]) + x[i*NXU+NX + 2] * sin(current_pos_for_mpc[2]);
   // float opt_theta_global = x[i*NXU+NX + 4] + current_pos_for_mpc[2];

   float opt_x_global[N];
   float opt_y_global[N];
   float opt_theta_global[N];

   float ref_x_global[N];
   float ref_y_global[N];
   float ref_theta_global[N];

   for (size_t i = 0; i < N; i++) 
   {
      opt_x_global[i] = current_pos_for_mpc[0] + x[i*NXU+NX + 2] * cos(current_pos_for_mpc[2]) - x[i*NXU+NX + 3] * sin(current_pos_for_mpc[2]);
      opt_y_global[i] = current_pos_for_mpc[1] + x[i*NXU+NX + 3] * cos(current_pos_for_mpc[2]) + x[i*NXU+NX + 2] * sin(current_pos_for_mpc[2]);
      opt_theta_global[i] = wrapAngle_rad_(x[i*NXU+NX + 4] + current_pos_for_mpc[2]);

      ref_x_global[i] = current_pos_for_mpc[0] + mLocalPoints[i].x * cos(current_pos_for_mpc[2]) - mLocalPoints[i].y * sin(current_pos_for_mpc[2]);
      ref_y_global[i] = current_pos_for_mpc[1] + mLocalPoints[i].y * cos(current_pos_for_mpc[2]) + mLocalPoints[i].x * sin(current_pos_for_mpc[2]);
      ref_theta_global[i] = wrapAngle_rad_(mLocalPoints[i].theta + current_pos_for_mpc[2]);
   }
   m_opt_x_global = opt_x_global[0];
   m_opt_y_global = opt_y_global[0];
   m_opt_theta_global = opt_theta_global[0];
   // float ref_x_global = current_pos_for_mpc[0] + mLocalPoints[i].x * cos(current_pos_for_mpc[2]) - mLocalPoints[i].y * sin(current_pos_for_mpc[2]);
   // float ref_y_global = current_pos_for_mpc[1] + mLocalPoints[i].y * cos(current_pos_for_mpc[2]) + mLocalPoints[i].x * sin(current_pos_for_mpc[2]);
   // float ref_theta_global = mLocalPoints[i].theta + current_pos_for_mpc[2];
   size_t i = 0;
   #ifndef HIDE_ALL_LOG
   std::cout<<"v["<<i<<"]: "<<x[i*NXU+NX]<<" ";
   std::cout<<"delta["<<i<<"]: "<<x[i*NXU+NX + 1]<<" "<<x[i*NXU+NX + 1] * 180/M_PI <<" ";
   // std::cout<<" ||  x["<<i+1<<"]: "<<x[i*NXU+NX + 2]<<" ";
   // std::cout<<"y["<<i+1<<"]: "<<x[i*NXU+NX + 3]<<" ";
   // std::cout<<"phi["<<i+1<<"]: "<<x[i*NXU+NX + 4]<<std::endl;
   std::cout<<" ||  x["<<i+1<<"]: "<<opt_x_global[i]<<" ";
   std::cout<<"y["<<i+1<<"]: "<<opt_y_global[i]<<" ";
   std::cout<<"phi["<<i+1<<"]: "<<opt_theta_global[i]<<std::endl;

   for (size_t i = 0 ; i < N; i++) 
   {
      std::cout<<" local_ref:   x["<<i<<"]: "<<mLocalPoints[i].x<<" ";
      std::cout<<"y["<<i<<"]: "<<mLocalPoints[i].y<<" ";
      std::cout<<"phi["<<i<<"]: "<<mLocalPoints[i].theta<<std::endl;
   }
   #endif
   //zwz

   // std::ofstream mpc_traj_log;
   // mpc_traj_log.open("/tmp/log/mpc_traj_log.log", std::ios::app);
   // // ref_x ref_y ref_phi opt_x opt_y opt_phi robot_x robot_y robot_phi v delta 
   // mpc_traj_log << ref_x_global[0] << " " << ref_y_global[0] << " " << ref_theta_global[0] << " " 
   //              << opt_x_global[0] << " " << opt_y_global[0] << " " << opt_theta_global[0] << " " 
   //              << current_pos_for_mpc[0] << " " << current_pos_for_mpc[1] << " " << current_pos_for_mpc[2] << " " 
   //              << x[i*NXU+NX] << " " << x[i*NXU+NX + 1] ;
   // for (size_t i = 1; i < N; i++) 
   // {
   //    // col 12-23 
   //    mpc_traj_log << " " << ref_x_global[i] << " " << ref_y_global[i] << " " << ref_theta_global[i];
   //    // col 24-35
   //    mpc_traj_log << " " << opt_x_global[i] << " " << opt_y_global[i] << " " << opt_theta_global[i];
   // }
   // mpc_traj_log << std::endl;
   // mpc_traj_log.close();
   #endif

}

void ROBOT_NLP::set_local_points(std::vector<MPCRefPoints>& localPoints)
{
   mLocalPoints = localPoints;
   // std::cout << "mLocalPoints[0].x: " << mLocalPoints[0].x << std::endl;
   // std::cout << "mLocalPoints[0].y: " << mLocalPoints[0].y << std::endl;
   // std::cout << "mLocalPoints[0].thtea: " << mLocalPoints[0].theta << std::endl;
}

void ROBOT_NLP::set_current_pose(const std::vector<float>& current_pose_)
{
   current_pos_for_mpc = current_pose_;
   // std::cout << "current_pos_for_mpc_x: " << current_pos_for_mpc[0] << std::endl;
   // std::cout << "current_pos_for_mpc_y: " << current_pos_for_mpc[1] << std::endl;
   // std::cout << "current_pos_for_mpc_theta: " << current_pos_for_mpc[2] << std::endl;
}

void ROBOT_NLP::set_bounds(const float vl, const float vu, const float dl, const float du)
{
   v_lower_bound = vl; 
   v_upper_bound = vu;
   delta_lower_bound = dl;
   delta_upper_bound = du;
   // std::cout << "v_lower_bound: " << v_lower_bound << std::endl;
   // std::cout << "v_upper_bound: " << v_upper_bound << std::endl;
   // std::cout << "delta_lower_bound: " << delta_lower_bound << std::endl;
   // std::cout << "delta_upper_bound: " << delta_upper_bound << std::endl;
}

void ROBOT_NLP::set_vehicle_para(const float lr_, const float lf_, const float l_, const float rho_)
{
   lr = lr_;
   lf = lf_;
   l = l_;
   rho = rho_;
   // std::cout << "lr: " << lr << std::endl;
   // std::cout << "lf: " << lf << std::endl;
   // std::cout << "l: " << l << std::endl;
   // std::cout << "rho: " << rho << std::endl;
}

void ROBOT_NLP::set_weights(const float Qx_, const float Qy_, const float Qphi_, const float Qdelta_)
{
   Qx = Qx_;
   Qy = Qy_;
   Qphi = Qphi_;
   Qdelta = Qdelta_;
   // std::cout << "Qx: " << Qx << std::endl;
   // std::cout << "Qy: " << Qy << std::endl;
   // std::cout << "Qphi: " << Qphi << std::endl;
   // std::cout << "Qdelta: " << Qdelta << std::endl;
}

void ROBOT_NLP::set_opti_para(const size_t NX_, const size_t NU_, const size_t NXU_, const size_t N_, const float T_MPC_)
{
   NX = NX_;
   NU = NU_;
   NXU = NXU_;
   N = N_;
   T_MPC = T_MPC_;
   // std::cout << "NX: " << NX << std::endl;
   // std::cout << "NU: " << NU << std::endl;
   // std::cout << "NXU: " << NXU << std::endl;
   // std::cout << "N: " << N << std::endl;
   // std::cout << "T_MPC: " << T_MPC << std::endl;
}

void ROBOT_NLP::get_opt_res(std::vector<float> &X_OPT_, std::vector<float> &opt_global_)
{
   X_OPT_.clear();
   opt_global_.clear();
   X_OPT_ = X_OPT;
   opt_global_.push_back(m_opt_x_global);
   opt_global_.push_back(m_opt_y_global);
   opt_global_.push_back(m_opt_theta_global);
}

