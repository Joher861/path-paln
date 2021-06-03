#ifndef __ROBOT_NLP_HPP__
#define __ROBOT_NLP_HPP__

#include "IpTNLP.hpp"

using namespace Ipopt;

// #define LINEARIZATION

struct MPCRefPoints
{
    float x;
    float y;
    float theta;
};

class ROBOT_NLP: public TNLP
{
public:
   /** default constructor */
   ROBOT_NLP();

   /** default destructor */
   virtual ~ROBOT_NLP();

   /**@name Overloaded from TNLP */
   //@{
   /** Method to return some info about the nlp */
   virtual bool get_nlp_info(
      Index&          n,
      Index&          m,
      Index&          nnz_jac_g,
      Index&          nnz_h_lag,
      IndexStyleEnum& index_style
   );

   /** Method to return the bounds for my problem */
   virtual bool get_bounds_info(
      Index   n,
      Number* x_l,
      Number* x_u,
      Index   m,
      Number* g_l,
      Number* g_u
   );

   /** Method to return the starting point for the algorithm */
   virtual bool get_starting_point(
      Index   n,
      bool    init_x,
      Number* x,
      bool    init_z,
      Number* z_L,
      Number* z_U,
      Index   m,
      bool    init_lambda,
      Number* lambda
   );

   /** Method to return the objective value */
   virtual bool eval_f(
      Index         n,
      const Number* x,
      bool          new_x,
      Number&       obj_value
   );

   /** Method to return the gradient of the objective */
   virtual bool eval_grad_f(
      Index         n,
      const Number* x,
      bool          new_x,
      Number*       grad_f
   );

   /** Method to return the constraint residuals */
   virtual bool eval_g(
      Index         n,
      const Number* x,
      bool          new_x,
      Index         m,
      Number*       g
   );

   /** Method to return:
    *   1) The structure of the Jacobian (if "values" is NULL)
    *   2) The values of the Jacobian (if "values" is not NULL)
    */
   virtual bool eval_jac_g(
      Index         n,
      const Number* x,
      bool          new_x,
      Index         m,
      Index         nele_jac,
      Index*        iRow,
      Index*        jCol,
      Number*       values
   );

   /** Method to return:
    *   1) The structure of the Hessian of the Lagrangian (if "values" is NULL)
    *   2) The values of the Hessian of the Lagrangian (if "values" is not NULL)
    */
   virtual bool eval_h(
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
   );

   /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
   virtual void finalize_solution(
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
   );
   //@}
   
   virtual void set_local_points(std::vector<MPCRefPoints>& localPoints);

   virtual void set_current_pose(const std::vector<float>& current_pose_);

   virtual void set_bounds(const float vl, const float vu, const float dl, const float du);

   virtual void set_vehicle_para(const float lr_, const float lf_, const float l_, const float rho_);

   virtual void set_weights(const float Qx_, const float Qy_, const float Qphi_, const float Qdelta_);

   virtual void set_opti_para(const size_t NX_, const size_t NU_, const size_t NXU_, const size_t N_, const float T_MPC_);

   virtual void get_opt_res(std::vector<float> &X_OPT_,  std::vector<float> &opt_global_);


private:
   /**@name Methods to block default compiler methods.
    *
    * The compiler automatically generates the following three methods.
    *  Since the default compiler implementation is generally not what
    *  you want (for all but the most simple classes), we usually
    *  put the declarations of these methods in the private section
    *  and never implement them. This prevents the compiler from
    *  implementing an incorrect "default" behavior without us
    *  knowing. (See Scott Meyers book, "Effective C++")
    */
   //@{
   ROBOT_NLP(
      const ROBOT_NLP&
   );

   ROBOT_NLP& operator=(
      const ROBOT_NLP&
   );

   std::vector<MPCRefPoints> mLocalPoints;

   std::vector<float> current_pos_for_mpc;

   float lr;
   float lf;
   float l;
   float rho;

   size_t NX;
   size_t NU;
   size_t NXU;
   size_t N;
   float T_MPC;

   std::vector<float> X_OPT;

   float v_lower_bound; 
   float v_upper_bound;
   float delta_lower_bound;
   float delta_upper_bound;

   float Qx;
   float Qy;
   float Qphi;
   float Qdelta;

   float m_opt_x_global = 0;
   float m_opt_y_global = 0;
   float m_opt_theta_global = 0;

   //@}
};

#endif
