#ifndef G2O_CONFIG_H
#define G2O_CONFIG_H


#define G2O_HAVE_OPENGL 0
#define G2O_OPENGL_FOUND 0
/* #undef G2O_OPENMP */
//#define G2O_OPENMP 0  //定义就会把代码加进去
#define G2O_SHARED_LIBS 1
#define G2O_LGPL_SHARED_LIBS 1

// available sparse matrix libraries
#define G2O_HAVE_CHOLMOD 0
#define G2O_HAVE_CSPARSE 1

#define G2O_CXX_COMPILER "GNU /usr/lib/ccache/x86_64-linux-gnu-g++"

#define G2O_DELETE_IMPLICITLY_OWNED_OBJECTS 0



#ifdef __cplusplus
//#define G2O_NUMBER_FORMAT_STR "%g"
//using number_t = float;
#define G2O_NUMBER_FORMAT_STR "%lg"
using number_t = double;
#include <g2o/core/eigen_types.h>
#endif

#endif
