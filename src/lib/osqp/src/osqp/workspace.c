/*
 * This file was autogenerated by OSQP-Python on October 19, 2022 at 14:23:34.
 * 
 * This file contains the workspace variables needed by OSQP.
 */

#include "types.h"
#include "qdldl_interface.h"

// Define data structure
c_int Pdata_i[9] = {
0,
1,
2,
3,
4,
5,
6,
7,
8,
};
c_int Pdata_p[10] = {
0,
1,
2,
3,
4,
5,
6,
7,
8,
9,
};
c_float Pdata_x[9] = {
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
};
csc Pdata = {9, 9, 9, Pdata_p, Pdata_i, Pdata_x, -1};
c_int Adata_i[25] = {
0,
1,
3,
4,
2,
3,
4,
0,
5,
6,
1,
5,
6,
2,
5,
6,
0,
7,
8,
1,
7,
8,
2,
7,
8,
};
c_int Adata_p[10] = {
0,
1,
4,
7,
10,
13,
16,
19,
22,
25,
};
c_float Adata_x[25] = {
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)0.99993925725781140912,
(c_float)-0.99985954009920885444,
(c_float)1.00000000000000000000,
(c_float)0.36394812248159674750,
(c_float)-0.57726917715069836490,
(c_float)1.00000000000000000000,
(c_float)-0.99979880589240377464,
(c_float)0.99971909993593410793,
(c_float)1.00000000000000000000,
(c_float)-0.57723411019180126402,
(c_float)0.57718808893977158192,
(c_float)1.00000000000000000000,
(c_float)0.42019206489154992124,
(c_float)-0.66647939995728933127,
(c_float)1.00000000000000000000,
(c_float)0.99979880589240377464,
(c_float)-0.99971909993593410793,
(c_float)1.00000000000000000000,
(c_float)-0.57723411019180126402,
(c_float)0.57718808893977158192,
(c_float)1.00000000000000000000,
(c_float)0.42019206489154992124,
(c_float)-0.66647939995728933127,
};
csc Adata = {25, 9, 9, Adata_p, Adata_i, Adata_x, -1};
c_float qdata[9] = {
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
};
c_float ldata[9] = {
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.09810000000000000664,
(c_float)-1064113132289802963587568762880.00000000000000000000,
(c_float)-1154538354301396779885683277824.00000000000000000000,
(c_float)-1228559420189553632351472320512.00000000000000000000,
(c_float)-1332958799914578773965478559744.00000000000000000000,
(c_float)-1228559420189553632351472320512.00000000000000000000,
(c_float)-1332958799914578773965478559744.00000000000000000000,
};
c_float udata[9] = {
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.09810000000000000664,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
};
OSQPData data = {9, 9, &Pdata, &Adata, qdata, ldata, udata};

// Define settings structure
OSQPSettings settings = {(c_float)0.10000000000000000555, (c_float)0.00000100000000000000, 10, 1, 0, (c_float)5.00000000000000000000, 4000, (c_float)0.00100000000000000002, (c_float)0.00100000000000000002, (c_float)0.00010000000000000000, (c_float)0.00010000000000000000, (c_float)1.00000000000000000000, (enum linsys_solver_type) LINSYS_SOLVER, 0, 25, 1, };

// Define scaling structure
c_float Dscaling[9] = {
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
};
c_float Dinvscaling[9] = {
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
};
c_float Escaling[9] = {
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.06411313228980297652,
(c_float)1.15453835430139672980,
(c_float)1.22855942018955355266,
(c_float)1.33295879991457866254,
(c_float)1.22855942018955355266,
(c_float)1.33295879991457866254,
};
c_float Einvscaling[9] = {
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)0.93974970297392945717,
(c_float)0.86614705892996790570,
(c_float)0.81396144424639282899,
(c_float)0.75021073424331197188,
(c_float)0.81396144424639282899,
(c_float)0.75021073424331197188,
};
OSQPScaling scaling = {(c_float)1.00000000000000000000, Dscaling, Escaling, (c_float)1.00000000000000000000, Dinvscaling, Einvscaling};

// Define linsys_solver structure
c_int linsys_solver_L_i[43] = {
1,
2,
12,
3,
4,
12,
4,
5,
11,
12,
5,
11,
12,
11,
12,
15,
8,
9,
8,
9,
9,
15,
10,
15,
11,
13,
15,
12,
13,
15,
13,
15,
16,
17,
15,
16,
17,
15,
16,
17,
16,
17,
17,
};
c_int linsys_solver_L_p[19] = {
0,
1,
3,
6,
10,
13,
16,
18,
20,
22,
24,
27,
30,
34,
37,
40,
42,
43,
43,
};
c_float linsys_solver_L_x[43] = {
(c_float)0.99999900000100006014,
(c_float)-0.99009999019702987777,
(c_float)-0.99009999019702987777,
(c_float)0.50234591352922097762,
(c_float)-0.50238596474112551249,
(c_float)0.49751243533575911782,
(c_float)-0.04782280043470586717,
(c_float)0.06346090295914015578,
(c_float)-0.05495875386955054870,
(c_float)0.04735888256971215737,
(c_float)-0.03705948566648133852,
(c_float)0.05245441874081151201,
(c_float)-0.04520085462200042203,
(c_float)-0.05394020840837975700,
(c_float)0.04648118472416934432,
(c_float)0.94635376707426976939,
(c_float)0.05772691771506983927,
(c_float)0.09998595400992088544,
(c_float)-0.03639481224815967891,
(c_float)-0.09999392572578114091,
(c_float)0.08992359759723259072,
(c_float)0.95550153496812384457,
(c_float)0.83927968221940374427,
(c_float)-0.07547104841543091081,
(c_float)-1.17746841345211761265,
(c_float)-1.17746841345211761265,
(c_float)-0.08886477563928539547,
(c_float)-0.02216135649779695307,
(c_float)0.52684398304695223558,
(c_float)0.06389636081536202716,
(c_float)0.01695429400134101927,
(c_float)-0.02814406247509658801,
(c_float)0.64960104954673913991,
(c_float)-0.64954926205442309328,
(c_float)0.00922680563029940233,
(c_float)-0.38169896204901054837,
(c_float)0.38166853032089209652,
(c_float)0.99999900000100006014,
(c_float)0.42019164469990522015,
(c_float)-0.66647873347855590698,
(c_float)0.13227806548329743008,
(c_float)-0.21655430395478703076,
(c_float)-0.09748821835270737868,
};
csc linsys_solver_L = {43, 18, 18, linsys_solver_L_p, linsys_solver_L_i, linsys_solver_L_x, -1};
c_float linsys_solver_Dinv[18] = {
(c_float)0.99999900000100006014,
(c_float)-0.99009999019702987777,
(c_float)0.50248706217717886702,
(c_float)-0.09521810120944021461,
(c_float)-0.09543563671105592672,
(c_float)0.94635376707426976939,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)0.95550153496812384457,
(c_float)0.83927968221940374427,
(c_float)-1.17746841345211761265,
(c_float)0.44743789050132054541,
(c_float)0.64973177175073348888,
(c_float)0.64239078147128403362,
(c_float)0.99999900000100006014,
(c_float)-0.34218326961196815494,
(c_float)-0.09089507237692490138,
(c_float)-0.09026316364258221703,
};
c_int linsys_solver_P[18] = {
0,
9,
3,
15,
14,
5,
13,
12,
2,
1,
10,
4,
6,
7,
8,
11,
16,
17,
};
c_float linsys_solver_bp[18];
c_float linsys_solver_sol[18];
c_float linsys_solver_rho_inv_vec[9] = {
(c_float)0.01000000000000000021,
(c_float)0.01000000000000000021,
(c_float)0.01000000000000000021,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
};
c_int linsys_solver_Pdiag_idx[9] = {
0,
1,
2,
3,
4,
5,
6,
7,
8,
};
c_int linsys_solver_KKT_i[43] = {
0,
0,
1,
2,
1,
2,
3,
2,
4,
5,
4,
3,
6,
7,
8,
7,
6,
9,
7,
6,
9,
10,
11,
10,
4,
3,
12,
1,
13,
10,
14,
8,
5,
14,
15,
12,
13,
14,
16,
12,
13,
14,
17,
};
c_int linsys_solver_KKT_p[19] = {
0,
1,
3,
5,
7,
9,
12,
13,
14,
17,
20,
22,
26,
28,
30,
31,
35,
39,
43,
};
c_float linsys_solver_KKT_x[43] = {
(c_float)1.00000099999999991773,
(c_float)1.00000000000000000000,
(c_float)-0.01000000000000000021,
(c_float)1.00000099999999991773,
(c_float)1.00000000000000000000,
(c_float)0.99971909993593410793,
(c_float)-10.00000000000000000000,
(c_float)-0.99979880589240377464,
(c_float)-10.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)0.42019206489154992124,
(c_float)-0.66647939995728933127,
(c_float)-10.00000000000000000000,
(c_float)-10.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)0.36394812248159674750,
(c_float)-0.57726917715069836490,
(c_float)1.00000099999999991773,
(c_float)0.99993925725781140912,
(c_float)-0.99985954009920885444,
(c_float)1.00000000000000000000,
(c_float)-0.01000000000000000021,
(c_float)1.00000099999999991773,
(c_float)1.00000000000000000000,
(c_float)-0.57723411019180126402,
(c_float)0.57718808893977158192,
(c_float)1.00000099999999991773,
(c_float)1.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)1.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)-0.01000000000000000021,
(c_float)0.99979880589240377464,
(c_float)-0.57723411019180126402,
(c_float)0.42019206489154992124,
(c_float)-10.00000000000000000000,
(c_float)-0.99971909993593410793,
(c_float)0.57718808893977158192,
(c_float)-0.66647939995728933127,
(c_float)-10.00000000000000000000,
};
csc linsys_solver_KKT = {43, 18, 18, linsys_solver_KKT_p, linsys_solver_KKT_i, linsys_solver_KKT_x, -1};
c_int linsys_solver_PtoKKT[9] = {
0,
17,
14,
3,
22,
9,
26,
28,
30,
};
c_int linsys_solver_AtoKKT[25] = {
1,
20,
18,
19,
31,
15,
16,
4,
7,
5,
23,
24,
25,
32,
10,
11,
27,
35,
39,
29,
36,
40,
33,
37,
41,
};
c_int linsys_solver_rhotoKKT[9] = {
2,
21,
34,
13,
12,
8,
6,
38,
42,
};
QDLDL_float linsys_solver_D[18] = {
1,
-1,
1,
-10,
-10,
1,
-10,
-10,
1,
1,
0,
2,
1,
1,
1,
-2,
-11,
-11,
};
QDLDL_int linsys_solver_etree[18] = {
1,
2,
3,
4,
5,
11,
8,
8,
9,
10,
11,
12,
13,
15,
15,
16,
17,
-1,
};
QDLDL_int linsys_solver_Lnz[18] = {
1,
2,
3,
4,
3,
3,
2,
2,
2,
2,
3,
3,
4,
3,
3,
2,
1,
0,
};
QDLDL_int   linsys_solver_iwork[54];
QDLDL_bool  linsys_solver_bwork[18];
QDLDL_float linsys_solver_fwork[18];
qdldl_solver linsys_solver = {QDLDL_SOLVER, &solve_linsys_qdldl, &update_linsys_solver_matrices_qdldl, &update_linsys_solver_rho_vec_qdldl, &linsys_solver_L, linsys_solver_Dinv, linsys_solver_P, linsys_solver_bp, linsys_solver_sol, linsys_solver_rho_inv_vec, (c_float)0.00000100000000000000, 9, 9, linsys_solver_Pdiag_idx, 9, &linsys_solver_KKT, linsys_solver_PtoKKT, linsys_solver_AtoKKT, linsys_solver_rhotoKKT, linsys_solver_D, linsys_solver_etree, linsys_solver_Lnz, linsys_solver_iwork, linsys_solver_bwork, linsys_solver_fwork, };

// Define solution
c_float xsolution[9];
c_float ysolution[9];

OSQPSolution solution = {xsolution, ysolution};

// Define info
OSQPInfo info = {0, "Unsolved", OSQP_UNSOLVED, 0.0, 0.0, 0.0};

// Define workspace
c_float work_rho_vec[9] = {
(c_float)100.00000000000000000000,
(c_float)100.00000000000000000000,
(c_float)100.00000000000000000000,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
};
c_float work_rho_inv_vec[9] = {
(c_float)0.01000000000000000021,
(c_float)0.01000000000000000021,
(c_float)0.01000000000000000021,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
};
c_int work_constr_type[9] = {
1,
1,
1,
0,
0,
0,
0,
0,
0,
};
c_float work_x[9];
c_float work_y[9];
c_float work_z[9];
c_float work_xz_tilde[18];
c_float work_x_prev[9];
c_float work_z_prev[9];
c_float work_Ax[9];
c_float work_Px[9];
c_float work_Aty[9];
c_float work_delta_y[9];
c_float work_Atdelta_y[9];
c_float work_delta_x[9];
c_float work_Pdelta_x[9];
c_float work_Adelta_x[9];
c_float work_D_temp[9];
c_float work_D_temp_A[9];
c_float work_E_temp[9];

OSQPWorkspace workspace = {
&data, (LinSysSolver *)&linsys_solver,
work_rho_vec, work_rho_inv_vec,
work_constr_type,
work_x, work_y, work_z, work_xz_tilde,
work_x_prev, work_z_prev,
work_Ax, work_Px, work_Aty,
work_delta_y, work_Atdelta_y,
work_delta_x, work_Pdelta_x, work_Adelta_x,
work_D_temp, work_D_temp_A, work_E_temp,
&settings, &scaling, &solution, &info};

