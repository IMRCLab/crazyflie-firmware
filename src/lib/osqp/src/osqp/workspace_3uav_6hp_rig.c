/*
 * This file was autogenerated by OSQP-Python on July 20, 2023 at 21:22:48.
 * 
 * This file contains the workspace variables needed by OSQP.
 */

#include "types.h"
#include "qdldl_interface.h"

// Define data structure
static c_int Pdata_p[10] = {
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
static c_int Pdata_i[9] = {
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
static c_float Pdata_x[9] = {
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
static csc Pdata = {9, 9, 9, Pdata_p, Pdata_i, Pdata_x, -1};
static c_int Adata_p[10] = {
0,
8,
16,
24,
32,
40,
48,
56,
64,
72,
};
static c_int Adata_i[72] = {
0,
1,
2,
3,
4,
5,
6,
7,
0,
1,
2,
3,
4,
5,
6,
7,
0,
1,
2,
3,
4,
5,
6,
7,
0,
1,
2,
3,
4,
5,
8,
9,
0,
1,
2,
3,
4,
5,
8,
9,
0,
1,
2,
3,
4,
5,
8,
9,
0,
1,
2,
3,
4,
5,
10,
11,
0,
1,
2,
3,
4,
5,
10,
11,
0,
1,
2,
3,
4,
5,
10,
11,
};
static c_float Adata_x[72] = {
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
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
static csc Adata = {72, 12, 9, Adata_p, Adata_i, Adata_x, -1};
static c_float qdata[9] = {
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
static c_float ldata[12] = {
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)-1000000000000000019884624838656.00000000000000000000,
(c_float)-1000000000000000019884624838656.00000000000000000000,
(c_float)-1000000000000000019884624838656.00000000000000000000,
(c_float)-1000000000000000019884624838656.00000000000000000000,
(c_float)-1000000000000000019884624838656.00000000000000000000,
(c_float)-1000000000000000019884624838656.00000000000000000000,
};
static c_float udata[12] = {
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
};
static OSQPData data = {9, 12, &Pdata, &Adata, qdata, ldata, udata};

// Define settings structure
static OSQPSettings settings = {(c_float)0.10000000000000000555, (c_float)0.00000100000000000000, 10, 1, 0, (c_float)5.00000000000000000000, 4000, (c_float)0.00100000000000000002, (c_float)0.00100000000000000002, (c_float)0.00010000000000000000, (c_float)0.00010000000000000000, (c_float)1.00000000000000000000, (enum linsys_solver_type) LINSYS_SOLVER, 0, 25, 1, };

// Define scaling structure
static c_float Dscaling[9] = {
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
static c_float Dinvscaling[9] = {
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
static c_float Escaling[12] = {
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
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
static c_float Einvscaling[12] = {
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
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
static OSQPScaling scaling = {(c_float)1.00000000000000000000, Dscaling, Escaling, (c_float)1.00000000000000000000, Dinvscaling, Einvscaling};

// Define linsys_solver structure
static c_int linsys_solver_L_p[22] = {
0,
3,
6,
9,
12,
15,
18,
26,
33,
39,
47,
54,
60,
68,
75,
81,
86,
90,
93,
95,
96,
96,
};
static c_int linsys_solver_L_i[96] = {
9,
10,
11,
9,
10,
11,
12,
13,
14,
12,
13,
14,
6,
7,
8,
6,
7,
8,
7,
8,
15,
16,
17,
18,
19,
20,
8,
15,
16,
17,
18,
19,
20,
15,
16,
17,
18,
19,
20,
10,
11,
15,
16,
17,
18,
19,
20,
11,
15,
16,
17,
18,
19,
20,
15,
16,
17,
18,
19,
20,
13,
14,
15,
16,
17,
18,
19,
20,
14,
15,
16,
17,
18,
19,
20,
15,
16,
17,
18,
19,
20,
16,
17,
18,
19,
20,
17,
18,
19,
20,
18,
19,
20,
19,
20,
20,
};
static c_float linsys_solver_L_x[96] = {
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)0.16666652777789350837,
(c_float)0.16666652777789350837,
(c_float)0.83333263888946751408,
(c_float)0.83333263888946751408,
(c_float)0.83333263888946751408,
(c_float)0.83333263888946751408,
(c_float)0.83333263888946751408,
(c_float)0.83333263888946751408,
(c_float)0.14285704081639941099,
(c_float)0.71428520408199702718,
(c_float)0.71428520408199702718,
(c_float)0.71428520408199702718,
(c_float)0.71428520408199702718,
(c_float)0.71428520408199702718,
(c_float)0.71428520408199702718,
(c_float)0.62499960937524412508,
(c_float)0.62499960937524412508,
(c_float)0.62499960937524412508,
(c_float)0.62499960937524412508,
(c_float)0.62499960937524412508,
(c_float)0.62499960937524412508,
(c_float)0.16666652777789350837,
(c_float)0.16666652777789350837,
(c_float)0.83333263888946751408,
(c_float)0.83333263888946751408,
(c_float)0.83333263888946751408,
(c_float)0.83333263888946751408,
(c_float)0.83333263888946751408,
(c_float)0.83333263888946751408,
(c_float)0.14285704081639941099,
(c_float)0.71428520408199702718,
(c_float)0.71428520408199702718,
(c_float)0.71428520408199702718,
(c_float)0.71428520408199702718,
(c_float)0.71428520408199702718,
(c_float)0.71428520408199702718,
(c_float)0.62499960937524412508,
(c_float)0.62499960937524412508,
(c_float)0.62499960937524412508,
(c_float)0.62499960937524412508,
(c_float)0.62499960937524412508,
(c_float)0.62499960937524412508,
(c_float)0.16666652777789350837,
(c_float)0.16666652777789350837,
(c_float)0.83333263888946751408,
(c_float)0.83333263888946751408,
(c_float)0.83333263888946751408,
(c_float)0.83333263888946751408,
(c_float)0.83333263888946751408,
(c_float)0.83333263888946751408,
(c_float)0.14285704081639941099,
(c_float)0.71428520408199702718,
(c_float)0.71428520408199702718,
(c_float)0.71428520408199702718,
(c_float)0.71428520408199702718,
(c_float)0.71428520408199702718,
(c_float)0.71428520408199702718,
(c_float)0.62499960937524412508,
(c_float)0.62499960937524412508,
(c_float)0.62499960937524412508,
(c_float)0.62499960937524412508,
(c_float)0.62499960937524412508,
(c_float)0.62499960937524412508,
(c_float)0.99822537600019367598,
(c_float)0.99822537600019367598,
(c_float)0.99822537600019367598,
(c_float)0.99822537600019367598,
(c_float)0.99822537600019367598,
(c_float)0.49955594998915914262,
(c_float)0.49955594998915914262,
(c_float)0.49955594998915914262,
(c_float)0.49955594998915914262,
(c_float)0.33313591933183328564,
(c_float)0.33313591933183328564,
(c_float)0.33313591933183328564,
(c_float)0.24988893818028751204,
(c_float)0.24988893818028751204,
(c_float)0.19992891411944221924,
};
static csc linsys_solver_L = {96, 21, 21, linsys_solver_L_p, linsys_solver_L_i, linsys_solver_L_x, -1};
static c_float linsys_solver_Dinv[21] = {
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)0.83333263888946751408,
(c_float)0.85714210204149843264,
(c_float)0.87499920312574797077,
(c_float)0.83333263888946751408,
(c_float)0.85714210204149843264,
(c_float)0.87499920312574797077,
(c_float)0.83333263888946751408,
(c_float)0.85714210204149843264,
(c_float)0.87499920312574797077,
(c_float)-0.17746239998063176313,
(c_float)-50.04440500108515266220,
(c_float)-66.68640806681808896883,
(c_float)-75.01110618197283486097,
(c_float)-80.00710858805747704992,
(c_float)-83.33827014526630705404,
};
static c_int linsys_solver_P[21] = {
18,
17,
20,
19,
16,
15,
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
11,
12,
13,
14,
10,
};
static c_float linsys_solver_bp[21];
static c_float linsys_solver_sol[21];
static c_float linsys_solver_rho_inv_vec[12] = {
(c_float)0.01000000000000000021,
(c_float)0.01000000000000000021,
(c_float)0.01000000000000000021,
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
static c_int linsys_solver_Pdiag_idx[9] = {
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
static c_int linsys_solver_KKT_p[22] = {
0,
1,
2,
3,
4,
5,
6,
9,
12,
15,
18,
21,
24,
27,
30,
33,
43,
53,
63,
73,
83,
93,
};
static c_int linsys_solver_KKT_i[93] = {
0,
1,
2,
3,
4,
5,
6,
5,
4,
7,
5,
4,
8,
5,
4,
9,
1,
0,
10,
1,
0,
11,
1,
0,
12,
3,
2,
13,
3,
2,
14,
3,
2,
6,
7,
8,
9,
10,
11,
12,
13,
14,
15,
6,
7,
8,
9,
10,
11,
12,
13,
14,
16,
6,
7,
8,
9,
10,
11,
12,
13,
14,
17,
6,
7,
8,
9,
10,
11,
12,
13,
14,
18,
6,
7,
8,
9,
10,
11,
12,
13,
14,
19,
6,
7,
8,
9,
10,
11,
12,
13,
14,
20,
};
static c_float linsys_solver_KKT_x[93] = {
(c_float)-10.00000000000000000000,
(c_float)-10.00000000000000000000,
(c_float)-10.00000000000000000000,
(c_float)-10.00000000000000000000,
(c_float)-10.00000000000000000000,
(c_float)-10.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)-0.01000000000000000021,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)-0.01000000000000000021,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)-0.01000000000000000021,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)-0.01000000000000000021,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)-0.01000000000000000021,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)-0.01000000000000000021,
};
static csc linsys_solver_KKT = {93, 21, 21, linsys_solver_KKT_p, linsys_solver_KKT_i, linsys_solver_KKT_x, -1};
static c_int linsys_solver_PtoKKT[9] = {
6,
9,
12,
15,
18,
21,
24,
27,
30,
};
static c_int linsys_solver_AtoKKT[72] = {
33,
83,
43,
53,
63,
73,
7,
8,
34,
84,
44,
54,
64,
74,
10,
11,
35,
85,
45,
55,
65,
75,
13,
14,
36,
86,
46,
56,
66,
76,
16,
17,
37,
87,
47,
57,
67,
77,
19,
20,
38,
88,
48,
58,
68,
78,
22,
23,
39,
89,
49,
59,
69,
79,
25,
26,
40,
90,
50,
60,
70,
80,
28,
29,
41,
91,
51,
61,
71,
81,
31,
32,
};
static c_int linsys_solver_rhotoKKT[12] = {
42,
92,
52,
62,
72,
82,
5,
4,
1,
0,
3,
2,
};
static QDLDL_float linsys_solver_D[21] = {
-10,
-10,
-10,
-10,
-10,
-10,
1,
1,
1,
1,
1,
1,
1,
1,
1,
-5,
0,
0,
0,
0,
0,
};
static QDLDL_int linsys_solver_etree[21] = {
9,
9,
12,
12,
6,
6,
7,
8,
15,
10,
11,
15,
13,
14,
15,
16,
17,
18,
19,
20,
-1,
};
static QDLDL_int linsys_solver_Lnz[21] = {
3,
3,
3,
3,
3,
3,
8,
7,
6,
8,
7,
6,
8,
7,
6,
5,
4,
3,
2,
1,
0,
};
static QDLDL_int   linsys_solver_iwork[63];
static QDLDL_bool  linsys_solver_bwork[21];
static QDLDL_float linsys_solver_fwork[21];
static qdldl_solver linsys_solver = {QDLDL_SOLVER, &solve_linsys_qdldl, &update_linsys_solver_matrices_qdldl, &update_linsys_solver_rho_vec_qdldl, &linsys_solver_L, linsys_solver_Dinv, linsys_solver_P, linsys_solver_bp, linsys_solver_sol, linsys_solver_rho_inv_vec, (c_float)0.00000100000000000000, 9, 12, linsys_solver_Pdiag_idx, 9, &linsys_solver_KKT, linsys_solver_PtoKKT, linsys_solver_AtoKKT, linsys_solver_rhotoKKT, linsys_solver_D, linsys_solver_etree, linsys_solver_Lnz, linsys_solver_iwork, linsys_solver_bwork, linsys_solver_fwork, };

// Define solution
static c_float xsolution[9];
static c_float ysolution[12];

static OSQPSolution solution = {xsolution, ysolution};

// Define info
static OSQPInfo info = {0, "Unsolved", OSQP_UNSOLVED, 0.0, 0.0, 0.0};

// Define workspace
static c_float work_rho_vec[12] = {
(c_float)100.00000000000000000000,
(c_float)100.00000000000000000000,
(c_float)100.00000000000000000000,
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
static c_float work_rho_inv_vec[12] = {
(c_float)0.01000000000000000021,
(c_float)0.01000000000000000021,
(c_float)0.01000000000000000021,
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
static c_int work_constr_type[12] = {
1,
1,
1,
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
static c_float work_x[9];
static c_float work_y[12];
static c_float work_z[12];
static c_float work_xz_tilde[21];
static c_float work_x_prev[9];
static c_float work_z_prev[12];
static c_float work_Ax[12];
static c_float work_Px[9];
static c_float work_Aty[9];
static c_float work_delta_y[12];
static c_float work_Atdelta_y[9];
static c_float work_delta_x[9];
static c_float work_Pdelta_x[9];
static c_float work_Adelta_x[12];
static c_float work_D_temp[9];
static c_float work_D_temp_A[9];
static c_float work_E_temp[12];

OSQPWorkspace workspace_3uav_6hp_rig = {
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
