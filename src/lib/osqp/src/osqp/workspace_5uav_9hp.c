/*
 * This file was autogenerated by OSQP-Python on July 12, 2023 at 15:21:56.
 * 
 * This file contains the workspace variables needed by OSQP.
 */

#include "types.h"
#include "qdldl_interface.h"

// Define data structure
static c_int Pdata_p[16] = {
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
10,
11,
12,
13,
14,
15,
};
static c_int Pdata_i[15] = {
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
10,
11,
12,
13,
14,
};
static c_float Pdata_x[15] = {
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
static csc Pdata = {15, 15, 15, Pdata_p, Pdata_i, Pdata_x, -1};
static c_int Adata_p[16] = {
0,
5,
10,
15,
20,
25,
30,
35,
40,
45,
50,
55,
60,
65,
70,
75,
};
static c_int Adata_i[75] = {
0,
3,
4,
5,
6,
1,
3,
4,
5,
6,
2,
3,
4,
5,
6,
0,
7,
8,
9,
10,
1,
7,
8,
9,
10,
2,
7,
8,
9,
10,
0,
11,
12,
13,
14,
1,
11,
12,
13,
14,
2,
11,
12,
13,
14,
0,
15,
16,
17,
18,
1,
15,
16,
17,
18,
2,
15,
16,
17,
18,
0,
19,
20,
21,
22,
1,
19,
20,
21,
22,
2,
19,
20,
21,
22,
};
static c_float Adata_x[75] = {
(c_float)1.00000000000000000000,
(c_float)0.89103152957089282182,
(c_float)0.60343484293843918653,
(c_float)0.16773655597217329571,
(c_float)0.72539687770744731843,
(c_float)1.00000000000000000000,
(c_float)0.36268753596460490130,
(c_float)0.99919971579282240715,
(c_float)0.98784815167469730568,
(c_float)0.08476648702784457534,
(c_float)1.00000000000000000000,
(c_float)0.99996340673217254391,
(c_float)0.08394436064918971152,
(c_float)0.99991191522068456621,
(c_float)0.99999273578680225594,
(c_float)1.00000000000000000000,
(c_float)0.89103152957089282182,
(c_float)0.60343484293843918653,
(c_float)0.16773655597217329571,
(c_float)0.72539687770744731843,
(c_float)1.00000000000000000000,
(c_float)0.36268753596460490130,
(c_float)0.99919971579282240715,
(c_float)0.98784815167469730568,
(c_float)0.08476648702784457534,
(c_float)1.00000000000000000000,
(c_float)0.99996340673217254391,
(c_float)0.08394436064918971152,
(c_float)0.99991191522068456621,
(c_float)0.99999273578680225594,
(c_float)1.00000000000000000000,
(c_float)0.89103152957089282182,
(c_float)0.60343484293843918653,
(c_float)0.16773655597217329571,
(c_float)0.72539687770744731843,
(c_float)1.00000000000000000000,
(c_float)0.36268753596460490130,
(c_float)0.99919971579282240715,
(c_float)0.98784815167469730568,
(c_float)0.08476648702784457534,
(c_float)1.00000000000000000000,
(c_float)0.99996340673217254391,
(c_float)0.08394436064918971152,
(c_float)0.99991191522068456621,
(c_float)0.99999273578680225594,
(c_float)1.00000000000000000000,
(c_float)0.89103152957089282182,
(c_float)0.60343484293843918653,
(c_float)0.16773655597217329571,
(c_float)0.72539687770744731843,
(c_float)1.00000000000000000000,
(c_float)0.36268753596460490130,
(c_float)0.99919971579282240715,
(c_float)0.98784815167469730568,
(c_float)0.08476648702784457534,
(c_float)1.00000000000000000000,
(c_float)0.99996340673217254391,
(c_float)0.08394436064918971152,
(c_float)0.99991191522068456621,
(c_float)0.99999273578680225594,
(c_float)1.00000000000000000000,
(c_float)0.89103152957089282182,
(c_float)0.60343484293843918653,
(c_float)0.16773655597217329571,
(c_float)0.72539687770744731843,
(c_float)1.00000000000000000000,
(c_float)0.36268753596460490130,
(c_float)0.99919971579282240715,
(c_float)0.98784815167469730568,
(c_float)0.08476648702784457534,
(c_float)1.00000000000000000000,
(c_float)0.99996340673217254391,
(c_float)0.08394436064918971152,
(c_float)0.99991191522068456621,
(c_float)0.99999273578680225594,
};
static csc Adata = {75, 23, 15, Adata_p, Adata_i, Adata_x, -1};
static c_float qdata[15] = {
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
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
static c_float ldata[23] = {
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)-1038145136240424988628515553280.00000000000000000000,
(c_float)-2268272461217911517485169901568.00000000000000000000,
(c_float)-1094299790684864613669861851136.00000000000000000000,
(c_float)-1007458997855180277753090932736.00000000000000000000,
(c_float)-1038145136240424988628515553280.00000000000000000000,
(c_float)-2268272461217911517485169901568.00000000000000000000,
(c_float)-1094299790684864613669861851136.00000000000000000000,
(c_float)-1007458997855180277753090932736.00000000000000000000,
(c_float)-1038145136240424988628515553280.00000000000000000000,
(c_float)-2268272461217911517485169901568.00000000000000000000,
(c_float)-1094299790684864613669861851136.00000000000000000000,
(c_float)-1007458997855180277753090932736.00000000000000000000,
(c_float)-1038145136240424988628515553280.00000000000000000000,
(c_float)-2268272461217911517485169901568.00000000000000000000,
(c_float)-1094299790684864613669861851136.00000000000000000000,
(c_float)-1007458997855180277753090932736.00000000000000000000,
(c_float)-1038145136240424988628515553280.00000000000000000000,
(c_float)-2268272461217911517485169901568.00000000000000000000,
(c_float)-1094299790684864613669861851136.00000000000000000000,
(c_float)-1007458997855180277753090932736.00000000000000000000,
};
static c_float udata[23] = {
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
(c_float)0.00000000000000000000,
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
static OSQPData data = {15, 23, &Pdata, &Adata, qdata, ldata, udata};

// Define settings structure
static OSQPSettings settings = {(c_float)0.10000000000000000555, (c_float)0.00000100000000000000, 10, 1, 0, (c_float)5.00000000000000000000, 4000, (c_float)0.00100000000000000002, (c_float)0.00100000000000000002, (c_float)0.00010000000000000000, (c_float)0.00010000000000000000, (c_float)1.00000000000000000000, (enum linsys_solver_type) LINSYS_SOLVER, 0, 25, 1, };

// Define scaling structure
static c_float Dscaling[15] = {
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
static c_float Dinvscaling[15] = {
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
static c_float Escaling[23] = {
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.03814513624042503714,
(c_float)2.26827246121791148425,
(c_float)1.09429979068486460392,
(c_float)1.00745899785518022718,
(c_float)1.03814513624042503714,
(c_float)2.26827246121791148425,
(c_float)1.09429979068486460392,
(c_float)1.00745899785518022718,
(c_float)1.03814513624042503714,
(c_float)2.26827246121791148425,
(c_float)1.09429979068486460392,
(c_float)1.00745899785518022718,
(c_float)1.03814513624042503714,
(c_float)2.26827246121791148425,
(c_float)1.09429979068486460392,
(c_float)1.00745899785518022718,
(c_float)1.03814513624042503714,
(c_float)2.26827246121791148425,
(c_float)1.09429979068486460392,
(c_float)1.00745899785518022718,
};
static c_float Einvscaling[23] = {
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)0.96325645142589100267,
(c_float)0.44086414533422785267,
(c_float)0.91382636505317493825,
(c_float)0.99259622687269655206,
(c_float)0.96325645142589100267,
(c_float)0.44086414533422785267,
(c_float)0.91382636505317493825,
(c_float)0.99259622687269655206,
(c_float)0.96325645142589100267,
(c_float)0.44086414533422785267,
(c_float)0.91382636505317493825,
(c_float)0.99259622687269655206,
(c_float)0.96325645142589100267,
(c_float)0.44086414533422785267,
(c_float)0.91382636505317493825,
(c_float)0.99259622687269655206,
(c_float)0.96325645142589100267,
(c_float)0.44086414533422785267,
(c_float)0.91382636505317493825,
(c_float)0.99259622687269655206,
};
static OSQPScaling scaling = {(c_float)1.00000000000000000000, Dscaling, Escaling, (c_float)1.00000000000000000000, Dinvscaling, Einvscaling};

// Define linsys_solver structure
static c_int linsys_solver_L_p[39] = {
0,
3,
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
36,
39,
42,
45,
48,
51,
54,
57,
60,
63,
66,
69,
72,
75,
78,
81,
84,
87,
90,
93,
96,
101,
105,
108,
110,
111,
111,
};
static c_int linsys_solver_L_i[111] = {
4,
5,
27,
4,
5,
27,
4,
5,
27,
4,
5,
27,
5,
27,
30,
27,
29,
30,
10,
11,
26,
10,
11,
26,
10,
11,
26,
10,
11,
26,
11,
26,
30,
26,
29,
30,
16,
17,
25,
16,
17,
25,
16,
17,
25,
16,
17,
25,
17,
25,
30,
25,
29,
30,
22,
23,
24,
22,
23,
24,
22,
23,
24,
22,
23,
24,
23,
24,
30,
24,
29,
30,
28,
29,
30,
28,
29,
30,
28,
29,
30,
28,
29,
30,
29,
30,
32,
30,
32,
33,
32,
33,
37,
32,
33,
37,
33,
34,
35,
36,
37,
34,
35,
36,
37,
35,
36,
37,
36,
37,
37,
};
static c_float linsys_solver_L_x[111] = {
(c_float)-0.09999927357868022837,
(c_float)-0.00847664870278445719,
(c_float)-0.07253968777074473739,
(c_float)-0.09999119152206846495,
(c_float)-0.09878481516746973057,
(c_float)-0.01677365559721732957,
(c_float)-0.00839443606491897115,
(c_float)-0.09991997157928224349,
(c_float)-0.06034348429384392420,
(c_float)-0.09999634067321726272,
(c_float)-0.03626875359646049291,
(c_float)-0.08910315295708928773,
(c_float)0.11679116286059872121,
(c_float)0.14106223540985129983,
(c_float)0.76882903947962411007,
(c_float)0.07867418160057319809,
(c_float)0.83783202282441149311,
(c_float)-0.09785137622751070852,
(c_float)-0.09999927357868022837,
(c_float)-0.00847664870278445719,
(c_float)-0.07253968777074473739,
(c_float)-0.09999119152206846495,
(c_float)-0.09878481516746973057,
(c_float)-0.01677365559721732957,
(c_float)-0.00839443606491897115,
(c_float)-0.09991997157928224349,
(c_float)-0.06034348429384392420,
(c_float)-0.09999634067321726272,
(c_float)-0.03626875359646049291,
(c_float)-0.08910315295708928773,
(c_float)0.11679116286059872121,
(c_float)0.14106223540985129983,
(c_float)0.76882903947962411007,
(c_float)0.07867418160057319809,
(c_float)0.83783202282441149311,
(c_float)-0.09785137622751070852,
(c_float)-0.09999927357868022837,
(c_float)-0.00847664870278445719,
(c_float)-0.07253968777074473739,
(c_float)-0.09999119152206846495,
(c_float)-0.09878481516746973057,
(c_float)-0.01677365559721732957,
(c_float)-0.00839443606491897115,
(c_float)-0.09991997157928224349,
(c_float)-0.06034348429384392420,
(c_float)-0.09999634067321726272,
(c_float)-0.03626875359646049291,
(c_float)-0.08910315295708928773,
(c_float)0.11679116286059872121,
(c_float)0.14106223540985129983,
(c_float)0.76882903947962411007,
(c_float)0.07867418160057319809,
(c_float)0.83783202282441149311,
(c_float)-0.09785137622751070852,
(c_float)-0.09999927357868022837,
(c_float)-0.00847664870278445719,
(c_float)-0.07253968777074473739,
(c_float)-0.09999119152206846495,
(c_float)-0.09878481516746973057,
(c_float)-0.01677365559721732957,
(c_float)-0.00839443606491897115,
(c_float)-0.09991997157928224349,
(c_float)-0.06034348429384392420,
(c_float)-0.09999634067321726272,
(c_float)-0.03626875359646049291,
(c_float)-0.08910315295708928773,
(c_float)0.11679116286059872121,
(c_float)0.14106223540985129983,
(c_float)0.76882903947962411007,
(c_float)0.07867418160057319809,
(c_float)0.83783202282441149311,
(c_float)-0.09785137622751070852,
(c_float)0.87875594219156694109,
(c_float)-0.06913540457856214017,
(c_float)-0.11588487328966425027,
(c_float)0.87875594219156694109,
(c_float)-0.06913540457856214017,
(c_float)-0.11588487328966425027,
(c_float)0.87875594219156694109,
(c_float)-0.06913540457856214017,
(c_float)-0.11588487328966425027,
(c_float)0.87875594219156694109,
(c_float)-0.06913540457856214017,
(c_float)-0.11588487328966425027,
(c_float)-0.07845099393784685238,
(c_float)-0.13149967874426343739,
(c_float)-0.28368603039235462671,
(c_float)-0.11641079151842326223,
(c_float)-0.02333885634399262415,
(c_float)-0.29749599300785067868,
(c_float)-0.04557619902177620363,
(c_float)-0.03772649939779536971,
(c_float)-0.32408077383293748541,
(c_float)-0.07253968777074473739,
(c_float)-0.00847664870278445719,
(c_float)-0.09999927357868022837,
(c_float)0.02587736872165794649,
(c_float)0.66269991084375379220,
(c_float)0.44880142098664244088,
(c_float)0.12475316192405674909,
(c_float)0.08784766402009325237,
(c_float)0.26091124975765117577,
(c_float)0.75561125138455642603,
(c_float)0.75555222204965488686,
(c_float)0.03314614870701611954,
(c_float)0.06147750902195071171,
(c_float)0.03443803289602367229,
(c_float)-0.08525352880494156682,
(c_float)0.07252090670717251109,
(c_float)0.00525248625289792027,
(c_float)-0.08653778347206900201,
};
static csc linsys_solver_L = {111, 38, 38, linsys_solver_L_p, linsys_solver_L_i, linsys_solver_L_x, -1};
static c_float linsys_solver_Dinv[38] = {
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)0.76882903947962411007,
(c_float)0.83783202282441149311,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)0.76882903947962411007,
(c_float)0.83783202282441149311,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)0.76882903947962411007,
(c_float)0.83783202282441149311,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)-0.10000000000000000555,
(c_float)0.76882903947962411007,
(c_float)0.83783202282441149311,
(c_float)0.87875594219156694109,
(c_float)0.87875594219156694109,
(c_float)0.87875594219156694109,
(c_float)0.87875594219156694109,
(c_float)-0.28368603039235462671,
(c_float)-0.29749599300785067868,
(c_float)-0.32408077383293748541,
(c_float)-0.10000000000000000555,
(c_float)0.74374462502230409555,
(c_float)0.76822207002051146052,
(c_float)-0.09364085164750307611,
(c_float)-0.09112722983285065359,
(c_float)-0.09351354338809084177,
(c_float)0.63683047625203847986,
};
static c_int linsys_solver_P[38] = {
25,
24,
23,
22,
5,
4,
29,
28,
27,
26,
8,
7,
33,
32,
31,
30,
11,
10,
37,
36,
35,
34,
14,
13,
12,
9,
6,
3,
15,
16,
17,
21,
0,
1,
18,
19,
20,
2,
};
static c_float linsys_solver_bp[38];
static c_float linsys_solver_sol[38];
static c_float linsys_solver_rho_inv_vec[23] = {
(c_float)0.01000000000000000021,
(c_float)0.01000000000000000021,
(c_float)0.01000000000000000021,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
};
static c_int linsys_solver_Pdiag_idx[15] = {
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
10,
11,
12,
13,
14,
};
static c_int linsys_solver_KKT_p[39] = {
0,
1,
2,
3,
4,
9,
14,
15,
16,
17,
18,
23,
28,
29,
30,
31,
32,
37,
42,
43,
44,
45,
46,
51,
56,
61,
66,
71,
76,
81,
86,
91,
92,
95,
98,
101,
104,
107,
113,
};
static c_int linsys_solver_KKT_i[113] = {
0,
1,
2,
3,
4,
3,
2,
1,
0,
5,
3,
2,
1,
0,
6,
7,
8,
9,
10,
9,
8,
7,
6,
11,
9,
8,
7,
6,
12,
13,
14,
15,
16,
15,
14,
13,
12,
17,
15,
14,
13,
12,
18,
19,
20,
21,
22,
21,
20,
19,
18,
23,
21,
20,
19,
18,
24,
21,
20,
19,
18,
25,
15,
14,
13,
12,
26,
9,
8,
7,
6,
27,
3,
2,
1,
0,
27,
26,
25,
24,
28,
5,
11,
17,
23,
29,
4,
10,
16,
22,
30,
31,
32,
28,
31,
33,
29,
31,
32,
33,
34,
32,
33,
35,
32,
33,
36,
37,
30,
34,
35,
36,
31,
};
static c_float linsys_solver_KKT_x[113] = {
(c_float)-10.00000000000000000000,
(c_float)-10.00000000000000000000,
(c_float)-10.00000000000000000000,
(c_float)-10.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)0.99996340673217254391,
(c_float)0.08394436064918971152,
(c_float)0.99991191522068456621,
(c_float)0.99999273578680225594,
(c_float)1.00000099999999991773,
(c_float)0.36268753596460490130,
(c_float)0.99919971579282240715,
(c_float)0.98784815167469730568,
(c_float)0.08476648702784457534,
(c_float)-10.00000000000000000000,
(c_float)-10.00000000000000000000,
(c_float)-10.00000000000000000000,
(c_float)-10.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)0.99996340673217254391,
(c_float)0.08394436064918971152,
(c_float)0.99991191522068456621,
(c_float)0.99999273578680225594,
(c_float)1.00000099999999991773,
(c_float)0.36268753596460490130,
(c_float)0.99919971579282240715,
(c_float)0.98784815167469730568,
(c_float)0.08476648702784457534,
(c_float)-10.00000000000000000000,
(c_float)-10.00000000000000000000,
(c_float)-10.00000000000000000000,
(c_float)-10.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)0.99996340673217254391,
(c_float)0.08394436064918971152,
(c_float)0.99991191522068456621,
(c_float)0.99999273578680225594,
(c_float)1.00000099999999991773,
(c_float)0.36268753596460490130,
(c_float)0.99919971579282240715,
(c_float)0.98784815167469730568,
(c_float)0.08476648702784457534,
(c_float)-10.00000000000000000000,
(c_float)-10.00000000000000000000,
(c_float)-10.00000000000000000000,
(c_float)-10.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)0.99996340673217254391,
(c_float)0.08394436064918971152,
(c_float)0.99991191522068456621,
(c_float)0.99999273578680225594,
(c_float)1.00000099999999991773,
(c_float)0.36268753596460490130,
(c_float)0.99919971579282240715,
(c_float)0.98784815167469730568,
(c_float)0.08476648702784457534,
(c_float)1.00000099999999991773,
(c_float)0.89103152957089282182,
(c_float)0.60343484293843918653,
(c_float)0.16773655597217329571,
(c_float)0.72539687770744731843,
(c_float)1.00000099999999991773,
(c_float)0.89103152957089282182,
(c_float)0.60343484293843918653,
(c_float)0.16773655597217329571,
(c_float)0.72539687770744731843,
(c_float)1.00000099999999991773,
(c_float)0.89103152957089282182,
(c_float)0.60343484293843918653,
(c_float)0.16773655597217329571,
(c_float)0.72539687770744731843,
(c_float)1.00000099999999991773,
(c_float)0.89103152957089282182,
(c_float)0.60343484293843918653,
(c_float)0.16773655597217329571,
(c_float)0.72539687770744731843,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)-0.01000000000000000021,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)-0.01000000000000000021,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)1.00000000000000000000,
(c_float)-0.01000000000000000021,
(c_float)-10.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)1.00000000000000000000,
(c_float)0.72539687770744731843,
(c_float)1.00000099999999991773,
(c_float)1.00000000000000000000,
(c_float)0.08476648702784457534,
(c_float)0.89103152957089282182,
(c_float)0.36268753596460490130,
(c_float)-10.00000000000000000000,
(c_float)0.60343484293843918653,
(c_float)0.99919971579282240715,
(c_float)-10.00000000000000000000,
(c_float)0.16773655597217329571,
(c_float)0.98784815167469730568,
(c_float)-10.00000000000000000000,
(c_float)1.00000099999999991773,
(c_float)1.00000000000000000000,
(c_float)0.99996340673217254391,
(c_float)0.08394436064918971152,
(c_float)0.99991191522068456621,
(c_float)0.99999273578680225594,
};
static csc linsys_solver_KKT = {113, 38, 38, linsys_solver_KKT_p, linsys_solver_KKT_i, linsys_solver_KKT_x, -1};
static c_int linsys_solver_PtoKKT[15] = {
92,
95,
107,
71,
9,
4,
66,
23,
18,
61,
37,
32,
56,
51,
46,
};
static c_int linsys_solver_AtoKKT[75] = {
93,
98,
101,
104,
94,
96,
99,
102,
105,
97,
108,
109,
110,
111,
112,
76,
72,
73,
74,
75,
81,
10,
11,
12,
13,
86,
5,
6,
7,
8,
77,
67,
68,
69,
70,
82,
24,
25,
26,
27,
87,
19,
20,
21,
22,
78,
62,
63,
64,
65,
83,
38,
39,
40,
41,
88,
33,
34,
35,
36,
79,
57,
58,
59,
60,
84,
52,
53,
54,
55,
89,
47,
48,
49,
50,
};
static c_int linsys_solver_rhotoKKT[23] = {
80,
85,
90,
100,
103,
106,
91,
3,
2,
1,
0,
17,
16,
15,
14,
31,
30,
29,
28,
45,
44,
43,
42,
};
static QDLDL_float linsys_solver_D[38] = {
-10,
-10,
-10,
-10,
1,
1,
-10,
-10,
-10,
-10,
1,
1,
-10,
-10,
-10,
-10,
1,
1,
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
-3,
-3,
-3,
-10,
1,
1,
-10,
-10,
-10,
1,
};
static QDLDL_int linsys_solver_etree[38] = {
4,
4,
4,
4,
5,
27,
10,
10,
10,
10,
11,
26,
16,
16,
16,
16,
17,
25,
22,
22,
22,
22,
23,
24,
28,
28,
28,
28,
29,
30,
32,
32,
33,
34,
35,
36,
37,
-1,
};
static QDLDL_int linsys_solver_Lnz[38] = {
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
3,
5,
4,
3,
2,
1,
0,
};
static QDLDL_int   linsys_solver_iwork[114];
static QDLDL_bool  linsys_solver_bwork[38];
static QDLDL_float linsys_solver_fwork[38];
static qdldl_solver linsys_solver = {QDLDL_SOLVER, &solve_linsys_qdldl, &update_linsys_solver_matrices_qdldl, &update_linsys_solver_rho_vec_qdldl, &linsys_solver_L, linsys_solver_Dinv, linsys_solver_P, linsys_solver_bp, linsys_solver_sol, linsys_solver_rho_inv_vec, (c_float)0.00000100000000000000, 15, 23, linsys_solver_Pdiag_idx, 15, &linsys_solver_KKT, linsys_solver_PtoKKT, linsys_solver_AtoKKT, linsys_solver_rhotoKKT, linsys_solver_D, linsys_solver_etree, linsys_solver_Lnz, linsys_solver_iwork, linsys_solver_bwork, linsys_solver_fwork, };

// Define solution
static c_float xsolution[15];
static c_float ysolution[23];

static OSQPSolution solution = {xsolution, ysolution};

// Define info
static OSQPInfo info = {0, "Unsolved", OSQP_UNSOLVED, 0.0, 0.0, 0.0};

// Define workspace
static c_float work_rho_vec[23] = {
(c_float)100.00000000000000000000,
(c_float)100.00000000000000000000,
(c_float)100.00000000000000000000,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
(c_float)0.10000000000000000555,
};
static c_float work_rho_inv_vec[23] = {
(c_float)0.01000000000000000021,
(c_float)0.01000000000000000021,
(c_float)0.01000000000000000021,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
(c_float)10.00000000000000000000,
};
static c_int work_constr_type[23] = {
1,
1,
1,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
};
static c_float work_x[15];
static c_float work_y[23];
static c_float work_z[23];
static c_float work_xz_tilde[38];
static c_float work_x_prev[15];
static c_float work_z_prev[23];
static c_float work_Ax[23];
static c_float work_Px[15];
static c_float work_Aty[15];
static c_float work_delta_y[23];
static c_float work_Atdelta_y[15];
static c_float work_delta_x[15];
static c_float work_Pdelta_x[15];
static c_float work_Adelta_x[23];
static c_float work_D_temp[15];
static c_float work_D_temp_A[15];
static c_float work_E_temp[23];

OSQPWorkspace workspace_5uav_9hp = {
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
