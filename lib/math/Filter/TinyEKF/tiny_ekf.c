/*
 * TinyEKF: Extended Kalman Filter for embedded processors
 *
 * Copyright (C) 2015 Simon D. Levy
 *
 * MIT License
 */

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

/* Cholesky-decomposition matrix-inversion code, adapated from
   http://jean-pierre.moreau.pagesperso-orange.fr/Cplus/choles_cpp.txt */

static int choldc1(double * a, double * p, int n) { //fungsi untuk....., parameter a berupa vektor, p berupa vektor, dan n berupa elemen matriks 
    int i,j,k;
    double sum;     // ini variabel apa ? h- variansi - nilai akar sum disimpan pada elemen mat p - ine 30

    for (i = 0; i < n; i++) {               // i sebagai counter iterasi teratas
        for (j = i; j < n; j++) {           // j sebagai kounter yang startnya i, awal iterasi bertambah seiring iterasi diatasnya
            sum = a[i*n+j];                 // asignment nilai variabel dengan elemen matriks a dengan cara .....
            for (k = i - 1; k >= 0; k--) {  // k sebagi counter iterasi terbawah, iterasi decrement,
                sum -= a[i*n+k] * a[j*n+k]; // kurangi nilai var sum dengan fungsi nilai 2 elemen matriks a
            }
            if (i == j) {                   // ketika kountr i = j , posisi diagonal matriks ? ...  
                if (sum <= 0) {             // dan nilai sum negatif (h- berkaitan dengan variansi negatif)
                    return 1; /* error */
                }
                p[i] = sqrt(sum);           // jika nilai sum positif, asignment nilai vektor P elemen i dengan nilai akar sum (h- standar deviasi) 
            }
            else {                          // jika nilai elemen i dan j tidak sama, isi nilai elemen vektor a dengan nilaai sum/p{i} (h- sum dan elemen vektor p memiliki besaran yang sama) 
                a[j*n+i] = sum / p[i];      // mengisi nilai elemen [j*n+i] vektor a dengan nilai sum/p (apa itu sum/p ? apa isi nilai vektro a)
            }
        }       //note : pahami maksud nilai [i*n+k] dan a[i*n+j], pahami nilai n, makna i dan j dalam matriks yang diakses, apa itu vektor p, apa itu vektor a
    }

    return 0; /* success */
}

static int choldcsl(double * A, double * a, double * p, int n) // fungsi untuk...., parameter yang sama dengan yang atas
{
    int i,j,k; double sum;
    for (i = 0; i < n; i++) 
        for (j = 0; j < n; j++) 
            a[i*n+j] = A[i*n+j];
    if (choldc1(a, p, n)) return 1;
    for (i = 0; i < n; i++) {
        a[i*n+i] = 1 / p[i];
        for (j = i + 1; j < n; j++) {
            sum = 0;
            for (k = i; k < j; k++) {
                sum -= a[j*n+k] * a[k*n+i];
            }
            a[j*n+i] = sum / p[j];
        }
    }

    return 0; /* success */
}


static int cholsl(double * A, double * a, double * p, int n) //fungsi matriks, parameter berbed pada A, selain itu sama 
{
    int i,j,k;
    if (choldcsl(A,a,p,n)) return 1;
    for (i = 0; i < n; i++) {
        for (j = i + 1; j < n; j++) {
            a[i*n+j] = 0.0;
        }
    }
    for (i = 0; i < n; i++) {
        a[i*n+i] *= a[i*n+i];
        for (k = i + 1; k < n; k++) {
            a[i*n+i] += a[k*n+i] * a[k*n+i];
        }
        for (j = i + 1; j < n; j++) {
            for (k = j; k < n; k++) {
                a[i*n+j] += a[k*n+i] * a[k*n+j];
            }
        }
    }
    for (i = 0; i < n; i++) {
        for (j = 0; j < i; j++) {
            a[i*n+j] = a[j*n+i];
        }
    }

    return 0; /* success */
}

static void zeros(double * a, int m, int n) // fungsi untuk membuat nilai matriks a (sebaagai parameter) menjadi bernilai 0
{                                           // m merupakan parameter 
    int j;
    for (j=0; j<m*n; ++j)                   //apa makna m*n ? yang jelas m*n masih merupakan elemen matriks a, h- elemen terakhir
        a[j] = 0;                           
}

#ifdef DEBUG
static void dump(double * a, int m, int n, const char * fmt)
{
    int i,j;

    char f[100];
    sprintf(f, "%s ", fmt);
    for(i=0; i<m; ++i) {
        for(j=0; j<n; ++j)
            printf(f, a[i*n+j]);
        printf("\n");
    }
}
#endif

/* C <- A * B */
static void mulmat(double * a, double * b, double * c, int arows, int acols, int bcols) // fungsi untuk perkalian matriks a dn b dengan hasil c
{
    int i, j,l;

    for(i=0; i<arows; ++i)
        for(j=0; j<bcols; ++j) {
            c[i*bcols+j] = 0;                                // makna ini apa ya ?
            for(l=0; l<acols; ++l)
                c[i*bcols+j] += a[i*acols+l] * b[l*bcols+j]; // nilaai matrks c yang diisi dengan hasil operasi elemen matriks a dan b 
        }
}

static void mulvec(double * a, double * x, double * y, int m, int n) // h- perkalian vektor (multiplication vector)
{
    int i, j;

    for(i=0; i<m; ++i) {
        y[i] = 0;
        for(j=0; j<n; ++j)
            y[i] += x[j] * a[i*n+j];
    }
}

static void transpose(double * a, double * at, int m, int n)
{
    int i,j;

    for(i=0; i<m; ++i)
        for(j=0; j<n; ++j) {
            at[j*m+i] = a[i*n+j];
        }
}

/* A <- A + B */
static void accum(double * a, double * b, int m, int n) // fungsi h - pengisian nilai a  dengan nilai a + b 
{        
    int i,j;

    for(i=0; i<m; ++i)
        for(j=0; j<n; ++j)
            a[i*n+j] += b[i*n+j];
}

/* C <- A + B */
static void add(double * a, double * b, double * c, int n) // fungsi h - pengisian nilai c dengan nilai a + b 
{
    int j;

    for(j=0; j<n; ++j)
        c[j] = a[j] + b[j];
}


/* C <- A - B */
static void sub(double * a, double * b, double * c, int n)
{
    int j;

    for(j=0; j<n; ++j)
        c[j] = a[j] - b[j];
}

static void negate(double * a, int m, int n) // fungsi membuat nilai matriks menjadi negatif
{        
    int i, j;

    for(i=0; i<m; ++i)
        for(j=0; j<n; ++j)
            a[i*n+j] = -a[i*n+j];
}

static void mat_addeye(double * a, int n)   //fungsi menambahkan nilai 1 pada setiap eleen (increment)
{
    int i;
    for (i=0; i<n; ++i)
        a[i*n+i] += 1;
}

/* TinyEKF code ------------------------------------------------------------------- */

#include "tiny_ekf.h"

typedef struct { // pengaksesan menggunakaan pointer, sehinga (nilai yang disimpan variabel input paramaeter) berubah 

    double * x;    /* state vector */

    double * P;  /* prediction error covariance */
    double * Q;  /* process noise covariance */
    double * R;  /* measurement error covariance */

    double * G;  /* Kalman gain; a.k.a. K */

    double * F;  /* Jacobian of process model */
    double * H;  /* Jacobian of measurement model */

    double * Ht; /* transpose of measurement Jacobian */
    double * Ft; /* transpose of process Jacobian */
    double * Pp; /* P, post-prediction, pre-update */

    double * fx;  /* output of user defined f() state-transition function */
    double * hx;  /* output of user defined h() measurement function */

    /* temporary storage */
    double * tmp0;
    double * tmp1;
    double * tmp2;
    double * tmp3;
    double * tmp4;
    double * tmp5; 

} ekf_t;    // adalah struct dengan nama ekf_t yang didefinisikan dalaam typedef

static void unpack(void * v, ekf_t * ekf, int n, int m)// fungsi untuk ... , parameter v adalah ..., parameter ekf adalah objek
{
    /* skip over n, m in data structure */
    char * cptr = (char *)v;        // membuat pointer bertipe char yg mengacu alamat yg sama dg yg diacu pointer v
    cptr += 2*sizeof(int);          // pengacuan alamat lompat sebanyak 2 data bertipe int   

    double * dptr = (double *)cptr; // membuat pointer bertipe double yang mengacu alamat yg sama dg yg diacu cptr
    ekf->x = dptr;                  // asignment nilai member ekf dengan nilai variabel yg diacu dptr
    dptr += n;                      // pengacuan dptr bergeser sejumlah n (jumlah state)  
    ekf->P = dptr;                  // jadi ini adalah pengisian nilai matrix P,Q,R,G,F,Ht dll
    dptr += n*n;                    // penggeseran pengacuan alamat menjadi n^2
    ekf->Q = dptr;
    dptr += n*n;
    ekf->R = dptr;
    dptr += m*m;
    ekf->G = dptr;                  // intinya, meng-unpack vektor input v yang berisi variabel2 observed       
    dptr += n*m;
    ekf->F = dptr;
    dptr += n*n;
    ekf->H = dptr;
    dptr += m*n;
    ekf->Ht = dptr;
    dptr += n*m;
    ekf->Ft = dptr;
    dptr += n*n;
    ekf->Pp = dptr;
    dptr += n*n;
    ekf->fx = dptr;
    dptr += n;
    ekf->hx = dptr;
    dptr += m;
    ekf->tmp0 = dptr;
    dptr += n*n;
    ekf->tmp1 = dptr;
    dptr += n*m;
    ekf->tmp2 = dptr;
    dptr += m*n;
    ekf->tmp3 = dptr;
    dptr += m*m;
    ekf->tmp4 = dptr;
    dptr += m*m;
    ekf->tmp5 = dptr;
  } 

void ekf_init(void * v, int n, int m)
{
    /* retrieve n, m and set them in incoming data structure */
    int * ptr = (int *)v;   // membuat pointer yang mengacu alamat yg sama dg v dg tipe pointer integer
    *ptr = n;               // member pertama ekf_t adalah n, ini adalah set nilai n 
    ptr++;                  // geser ke alamat yang diacu berikutnya yaitu nilai m - lihat struct_config
    *ptr = m;               // asigment nilai m.......done 

    /* unpack rest of incoming structure for initlization */
    ekf_t ekf;              // objek ekf dibuat untuk lokal scoop ini saja
    unpack(v, &ekf, n, m);  // semacam copy objek, nilai ekf diisi nilai v, v yg dicopykan ke ekf

    /* zero-out matrices */
    zeros(ekf.P, n, n);     // fungsi asigment nilai 0 untuk member ekf
    zeros(ekf.Q, n, n);
    zeros(ekf.R, m, m);
    zeros(ekf.G, n, m);
    zeros(ekf.F, n, n);
    zeros(ekf.H, m, n);
}

int ekf_step(void * v, double * z)  // inti dari sensor fusion, inti program dan perhitungan, ayo ikuti
{        
    /* unpack incoming structure */

    int * ptr = (int *)v;           // membuat pointer bertipe int yang mengacu alamt yg sama dg alamat yg dicu pointer v
    int n = *ptr;                   // n berisi nilai variabel yang diacu pointer ptr saa ini
    ptr++;                          // pointer bergeser ke alamat setelahnya
    int m = *ptr;                   // nilai m berisi nilai variabel yang diacu sebelah n, alamat memori setelah n

    ekf_t ekf;                      // instansiasi objek
    unpack(v, &ekf, n, m);          // fungsi untuk ..... (operasi matriks) - h mengisi nilai parameter pada ekf, cari arti parameter
 
    /* P_k = F_{k-1} P_{k-1} F^T_{k-1} + Q_{k-1} */     // h- matriks p kovariansi eror prediksi (prediction error covariance)
    mulmat(ekf.F, ekf.P, ekf.tmp0, n, n, n);            // p ukuran [state][state]
    transpose(ekf.F, ekf.Ft, n, n);                     // pengoperasian matriks dan nilai matriks erubah di sini
    mulmat(ekf.tmp0, ekf.Ft, ekf.Pp, n, n, n);
    accum(ekf.Pp, ekf.Q, n, n);

    /* G_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1} */      // matiks G, matriks kalman gain [state][observed] 
    transpose(ekf.H, ekf.Ht, m, n);
    mulmat(ekf.Pp, ekf.Ht, ekf.tmp1, n, n, m);
    mulmat(ekf.H, ekf.Pp, ekf.tmp2, m, n, n);
    mulmat(ekf.tmp2, ekf.Ht, ekf.tmp3, m, n, m);
    accum(ekf.tmp3, ekf.R, m, m);
    if (cholsl(ekf.tmp3, ekf.tmp4, ekf.tmp5, m)) return 1;
    mulmat(ekf.tmp1, ekf.tmp4, ekf.G, n, m, m);

    /* \hat{x}_k = \hat{x_k} + G_k(z_k - h(\hat{x}_k)) */
    sub(z, ekf.hx, ekf.tmp5, m);
    mulvec(ekf.G, ekf.tmp5, ekf.tmp2, n, m);
    add(ekf.fx, ekf.tmp2, ekf.x, n);

    /* P_k = (I - G_k H_k) P_k */               //jadi, yang terjadi ketika progam ini dijalankan adalah mengoperasikan nilai elemen matriks
    mulmat(ekf.G, ekf.H, ekf.tmp0, n, m, n);    // fungsi perkalian matriks (mat a, mat b, mat hasil, ukuran, ukuran,ukuran)
    negate(ekf.tmp0, n, n);                     // pengoperasian dilakukan dengan pointer
    mat_addeye(ekf.tmp0, n);
    mulmat(ekf.tmp0, ekf.Pp, ekf.P, n, n, n);

    /* success */
    return 0;
}
