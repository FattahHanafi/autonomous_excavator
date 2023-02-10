#include <gsl/gsl_bspline.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_statistics.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

/* number of data points to fit */
#define N 1e3

/* number of fit coefficients */
#define NCOEFFS 12

/* nbreak = ncoeffs + 2 - k = ncoeffs - 2 since k = 4 */
#define NBREAK (NCOEFFS - 2)

struct Timer {
  Timer() {
    m_start = std::chrono::high_resolution_clock().now();
  }
  ~Timer() {
    m_end = std::chrono::high_resolution_clock().now();
    std::cout << "It took = " << std::chrono::duration_cast<std::chrono::microseconds>(m_end - m_start).count() << " micros" << std::endl;
  }
  void end() {
    m_end = std::chrono::high_resolution_clock().now();
    std::cout << "It took = " << std::chrono::duration_cast<std::chrono::microseconds>(m_end - m_start).count() << " micros" << std::endl;
  }

 private:
  std::chrono::system_clock::time_point m_start;
  std::chrono::system_clock::time_point m_end;
};

int main(void) {
  const size_t n = N;
  const size_t ncoeffs = NCOEFFS;
  const size_t nbreak = NBREAK;
  size_t i, j;
  gsl_bspline_workspace *bw;
  gsl_vector *B;
  double dy;
  gsl_rng *r;
  gsl_vector *c, *w;
  gsl_vector *x, *y;
  gsl_matrix *X, *cov;
  gsl_multifit_linear_workspace *mw;
  double chisq, Rsq, dof, tss;

  {
    Timer timer;
    std::thread thr1{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr2{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr3{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr4{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr5{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr6{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr7{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr8{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr9{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr10{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr11{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr12{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr13{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr14{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr15{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr16{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr17{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr18{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr19{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr20{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr21{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr22{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr23{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr24{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr25{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr26{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr27{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr28{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr29{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr30{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr31{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr32{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr33{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr34{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr35{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr36{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr37{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr38{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr39{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr40{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr41{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr42{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr43{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr44{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr45{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr46{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr47{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr48{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr49{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr50{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr51{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr52{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr53{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr54{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr55{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr56{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr57{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr58{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr59{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr60{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr61{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr62{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr63{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr64{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr65{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr66{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr67{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr68{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr69{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr70{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr71{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr72{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr73{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr74{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr75{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr76{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr77{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr78{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr79{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr80{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr81{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr82{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr83{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr84{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr85{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr86{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr87{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr88{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr89{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr90{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr91{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr92{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr93{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr94{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr95{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr96{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr97{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr98{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr99{[] { std::cout << "Joinable std::thread" << std::endl; }};
    std::thread thr100{[] { std::cout << "Joinable std::thread" << std::endl; }};

    thr1.join();
    thr2.join();
    thr3.join();
    thr4.join();
    thr5.join();
    thr6.join();
    thr7.join();
    thr8.join();
    thr9.join();
    thr10.join();
    thr11.join();
    thr12.join();
    thr13.join();
    thr14.join();
    thr15.join();
    thr16.join();
    thr17.join();
    thr18.join();
    thr19.join();
    thr20.join();
    thr21.join();
    thr22.join();
    thr23.join();
    thr24.join();
    thr25.join();
    thr26.join();
    thr27.join();
    thr28.join();
    thr29.join();
    thr30.join();
    thr31.join();
    thr32.join();
    thr33.join();
    thr34.join();
    thr35.join();
    thr36.join();
    thr37.join();
    thr38.join();
    thr39.join();
    thr40.join();
    thr41.join();
    thr42.join();
    thr43.join();
    thr44.join();
    thr45.join();
    thr46.join();
    thr47.join();
    thr48.join();
    thr49.join();
    thr50.join();
    thr51.join();
    thr52.join();
    thr53.join();
    thr54.join();
    thr55.join();
    thr56.join();
    thr57.join();
    thr58.join();
    thr59.join();
    thr60.join();
    thr61.join();
    thr62.join();
    thr63.join();
    thr64.join();
    thr65.join();
    thr66.join();
    thr67.join();
    thr68.join();
    thr69.join();
    thr70.join();
    thr71.join();
    thr72.join();
    thr73.join();
    thr74.join();
    thr75.join();
    thr76.join();
    thr77.join();
    thr78.join();
    thr79.join();
    thr80.join();
    thr81.join();
    thr82.join();
    thr83.join();
    thr84.join();
    thr85.join();
    thr86.join();
    thr87.join();
    thr88.join();
    thr89.join();
    thr90.join();
    thr91.join();
    thr92.join();
    thr93.join();
    thr94.join();
    thr95.join();
    thr96.join();
    thr97.join();
    thr98.join();
    thr99.join();
    thr100.join();
  }
  gsl_rng_env_setup();
  r = gsl_rng_alloc(gsl_rng_default);

  /* allocate a cubic bspline workspace (k = 4) */
  bw = gsl_bspline_alloc(4, nbreak);
  B = gsl_vector_alloc(ncoeffs);

  x = gsl_vector_alloc(n);
  y = gsl_vector_alloc(n);
  X = gsl_matrix_alloc(n, ncoeffs);
  c = gsl_vector_alloc(ncoeffs);
  w = gsl_vector_alloc(n);
  cov = gsl_matrix_alloc(ncoeffs, ncoeffs);
  mw = gsl_multifit_linear_alloc(n, ncoeffs);

  /* this is the data to be fitted */
  for (i = 0; i < n; ++i) {
    double sigma;
    double xi = (15.0 / (N - 1)) * i;
    double yi = cos(xi) * exp(-0.1 * xi);

    sigma = 0.1 * yi;
    dy = gsl_ran_gaussian(r, sigma);
    yi += dy;

    gsl_vector_set(x, i, xi);
    gsl_vector_set(y, i, yi);
    gsl_vector_set(w, i, 1.0 / (sigma * sigma));
  }

  /* use uniform breakpoints on [0, 15] */
  gsl_bspline_knots_uniform(0.0, 15.0, bw);

  /* construct the fit matrix X */
  for (i = 0; i < n; ++i) {
    double xi = gsl_vector_get(x, i);

    /* compute B_j(xi) for all j */
    gsl_bspline_eval(xi, B, bw);

    /* fill in row i of X */
    for (j = 0; j < ncoeffs; ++j) {
      double Bj = gsl_vector_get(B, j);
      gsl_matrix_set(X, i, j, Bj);
    }
  }

  /* do the fit */
  gsl_multifit_wlinear(X, w, y, c, cov, &chisq, mw);

  dof = n - ncoeffs;
  tss = gsl_stats_wtss(w->data, 1, y->data, 1, y->size);
  Rsq = 1.0 - chisq / tss;

  // fprintf(stderr, "chisq/dof = %e, Rsq = %f\n", chisq / dof, Rsq);

  // printf("\n\n");

  /* output the smoothed curve */
  {
    double xi, yi, yerr;

    for (xi = 0.0; xi < 15.0; xi += (15.0 / N)) {
      gsl_bspline_eval(xi, B, bw);
      gsl_multifit_linear_est(B, c, cov, &yi, &yerr);
      // printf("%f %f\n", xi, yi);
    }
  }

  gsl_rng_free(r);
  gsl_bspline_free(bw);
  gsl_vector_free(B);
  gsl_vector_free(x);
  gsl_vector_free(y);
  gsl_matrix_free(X);
  gsl_vector_free(c);
  gsl_vector_free(w);
  gsl_matrix_free(cov);
  gsl_multifit_linear_free(mw);

  return 0;
} /* main() */
