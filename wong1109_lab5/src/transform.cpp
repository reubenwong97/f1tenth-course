#include "wong1109_lab5/transform.h"
#include "geometry_msgs/Point.h"
#include "ros/console.h"
#include "ros/ros.h"
#include <Eigen/Geometry>
#include <cmath>
#include <complex>

using namespace std;

void printMatrixShape(const Eigen::MatrixXf &mat) {
  cout << "shape: (" << mat.rows() << "," << mat.cols() << ")" << endl;
}

void transformPoints(const vector<Point> &points, Transform &t,
                     vector<Point> &transformed_points) {
  transformed_points.clear();
  for (int i = 0; i < points.size(); i++) {
    transformed_points.push_back(t.apply(points[i]));
    // printf("%f %transformed_points.back().r,
    // transformed_points.back().theta);
  }
}

// returns the largest real root to ax^3 + bx^2 + cx + d = 0
complex<float> get_cubic_root(float a, float b, float c, float d) {
  // std::cout<< "a= " << a<< ";  b= " << b<< ";  c= " << c<< ";  d= " <<
  // d<<";"<<std::endl;
  // Reduce to depressed cubic
  float p = c / a - b * b / (3 * a * a);
  float q = 2 * b * b * b / (27 * a * a * a) + d / a - b * c / (3 * a * a);

  // std::cout<<"p = "<<p<<";"<<std::endl;
  // std::cout<<"q = "<<q<<";"<<std::endl;

  complex<float> xi(-.5, sqrt(3) / 2);

  complex<float> inside = sqrt(q * q / 4 + p * p * p / 27);

  complex<float> root;

  for (float k = 0; k < 3; ++k) {
    // get root for 3 possible values of k
    root = -b / (3 * a) + pow(xi, k) * pow(-q / 2.f + inside, 1.f / 3.f) +
           pow(xi, 2.f * k) * pow(-q / 2.f - inside, 1.f / 3.f);
    // std::cout<<"RootTemp: "<< root<<std::endl;
    if (root.imag() != 0) {
      return root;
    }
  }

  return root;
}

// returns the largest real root to ax^4 + bx^3 + cx^2 + dx + e = 0
float greatest_real_root(float a, float b, float c, float d, float e) {
  // Written with inspiration from:
  // https://en.wikipedia.org/wiki/Quartic_function#General_formula_for_roots
  // std::cout<< "a= " << a<< ";  b= " << b<< ";  c= " << c<< ";  d= " << d<< ";
  // e= " << e<<";"<<std::endl;

  // Reduce to depressed Quadratic
  float p = (8 * a * c - 3 * b * b) / (8 * a * a);
  float q = (b * b * b - 4 * a * b * c + 8 * a * a * d) / (8 * a * a * a);
  float r = (-3 * b * b * b * b + 256 * a * a * a * e - 64 * a * a * b * d +
             16 * a * b * b * c) /
            (256 * a * a * a * a);

  // std::cout<<"p = "<<p<<";"<<std::endl;
  // std::cout<<"q = "<<q<<";"<<std::endl;
  // std::cout<<"r = "<<r<<";"<<std::endl;

  // Ferrari's Solution for Quartics: 8m^3 + 8pm^2 + (2p^2-8r)m - q^2 = 0
  complex<float> m = get_cubic_root(8, 8 * p, 2 * p * p - 8 * r, -q * q);

  complex<float> root1 =
      -b / (4 * a) +
      (sqrt(2.f * m) + sqrt(-(2 * p + 2.f * m + sqrt(2.f) * q / sqrt(m)))) /
          2.f;
  complex<float> root2 =
      -b / (4 * a) +
      (sqrt(2.f * m) - sqrt(-(2 * p + 2.f * m + sqrt(2.f) * q / sqrt(m)))) /
          2.f;
  complex<float> root3 =
      -b / (4 * a) +
      (-sqrt(2.f * m) + sqrt(-(2 * p + 2.f * m - sqrt(2.f) * q / sqrt(m)))) /
          2.f;
  complex<float> root4 =
      -b / (4 * a) +
      (-sqrt(2.f * m) - sqrt(-(2 * p + 2.f * m - sqrt(2.f) * q / sqrt(m)))) /
          2.f;

  vector<complex<float>> roots{root1, root2, root3, root4};

  float max_real_root = 0.f;

  for (complex<float> root : roots) {
    if (root.imag() == 0) {
      max_real_root = max(max_real_root, root.real());
    }
  }
  // std::cout<<"Max real root:" << max_real_root<<std::endl;

  return max_real_root;
}

void updateTransform(vector<Correspondence> &corresponds,
                     Transform &curr_trans) {
  // Written with inspiration from:
  // https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c You can use the
  // helper functions which are defined above for finding roots and transforming
  // points as and when needed. use helper functions and structs in transform.h
  // and correspond.h input : corresponds : a struct vector of Correspondene
  // struct object defined in correspond. input : curr_trans : A Transform
  // object refernece output : update the curr_trans object. Being a call by
  // reference function, Any changes you make to curr_trans will be reflected in
  // the calling function in the scan_match.cpp program/

  // You can change the number of iterations here. More the number of
  // iterations, slower will be the convergence but more accurate will be the
  // results. You need to find the right balance.

  // ROS_INFO("Entering update transform");
  int number_iter = 2;

  for (int i = 0; i < number_iter; i++) {

    Eigen::MatrixXf M_i(2, 4);
    Eigen::Matrix2f C_i;
    Eigen::Vector2f pi_i;
    Eigen::Vector2f n_i;

    // Fill in the values for the matrices
    Eigen::Matrix4f M, W;
    // Eigen::MatrixXf g(4, 1);
    Eigen::MatrixXf g(1, 4);
    // Eigen::MatrixXf g_t(1, 4);

    M << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    W << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

    g << 0, 0, 0, 0;
    // g_t << 0, 0, 0, 0;

    // ROS_INFO("Filling in M and g");
    for (Correspondence c : corresponds) {
      n_i = c.getNormalNorm();
      M_i << 1, 0, c.p->getX(), -c.p->getY(), 0, 1, c.p->getY(), c.p->getX();
      C_i << n_i * n_i.transpose();
      pi_i << c.pj1->getX(), c.pj1->getY();

      M += M_i.transpose() * C_i * M_i;
      g -= 2 * pi_i.transpose() * C_i * M_i;
    }
    // ROS_INFO("Completed filling M and g");

    // Define sub-matrices A, B, D from M
    // ROS_INFO("Computing res matrix");
    Eigen::MatrixXf res = 2 * M + 2 * W;
    // g << g_t.transpose();
    // ROS_INFO("g has shape (%ld, %ld)", g.rows(), g.cols());
    Eigen::Matrix2f A, B, D;
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(2, 2);
    // Eigen::Matrix2f I = Eigen::Matrix<float, 2, 2>::Identity();
    // ROS_INFO("Computed res matrix, filling in A B and D");

    A = res.block(0, 0, 2, 2);
    B = res.block(0, 2, 2, 2);
    D = res.block(2, 2, 2, 2);
    D -= 2 * I;
    cout << "res: " << res << endl;
    cout << "A: " << A << endl;
    cout << "B: " << B << endl;
    cout << "D: " << D << endl;

    // define S and S_A matrices from the matrices A B and D
    Eigen::Matrix2f S;
    Eigen::Matrix2f S_A;

    // ROS_INFO("Computing S and S_A");
    S << D - B.transpose() * A.inverse() * B;
    S_A << S.determinant() * S.inverse();
    // ROS_INFO("Completed computation of S and S_A");

    // find the coefficients of the quadratic function of lambda
    float pow_2;
    float pow_1;
    float pow_0;

    Eigen::MatrixXf interm_pow2(4, 4);
    Eigen::MatrixXf interm_pow1(4, 4);
    Eigen::MatrixXf interm_pow0(4, 4);

    // ROS_INFO("Computing intermediate matrices");
    // compute matrix for pow 2 term
    interm_pow2.topLeftCorner(2, 2) =
        A.inverse() * B * B.transpose() * A.inverse().transpose();
    interm_pow2.topRightCorner(2, 2) = -A.inverse() * B;
    interm_pow2.bottomLeftCorner(2, 2) = -A.inverse() * B;
    interm_pow2.bottomRightCorner(2, 2) = Eigen::MatrixXf::Identity(2, 2);

    // compute matrix for pow 1 term
    interm_pow1.topLeftCorner(2, 2) =
        A.inverse() * B * S_A * B.transpose() * A.inverse().transpose();
    interm_pow1.topRightCorner(2, 2) = -A.inverse() * B * S_A;
    interm_pow1.bottomLeftCorner(2, 2) = -A.inverse() * B * S_A;
    interm_pow1.bottomRightCorner(2, 2) = S_A;

    // compute matrix for pow 0 term
    interm_pow0.topLeftCorner(2, 2) = A.inverse() * B * S_A.transpose() * S_A *
                                      B.transpose() * A.inverse().transpose();
    interm_pow0.topRightCorner(2, 2) = -A.inverse() * B * S_A.transpose() * S_A;
    interm_pow0.bottomLeftCorner(2, 2) =
        -A.inverse() * B * S_A.transpose() * S_A;
    interm_pow0.bottomRightCorner(2, 2) = S_A.transpose() * S_A;

    // ROS_INFO("Intermediate matrices computed");

    // compute actual coefficients
    // pow_2 = 4 * (g.transpose() * interm_pow2 * g)(0);
    // pow_1 = 4 * (g.transpose() * interm_pow1 * g)(0);
    // pow_0 = (g.transpose() * interm_pow0 * g)(0);

    // HACK: annoying, have to flip the transposes around
    pow_2 = 4 * (g * interm_pow2 * g.transpose())(0);
    pow_1 = 4 * (g * interm_pow1 * g.transpose())(0);
    pow_0 = (g * interm_pow0 * g.transpose())(0);

    // find the value of lambda by solving the equation formed. You can use the
    // greatest real root function
    float lambda;
    // ROS_INFO("Computing lambda");
    lambda = greatest_real_root(0, 0, pow_2, pow_1, pow_0);
    cout << "M: " << M << endl;
    cout << "g: " << g << endl;
    cout << "pow2: " << pow_2 << endl;
    cout << "pow1: " << pow_1 << endl;
    cout << "pow0: " << pow_0 << endl;
    cout << "lambda: " << lambda << endl;

    // find the value of x which is the vector for translation and rotation
    Eigen::Vector4f x;
    // x = -(2 * M + 2 * lambda * W.inverse()).transpose() *
    //     g;
    // lecture formula instead of lab
    // Convert from x to new transform
    // HACK: due to inconsistency, g becomes g.T
    x = -((2 * M + 2 * lambda * W).inverse()).transpose() * g.transpose();
    // ROS_INFO("x is: %f %f %f %f", x(0), x(1), x(2), x(3));
    // ROS_INFO("x has been computed");

    float theta = atan2(x(3), x(2));
    curr_trans = Transform(x(0), x(1), theta);
  }
}
