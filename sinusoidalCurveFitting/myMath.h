#pragma once

#include <cstdint>
#include <valarray>

#define PI 3.14159265359

class myMath
{
public:
  static void sinusoidLeastSquaresFit(double * xbuf, double * ybuf, int size, double * results);

//private:
  static double * filterData(double * rawData, int length, int rollingAvgSize);
  /* matrix operations */
  static double ** createMatrix(int rows, int cols);
  static void deleteMatrix(double ** matrix, int rows);
  static double ** invertMatrix(double ** matrix, int rows);
  static double determinant(double ** matrix, int size);
  static double ** getMinorMatrix(double ** matrix, int row, int col, int size);
  static double ** matrixMult(double ** matrix1, int rows1, int cols1, double ** matrix2, int rows2, int cols2);

  /* Newton-raphson method */
  static void NewtonRaphson(double * initialGuessParams, double * xbuf, double * ybuf, int length, double * resultsBuf, bool lockedFrequency = false);
  static double y_model(double * paramsBuf, double x);
  static double dedX(double * paramsBuf, double * xbuf, double * ybuf, int size, double(*dydX)(double *, double));
  static double de2dXdY(double * paramsBuf, double * xbuf, double * ybuf, int size,
    double(*dydX)(double *, double),
    double(*dydY)(double *, double),
    double(*dy2dXdY)(double *, double));

  /* derivatives */
  static double dydw(double * paramsBuf, double x);
  static double dyda(double * paramsBuf, double x);
  static double dydb(double * paramsBuf, double x);
  static double dydc(double * paramsBuf, double x);
  static double d2ydw2(double * paramsBuf, double x);
  static double d2ydwda(double * paramsBuf, double x);
  static double d2ydwdb(double * paramsBuf, double x);
  static double d2ydwdc(double * paramsBuf, double x);
  static double d2yda2(double * paramsBuf, double x);
  static double d2ydadb(double * paramsBuf, double x);
  static double d2ydadc(double * paramsBuf, double x);
  static double d2ydb2(double * paramsBuf, double x);
  static double d2ydbdc(double * paramsBuf, double x);
  static double d2ydc2(double * paramsBuf, double x);
};