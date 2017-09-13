#include "myMath.h"




double * myMath::filterData(double * rawData, int length, int rollingAvgSize)
{
  int newLength = length - rollingAvgSize;
  double * filteredData = new double[newLength];

  /* low-pass filter */
  for (int i = 0; i < newLength; i++)
  {
    filteredData[i] = 0;
    for (int j = 0; j < rollingAvgSize; j++)
    {
      filteredData[i] += rawData[i + j];
    }
    filteredData[i] /= rollingAvgSize;
  }

  /* "high-pass filter" (remove average value) */
  double avg = 0;
  for (int i = 0; i < newLength; i++)
  {
    avg += filteredData[i];
  }
  avg /= newLength;
  for (int i = 0; i < newLength; i++)
  {
    filteredData[i] -= avg;
  }

  return filteredData;
}

void myMath::sinusoidLeastSquaresFit(double * xbuf, double * ybuf, int size, double * results)
{
  double * S = new double[size];
  double * SS = new double[size];
  S[0] = SS[0] = 0;
  for (int i = 1; i < size; i++)
  {
    S[i] = S[i - 1] + 0.5*(ybuf[i] + ybuf[i - 1])*(xbuf[i] - xbuf[i - 1]);
    SS[i] = SS[i - 1] + 0.5*(S[i] + S[i - 1])*(xbuf[i] - xbuf[i - 1]);
  }

  double sum_x = 0, sum_x2 = 0, sum_x3 = 0, sum_x4 = 0, sum_SS = 0, sum_SS2 = 0, sum_xSS = 0, sum_x2SS = 0, sum_y = 0, sum_yx = 0, sum_yx2 = 0, sum_ySS = 0;

  for (int i = 0; i < size; i++)
  {
    sum_x += xbuf[i];
    sum_x2 += xbuf[i] * xbuf[i];
    sum_x3 += xbuf[i] * xbuf[i] * xbuf[i];
    sum_x4 += xbuf[i] * xbuf[i] * xbuf[i] * xbuf[i];
    sum_SS += SS[i];
    sum_SS2 += SS[i] * SS[i];
    sum_xSS += xbuf[i] * SS[i];
    sum_x2SS += xbuf[i] * xbuf[i] * SS[i];
    sum_y += ybuf[i];
    sum_yx += ybuf[i] * xbuf[i];
    sum_yx2 += ybuf[i] * xbuf[i] * xbuf[i];
    sum_ySS += ybuf[i] * SS[i];
  }

  double ** matrix1 = createMatrix(4, 4);
  double ** matrix2 = createMatrix(4, 1);

  /* Initialize matrix1 */
  matrix1[0][0] = sum_SS2;
  matrix1[0][1] = sum_x2SS;
  matrix1[0][2] = sum_xSS;
  matrix1[0][3] = sum_SS;
  
  matrix1[1][0] = sum_x2SS;
  matrix1[1][1] = sum_x4;
  matrix1[1][2] = sum_x3;
  matrix1[1][3] = sum_x2;
  
  matrix1[2][0] = sum_xSS;
  matrix1[2][1] = sum_x3;
  matrix1[2][2] = sum_x2;
  matrix1[2][3] = sum_x;

  matrix1[3][0] = sum_SS;
  matrix1[3][1] = sum_x2;
  matrix1[3][2] = sum_x;
  matrix1[3][3] = size;

  /* initialize matrix2 */
  matrix2[0][0] = sum_ySS;
  matrix2[1][0] = sum_yx2;
  matrix2[2][0] = sum_yx;
  matrix2[3][0] = sum_y;

  double ** invMatrix1 = invertMatrix(matrix1, 4);
  double ** matrixABCD = matrixMult(invMatrix1, 4, 4, matrix2, 4, 1);
  double A1 = matrixABCD[0][0];
  double B1 = matrixABCD[1][0];
  double C1 = matrixABCD[2][0];
  double D1 = matrixABCD[3][0];
  double w1 = sqrt(-A1);
  double a1 = 2 * B1 / w1;
  double b1 = (B1 * xbuf[0] * xbuf[0] + C1 * xbuf[0] + D1 - a1) * sin(w1 * xbuf[0]) + (C1 + 2 * B1 * xbuf[0]) / w1 * cos(w1 * xbuf[0]);
  double c1 = (B1 * xbuf[0] * xbuf[0] + C1 * xbuf[0] + D1 - a1) * cos(w1 * xbuf[0]) + (C1 + 2 * B1 * xbuf[0]) / w1 * sin(w1 * xbuf[0]);

  /* Part 2 */
  double a2 = a1;
  double p1 = sqrt(b1 * b1 + c1 * c1);
  double p2 = p1;
  double theta1 = 0;
  if (b1 > 0)
    theta1 = atan(c1 / b1);
  else if (b1 < 0)
    theta1 = atan(c1 / b1) + PI;
  else if (b1 == 0)
  {
    if (c1 > 0)
      theta1 = PI / 2;
    else
      theta1 = -PI / 2;
  }

  double * K = new double[size];
  for (int i = 0; i < size; i++)
  {
    K[i] = round((w1 * xbuf[i] + theta1) / PI);
  }

  double * Q = new double[size];
  for (int i = 0; i < size; i++)
  {
    if (p2*p2 > pow(ybuf[i] - a2, 2))
      Q[i] = pow(-1, K[i]) * atan((ybuf[i] - a2) / (p2*p2 - pow((ybuf[i] - a2), 2))) + PI * K[i];
    else
    {
      if (ybuf[i] > a2)
      {
        Q[i] = PI / 2 * pow(-1, K[i]) + PI * K[i];
      }
      else
      {
        Q[i] = -PI / 2 * pow(-1, K[i]) + PI * K[i];
      }
    }
  }

  double sum_Q = 0, sum_Qx = 0;
  for (int i = 0; i < size; i++)
  {
    sum_Q += Q[i];
    sum_Qx += Q[i] * xbuf[i];
  }
  double ** matrix3 = createMatrix(2, 2);
  matrix3[0][0] = sum_x2;
  matrix3[0][1] = sum_x;
  matrix3[1][0] = sum_x;
  matrix3[1][1] = size;
  double ** matrix4 = createMatrix(2, 1);
  matrix4[0][0] = sum_Qx;
  matrix4[1][0] = sum_Q;
  double ** invMatrix3 = invertMatrix(matrix3, 2);
  double ** matrixEF = matrixMult(invMatrix3, 2, 2, matrix4, 2, 1);
  double w2 = matrixEF[0][0];
  double theta2 = matrixEF[1][0];
  double b2 = p2 * cos(theta2);
  double c2 = p2 * sin(theta2);

  /* Part 3 */
  double w3 = w2;
  double sum_sin_w3x = 0, sum_cos_w3x = 0, sum_sin2_w3x = 0, sum_cos2_w3x = 0, sum_sincos_w3x = 0, sum_ysin_w3x = 0, sum_ycos_w3x = 0;
  for (int i = 0; i < size; i++)
  {
    double arg = w3 * xbuf[i];
    sum_sin_w3x += sin(arg);
    sum_cos_w3x += cos(arg);
    sum_sin2_w3x += pow(sin(arg), 2);
    sum_cos2_w3x += pow(cos(arg), 2);
    sum_sincos_w3x += sin(arg) * cos(arg);
    sum_ysin_w3x += ybuf[i] * sin(arg);
    sum_ycos_w3x += ybuf[i] * cos(arg);
  }
  
  double ** matrix5 = createMatrix(3, 3);
  matrix5[0][0] = size;
  matrix5[0][1] = sum_sin_w3x;
  matrix5[0][2] = sum_cos_w3x;
  matrix5[1][0] = sum_sin_w3x;
  matrix5[1][1] = sum_sin2_w3x;
  matrix5[1][2] = sum_sincos_w3x;
  matrix5[2][0] = sum_cos_w3x;
  matrix5[2][1] = sum_sincos_w3x;
  matrix5[2][2] = sum_cos2_w3x;

  double ** matrix6 = createMatrix(3, 1);
  matrix6[0][0] = sum_y;
  matrix6[1][0] = sum_ysin_w3x;
  matrix6[2][0] = sum_ycos_w3x;

  double ** invMatrix5 = invertMatrix(matrix5, 3);

  double ** matrixGHI = matrixMult(invMatrix5, 3, 3, matrix6, 3, 1);
  results[0] = w3;
  results[1] = matrixGHI[0][0];
  results[2] = matrixGHI[1][0];
  results[3] = matrixGHI[2][0];


  /* clean up */
  deleteMatrix(matrix1, 4);
  deleteMatrix(matrixABCD, 4);
  deleteMatrix(matrix2, 4);
  deleteMatrix(invMatrix1, 4);
  deleteMatrix(matrix3, 2);
  deleteMatrix(matrix4, 2);
  deleteMatrix(invMatrix3, 2);
  deleteMatrix(matrixEF, 2);
  deleteMatrix(matrix5, 3);
  deleteMatrix(matrix6, 3);
  deleteMatrix(invMatrix5, 3);
  deleteMatrix(matrixGHI, 3);
  delete[] S;
  delete[] SS;
  delete[] K;
  delete[] Q;
}

double ** myMath::createMatrix(int rows, int cols)
{
  double ** matrix = new double*[rows];
  for (int i = 0; i < rows; i++)
  {
    matrix[i] = new double[cols];
  }

  return matrix;
}

void myMath::deleteMatrix(double ** matrix, int rows)
{
  for (int i = 0; i < rows; i++)
  {
    delete[] matrix[i];
  }
}

double ** myMath::invertMatrix(double ** matrix, int size)
{

  /* Get matrix of minors */
  double ** matrixOfMinors = createMatrix(size, size);
  for (int r = 0; r < size; r++)
  {
    for (int c = 0; c < size; c++)
    {
      double ** minorMatrix = getMinorMatrix(matrix, r, c, size);
      matrixOfMinors[r][c] =determinant(minorMatrix, size - 1);
      deleteMatrix(minorMatrix, size - 1);
    }
  }

  /* matrix of cofactors */
  for (int r = 0; r < size; r++)
  {
    for (int c = 0; c < size; c++)
    {
      matrixOfMinors[r][c] *= ((c % 2) == 0 ? 1 : -1) * ((r % 2) == 0 ? 1 : -1);
    }
  }

  /* get Adjugate */
  double ** inverseMatrix = createMatrix(size, size);
  double determinant_ = determinant(matrix, size);
  for (int r = 0; r < size; r++)
  {
    for (int c = 0; c < size; c++)
    {
      inverseMatrix[c][r] = matrixOfMinors[r][c];
      inverseMatrix[c][r] /= determinant_;
    }
  }
  

  deleteMatrix(matrixOfMinors, size);
  return inverseMatrix;
}

double myMath::determinant(double ** matrix, int size)
{
  if (size == 1)
    return matrix[0][0];
  else
  {
    double sum = 0;
    for (int i = 0; i < size; i++)
    {
      double ** minorMatrix = getMinorMatrix(matrix, 0, i, size);
      double term = matrix[0][i] * determinant(minorMatrix, size - 1);
      term *= (i % 2) == 0 ? 1 : -1;
      sum += term;
      deleteMatrix(minorMatrix, size - 1);
    }
    return sum;    
  }
}

double ** myMath::getMinorMatrix(double ** matrix, int row, int col, int size)
{
  double ** minorMatrix = createMatrix(size - 1, size - 1);
  for (int c = 0, srcCol = 0; c < size - 1; c++, srcCol++)
  {
    if (c == col)
      srcCol++;
    for (int r = 0, srcRow = 0; r < size - 1; r++, srcRow++)
    {
      if (r == row)
        srcRow++;
      minorMatrix[r][c] = matrix[srcRow][srcCol];
    }
  }
  return minorMatrix;
}

double ** myMath::matrixMult(double ** matrix1, int rows1, int cols1, double ** matrix2, int rows2, int cols2)
{
  if (cols1 != rows2)
    return NULL;

  double ** result = createMatrix(rows1, cols2);
  for (int r = 0; r < rows1; r++)
  {
    for (int c = 0; c < cols2; c++)
    {
      double sum = 0;
      for (int i = 0; i < cols1; i++)
        sum += matrix1[r][i] * matrix2[i][c];
      result[r][c] = sum;
    }
  }
  return result;
}

/* Newton-Raphson method */
void myMath::NewtonRaphson(double * initialGuessParams, double * xbuf, double * ybuf, int length, double * resultsBuf, bool lockedFrequency)
{
  if (lockedFrequency == false)
  {
    double ** doubleDerivMatrix = createMatrix(4, 4);

    double(*ptrFnDblDerivs[4][4])(double *, double);
    ptrFnDblDerivs[0][0] = d2ydw2;
    ptrFnDblDerivs[0][1] = d2ydwda;
    ptrFnDblDerivs[0][2] = d2ydwdb;
    ptrFnDblDerivs[0][3] = d2ydwdc;
    ptrFnDblDerivs[1][0] = d2ydwda;
    ptrFnDblDerivs[1][1] = d2yda2;
    ptrFnDblDerivs[1][2] = d2ydadb;
    ptrFnDblDerivs[1][3] = d2ydadc;
    ptrFnDblDerivs[2][0] = d2ydwdb;
    ptrFnDblDerivs[2][1] = d2ydadb;
    ptrFnDblDerivs[2][2] = d2ydb2;
    ptrFnDblDerivs[2][3] = d2ydbdc;
    ptrFnDblDerivs[3][0] = d2ydwdc;
    ptrFnDblDerivs[3][1] = d2ydadc;
    ptrFnDblDerivs[3][2] = d2ydbdc;
    ptrFnDblDerivs[3][3] = d2ydc2;

    double(*ptrFnDerivs[4])(double *, double);
    ptrFnDerivs[0] = dydw;
    ptrFnDerivs[1] = dyda;
    ptrFnDerivs[2] = dydb;
    ptrFnDerivs[3] = dydc;

    for (int r = 0; r < 4; r++)
    {
      for (int c = 0; c < 4; c++)
      {
        doubleDerivMatrix[r][c] = de2dXdY(initialGuessParams, xbuf, ybuf, length, ptrFnDerivs[r], ptrFnDerivs[c], ptrFnDblDerivs[r][c]);
      }
    }

    double ** singleDerivMatrix = createMatrix(4, 1);
    for (int r = 0; r < 4; r++)
    {
      singleDerivMatrix[r][0] = dedX(initialGuessParams, xbuf, ybuf, length, ptrFnDerivs[r]);
    }

    double ** dblDerivMatrInverse = invertMatrix(doubleDerivMatrix, 4);
    double ** results = matrixMult(dblDerivMatrInverse, 4, 4, singleDerivMatrix, 4, 1);
    for (int i = 0; i < 4; i++)
    {
      resultsBuf[i] = initialGuessParams[i] - results[i][0];
    }

    deleteMatrix(dblDerivMatrInverse, 4);
    deleteMatrix(doubleDerivMatrix, 4);
    deleteMatrix(singleDerivMatrix, 4);
    deleteMatrix(results, 4);
  }
  else
  {
    double ** doubleDerivMatrix = createMatrix(3, 3);

    double(*ptrFnDblDerivs[3][3])(double *, double);
    ptrFnDblDerivs[0][0] = d2yda2;
    ptrFnDblDerivs[0][1] = d2ydadb;
    ptrFnDblDerivs[0][2] = d2ydadc;
    ptrFnDblDerivs[1][0] = d2ydadb;
    ptrFnDblDerivs[1][1] = d2ydb2;
    ptrFnDblDerivs[1][2] = d2ydbdc;
    ptrFnDblDerivs[2][0] = d2ydadc;
    ptrFnDblDerivs[2][1] = d2ydbdc;
    ptrFnDblDerivs[2][2] = d2ydc2;

    double(*ptrFnDerivs[3])(double *, double);
    ptrFnDerivs[0] = dyda;
    ptrFnDerivs[1] = dydb;
    ptrFnDerivs[2] = dydc;

    for (int r = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        doubleDerivMatrix[r][c] = de2dXdY(initialGuessParams, xbuf, ybuf, length, ptrFnDerivs[r], ptrFnDerivs[c], ptrFnDblDerivs[r][c]);
      }
    }

    double ** singleDerivMatrix = createMatrix(3, 1);
    for (int r = 0; r < 3; r++)
    {
      singleDerivMatrix[r][0] = dedX(initialGuessParams, xbuf, ybuf, length, ptrFnDerivs[r]);
    }

    double ** dblDerivMatrInverse = invertMatrix(doubleDerivMatrix, 3);
    double ** results = matrixMult(dblDerivMatrInverse, 3, 3, singleDerivMatrix, 3, 1);
    for (int i = 0; i < 3; i++)
    {
      resultsBuf[i + 1] = initialGuessParams[i + 1] - results[i][0];
    }

    deleteMatrix(dblDerivMatrInverse, 3);
    deleteMatrix(doubleDerivMatrix, 3);
    deleteMatrix(singleDerivMatrix, 3);
    deleteMatrix(results, 3);
  }
}

double myMath::y_model(double * paramsBuf, double x)
{
  double w = paramsBuf[0];
  double a = paramsBuf[1];
  double b = paramsBuf[2];
  double c = paramsBuf[3];
  return a + b*sin(w*x) + c*cos(w*x);
}

double myMath::dedX(double * paramsBuf, double * xbuf, double * ybuf, int size, double (*dydX)(double *, double))
{
  double sum = 0;
  for (int i = 0; i < size; i++)
  {
    sum += 2 * (y_model(paramsBuf, xbuf[i]) - ybuf[i]) * dydX(paramsBuf, xbuf[i]);
  }
  return sum;
}

double myMath::de2dXdY(double * paramsBuf, double * xbuf, double * ybuf, int size,
  double(*dydX)(double *, double),
  double(*dydY)(double *, double),
  double(*d2ydXdY)(double *, double))
{
  double sum = 0;
  for (int i = 0; i < size; i++)
  {
    sum += 2 * (y_model(paramsBuf, xbuf[i]) - ybuf[i])*d2ydXdY(paramsBuf, xbuf[i]) + 2 * dydX(paramsBuf, xbuf[i])*dydY(paramsBuf, xbuf[i]);
  }
  return sum;
}

double myMath::dydw(double * paramsBuf, double x)
{
  double w = paramsBuf[0];
  double b = paramsBuf[2];
  double c = paramsBuf[3];
  return x*(b*cos(w*x) - c*sin(w*x));
}

double myMath::dyda(double * paramsBuf, double x)
{
  return 1;
}

double myMath::dydb(double * paramsBuf, double x)
{
  double w = paramsBuf[0];
  return sin(w*x);
}

double myMath::dydc(double * paramsBuf, double x)
{
  double w = paramsBuf[0];
  return cos(w*x);
}

double myMath::d2ydw2(double * paramsBuf, double x)
{
  double w = paramsBuf[0];
  double b = paramsBuf[2];
  double c = paramsBuf[3];
  return -x*x*b*sin(w*x) - x*x*c*cos(w*x);
}

double myMath::d2ydwda(double * paramsBuf, double x)
{
  return 0;
}

double myMath::d2ydwdb(double * paramsBuf, double x)
{
  double w = paramsBuf[0];
  return x*cos(w*x);
}

double myMath::d2ydwdc(double * paramsBuf, double x)
{
  double w = paramsBuf[0];
  return -x*sin(w*x);
}

double myMath::d2yda2(double * paramsBuf, double x)
{
  return 0;
}

double myMath::d2ydadb(double * paramsBuf, double x)
{
  return 0;
}

double myMath::d2ydadc(double * paramsBuf, double x)
{
  return 0;
}

double myMath::d2ydb2(double * paramsBuf, double x)
{
  return 0;
}

double myMath::d2ydbdc(double * paramsBuf, double x)
{
  return 0;
}

double myMath::d2ydc2(double * paramsBuf, double x)
{
  return 0;
}