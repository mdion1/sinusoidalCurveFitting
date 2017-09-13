#include <iostream>
#include <fstream>
#include "myMath.h"
#include <global_typedefs.h>
#include <ExperimentNode.h>
#include "ExperimentCalcHelper.h"

using namespace std;

double importData(const std::string filename, double * xvar, double * y1var, double * y2var, double& frequency);

double importData(const std::string filename, double * xvar, double * y1var, double * y2var, double& frequency)
{
  /* Data collection */
  ifstream fin;
  fin.open(filename, ifstream::in);
  double numLines = -1;
  double dummyVar1, dummyVar2;
  while (!fin.eof())
  {
    fin >> dummyVar1 >> dummyVar2;
    numLines++;
  }
  double frequency;
  xvar = new double[numLines];
  y1var = new double[numLines];
  y2var = new double[numLines];

  fin.seekg(0, fin.beg);
  fin >> frequency;
  for (int i = 0; i < numLines; i++)
  {
    xvar[i] = i;
    fin >> y1var[i] >> y2var[i];
  }

  return numLines;
}

int main(void)
{
  double *xvar, *y1var, *y2var, frequency;
  double numLines = importData("C:/Users/Matt/Desktop/data.txt", xvar, y1var, y2var, frequency);
  

  /* Part 1: least squares regression first guess*/
  double results1[4];
  double results2[4];
  double * filtered_y1 = myMath::filterData(y1var, numLines, 20);
  double * filtered_y2 = myMath::filterData(y2var, numLines, 20);
  myMath::sinusoidLeastSquaresFit(xvar, filtered_y1, numLines - 20, results1);
  myMath::sinusoidLeastSquaresFit(xvar, filtered_y2, numLines - 20, results2);

  double Magnitude1 = sqrt(pow(results1[2], 2) + pow(results1[3], 2));
  double Magnitude2 = sqrt(pow(results2[2], 2) + pow(results2[3], 2));
  double phase1 = atan2(results1[2], results1[3]) * 180 / PI;
  double phase2 = atan2(results2[2], results2[3]) * 180 / PI;

  ofstream fout;
  fout.open("C:/Users/Matt/Desktop/fittingResults.txt", ofstream::out);
  fout << "LSR curve 1: " << '\t' << results1[0] << '\t' << results1[1] << '\t' << results1[2] << '\t' << results1[3] << endl;
  fout << "LSR curve 2: " << '\t' << results2[0] << '\t' << results2[1] << '\t' << results2[2] << '\t' << results2[3] << endl;

  //results1[0] = results2[0] = (results1[0] + results2[0]) / 2;
  results2[0] = results1[0];
  results2[1] = results1[1];
  results2[2] = results1[2];
  results2[3] = results1[3];

  /* Part 2: Newton-Raphson method */
  for (int i = 0; i < 10; i++)
  {
    //myMath::NewtonRaphson(results1, xvar, y1var, numLines, results1);
    myMath::NewtonRaphson(results2, xvar, filtered_y2, numLines - 20, results2);
    results1[0] = results2[0];
    //myMath::NewtonRaphson(results2, xvar, y2var, numLines, results2, true);
    myMath::NewtonRaphson(results1, xvar, filtered_y1, numLines - 20, results1, true);

    Magnitude1 = sqrt(pow(results1[2], 2) + pow(results1[3], 2));
    Magnitude2 = sqrt(pow(results2[2], 2) + pow(results2[3], 2));
    phase1 = atan2(results1[2], results1[3]) * 180 / PI;
    phase2 = atan2(results2[2], results2[3]) * 180 / PI;
  }

  fout << "NR curve 1: " << '\t' << results1[0] << '\t' << results1[1] << '\t' << results1[2] << '\t' << results1[3] << endl;
  fout << "NR curve 2: " << '\t' << results2[0] << '\t' << results2[1] << '\t' << results2[2] << '\t' << results2[3] << endl;

  delete[] xvar;
  delete[] y1var;
  delete[] y2var;
  delete[] filtered_y1;
  delete[] filtered_y2;
  return 0;
}
