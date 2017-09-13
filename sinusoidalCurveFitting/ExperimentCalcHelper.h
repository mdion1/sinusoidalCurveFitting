#ifndef _EXPERIMENT_CALC_HELPER
#define _EXPERIMENT_CALC_HELPER

#include <ExperimentNode.h>
#include <ExternalStructures.h>
#include <global_typedefs.h>
#include <cal.h>
#include <math.h>
#include <qlist.h>

#define SQUIDSTAT_PIC_MIN_ADCDC_TIMER_PERIOD (10 * MICROSECONDS)
#define SQUIDSTAT_PIC_MAX_ADCDC_BUF_SIZE 512
#define SQUIDSTAT_TEENSY_MIN_ADCDC_TIMER_PERIOD (500 * MICROSECONDS)
#define SQUIDSTAT_TEENSY_MAX_ADCDC_BUF_SIZE 512
#define SQUIDSTAT_PIC_TIMER_CLK_SPEED 1e8
#define SQUIDSTAT_MAX_ADC_AC_BUF_SIZE 1024.0
#define HF_CUTOFF_VALUE 500
#define SIGNAL_GEN_RESOLUTION 1024
#define MIN_TICKS_FOR_USB_TRANSMISSION (80 * MILLISECONDS)
#define MAX_CURRENT 1.0e10
#define MAX_VOLTAGE 30

class ExperimentCalcHelperClass
{
public:
  static void GetSamplingParams_staticDAC(HardwareModel_t HWversion, ExperimentNode_t * pNode, const double targetSamplingInterval, double MaxUpdateInterval = 1);
  static void GetSamplingParams_potSweep(HardwareModel_t HWversion, const cal_t * calData, ExperimentNode_t * pNode,
    double dEdt, const double targetSamplingInterval = 0);
  static void GetSamplingParams_galvSweep(HardwareModel_t HWversion, const cal_t * calData, ExperimentNode_t * pNode,
    double dIdt, currentRange_t currentRange, const double samplingInterval = 0);
  static void GetSamplingParameters_pulse(HardwareModel_t HWversion, double t_period, double t_pulsewidth, ExperimentNode_t * pNode);
  static currentRange_t GetMinCurrentRange(HardwareModel_t HWversion, const cal_t * calData, double targetCurrent);
  static int16_t GetBINCurrent(const cal_t * calData, currentRange_t currentRange, double targetCurrent);
  static int16_t GetBINVoltageForDAC(const cal_t * calData, double targetVoltage);
  static ProcessedDCData ProcessDCDataPoint(const cal_t * calData, ExperimentalDcData rawData);
  static double GetUnitsMultiplier(QString units_str);

  /* AC methods */
  static currentRange_t GetMinCurrentRange_DACac(const cal_t * calData, double targetCurrentAmp);
  static QList<double> calculateFrequencyList(double lowerFreq, double upperFreq, double pointsPerDecade);
  static void calcACSamplingParams(const cal_t * calData, ExperimentNode_t * pNode);
  static double calcNumberOfCycles(const ExperimentalAcData);

  /* sinusoidal curve-fitting */
  static ComplexDataPoint_t AnalyzeFRA(double frequency, int16_t * bufCurrent, int16_t * bufEWE, double gainEWE, double gainI, double approxNumCycles, const cal_t * calData, currentRange_t range);

private:
  /* matrix operations */
  static double ** createMatrix(int rows, int cols);
  static void deleteMatrix(double ** matrix, int rows);
  static double ** invertMatrix(double ** matrix, int rows);
  static double determinant(double ** matrix, int size);
  static double ** getMinorMatrix(double ** matrix, int row, int col, int size);
  static double ** matrixMult(double ** matrix1, int rows1, int cols1, double ** matrix2, int rows2, int cols2);

  /* Newton-raphson method */
  static void sinusoidLeastSquaresFit(double * xbuf, double * ybuf, int size, double * results);
  static void NewtonRaphson(double * initialGuessParams, double * xbuf, double * ybuf, int length, double * resultsBuf, bool lockedFrequency = false);
  static double * filterData(int16_t * rawData, int length, int rollingAvgSize);
  static double getError(double * rawData, double * resultsBuf, int len);
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


#endif	//_EXPERIMENT_CALC_HELPER