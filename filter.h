#pragma once
#include <stdlib.h>

class IIR_doubleFilter{
	public:
	IIR_doubleFilter(double f);
	~IIR_doubleFilter();
	void addValue(double v);
        void reset();
	double getCurrentValue();
        void setValue(double v);

	private:
        double old_factor;
        double new_factor;
        double current_val;
};

class FIR_doubleFilter{
      public:
      FIR_doubleFilter(int i);
      ~FIR_doubleFilter();
      void addValue(double v);
      void fill(double v);
      double getCurrentValue();
      
      private:
      int arraySize;
      int index;
      double *array;
};


class IIR_integerFilter{
	public:
	IIR_integerFilter(int f);
	~IIR_integerFilter();
	void addValue(int v);
        void reset();
	int getCurrentValue();
        void setValue(int v);

	private:
        int old_factor;
        int new_factor;
        int current_val;
};

class FIR_integerFilter{
      public:
      FIR_integerFilter(int i);
      ~FIR_integerFilter();
      void addValue(int v);
      void fill(int v);
      int getCurrentValue();
      
      private:
      int arraySize;
      int index;
      int *array;
};
