#pragma once
#include "filter.h"
#include <stdlib.h>

/*
IIR Filter calculations:
y[n] = A*y[n-1] + B*x[n]
DC gain = B / (1 - A)
For gain = 1, we need to have 1 = A + B

digital cut off frequency (radians):
w_3db = acos((1 - 4A + A^2) / 2A)

if filter is updated at a frequency of K hertz, then cuttof frequency is
F_3db = w_3dB / 2 / pi * K  (hertz)

*/

IIR_doubleFilter::IIR_doubleFilter(double f){
        new_factor = f;
        old_factor = 1.0 - f;
	//initialize value
        current_val = 0;        
}

IIR_doubleFilter::~IIR_doubleFilter(){
}

void IIR_doubleFilter::addValue(double v){
      current_val = current_val * old_factor + v * new_factor;
}

void IIR_doubleFilter::setValue(double v){
      current_val = v;
}

void IIR_doubleFilter::reset(){
      current_val = 0;
}

double IIR_doubleFilter::getCurrentValue(){
      return current_val;
}

FIR_doubleFilter::FIR_doubleFilter(int s){
	arraySize = s;
	index = 0; 
	array = (double *) malloc(sizeof(double)*arraySize);
	fill(0);
}

FIR_doubleFilter::~FIR_doubleFilter(){
}

void FIR_doubleFilter::fill(double v){
	for(int i = 0; i < arraySize; i++)
		array[i] = v;
}

void FIR_doubleFilter::addValue(double v){
	index++;
	if(index == arraySize)
		index = 0;	
	array[index] = v;
}

double FIR_doubleFilter::getCurrentValue(){
	double sum = 0;
	for(int i = 0; i < arraySize; i++){
		sum  = sum + array[i];
	}	
	return sum / arraySize;
}


IIR_integerFilter::IIR_integerFilter(int f){
        new_factor = f;
        old_factor = 1.0 - f;
	//initialize value
        current_val = 0;        
}

IIR_integerFilter::~IIR_integerFilter(){
}

void IIR_integerFilter::addValue(int v){
      current_val = current_val * old_factor + v * new_factor;
}

void IIR_integerFilter::setValue(int v){
      current_val = v;
}

void IIR_integerFilter::reset(){
      current_val = 0;
}

int IIR_integerFilter::getCurrentValue(){
      return current_val;
}

FIR_integerFilter::FIR_integerFilter(int s){
	arraySize = s;
	index = 0; 
	array = (int *) malloc(sizeof(int)*arraySize);
	fill(0);
}

FIR_integerFilter::~FIR_integerFilter(){
}

void FIR_integerFilter::fill(int v){
	for(int i = 0; i < arraySize; i++)
		array[i] = v;
}

void FIR_integerFilter::addValue(int v){
	index++;
	if(index == arraySize)
		index = 0;	
	array[index] = v;
}

int FIR_integerFilter::getCurrentValue(){
	int sum = 0;
	for(int i = 0; i < arraySize; i++){
		sum  = sum + array[i];
	}	
	return sum / arraySize;
}
