#pragma once

#include "FilterClass.h"
#include "FilterWithBuffer.h"

// 1st parameter <T> is the type of data being filtered.
// 2nd parameter <U> is a larger data type used during summation to prevent overflows
// 3rd parameter <WINDOWS_SIZE> is the number of elements in the filter

template <class T, class U, uint8_t WINDOWS_SIZE>
class MovingAverage : public FilterWithBuffer<T,WINDOWS_SIZE>
{
    public:
    MovingAverage() : FilterWithBuffer<T,WINDOWS_SIZE>(), _num_samples(0){

    };

    virtual T apply(T sample) override;

    virtual void reset() override;

    protected:

    uint8_t _num_samples;
};

// Typedef for convenience (1st argument is the data type, 2nd is a larger datatype to handle overflows, 3rd is buffer size)
typedef MovingAverage<int8_t,int16_t,2> MovingAverageInt8_Size2;
typedef MovingAverage<int8_t,int16_t,3> MovingAverageInt8_Size3;
typedef MovingAverage<int8_t,int16_t,4> MovingAverageInt8_Size4;
typedef MovingAverage<int8_t,int16_t,5> MovingAverageInt8_Size5;
typedef MovingAverage<uint8_t,uint16_t,2> MovingAverageUInt8_Size2;
typedef MovingAverage<uint8_t,uint16_t,3> MovingAverageUInt8_Size3;
typedef MovingAverage<uint8_t,uint16_t,4> MovingAverageUInt8_Size4;
typedef MovingAverage<uint8_t,uint16_t,5> MovingAverageUInt8_Size5;

typedef MovingAverage<int16_t,int32_t,2> MovingAverageInt16_Size2;
typedef MovingAverage<int16_t,int32_t,3> MovingAverageInt16_Size3;
typedef MovingAverage<int16_t,int32_t,4> MovingAverageInt16_Size4;
typedef MovingAverage<int16_t,int32_t,5> MovingAverageInt16_Size5;
typedef MovingAverage<uint16_t,uint32_t,2> MovingAverageUInt16_Size2;
typedef MovingAverage<uint16_t,uint32_t,3> MovingAverageUInt16_Size3;
typedef MovingAverage<uint16_t,uint32_t,4> MovingAverageUInt16_Size4;
typedef MovingAverage<uint16_t,uint32_t,5> MovingAverageUInt16_Size5;

typedef MovingAverage<int32_t,float,2> MovingAverageInt32_Size2;
typedef MovingAverage<int32_t,float,3> MovingAverageInt32_Size3;
typedef MovingAverage<int32_t,float,4> MovingAverageInt32_Size4;
typedef MovingAverage<int32_t,float,5> MovingAverageInt32_Size5;
typedef MovingAverage<uint32_t,float,2> MovingAverageUInt32_Size2;
typedef MovingAverage<uint32_t,float,3> MovingAverageUInt32_Size3;
typedef MovingAverage<uint32_t,float,4> MovingAverageUInt32_Size4;
typedef MovingAverage<uint32_t,float,5> MovingAverageUInt32_Size5;

typedef MovingAverage<float,float,2> MovingAverageFloat_Size2;
typedef MovingAverage<float,float,3> MovingAverageFloat_Size3;
typedef MovingAverage<float,float,4> MovingAverageFloat_Size4;
typedef MovingAverage<float,float,5> MovingAverageFloat_Size5;

typedef MovingAverage<double,double,2> MovingAverageDouble_Size2;
typedef MovingAverage<double,double,3> MovingAverageDouble_Size3;
typedef MovingAverage<double,double,4> MovingAverageDouble_Size4;
typedef MovingAverage<double,double,5> MovingAverageDouble_Size5;

template <class T, class U, uint8_t WINDOWS_SIZE>
T MovingAverage<T,U,WINDOWS_SIZE>::apply(T sample)
{
    U result = 0;

    FilterWithBuffer<T,WINDOWS_SIZE>::apply(sample);

    _num_samples++;
    if (_num_samples > WINDOWS_SIZE || _num_samples == 0)
    {
        _num_samples = WINDOWS_SIZE;
    }

    for(uint8_t i = 0; i<WINDOWS_SIZE; i++)
    {
        result+=FilterWithBuffer<T,WINDOWS_SIZE>::samples[i];
    }

    return (T)( result/_num_samples );
}

template <class T, class U, uint8_t WINDOWS_SIZE>
void MovingAverage<T,U,WINDOWS_SIZE>::reset()
{
    FilterWithBuffer<T, WINDOWS_SIZE>::reset();

    _num_samples = 0;

}