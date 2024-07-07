/*
 * fft.h
 *
 *
 *
 */

#ifndef INC_FFT_H_
#define INC_FFT_H_


struct cmpx
{
float real;
float imag;
};

/* Función para realizar la FFT de N puntos
 * La salida se carga en el arreglo Y pasado por referencia */
void FFT(struct cmpx *Y, int N);

#endif /* INC_FFT_H_ */
