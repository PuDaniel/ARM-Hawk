/*
 * ArmHawk_math.hpp
 *
 *  Created on: May 19, 2018
 *      Author: Daniel
 */

#ifndef ARMHAWK_MATH_HPP_
#define ARMHAWK_MATH_HPP_

// Built-in modules
#include "arm_math.h"
#include <cmath>
#include <cstdint>
#include <array>

// Own modules
#include "System.hpp"

// Class definitions

/*
 * Create a (mathematical) matrix object.
 * External indexing starts at (0, 0) and ends at (m-1, n-1)
 * Values are stored by appending all rows after each other:
 * A = [a_00     a_01     ...      a_0(n-1)
 *      a_10     a_11     ...      a_1(n-1)
 *      ...      ...      ...      ...
 *      a_(m-1)0 a_(m-1)1 ...      a_(m-1)(n-1)]
 *
 * => value = [a_00 a_01 ... a_0(n-1) a_10 ... a_1(n-1) ... a_(m-1)0 ... a_(m-1)(n-1)]
 *
 * @template <M>	Amount of rows of the matrix
 * @template <N>	Amount of columns of the matrix
 * @template <T>	Type of matrix elements
 */
template <uint8_t M, uint8_t N>
class Matrix
{
protected:
	static_assert(M > 0, "Number of rows of a matrix can't be zero");
	static_assert(N > 0, "Number of columns of a matrix can't be zero");

	// Stores the matrix data in vectorized form
	std::array<float, M * N> value;

	// CMSIS DSP library matrix instance
	arm_matrix_instance_f32 cmsis_mat;

	// Compute the index to easily access the internal stored, vectorized data
	uint16_t index(uint8_t i, uint8_t j) const { return (uint16_t)i * N + (uint16_t)j; }

public:
	// Matrix access functions
	float& operator()(uint8_t i, uint8_t j);
	float operator()(uint8_t i, uint8_t j) const;

	// Operators for matrix addition
	Matrix<M, N>& operator+=(float rhs);
	Matrix<M, N>& operator+=(const Matrix<M, N>& rhs);

	// Operators for matrix subtracion
	Matrix<M, N> operator-() const;
	Matrix<M, N>& operator-=(float rhs);
	Matrix<M, N>& operator-=(const Matrix<M, N>& rhs);

	// Operators for matrix multiplication
	Matrix<M, N>& operator*=(float rhs);
	template <uint8_t M_, uint8_t N_, uint8_t P_>
	friend Matrix<M_, N_> operator*(const Matrix<M_, P_>& lhs, const Matrix<P_, N_>& rhs);

	// Operators for matrix division (only with scalars!)
	Matrix<M, N>& operator/=(float rhs);

	// Other matrix manipulations
	template <uint8_t M_, uint8_t N_>
	friend Matrix<N_, M_> transpose(const Matrix<M_, N_>& mat);
	template <uint8_t M_, uint8_t N_>
	friend Matrix<M_, N_> inverse(const Matrix<M_, N_>& mat);

	// Matrix conversion functions
	int string(char* ascii);

	// Matrix checks
	char finite() const;

	// Constructors
	Matrix();
	Matrix(float init_val);
	Matrix(const std::array<float, M * N>& init_vect);

	// Copy constructor
	Matrix(const Matrix<M, N>& mat);

	// Move constructor
	Matrix(Matrix<M, N>&& mat);

	// Copy assignment
	Matrix<M, N>& operator=(const Matrix<M, N>& mat);

	// Move assignemnt
	Matrix<M, N>& operator=(Matrix<M, N>&& mat);
};

/*
 * Create a (mathematical) vector object.
 * Is a single column Matrix object with special functions.
 * External indexing starts at (0) and ends at (m-1):
 * x = [x_0
 *      x_1
 *      ...
 *      x_(m-1)]
 *
 * => value = [x_0 x_1 ... x_(m-1)]
 *
 * @template <M>	Amount of elements (rows) of the vector
 * @template <T>	Type of vector elements
 */
template <uint8_t M>
class Vector : public Matrix<M, 1>
{
public:
	// Vector access functions
	float& operator()(uint8_t i);
	float operator()(uint8_t i) const;

	// Other vector manipulations
	float dot(const Vector& rhs) const;
	Vector<3> cross(const Vector<3>& rhs) const;
	template <uint8_t M_>
	friend float norm(const Vector<M_>& vect);

	// Constructors
	Vector();
	Vector(float init_val);
	Vector(const std::array<float, M>& init_vect);
	Vector(const Matrix<M, 1>& mat);

	// Copy constructor
	Vector(const Vector<M>& vect);

	// Move constructor
	Vector(Vector<M>&& vect);

	// Copy assignment
	Vector<M>& operator=(const Vector<M>& vect);

	// Move assignemnt
	Vector<M>& operator=(Vector<M>&& vect);
};

/*
 * Creates a First-In First-Out container object.
 *
 * @template <M>	Size of the FIFO
 * @template <T>	Type of FIFO elements
 */
template <uint16_t M, typename T>
class FIFO
{
	static_assert(M > 0, "Minimum FIFO size is 1");

	// Ring buffer for stroring the FIFO elements
	std::array<T, M> buffer;

	// Index of the oldest element in the FIFO
	int top;

	// Index of the newest element in the FIFO
	int bottom;

public:
	// Query functions
	int size() const;
	char empty() const;

	// Modifiy functions
	void push(const T& new_val);
	T& pop();

	// Access functions
	T& operator[] (uint16_t index);

	FIFO();
};

/*
 * Create a transfer function for filtering values.
 * Transfer function has the form:
 *
 * b0 + b1*z^-1 + b2*z^-2 + ... + bM*z^-M
 * --------------------------------------
 * a0 + a1*z^-1 + a2*z^-2 + ... + aN*z^-N
 *
 * @template <M>	Degree of the numerator polynomial
 * @template <N>	Degree of the denominator coefficients
 * @template <T>	Type of filtered elements
 */
template <uint8_t M, uint8_t N, typename T>
class Filter
{
	static_assert(M > 0, "Minimum degree of numerator polynomial is 1");
	static_assert(N > 0, "Minimum degree of denominator polynomial is 1");

	// Input signal vector; Index 0 = newest value, index M = oldest value
	std::array<T, M> x;

	// Output signal vector; Index 0 = newest value, index N = oldest value
	std::array<T, N> y;

public:
	// Denominator coefficients
	std::array<double, N> a;

	// Numerator coefficients
	std::array<double, M> b;

	// Holds the last added value without any filtering
	T raw;

	// Holds the latest filtered output value
	T filtered;

	T& update(const T& new_val);

	void reset(const T& reset_val);
	void reset();

	Filter(const std::array<double, N>& denom, const std::array<double, M>& numer);
	Filter(const std::array<double, N>& denom, const std::array<double, M>& numer, const T& init_val);
};

/*
 * Get the sign of a value
 * Returns 1, if value greater than 0;
 * returns -1, if value smaller than 0;
 * returns 0, if value equals 0
 * @param val	The value to be checked
 * @return		Sign of value
 */
template <typename T>
int sgn(T val) { return (val > T(0)) - (val < T(0)); }

// Additional print function for matrices and vectors
template <uint8_t M, uint8_t N>
void print(const char* message, Matrix<M, N> mat, char priority);

// Include the template implementations
#include "ArmHawk_math.tpp"

#endif /* ARMHAWK_MATH_HPP_ */
