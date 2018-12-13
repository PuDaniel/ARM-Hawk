/*
 * ArmHawk_math.tpp
 *
 *  Created on: Sep 1, 2018
 *      Author: Daniel
 */

#ifndef ARMHAWK_MATH_TPP_
#define ARMHAWK_MATH_TPP_

/*
 * Standard initialization of a matrix
 */
template <uint8_t M, uint8_t N>
Matrix<M, N>::Matrix()
{
	arm_mat_init_f32(&this->cmsis_mat, M, N, this->value.data());
}

/*
 * Initialize a matrix uniformly with a given value
 * @param init_val	Value, all matrix elements should be initialized with
 */
template <uint8_t M, uint8_t N>
Matrix<M, N>::Matrix(float init_val)
{
	this->value.fill(init_val);

	arm_mat_init_f32(&this->cmsis_mat, M, N, this->value.data());
}

/*
 * Initialize a matrix with given data
 * Clips init_vect if its too long and pads with zero if its too short
 * @param init_vect Initialization matrix in vectorized form
 */
template <uint8_t M, uint8_t N>
Matrix<M, N>::Matrix(const std::array<float, M * N>& init_vect)
	: value(init_vect)
{
	arm_mat_init_f32(&this->cmsis_mat, M, N, this->value.data());
}

/*
 * Copy constructor: Initialize a matrix from another matrix
 * @param mat		Other matrix to copy data from
 */
template <uint8_t M, uint8_t N>
Matrix<M, N>::Matrix(const Matrix<M, N>& mat)
{
	this->value = mat.value;

	arm_mat_init_f32(&this->cmsis_mat, M, N, this->value.data());
}

/*
 * Move constructor: Initialize a matrix with data from another temporary matrix
 * @param mat		Other matrix, which data will be owned by this matrix
 */
template <uint8_t M, uint8_t N>
Matrix<M, N>::Matrix(Matrix<M, N>&& mat)
{
	this->value = mat.value;

	arm_mat_init_f32(&this->cmsis_mat, M, N, this->value.data());
}

/*
 * Copy assignment: Set matrix data from another matrix
 * @param mat		Other matrix to copy data from
 */
template <uint8_t M, uint8_t N>
Matrix<M, N>& Matrix<M, N>::operator=(const Matrix<M, N>& mat)
{
	this->value = mat.value;

	return *this;
}

/*
 * Move assignment: Set matrix with data from another temporary matrix
 * @param mat		Other matrix, which data will be owned by this matrix
 */
template <uint8_t M, uint8_t N>
Matrix<M, N>& Matrix<M, N>::operator=(Matrix<M, N>&& mat)
{
	this->value = mat.value;

	return *this;
}

/*
 * Return a reference to the matrix element with index (i, j)
 * @param i			Row of the required element
 * @param j			Column of the required element
 * @return 			Reference to the element (i, j)
 */
template <uint8_t M, uint8_t N>
inline float& Matrix<M, N>::operator()(uint8_t i, uint8_t j)
{
	if(i >= M)
		i = M - 1;

	if(j >= N)
		j = N - 1;

	return this->value[this->index(i, j)];
}

/*
 * Return the matrix element with index (i, j)
 * @param i			Row of the required element
 * @param j			Column of the required element
 * @return 			Element (i, j)
 */
template <uint8_t M, uint8_t N>
inline float Matrix<M, N>::operator()(uint8_t i, uint8_t j) const
{
	if(i >= M)
		i = M - 1;

	if(j >= N)
		j = N - 1;

	return this->value[this->index(i, j)];
}

/*
 * Add a scalar value to the current matrix
 * @param rhs		Right hand side scalar value
 * @return 			Reference to own matrix
 */
template <uint8_t M, uint8_t N>
Matrix<M, N>& Matrix<M, N>::operator+=(float rhs)
{
	for(int i = 0; i < M; i++)
		for(int j = 0; j < N; j++)
			this->value[this->index(i, j)] += rhs;

	return *this;
}

/*
 * Add another matrix to current one
 * @param rhs		Right hand side matrix
 * @return 			Reference to own matrix
 */
template <uint8_t M, uint8_t N>
Matrix<M, N>& Matrix<M, N>::operator+=(const Matrix<M, N>& rhs)
{
	for(int i = 0; i < M; i++)
		for(int j = 0; j < N; j++)
			this->value[this->index(i, j)] += rhs.value[rhs.index(i, j)];

	return *this;
}

/*
 * Add a matrix and a scalar value
 * @param lhs		Left hand side matrix
 * @param rhs		Right hand side scalar value
 * @return 			Result of addition
 */
template <uint8_t M, uint8_t N>
Matrix<M, N> operator+(const Matrix<M, N>& lhs, float rhs)
{
	Matrix<M, N> result(lhs);
	result += rhs;
	return result;
}

/*
 * Add a matrix and a scalar value
 * @param lhs		Left hand side scalar value
 * @param rhs		Right hand side matrix
 * @return 			Result of addition
 */
template <uint8_t M, uint8_t N>
Matrix<M, N> operator+(float lhs, const Matrix<M, N>& rhs)
{
	Matrix<M, N> result(rhs);
	result += lhs;
	return result;
}

/*
 * Add two matrices
 * @param lhs		Left hand side matrix
 * @param rhs		Right hand side matrix
 * @return 			Result of addition
 */
template <uint8_t M, uint8_t N>
Matrix<M, N> operator+(const Matrix<M, N>& lhs, const Matrix<M, N>& rhs)
{
	Matrix<M, N> result(lhs);
	result += rhs;
	return result;
}

/*
 * Negate the matrix
 * @param rhs		Right hand side matrix to be negated
 * @return 			Result of negation
 */
template <uint8_t M, uint8_t N>
Matrix<M, N> Matrix<M, N>::operator-() const
{
	Matrix<M, N> result(*this);

	for(int i = 0; i < M; i++)
		for(int j = 0; j < N; j++)
			result.value[result.index(i, j)] = -this->value[this->index(i, j)];

	return result;
}

/*
 * Substract a scalar value from the current matrix
 * @param rhs		Right hand side scalar value
 * @return 			Reference to own matrix
 */
template <uint8_t M, uint8_t N>
Matrix<M, N>& Matrix<M, N>::operator-=(float rhs)
{
	for(int i = 0; i < M; i++)
		for(int j = 0; j < N; j++)
			this->value[this->index(i, j)] -= rhs;

	return *this;
}

/*
 * Substract another matrix from current one
 * @param rhs		Right hand side matrix
 * @return 			Reference to own matrix
 */
template <uint8_t M, uint8_t N>
Matrix<M, N>& Matrix<M, N>::operator-=(const Matrix<M, N>& rhs)
{
	for(int i = 0; i < M; i++)
		for(int j = 0; j < N; j++)
			this->value[this->index(i, j)] -= rhs.value[rhs.index(i, j)];

	return *this;
}

/*
 * Substract a scalar value from a matrix
 * @param lhs		Left hand side matrix
 * @param rhs		Right hand side scalar value
 * @return 			Result of substraction
 */
template <uint8_t M, uint8_t N>
Matrix<M, N> operator-(const Matrix<M, N>& lhs, float rhs)
{
	Matrix<M, N> result(lhs);
	result -= rhs;
	return result;
}

/*
 * Substract a matrix from a scalar value
 * @param lhs		Left hand side scalar value
 * @param rhs		Right hand side matrix
 * @return 			Result of substraction
 */
template <uint8_t M, uint8_t N>
Matrix<M, N> operator-(float lhs, const Matrix<M, N>& rhs)
{
	Matrix<M, N> result(-rhs);
	result += lhs;
	return result;
}

/*
 * Substract a matrix from another
 * @param lhs		Left hand side matrix
 * @param rhs		Right hand side matrix
 * @return 			Result of substraction
 */
template <uint8_t M, uint8_t N>
Matrix<M, N> operator-(const Matrix<M, N>& lhs, const Matrix<M, N>& rhs)
{
	Matrix<M, N> result(lhs);
	result -= rhs;
	return result;
}

/*
 * Multiply a scalar value to the current matrix
 * @param rhs		Right hand side scalar value
 * @return 			Reference to own matrix
 */
template <uint8_t M, uint8_t N>
Matrix<M, N>& Matrix<M, N>::operator*=(float rhs)
{
	for(int i = 0; i < M; i++)
		for(int j = 0; j < N; j++)
			this->value[this->index(i, j)] *= rhs;

	return *this;
}

/*
 * Multiply a matrix and a scalar value
 * @param lhs		Left hand side matrix
 * @param rhs		Right hand side scalar value
 * @return 			Result of multiplication
 */
template <uint8_t M, uint8_t N>
Matrix<M, N> operator*(const Matrix<M, N>& lhs, float rhs)
{
	Matrix<M, N> result(lhs);

	result *= rhs;

	return result;
}

/*
 * Multiply a matrix and a scalar value
 * @param lhs		Left hand side scalar value
 * @param rhs		Right hand side matrix
 * @return 			Result of multiplication
 */
template <uint8_t M, uint8_t N>
Matrix<M, N> operator*(float lhs, const Matrix<M, N>& rhs)
{
	Matrix<M, N> result(rhs);

	result *= lhs;

	return result;
}

/*
 * Multiply two matrices
 * @param lhs		Left hand side matrix
 * @param rhs		Right hand side matrix
 * @return 			Result of multiplication
 */
template <uint8_t M, uint8_t N, uint8_t P>
Matrix<M, N> operator*(const Matrix<M, P>& lhs, const Matrix<P, N>& rhs)
{
	Matrix<M, N> result;

	arm_mat_mult_f32(&lhs.cmsis_mat, &rhs.cmsis_mat, &result.cmsis_mat);

	return result;
}

/*
 * Divide the current matrix by a scalar value
 * @param rhs		Right hand side scalar value
 * @return 			Reference to own matrix
 */
template <uint8_t M, uint8_t N>
Matrix<M, N>& Matrix<M, N>::operator/=(float rhs)
{
	for(int i = 0; i < M; i++)
		for(int j = 0; j < N; j++)
			this->value[this->index(i, j)] /= rhs;

	return *this;
}

/*
 * Divide a matrix by a scalar value
 * @param lhs		Left hand side matrix
 * @param rhs		Right hand side scalar value
 * @return 			Result of division
 */
template <uint8_t M, uint8_t N>
Matrix<M, N> operator/(const Matrix<M, N>& lhs, float rhs)
{
	Matrix<M, N> result(lhs);

	result /= rhs;

	return result;
}

/*
 * Transposes the matrix
 * @return			The transposed matrix
 */
template <uint8_t M, uint8_t N>
Matrix<N, M> transpose(const Matrix<M, N>& mat)
{
	Matrix<N, M> result;

	arm_mat_trans_f32(&mat.cmsis_mat, &result.cmsis_mat);

	return result;
}

/*
 * Compute the inverse of a square matrix
 * @param mat		Matrix to be inverted
 * @return			The inverted matrix
 */
template <uint8_t M, uint8_t N>
Matrix<M, N> inverse(const Matrix<M, N>& mat)
{
	static_assert(M == N, "Can only compute the inverse of a square matrix");

	Matrix<M, M> result;

	arm_mat_inverse_f32(&mat.cmsis_mat, &result.cmsis_mat);

	return result;
}

/*
 * Convert the matrix to a readable, Matlab-like string
 * @param ascii		Byte buffer to write the string to
 * @return 			Length of string in characters
 */
template <uint8_t M, uint8_t N>
int Matrix<M, N>::string(char* ascii)
{
	// Start matrix
	ascii[0] = '[';
	int length = 1;

	// Add rows and columns
	for(int i = 0; i < M; i++)
	{
		for(int j = 0; j < N; j++)
		{
			length += toString(this->value[this->index(i, j)], 2, ascii + length);

			if(j != (N - 1))
			{
				ascii[length] = ',';
				ascii[length + 1] = ' ';
				length += 2;
			}
			else if(i != (M - 1))
			{
				ascii[length] = ';';
				ascii[length + 1] = ' ';
				length += 2;
			}
		}
	}

	// End matrix
	ascii[length] = ']';
	length ++;

	ascii[length] = '\0';

	return length;
}

/*
 * Check if any of the matrix elements is NAN or INF
 * @return 			1 if all elements are finite, 0 else
 */
template <uint8_t M, uint8_t N>
char Matrix<M, N>::finite() const
{
	for(int i = 0; i < M; i++)
		for(int j = 0; j < N; j++)
			if(!isfinite(this->value[this->index(i, j)]))
				return 0;

	return 1;
}

/*
 *Standard initialization of a vector
 */
template <uint8_t M>
Vector<M>::Vector() : Matrix<M, 1>() {}

/*
 * Initialize a vector uniformly with a given value
 * @param init_val	Value, all vector elements should be initialized with
 */
template <uint8_t M>
Vector<M>::Vector(float init_val) : Matrix<M, 1>(init_val) {}

/*
 * Initialize a vector with given data
 * Clips init_vect if its too long and pads with zero if its too short
 * @param init_vect Initialization vector
 */
template <uint8_t M>
Vector<M>::Vector(const std::array<float, M>& init_vect) : Matrix<M, 1>(init_vect) {}

/*
 * Initialize a vector from a m x 1 matrix
 * @param mat			Matrix, to initialize the vector from
 */
template <uint8_t M>
Vector<M>::Vector(const Matrix<M, 1>& mat) : Matrix<M, 1>(mat) {}

/*
 * Copy constructor: Initialize a vector from another vector
 * @param vect		Other vector to copy data from
 */
template <uint8_t M>
Vector<M>::Vector(const Vector<M>& vect) : Matrix<M, 1>(vect.value) {}

/*
 * Move constructor: Initialize a vector with data from another temporary vector
 * @param mat		Other vector, which data will be owned by this matrix
 */
template <uint8_t M>
Vector<M>::Vector(Vector<M>&& vect)
{
	this->value = vect.value;

	arm_mat_init_f32(&this->cmsis_mat, M, 1, this->value.data());
}

/*
 * Copy assignment: Set vector data from another vector
 * @param mat		Other vector to copy data from
 */
template <uint8_t M>
Vector<M>& Vector<M>::operator=(const Vector<M>& vect)
{
	this->value = vect.value;

	return *this;
}

/*
 * Move assignment: Set vector with data from another temporary vector
 * @param mat		Other vector, which data will be owned by this vector
 */
template <uint8_t M>
Vector<M>& Vector<M>::operator=(Vector<M>&& vect)
{
	this->value = vect.value;

	return *this;
}

/*
 * Return a reference to the vector element with index (i)
 * @param i			Row of the required element
 * @return 			Reference to the element (i)
 */
template <uint8_t M>
inline float& Vector<M>::operator()(uint8_t i)
{
	if(i >= M)
		i = M - 1;

	return this->value[i];
}

/*
 * Return the vector element with index (i)
 * @param i			Row of the required element
 * @return 			Element (i)
 */
template <uint8_t M>
inline float Vector<M>::operator()(uint8_t i) const
{
	if(i >= M)
		i = M - 1;

	return this->value[i];
}

/*
 * Add two vectors
 * @param lhs		Left hand side vector
 * @param rhs		Right hand side vector
 * @return 			Result of addition
 */
template <uint8_t M>
Vector<M> operator+(const Vector<M>& lhs, const Vector<M>& rhs)
{
	Vector<M> result(lhs);
	result += rhs;
	return result;
}

/*
 * Substract a vector from another
 * @param lhs		Left hand side vector
 * @param rhs		Right hand side vector
 * @return 			Result of substraction
 */
template <uint8_t M>
Vector<M> operator-(const Vector<M>& lhs, const Vector<M>& rhs)
{
	Vector<M> result(lhs);
	result -= rhs;
	return result;
}

/*
 * Multiply a matrix with a vector
 * @param lhs		Left hand side matrix
 * @param rhs		Right hand side vector
 * @return 			Result of multiplication (vector)
 */
template <uint8_t M, uint8_t N>
Vector<M> operator*(const Matrix<M, N>& lhs, const Vector<N>& rhs)
{
	Matrix<N, 1> rhs_mat(rhs);

	return lhs * rhs_mat;
}

/*
 * Multiply a vector with a 1xn matrix (outer/tensor product)
 * @param lhs		Left hand side vector
 * @param rhs		Right hand side 1xn matrix
 * @return 			Result of multiplication (matrix)
 */
template <uint8_t M, uint8_t N>
Matrix<M, N> operator*(const Vector<M>& lhs, const Matrix<1, N>& rhs)
{
	Matrix<M, 1> lhs_mat(lhs);

	return lhs_mat * rhs;
}

/*
 * Compute the dot product between two vectors
 * @param other		Right side vector
 * @return			Scalar result of dot product
 */
template <uint8_t M>
float Vector<M>::dot(const Vector<M> &rhs) const
{
	float result;

	for(int i = 0; i < M; i++)
		result += this->value[i] * rhs.value[i];

	return result;
}

/*
 * Compute the cross product between two three dimensional vectors
 * @param other		Right side three dimensional vector
 * @return			Three dimensional vector result of cross product
 */
template <uint8_t M>
Vector<3> Vector<M>::cross(const Vector<3> &rhs) const
{
	static_assert(M == 3, "Cross product is only defined for vectors with length = 3");

	Vector<3> result;

	result.value[0] = this->value[1] * rhs.value[2] - this->value[2] * rhs.value[1];
	result.value[1] = this->value[2] * rhs.value[0] - this->value[0] * rhs.value[2];
	result.value[2] = this->value[0] * rhs.value[1] - this->value[1] * rhs.value[0];

	return result;
}

/*
 * Compute the cross product between two three dimensional vectors
 * @param other		Right side three dimensional vector
 * @return			Three dimensional vector result of cross product
 */
template <uint8_t M>
float norm(const Vector<M>& vect)
{
	float result = 0;

	for(int i = 0; i < M; i++)
		result += vect.value[i] * vect.value[i];

	result = sqrt(result);

	return result;
}

/*
 * Initialize an empty FIFO buffer
 */
template <uint16_t M, typename T>
FIFO<M, T>::FIFO() : top(-1), bottom(0) {}

/*
 * Get the amount of elements, stored in the FIFO buffer
 * @return				Amount of elements in the FIFO
 */
template <uint16_t M, typename T>
int FIFO<M, T>::size() const
{
	if(this->top < 0)
		return 0;
	else if(this->top >= this->bottom)
		return this->top - this->bottom + 1;
	else
		return (M - this->bottom) + this->top + 1;
}

/*
 * Check if the FIFO is empty
 * @return				1, if FIFO is empty, else 0
 */
template <uint16_t M, typename T>
char FIFO<M, T>::empty() const
{
	if(this->top < 0)
		return 1;
	else
		return 0;
}

/*
 * Add a new element to the bottom of the FIFO buffer (newest)
 * If the buffer is already full, discard the oldest element
 * @param new_val		New element, to be added to the FIFO
 */
template <uint16_t M, typename T>
void FIFO<M, T>::push(const T& new_val)
{
	// Overwrite oldest element, if already full
	if(this->size() == M)
	{
		if(this->top == 0)
			this->top = M - 1;
		else
			this->top--;
	}

	// Adjust position to newly added element
	if(this->bottom == 0)
		this->bottom = M - 1;
	else
		this->bottom--;

	// Mark buffer as not empty, if it was so
	if(this->empty())
		this->top = this->bottom;

	this->buffer[this->bottom] = new_val;
}

/*
 * Get one element from the top of the FIFO (oldest)
 * If the buffer is already empty, return 0
 * @return				Oldest element in the FIFO
 */
template <uint16_t M, typename T>
T& FIFO<M, T>::pop()
{
	// Get oldest value, if not empty
	if(!this->empty())
	{
		uint16_t index = this->top;

		// Adjust top position to next oldest element, if not yet empty
		if(this->size() == 1)
			this->top = -1;
		else if(this->top == 0)
			this->top = M - 1;
		else
			this->top--;

		return this->buffer[index];
	}
	else
	{
		return this->buffer[this->bottom];
	}
}

/*
 * Get element with given index, beginning with 0 at oldest element
 * If there are not enough elemnts stored, return the newest one
 * If there are no elements stored, return a zero element
 * @param index		Index of element, beginning with oldest one
 * @return			Reference to element at given index
 */
template <uint16_t M, typename T>
T& FIFO<M, T>::operator[] (uint16_t index)
{
	if(index < this->size())
	{
		if((this->top - index) >= 0)
			return this->buffer[this->top - index];
		else
			return this->buffer[this->top - index + M];
	}
	else
	{
		return this->buffer[this->bottom];
	}
}

/*
 * Initializes the filter with 0
 * @param a			An array with the filter denominator coefficients
 * @param b			An array with the filter numerator coefficients
 */
template <uint8_t M, uint8_t N, typename T>
Filter<M, N, T>::Filter(const std::array<double, N>& denom, const std::array<double, M>& numer)
	: x{}, y{}, a(denom), b(numer), raw(0), filtered(0) {}

/*
 * Initializes the filter with a constant value
 * @param a			An array with the filter denominator coefficients
 * @param b			An array with the filter numerator coefficients
 * @param init_val	The initial value for the filter output
 */
template <uint8_t M, uint8_t N, typename T>
Filter<M, N, T>::Filter(const std::array<double, N>& denom, const std::array<double, M>& numer, const T& init_val)
	: a(denom), b(numer), raw(init_val), filtered(init_val)
{
	for(int i = 0; i < M; i++)
		this->x[i] = init_val;

	for(int j = 0; j < N; j++)
		this->y[j] = init_val;
}

/*
 * Updates the filter with a new input value
 * @param new_val		The new input value
 */
template <uint8_t M, uint8_t N, typename T>
T& Filter<M, N, T>::update(const T& new_val)
{
	// Shift the input vector by one place and add the new input value
	for(int i = M - 1; i > 0; i--)
		this->x[i] = this->x[i-1];

	this->x[0] = new_val;

	// Shift the output vector by one place
	for(int i = N - 1; i > 0; i--)
		this->y[i] = this->y[i-1];

	// Compute the new output value
	this->y[0] = 0;

	for(int i = M - 1; i >= 0; i--)
		this->y[0] += this->b[i] * this->x[i];

	for(int i = N - 1; i > 0; i--)
		this->y[0] -= this->a[i] * this->y[i];

	this->y[0] /= this->a[0];

	this->raw = this->x[0];
	this->filtered = this->y[0];

	return this->filtered;
}

/*
 * Resets the filter to a given value
 * @param reset_val		Value, the filter should be reset to
 */
template <uint8_t M, uint8_t N, typename T>
void Filter<M, N, T>::reset(const T& reset_val)
{
	for(int i = 0; i < M; i++)
		this->x[i] = reset_val;

	for(int i = 0; i < N; i++)
		this->y[i] = reset_val;
}

/*
 * Resets the filter to zero
 */
template <uint8_t M, uint8_t N, typename T>
void Filter<M, N, T>::reset()
{
	this->reset(T(0));
}

/**
 * Put the given message into the message queue for further handling with an additional matrix
 * @param message 	The message string (max 64 characters minus the characters in number)
 * @param mat		A matrix which gets added after the message string
 * @param priority	The priority of the Message object (Warning, Caution, Note, Debug)
 */
template <uint8_t M, uint8_t N>
void print(const char* message, Matrix<M, N> mat, char priority)
{
	char mBuffer[100];				// Buffer slightly larger than needed to avoid overflow
	strcpy(mBuffer, message);
	mat.string(mBuffer + strlen(message));

	print(mBuffer, priority);		// Maximum length of 64 characters will be checked here
}

#endif /* ARMHAWK_MATH_TPP_ */
