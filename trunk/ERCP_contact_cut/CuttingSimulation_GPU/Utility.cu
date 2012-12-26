#include <cuda.h>
#include <cutil_inline.h>

template <int BLOCK_SIZE> __global__ void
matrixMulCUDA(float *C, float *A, float *B, int wA, int wB)
{
	    // Block index
    int bx = blockIdx.x;
    int by = blockIdx.y;

    // Thread index
    int tx = threadIdx.x;
    int ty = threadIdx.y;

    // Index of the first sub-matrix of A processed by the block
    int aBegin = wA * BLOCK_SIZE * by;

    // Index of the last sub-matrix of A processed by the block
    int aEnd   = aBegin + wA - 1;

    // Step size used to iterate through the sub-matrices of A
    int aStep  = BLOCK_SIZE;

    // Index of the first sub-matrix of B processed by the block
    int bBegin = BLOCK_SIZE * bx;

    // Step size used to iterate through the sub-matrices of B
    int bStep  = BLOCK_SIZE * wB;

    // Csub is used to store the element of the block sub-matrix
    // that is computed by the thread
    float Csub = 0;

    // Loop over all the sub-matrices of A and B
    // required to compute the block sub-matrix
    for (int a = aBegin, b = bBegin;
         a <= aEnd;
         a += aStep, b += bStep)
    {
        // Declaration of the shared memory array As used to
        // store the sub-matrix of A
        __shared__ float As[BLOCK_SIZE][BLOCK_SIZE];

        // Declaration of the shared memory array Bs used to
        // store the sub-matrix of B
        __shared__ float Bs[BLOCK_SIZE][BLOCK_SIZE];

        // Load the matrices from device memory
        // to shared memory; each thread loads
        // one element of each matrix
        As[ty][tx] = A[a + wA * ty + tx];
        Bs[ty][tx] = B[b + wB * ty + tx];

        // Synchronize to make sure the matrices are loaded
        __syncthreads();

        // Multiply the two matrices together;
        // each thread computes one element
        // of the block sub-matrix
#pragma unroll

        for (int k = 0; k < BLOCK_SIZE; ++k)
        {
            Csub += As[ty][k] * Bs[k][tx];
        }

        // Synchronize to make sure that the preceding
        // computation is done before loading two new
        // sub-matrices of A and B in the next iteration
        __syncthreads();
    }

    // Write the block sub-matrix to device memory;
    // each thread writes one element
    int c = wB * BLOCK_SIZE * by + BLOCK_SIZE * bx;
    C[c + wB * ty + tx] = Csub;
}

extern "C" int matrix_mul_cuda( float* h_C, float* h_A, float* h_B, int ra, int ca, int cb )
{
	int status = 0; // success
	int rb = ca;

	int block_size = 16;

	unsigned int mem_size_A = sizeof(float) *ra*ca;
	unsigned int mem_size_B = sizeof(float) *rb*cb;
	unsigned int mem_sie_C = sizeof(float) *ra*cb;
	
	float *d_A, *d_B, *d_C;
	
	cudaError_t error;
	cudaMalloc((void **) &d_A, mem_size_A);
	cudaMalloc((void**) &d_B, mem_size_B);
	cudaMalloc((void**) &d_C, mem_sie_C);

	cudaMemcpy(d_A, h_A, mem_size_A, cudaMemcpyHostToDevice);
	cudaMemcpy(d_B, h_B, mem_size_B, cudaMemcpyHostToDevice);

	//ececute parameters
    dim3 threads(block_size, block_size);
    dim3 grid(cb / threads.x, ra / threads.y);

	matrixMulCUDA<16><<< grid, threads >>>(d_C, d_A, d_B, ca, cb);
	cudaDeviceSynchronize();

	cudaMemcpy(h_C, d_C, mem_sie_C, cudaMemcpyDeviceToHost);
    cudaFree(d_A);
    cudaFree(d_B);
    cudaFree(d_C);

	return status;
}
