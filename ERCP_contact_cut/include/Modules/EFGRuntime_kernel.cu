#define GRAVITY 98
#define DIM 3
#define SDIM 6

#include <cuda.h>
#include <cutil_inline.h>

int  d_NbNode;
int	 d_NbNodeAdded;
int	 d_NbNodeInside;

float* d_NodePos0;
float* d_NodePos;
float* d_NodeVel;
float* d_NodeDis;
float* d_NodeForce;
float* d_NodeStrain;
float* d_NodeStress;
float* d_NodeVolume;
float* d_NodeMass;

int* d_NodeIdx;
int* d_NeighborNodeIdx;
int* d_NbNeighborNode;
int  d_NbNeighborMax;

float* d_ShapeFuncDerivAtNode;
float* d_ShapeFuncDerivXAtNode;
float* d_ShapeFuncDerivYAtNode;
float* d_ShapeFuncDerivZAtNode;
float* d_ShapeFuncDerivAtNodeInv;
float* d_ShapeFuncDerivXAtNodeInv;
float* d_ShapeFuncDerivYAtNodeInv;
float* d_ShapeFuncDerivZAtNodeInv;

// Stress point
int	 d_NbStressPoint;
float* d_StressVolume;
float* d_StressPos0;
float* d_StressPos;

int* d_NeighborStressPointIdx;
int* d_NbNeighborStressPoint;
int* d_StressNeighborNodeIdx;
int* d_NbStressNeighborNode;

float* d_ShapeFuncDerivAtStressPoint;
float* d_ShapeFuncDerivAtStressPointInv;

float* d_StressPointStrain;
float* d_StressPointStress;

float* d_MaterialStiffness;
int* d_FixedConstraint;

__global__ 
void updateValueFloat_k(float* previousValue, float* updatedValue, int* updatedIdx, int nbUpdated)
{
	int idx=blockIdx.x*blockDim.x+threadIdx.x;
	int _idx=updatedIdx[idx];
	if(idx<nbUpdated)
		previousValue[_idx]=updatedValue[idx];
}

__global__ 
void updateValueInt_k(int* previousValue, int* updatedValue, int* updatedIdx, int nbUpdated)
{
	int idx=blockIdx.x*blockDim.x+threadIdx.x;
	int _idx=updatedIdx[idx];
	if(idx<nbUpdated)
		previousValue[_idx]=updatedValue[idx];
}

__global__ 
void computeStrain_k(int* neighborNodeIdx, int* nbNeighborNode, int nbNeighborMax, float* shapeFuncDerivAtNodeInv, float* strain, float* nodeDis, int size)
{
	int nodeIdx = blockIdx.x;
	int tx = threadIdx.x;

	__shared__ float _strain[6];
	_strain[tx]=0;
		
	int idx1=tx%3;
	int idx2=tx-3;
	int idx3=(tx-2)%3;
	
	if(tx<3)
	{
		for(int i=0;i<nbNeighborNode[nodeIdx];i++)
		{
			float dis;
			int idx=nbNeighborMax*nodeIdx+i;
			dis=nodeDis[neighborNodeIdx[idx]*3+tx]-nodeDis[nodeIdx*3+tx];
			_strain[tx]+=shapeFuncDerivAtNodeInv[idx+size*tx]*dis;
		}
	}
	else
	{
		for(int i=0;i<nbNeighborNode[nodeIdx];i++)
		{
			float dis[2];
			int idx=nbNeighborMax*nodeIdx+i;
			dis[0]=nodeDis[neighborNodeIdx[idx]*3+idx2]-nodeDis[nodeIdx*3+idx2];
			dis[1]=nodeDis[neighborNodeIdx[idx]*3+idx3]-nodeDis[nodeIdx*3+idx3];
			_strain[tx]+=(shapeFuncDerivAtNodeInv[idx+size*idx3]*dis[0]+shapeFuncDerivAtNodeInv[idx+size*idx2]*dis[1]);
		}
	}
	strain[nodeIdx*6+tx]=_strain[tx];
}

__global__ 
void computeStress_k(float* nodeStrain, float* nodeStress, float* materialStiffness)
{
	int nodeIdx = blockIdx.x;
	int idx=threadIdx.x;
	
	__shared__ float stiffness[6][6];
	for(int i=0;i<6;i++)
	{
		stiffness[i][idx]=materialStiffness[6*i+idx];
	}

	__shared__ float strain[6];
	strain[idx]=nodeStrain[nodeIdx*6+idx];

	// Synchronize to make sure the matrices are loaded
    __syncthreads();

	nodeStress[nodeIdx*6+idx]=0;
	for(int i=0;i<6;i++)
	{
		nodeStress[nodeIdx*6+idx]+=stiffness[idx][i]*strain[idx];
	}
}

__global__ 
void computeForce_k(int* neighborNodeIdx, int* nbNeighborNode, int nbNeighborMax, float* shapeFuncDrv, float* stress, float* volume, float* vel, float* mass, float damping, float* force, int size)
{
	int nodeIdx = blockIdx.x;
	int tx = threadIdx.x;

	int idx1=size*tx;
	int idx2=size*((tx+1)%3);
	int idx3=size*((tx+2)%3);
	int idx4=(tx+2)%3+3;
	float _force=0;
	
	for(int i=0;i<nbNeighborNode[nodeIdx];i++)
	{
		int idx=nbNeighborMax*nodeIdx+i;
		_force-=(shapeFuncDrv[idx+idx1]*stress[neighborNodeIdx[idx]*6+tx]+shapeFuncDrv[idx+idx2]*stress[neighborNodeIdx[idx]*6+tx+3]+shapeFuncDrv[idx+idx3]*stress[neighborNodeIdx[idx]*6+idx4])*volume[nodeIdx];
	}

	//Add gravity
	if(tx==1)
		_force-=GRAVITY*mass[nodeIdx];

	//Add damping force
	_force-=vel[nodeIdx*3+tx]*damping;

	force[nodeIdx*3+tx]=_force;
}

__global__ 
void computeForce_k(int* neighborNodeIdx, int* nbNeighborNode, int nbNeighborMax, float* shapeFuncDrv, float* stress, float* volume, float* pos, float* vel, float* mass, float damping, float* force, int size)
{
	int nodeIdx = blockIdx.x;
	int tx = threadIdx.x;

	int idx1=size*tx;
	int idx2=size*((tx+1)%3);
	int idx3=size*((tx+2)%3);
	int idx4=(tx+2)%3+3;
	float _force=0;
	
	for(int i=0;i<nbNeighborNode[nodeIdx];i++)
	{
		int idx=nbNeighborMax*nodeIdx+i;
		_force-=(shapeFuncDrv[idx+idx1]*stress[neighborNodeIdx[idx]*6+tx]+shapeFuncDrv[idx+idx2]*stress[neighborNodeIdx[idx]*6+tx+3]+shapeFuncDrv[idx+idx3]*stress[neighborNodeIdx[idx]*6+idx4])*volume[nodeIdx];
	}

	//Add gravity
	if(tx==1)
		_force-=GRAVITY*mass[nodeIdx];

	//Add damping force
	_force-=vel[nodeIdx*3+tx]*damping;

	force[nodeIdx*3+tx]=_force;
}

__global__ 
void computeForceStress_k(int* neighborNodeIdx, int* nbNeighborNode, int nbNeighborMax, float* shapeFuncDrv, float* stress, float* volume, float* force, int size)
{
	int nodeIdx = blockIdx.x;
	int tx = threadIdx.x;

	int idx1=size*tx;
	int idx2=size*((tx+1)%3);
	int idx3=size*((tx+2)%3);
	int idx4=(tx+2)%3+3;
	float _force=0;
	
	for(int i=0;i<nbNeighborNode[nodeIdx];i++)
	{
		int idx=nbNeighborMax*nodeIdx+i;
		_force-=(shapeFuncDrv[idx+idx1]*stress[neighborNodeIdx[idx]*6+tx]+shapeFuncDrv[idx+idx2]*stress[neighborNodeIdx[idx]*6+tx+3]+shapeFuncDrv[idx+idx3]*stress[neighborNodeIdx[idx]*6+idx4])*volume[nodeIdx];
	}
	force[nodeIdx*3+tx]+=_force;
}

__global__ 
void explicitIntegration_k(float* dis, float* pos, float* pos0, float* vel, float* force, float* mass, int* constraint, int nbConstraint, float dt)
{
	int nodeIdx = blockIdx.x;
	int tx = threadIdx.x;

	float dv=force[nodeIdx*3+tx]/mass[nodeIdx]*dt;
	vel[nodeIdx*3+tx]+=dv;
	dis[nodeIdx*3+tx]+=(vel[nodeIdx*3+tx]*dt);
	pos[nodeIdx*3+tx]=pos0[nodeIdx*3+tx]+dis[nodeIdx*3+tx];
}

__global__ 
void explicitIntegrationConst_k(float* dis, float* pos, float* pos0, float* force, float* mass, float dt)
{
	int nodeIdx = blockIdx.x;
	int tx = threadIdx.x;

	float dv=force[nodeIdx*3+tx]/mass[nodeIdx]*dt;
	dis[nodeIdx*3+tx]+=(dv*dt);
	pos[nodeIdx*3+tx]=pos0[nodeIdx*3+tx]+dis[nodeIdx*3+tx];
}

__global__ 
void fixedConstraint_k(float* dis, float* pos, float* pos0, int* constraint)
{
	int idx=blockIdx.x;
	int tx=threadIdx.x;

	pos[constraint[idx]*3+tx]=pos0[constraint[idx]*3+tx];
	dis[constraint[idx]*3+tx]=0;
}

extern "C" void d_initGPU(int nbNode, int nbNodeAdded, float* nodeVolume, float* nodePos0, float* nodePos, float* nodeVel, int nbNodeInside)
{
	// Initialize
	int devID;
	cudaDeviceProp props;
	d_NbNode=nbNode;
	d_NbNodeAdded=nbNodeAdded;
	d_NbNodeInside=nbNodeInside;
	d_NbStressPoint=-1;

	// get number of SMs on this GPU
	cutilSafeCall(cudaGetDevice(&devID));
	cutilSafeCall(cudaGetDeviceProperties(&props, devID));

	// Memory allocation
	int size=(nbNode+nbNodeAdded)*sizeof(float);
	cutilSafeCall(cudaMalloc((void**) &d_NodeVolume, size));
	cutilSafeCall(cudaMemcpy(d_NodeVolume, nodeVolume, size, cudaMemcpyHostToDevice) );

	size=nbNode*DIM*sizeof(float);
	cutilSafeCall(cudaMalloc((void**) &d_NodePos0, size));
	cutilSafeCall(cudaMalloc((void**) &d_NodePos, size));
	cutilSafeCall(cudaMalloc((void**) &d_NodeVel, size));
	cutilSafeCall(cudaMemcpy(d_NodePos0, nodePos0, size, cudaMemcpyHostToDevice) );
	cutilSafeCall(cudaMemcpy(d_NodePos, nodePos, size, cudaMemcpyHostToDevice) );
	cutilSafeCall(cudaMemcpy(d_NodeVel, nodeVel, size, cudaMemcpyHostToDevice) );

	cutilSafeCall(cudaMalloc((void**) &d_NodeDis, size));
	cutilSafeCall(cudaMalloc((void**) &d_NodeForce, size));
	// zero default value for nodeDis, nodeForce, nodeStrain and nodeStress
	{
		float* temp0 = (float*)malloc(size);
		int i=0;
		for(i=0; i<nbNode*DIM; i++)
		{
			temp0[i] = 0;
		}
		cutilSafeCall(cudaMemcpy(d_NodeDis, temp0, size, cudaMemcpyHostToDevice) );
		cutilSafeCall(cudaMemcpy(d_NodeForce, temp0, size, cudaMemcpyHostToDevice) );
		free(temp0);
	}

	size=nbNode*SDIM*sizeof(float);
	cutilSafeCall(cudaMalloc((void**) &d_NodeStrain, size));
	cutilSafeCall(cudaMalloc((void**) &d_NodeStress, size));
		// zero default value for nodeDis, nodeForce, nodeStrain and nodeStress
	{
		float* temp0 = (float*)malloc(size);
		int i=0;
		for(i=0; i<nbNode*SDIM; i++)
		{
			temp0[i] = 0;
		}
		cutilSafeCall(cudaMemcpy(d_NodeStrain, temp0, size, cudaMemcpyHostToDevice) );
		cutilSafeCall(cudaMemcpy(d_NodeStress, temp0, size, cudaMemcpyHostToDevice) );
		free(temp0);
	}
}

extern "C" void d_initStressPoint(int nbStress, float* stressVolume, float* stressPos0, float* stressPos)
{
	d_NbStressPoint=nbStress;
	
	// Memory allocation
	int size=(d_NbStressPoint+d_NbNodeAdded)*sizeof(float);
	cutilSafeCall(cudaMalloc((void**) &d_StressVolume, size));
	cutilSafeCall(cudaMemcpy(d_StressVolume, stressVolume, size, cudaMemcpyHostToDevice));
	
	size=nbStress*DIM*sizeof(float);
	cutilSafeCall(cudaMalloc((void**) &d_StressPos0, size));
	cutilSafeCall(cudaMalloc((void**) &d_StressPos, size));
	cutilSafeCall(cudaMemcpy(d_StressPos0, stressPos0, size, cudaMemcpyHostToDevice) );
	cutilSafeCall(cudaMemcpy(d_StressPos, stressPos, size, cudaMemcpyHostToDevice) );
	
	size=nbStress*SDIM*sizeof(float);
	cutilSafeCall(cudaMalloc((void**) &d_StressPointStrain, size));
	cutilSafeCall(cudaMalloc((void**) &d_StressPointStress, size));
}

extern "C" void d_initNeighborInformation(int* neighborNodeIdx, int* nbNeighborNode, int nbNeighborMax)
{
	int size=(d_NbNode+d_NbNodeAdded)*sizeof(int);
	cutilSafeCall(cudaMalloc((void**) &d_NodeIdx, size));

	size=nbNeighborMax*(d_NbNode+d_NbNodeAdded)*sizeof(int);
	cutilSafeCall(cudaMalloc((void**) &d_NeighborNodeIdx, size));
	cutilSafeCall(cudaMemcpy(d_NeighborNodeIdx, neighborNodeIdx, size, cudaMemcpyHostToDevice));

	size=(d_NbNode+d_NbNodeAdded)*sizeof(int);
	cutilSafeCall(cudaMalloc((void**) &d_NbNeighborNode, size));

	cutilSafeCall(cudaMemcpy(d_NbNeighborNode, nbNeighborNode, size, cudaMemcpyHostToDevice));
	d_NbNeighborMax=nbNeighborMax;
}

extern "C" void d_initNeighborInformationStress(int* neighborStressPointIdx, int* nbNeighborStressPoint, int* stressNeighborNodeIdx, int* nbStressNeighborNode)
{
	int size=d_NbNeighborMax*(d_NbNode+d_NbNodeAdded)*sizeof(int);
	cutilSafeCall(cudaMalloc((void**) &d_NeighborStressPointIdx, size));
	cutilSafeCall(cudaMemcpy(d_NeighborStressPointIdx, neighborStressPointIdx, size, cudaMemcpyHostToDevice));
	
	size=(d_NbNode+d_NbNodeAdded)*sizeof(int);
	cutilSafeCall(cudaMalloc((void**) &d_NbNeighborStressPoint, size));
	cutilSafeCall(cudaMemcpy(d_NbNeighborStressPoint, nbNeighborStressPoint, size, cudaMemcpyHostToDevice));
	
	size=d_NbNeighborMax*(d_NbStressPoint+d_NbNodeAdded)*sizeof(int);
	cutilSafeCall(cudaMalloc((void**) &d_StressNeighborNodeIdx, size));
	cutilSafeCall(cudaMemcpy(d_StressNeighborNodeIdx, stressNeighborNodeIdx, size, cudaMemcpyHostToDevice));
	
	size=(d_NbStressPoint+d_NbNodeAdded)*sizeof(int);
	cutilSafeCall(cudaMalloc((void**) &d_NbStressNeighborNode, size));
	cutilSafeCall(cudaMemcpy(d_NbStressNeighborNode, nbStressNeighborNode, size, cudaMemcpyHostToDevice));
}

extern "C" void d_initShapeFuncValue(float* shapeFuncDerivAtNode, float* shapeFuncDerivAtNodeInv)
{
	int size=(d_NbNode+d_NbNodeAdded)*d_NbNeighborMax*sizeof(float)*3;
	cutilSafeCall(cudaMalloc((void**) &d_ShapeFuncDerivAtNode, size));
	cutilSafeCall(cudaMalloc((void**) &d_ShapeFuncDerivAtNodeInv, size));

	cutilSafeCall(cudaMemcpy(d_ShapeFuncDerivAtNode, shapeFuncDerivAtNode, size, cudaMemcpyHostToDevice));
	cutilSafeCall(cudaMemcpy(d_ShapeFuncDerivAtNodeInv, shapeFuncDerivAtNodeInv, size, cudaMemcpyHostToDevice));
}

extern "C" void d_initShapeFuncValueStressPoint(float* shapeFuncDerivAtStressPoint, float* shapeFuncDerivAtStressPointInv)
{
	int size=(d_NbNode+d_NbNodeAdded)*d_NbNeighborMax*sizeof(float)*3;
	cutilSafeCall(cudaMalloc((void**) &d_ShapeFuncDerivAtStressPoint, size));
	cutilSafeCall(cudaMemcpy(d_ShapeFuncDerivAtStressPoint, shapeFuncDerivAtStressPoint, size, cudaMemcpyHostToDevice));
	
	size=(d_NbStressPoint+d_NbNodeAdded)*d_NbNeighborMax*sizeof(float)*3;
	cutilSafeCall(cudaMalloc((void**) &d_ShapeFuncDerivAtStressPointInv, size));
	cutilSafeCall(cudaMemcpy(d_ShapeFuncDerivAtStressPointInv, shapeFuncDerivAtStressPointInv, size, cudaMemcpyHostToDevice));
}


extern "C" void d_initMaterialStiffness(float* materialStiffness, float* nodeMass)
{
	CUresult error;
	int size=SDIM*SDIM*sizeof(float);
	cutilSafeCall(cudaMalloc((void**) &d_MaterialStiffness, size));
	cutilSafeCall(cudaMemcpy(d_MaterialStiffness, materialStiffness, size, cudaMemcpyHostToDevice));

	size=(d_NbNode+d_NbNodeAdded)*sizeof(float);
	cutilSafeCall(cudaMalloc((void**) &d_NodeMass, size));
	cutilSafeCall(cudaMemcpy(d_NodeMass, nodeMass, size, cudaMemcpyHostToDevice));
}

extern "C" void d_explicitIntegration(float dt, int nbFixedConstraint)
{
	dim3 threadsPerBlock(DIM);
	dim3 blocksPerGrid(d_NbNode);

	explicitIntegration_k<<< blocksPerGrid, threadsPerBlock >>> (d_NodeDis, d_NodePos, d_NodePos0, d_NodeVel, d_NodeForce, d_NodeMass, d_FixedConstraint, nbFixedConstraint, dt);

	if(nbFixedConstraint > 0)
	{
		blocksPerGrid.x = nbFixedConstraint;
		fixedConstraint_k<<< blocksPerGrid, threadsPerBlock >>> (d_NodeDis, d_NodePos, d_NodePos0, d_FixedConstraint);
	}
}

extern "C" void d_explicitIntegrationConst(float dt)
{
	dim3 threadsPerBlock(DIM);
	dim3 blocksPerGrid(d_NbNode);
	
	explicitIntegrationConst_k<<< blocksPerGrid, threadsPerBlock >>> (d_NodeDis, d_NodePos, d_NodePos0, d_NodeForce, d_NodeMass, dt);
}

extern "C" void d_computeStrain()
{
	dim3 threadsPerBlock(SDIM);
	dim3 blocksPerGrid(d_NbNode);
	
	int size=(d_NbNode+d_NbNodeAdded)*d_NbNeighborMax;
	computeStrain_k<<< blocksPerGrid, threadsPerBlock >>>(d_NeighborNodeIdx, d_NbNeighborNode, d_NbNeighborMax, d_ShapeFuncDerivAtNodeInv, d_NodeStrain, d_NodeDis, size);
	
	if(d_NbStressPoint>0)
	{
		size=(d_NbStressPoint+d_NbNodeAdded)*d_NbNeighborMax;	
		blocksPerGrid.x=d_NbStressPoint;
		computeStrain_k<<< blocksPerGrid, threadsPerBlock >>>(d_StressNeighborNodeIdx, d_NbStressNeighborNode, d_NbNeighborMax, d_ShapeFuncDerivAtStressPointInv, d_StressPointStrain, d_NodeDis, size);
	}	
}

extern "C" void d_computeStress()
{
	dim3 threadsPerBlock(SDIM);
	dim3 blocksPerGrid(d_NbNode);

	computeStress_k<<< blocksPerGrid, threadsPerBlock >>>(d_NodeStrain, d_NodeStress, d_MaterialStiffness);
	
	if(d_NbStressPoint>0)
	{
		blocksPerGrid.x=d_NbStressPoint;
		computeStress_k<<< blocksPerGrid, threadsPerBlock >>>(d_StressPointStrain, d_StressPointStress, d_MaterialStiffness);
	}
}

extern "C" void d_computeForce(float damping)
{
	dim3 threadsPerBlock(DIM);
	dim3 blocksPerGrid(d_NbNode);

	int size=(d_NbNode+d_NbNodeAdded)*d_NbNeighborMax;
	computeForce_k<<< blocksPerGrid, threadsPerBlock >>>(d_NeighborNodeIdx, d_NbNeighborNode, d_NbNeighborMax, d_ShapeFuncDerivAtNode, d_NodeStress, d_NodeVolume, d_NodePos0, d_NodeVel, d_NodeMass, damping, d_NodeForce, size);
	
	if(d_NbStressPoint>0)
	{
		computeForceStress_k<<< blocksPerGrid, threadsPerBlock >>>(d_NeighborStressPointIdx, d_NbNeighborStressPoint, d_NbNeighborMax, d_ShapeFuncDerivAtStressPoint, d_StressPointStress, d_StressVolume, d_NodeForce, size);
	}
}

extern "C" void d_initFixedConstraint(int nbFixedConstraint, int* fixedIdx)
{
	int size=nbFixedConstraint*sizeof(int);
	cutilSafeCall(cudaMalloc((void**) &d_FixedConstraint, size));
	cutilSafeCall(cudaMemcpy(d_FixedConstraint, fixedIdx, size, cudaMemcpyHostToDevice));
}

extern "C" void d_getPosition(float* nodePos)
{
	int size=d_NbNode*DIM*sizeof(float);
	cutilSafeCall(cudaMemcpy(nodePos, d_NodePos, size, cudaMemcpyDeviceToHost));
}

extern "C" void d_getDisplacement(float* nodeDis)
{
	int size=d_NbNode*DIM*sizeof(float);
	cutilSafeCall(cudaMemcpy(nodeDis, d_NodeDis, size, cudaMemcpyDeviceToHost));
}

extern "C" void d_getMass(float* nodeMass)
{
	int size=d_NbNode*sizeof(float);
	cutilSafeCall(cudaMemcpy(nodeMass, d_NodeMass, size, cudaMemcpyDeviceToHost));
}

extern "C" void d_getStressPointStrain(float* stressPointStrain)
{
	int size=d_NbStressPoint*sizeof(float)*SDIM;
	cutilSafeCall(cudaMemcpy(stressPointStrain, d_StressPointStrain, size, cudaMemcpyDeviceToHost));
}
extern "C" void d_getStressPointStress(float* stressPointStress)
{
	int size=d_NbStressPoint*sizeof(float)*SDIM;
	cutilSafeCall(cudaMemcpy(stressPointStress, d_StressPointStress, size, cudaMemcpyDeviceToHost));
}
extern "C" void d_getNodeStrain(float* nodeStrain)
{
	int size=d_NbNode*sizeof(float)*SDIM;
	cutilSafeCall(cudaMemcpy(nodeStrain, d_NodeStrain, size, cudaMemcpyDeviceToHost));
}
extern "C" void d_getNodeStress(float* nodeStress)
{
	int size=d_NbStressPoint*sizeof(float)*SDIM;
	cutilSafeCall(cudaMemcpy(nodeStress, d_NodeStress, size, cudaMemcpyDeviceToHost));
}

extern "C" void d_setForce(float* nodeForce)
{
	int size=d_NbNode*sizeof(float)*DIM;
	cutilSafeCall(cudaMemcpy(d_NodeForce, nodeForce, size, cudaMemcpyHostToDevice));
}

extern "C" void d_updateNbNeighborNodeFull(int* nbNeighborNode, int size)
{
	cudaMemcpy(d_NbNeighborNode, nbNeighborNode, size, cudaMemcpyHostToDevice);
}
extern "C" void d_updateNbNeighborNodePartial(int* updatedValue, int* updatedIdx, int nbUpdated)
{
	int size=nbUpdated*sizeof(int);
	int* d_UpdatedValue;
	int* d_UpdatedIdx;
	cudaMalloc((void**) &d_UpdatedValue, size);
	cudaMalloc((void**) &d_UpdatedIdx, size);
	cudaMemcpy(d_UpdatedValue, updatedValue, size, cudaMemcpyHostToDevice);
	cudaMemcpy(d_UpdatedIdx, updatedIdx, size, cudaMemcpyHostToDevice);

	int nbBlock=nbUpdated/256+1;
	dim3 threadsPerBlock(256);
	dim3 blocksPerGrid(nbBlock);
	updateValueInt_k<<< blocksPerGrid, threadsPerBlock >>> (d_NbNeighborNode, updatedValue, updatedIdx, nbUpdated);
}
extern "C" void d_updateNeighborNodeIdxFull(int* neighborNodeIdx, int size)
{
	cudaMemcpy(d_NeighborNodeIdx, neighborNodeIdx, size, cudaMemcpyHostToDevice);
}
extern "C" void d_updateNeighborNodeIdxPartial(int* updatedValue, int* updatedIdx, int nbUpdated)
{
	int size=nbUpdated*sizeof(int);
	int* d_UpdatedValue;
	int* d_UpdatedIdx;
	cudaMalloc((void**) &d_UpdatedValue, size);
	cudaMalloc((void**) &d_UpdatedIdx, size);
	cudaMemcpy(d_UpdatedValue, updatedValue, size, cudaMemcpyHostToDevice);
	cudaMemcpy(d_UpdatedIdx, updatedIdx, size, cudaMemcpyHostToDevice);

	int nbBlock=nbUpdated/256+1;
	dim3 threadsPerBlock(256);
	dim3 blocksPerGrid(nbBlock);
	updateValueInt_k<<< blocksPerGrid, threadsPerBlock >>> (d_NeighborNodeIdx, d_UpdatedValue, d_UpdatedIdx, nbUpdated);
}

extern "C" void d_updateShapeFuncDrvFull(float* value, int size)
{	
	cudaMemcpy(d_ShapeFuncDerivAtNode, value, size, cudaMemcpyHostToDevice);
}

extern "C" void d_updateShapeFuncDrvInvFull(float* value, int size)
{	
	cudaMemcpy(d_ShapeFuncDerivAtNodeInv, value, size, cudaMemcpyHostToDevice);
}

extern "C" void d_updateShapeFuncDrvPartial(float* updatedValue, int* updatedIdx, int nbUpdated)
{
	int size=nbUpdated*sizeof(float);
	float* d_UpdatedValue;
	cudaMalloc((void**) &d_UpdatedValue, size);
	cudaMemcpy(d_UpdatedValue, updatedValue, size, cudaMemcpyHostToDevice);

	size=nbUpdated*sizeof(int);
	int* d_UpdatedIdx;
	cudaMalloc((void**) &d_UpdatedIdx, size);
	cudaMemcpy(d_UpdatedIdx, updatedIdx, size, cudaMemcpyHostToDevice);

	int nbBlock=nbUpdated/256+1;
	dim3 threadsPerBlock(256);
	dim3 blocksPerGrid(nbBlock);
	updateValueFloat_k<<< blocksPerGrid, threadsPerBlock >>> (d_ShapeFuncDerivAtNode, d_UpdatedValue, d_UpdatedIdx, nbUpdated);
}

extern "C" void d_updateShapeFuncDrvInvPartial(float* updatedValue, int* updatedIdx, int nbUpdated)
{
	int size=nbUpdated*sizeof(float);
	float* d_UpdatedValue;
	cudaMalloc((void**) &d_UpdatedValue, size);
	cudaMemcpy(d_UpdatedValue, updatedValue, size, cudaMemcpyHostToDevice);

	size=nbUpdated*sizeof(int);
	int* d_UpdatedIdx;
	cudaMalloc((void**) &d_UpdatedIdx, size);
	cudaMemcpy(d_UpdatedIdx, updatedIdx, size, cudaMemcpyHostToDevice);

	int nbBlock=nbUpdated/256+1;
	dim3 threadsPerBlock(256);
	dim3 blocksPerGrid(nbBlock);
	updateValueFloat_k<<< blocksPerGrid, threadsPerBlock >>> (d_ShapeFuncDerivAtNodeInv, d_UpdatedValue, d_UpdatedIdx, nbUpdated);
}