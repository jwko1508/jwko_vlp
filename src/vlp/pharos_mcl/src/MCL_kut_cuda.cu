#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cuda.h>
#include <cuda_runtime.h>
#include <time.h>
#include <math.h>

#define point_size 30000
static int *d_VMap;
static int *d_RMap;

static float *d_point_x = new float[point_size];
static float *d_point_y = new float[point_size];
static int *d_point_i = new int[point_size];
static double *d_w = new double[point_size];
static float *d_transed_point_x = new float[point_size];
static float *d_transed_point_y = new float[point_size];

__global__ void MeasInRMap(float *d_point_x, float *d_point_y, int *d_point_i, int *d_Map, float Map_resolution,
                           unsigned int Map_width, unsigned int Map_height, double Map_origin_x, double Map_origin_y, double *d_w)
{
    int tid = blockIdx.x;
    int xIndex, yIndex;
    int mapmeas;

    double resolutionInverse = 1/Map_resolution;
    if(tid < point_size){
        xIndex = (int)((d_point_x[tid] - Map_origin_x)*resolutionInverse);
        yIndex = (int)((d_point_y[tid] - Map_origin_y)*resolutionInverse);

        if(xIndex < Map_width && yIndex < Map_height){
            int mapIndex = Map_width*yIndex+xIndex;
            mapmeas = d_Map[mapIndex];
            if(mapmeas <0)
                mapmeas +=256;

            if(mapmeas > 100)
                d_w[tid] = 1;
            else
                d_w[tid] = 0;

            if(d_w[tid]>100)
                printf("road weight : %f\n",d_w[tid]);
        }
        else{
            d_w[tid]=0;
            printf("Out of RMap size!!!!!!!!!\n");
        }

    }
    else
        printf("Out of Road Point size!!!!!!!\n");

    }

__global__ void MeasInVMap(float *d_point_x, float *d_point_y, int *d_point_i, int *d_Map, float Map_resolution,
                          unsigned int Map_width, unsigned int Map_height, double Map_origin_x, double Map_origin_y, double *d_w)
{
    int tid = blockIdx.x;
    int xIndex, yIndex;
    int mapmeas;

    double resolutionInverse = 1/Map_resolution;
    if(tid<point_size){
        xIndex = (int)((d_point_x[tid] - Map_origin_x)*resolutionInverse);
        yIndex = (int)((d_point_y[tid] - Map_origin_y)*resolutionInverse);

        if(xIndex < Map_width && yIndex < Map_height){
            int mapIndex = Map_width*yIndex+xIndex;
            mapmeas = d_Map[mapIndex];
            if(mapmeas < 0)
                mapmeas +=256;

            int b1=0, b2=0;
            b1 = mapmeas&d_point_i[tid];
            int shBits;
            for (shBits=0; b1!=0;shBits++){
                b1 = b1 & (b1 -1);
            }

            // b2 = shBits*shBits;
            
            // if(mapmeas == d_point_i[tid] && b2!=1)
            //     b2=b2*2;

            // d_w[tid] = b2;
            d_w[tid] = shBits;
        }
        else{
printf("VMap%d\t%d\t%d\t%d\n",xIndex,Map_width,yIndex,Map_height);
            d_w[tid] = 0;
            printf("Out of VMap size!!!!!!!!!\n");
        }
    }
    else
        printf("Out of Vertical Point size!!!!!!!!!\n");

}

__global__ void Transformcuda(float *d_trans_point_x, float *d_trans_point_y, float *d_transed_point_x, float *d_transed_point_y, float Tx, float Ty, float theta)
{
    int tid = blockIdx.x;

    d_transed_point_x[tid] = d_trans_point_x[tid]*cos(theta) - d_trans_point_y[tid]*sin(theta) + Tx;
    d_transed_point_y[tid] = d_trans_point_x[tid]*sin(theta) + d_trans_point_y[tid]*cos(theta) + Ty;
}

double *MeasInMapCUDA(int N, float *point_x, float *point_y , int *point_i, int *Map, float Map_resolution,
                      unsigned int Map_width, unsigned int Map_height, double Map_origin_x, double Map_origin_y, float Tx, float Ty, float theta, double *w, std::string type)
{
    // Device copies of three inputs and output, size of allocated memory, num of threads and blocks
    cudaMemcpy(d_point_x,point_x,N*sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_point_y,point_y,N*sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_point_i,point_i,N*sizeof(int), cudaMemcpyHostToDevice);

    Transformcuda<<<N,1>>>(d_point_x, d_point_y, d_transed_point_x, d_transed_point_y, Tx, Ty, theta);
    if(type == "vertical")
        MeasInVMap<<<N,1>>>(d_transed_point_x, d_transed_point_y, d_point_i, d_VMap, Map_resolution, Map_width, Map_height, Map_origin_x, Map_origin_y, d_w);
    else if(type == "road")
        MeasInRMap<<<N,1>>>(d_transed_point_x, d_transed_point_y, d_point_i, d_RMap, Map_resolution, Map_width, Map_height, Map_origin_x, Map_origin_y, d_w);

    cudaMemcpy(w, d_w, N*sizeof(double), cudaMemcpyDeviceToHost);

    return w;
}

void CopyVMapCUDA(int *Map, unsigned int Map_width, unsigned int Map_height){
    d_VMap = new int[Map_width*Map_height];

    cudaMalloc((void **)&d_VMap, Map_width*Map_height*sizeof(int));
    cudaMemcpy(d_VMap, Map, Map_width*Map_height*sizeof(int), cudaMemcpyHostToDevice);


}
void CopyRMapCUDA(int *Map, unsigned int Map_width, unsigned int Map_height) {
    d_RMap = new int[Map_width*Map_height];

    cudaMalloc((void **)&d_RMap, Map_width * Map_height * sizeof(int));
    cudaMemcpy(d_RMap, Map, Map_width * Map_height * sizeof(int), cudaMemcpyHostToDevice);

    cudaMalloc((void **)&d_point_x, point_size*sizeof(float));
    cudaMalloc((void **)&d_point_y, point_size*sizeof(float));
    cudaMalloc((void **)&d_point_i, point_size*sizeof(int));

    cudaMalloc((void **)&d_transed_point_x, point_size*sizeof(float));
    cudaMalloc((void **)&d_transed_point_y, point_size*sizeof(float));

    cudaMalloc((void **)&d_w, point_size*sizeof(double));
}


void CUDAFree(){
    cudaFree(&d_VMap); cudaFree(&d_RMap); cudaFree(&d_point_x); cudaFree(&d_point_y); cudaFree(&d_point_i); cudaFree(&d_w);
    cudaFree(&d_transed_point_x);cudaFree(&d_transed_point_y);

}