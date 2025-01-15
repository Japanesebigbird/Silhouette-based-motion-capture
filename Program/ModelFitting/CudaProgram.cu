							static double M_PI		= 3.141592653589793238462643383279;
	__device__	static double M_PI_d	= 3.141592653589793238462643383279;

	__device__	static unsigned long long	BitList_d[65]={	1,											2,											4,											8,
																												16,											32,											64,											128,
																												256,										512,										1024,										2048,
																												4096,										8192,										16384,									32768,
																												65536,									131072,									262144,									524288,
																												1048576,								2097152,								4194304,								8388608,
																												16777216,								33554432,								67108864,								134217728,
																												268435456,							536870912,							1073741824,							2147483648,
																												4294967296,							8589934592,							17179869184,						34359738368,
																												68719476736,						137438953472,						274877906944,						549755813888,
																												1099511627776,					2199023255552,					4398046511104,					8796093022208,
																												17592186044416,					35184372088832,					70368744177664,					140737488355328,
																												281474976710656,				562949953421312,				1125899906842624,				2251799813685248,
																												4503599627370496,				9007199254740992,				18014398509481984,			36028797018963968,
																												72057594037927936,			144115188075855872,			288230376151711744,			576460752303423488,
																												1152921504606846976,		2305843009213693952,		4611686018427387904,		9223372036854775808	};


							static unsigned long long	BitList[65]={		1,											2,											4,											8,
																												16,											32,											64,											128,
																												256,										512,										1024,										2048,
																												4096,										8192,										16384,									32768,
																												65536,									131072,									262144,									524288,
																												1048576,								2097152,								4194304,								8388608,
																												16777216,								33554432,								67108864,								134217728,
																												268435456,							536870912,							1073741824,							2147483648,
																												4294967296,							8589934592,							17179869184,						34359738368,
																												68719476736,						137438953472,						274877906944,						549755813888,
																												1099511627776,					2199023255552,					4398046511104,					8796093022208,
																												17592186044416,					35184372088832,					70368744177664,					140737488355328,
																												281474976710656,				562949953421312,				1125899906842624,				2251799813685248,
																												4503599627370496,				9007199254740992,				18014398509481984,			36028797018963968,
																												72057594037927936,			144115188075855872,			288230376151711744,			576460752303423488,
																												1152921504606846976,		2305843009213693952,		4611686018427387904,		9223372036854775808	};


							static unsigned char BitSize		= sizeof(unsigned long long	)	*	8;
	__device__	static unsigned char BitSize_d	= sizeof(unsigned long long	)	*	8;


	__device__		unsigned char	List01[7][4]	=	{	{0,2,4,6},{0,1,4,5},{0,1,4,5},{0,1,2,3},{0,1,2,3},{0,1,2,3},{0,1,2,3}	};
	__device__		unsigned char	List02[7][4]	=	{	{1,3,5,7},{2,3,6,7},{3,2,7,6},{4,5,6,7},{5,6,7,4},{6,7,4,5},{7,4,5,6}	};



#include	<stdio.h>
#include	<string.h>
#include	<stdlib.h>
#include	<math.h>
#include	<time.h>
#include	<float.h>
#include	"MT.h"

#include <curand.h>
#include <curand_kernel.h>


#include <curand_mtgp32_host.h>
#include <curand_mtgp32dc_p_11213.h>
#include	<windows.h>


	typedef	struct
	{
		unsigned long long	Val[32];
	}	ULL32;


	typedef	struct
	{
		float3	Val[32];
	}	FLO332;


	typedef	struct
	{
		unsigned int				MinMax_Gro;
		unsigned int			 	HideFlag_1;
		unsigned long long	HideData[32];
		float3							LineCo[32];
	}	GROUP_DATA;

	typedef	struct
	{
		unsigned short int	nVertex;
		unsigned short int	nTriangle;
		unsigned short int	nTriangle_32;
		unsigned short int	Kintree_Table[24*2+100];
		ushort3							NewV_Point_32[13792+100];
		ushort3							V_Point3[13776+100];
		ushort3							LandMark01_Vertex[30];
		ushort3							LandMark02_Vertex[30];
		float3							LandMark01_Weight[30];
		float3							LandMark02_Weight[30];
		float								Shape_Dirs[6890*3*300+5000];
		float4							Shape_Dirs4[6890*3*(300/4)+1000];
		int4								Shape_Dirs4_Int[6890*3*(300/4)+1000];
		float								V_Temp[6890*3+100];
		float								J_Regressor[6890*24+100];
		float								Pose_Dirs[6890*3*93+100];
		float3							Pose_Dirs3[6890*3*31+100];
		float								Weights[6890*24+100];
		float4							Weights4[6890*(24/4)+100];

		unsigned short int	J_Reg_Vertex[270];
		unsigned short int	J_Reg_Joint[270];
		float								J_Reg_Co[270];
		unsigned short int	Pose_DirsVer2_Vertex[163534+30];
		unsigned short int	Pose_DirsVer2_FeatITI[163534+30];
		float								Pose_DirsVer2_Co[490602+100];
		unsigned int				Pose_DirsVer2_Start_nFeat[6890*2+100];

		unsigned short int	WeightsVer2_JointITI[18948+100];
		float								WeightsVer2_Co[18948+100];
		unsigned int				WeightsVer2_Start_nFeat[6890*2+100];
	}	STAR_Parameter;


	typedef	struct
	{
		float								TempObje;
		float								TempObje_Betas;
		float								V_Pose[6890*3+100];
		float								V_Pose_Ori[6890*3+100];
		float3							Vertex[6890+10];
		float								Feature[95];
		float								HTrans2[25][4][4];
		float3							GrobalJoint_3[(24)+5];
		float								Model_Joint[24*3+5];

		float2							GrobalJoint_Degi_2_Cam[8][(24)+5];
		float2							Keypoint_Repro_2_Cam[8][(24)+5];
		unsigned long long 	Label_Opt[8][(1080*1920/64)+50];
		GROUP_DATA					GroupData[8][450];
		unsigned char				MinMax_All[8][4];
		unsigned char				ReturnFlag;
	}	STAR_Particle;

	typedef	struct
	{
		unsigned char				nCam;
		unsigned char				nImage;
		unsigned char				nKeyPoint;
		unsigned char				ObjeFlag;
		int									Pix_Yoko;
		int									Pix_Tate;
		int									Pix_Yoko_8;
		int									Pix_Tate_8;
		float								OriUo;
		float								OriVo;
		float								Keypoint[300][45];
		float								KeypointAAA[20][45];
		unsigned long long	Label[300][(1080*1920/64)+50];
		unsigned char				MinMax_All[300][10];
		double							CamPara[10];
		double							CL[3];
		double							Rm[9];
		double							F[1];

	}	Camera_Parameter;

	typedef	struct
	{
		int			nValue;
		int			nPartical;
		int			nPartical_Lo;
		int			nLocal;
		int			Still;
		int			Still02;
	}	OptP;


		double	*mb;
		double	*rb;
		double	*Part_Dis;
		double	*Part_Dis_PB;
		double	*Part_Vel;
		double	*Part_PB;
		double	*Part_Obje;
		int			*Part_Rank;
		double	*Part_NewDis;
		double	*Part_NewVel;


		double	*mb_d;
		double	*rb_d;
		double	*Part_Dis_d;
		double	*Part_Dis_PB_d;
		double	*Part_Vel_d;
		double	*Part_PB_d;
		double	*Part_Obje_d;
		int			*Part_Rank_d;
		double	*Part_NewDis_d;
		double	*Part_NewVel_d;




	OptP				OptPara[1];
	OptP				*OptPara_d;

	cudaError_t		ErrorID;

	curandStateMtgp32 		*devMTGPStates;
	mtgp32_kernel_params	*devKernelParams;


	STAR_Parameter	*STAR_Para;
	STAR_Parameter	*STAR_Para_d;

	STAR_Particle	*STAR_Partic;
	STAR_Particle	*STAR_Partic_d;


	Camera_Parameter	*Cam_Para;
	Camera_Parameter	*Cam_Para_d;


	void		Get_BasePara(	void	);
	void		Get_BasePara_GPU(	void	);


	#include	"CudaProgram_Obje.h"


	typedef	void	(*Func_Point)(	STAR_Parameter		STAR_Para[],	STAR_Particle		STAR_Partic[],
																Camera_Parameter	Cam_Para[],		OptP						OptPara[],
																float							Part_Dis[],		float						Part_Dis_PB[],
																float							Part_Obje[],	float						Part_PB[],
																float							rb[],					float						mb[]	);

	#include	"CudaProgram_Partical_Swarm.h"

	int	main(void)
	{

/*
    int AAA;    //デバイス数
   	cudaGetDeviceCount(&AAA);

		printf("AA %d \n",	AAA	);

		cudaDeviceProp dev;
 
		// デバイスプロパティ取得
    cudaGetDeviceProperties(&dev, 0);

		printf("device %d\n", 0);
		printf(" device name : %s\n", dev.name);
		printf(" total global memory : %d (MB)\n", dev.totalGlobalMem/1024/1024);
		printf(" shared memory / block : %d (KB)\n", dev.sharedMemPerBlock/1024);
		printf(" register / block : %d\n", dev.regsPerBlock);
		printf(" warp size : %d\n", dev.warpSize);
		printf(" max pitch : %d (B)\n", dev.memPitch);
		printf(" max threads / block : %d\n", dev.maxThreadsPerBlock);
		printf(" max size of each dim. of block : (%d, %d, %d)\n", dev.maxThreadsDim[0], dev.maxThreadsDim[1], dev.maxThreadsDim[2]);
		printf(" max size of each dim. of grid  : (%d, %d, %d)\n", dev.maxGridSize[0], dev.maxGridSize[1], dev.maxGridSize[2]);
		printf(" clock rate : %d (MHz)\n", dev.clockRate/1000);
		printf(" total constant memory : %d (KB)\n", dev.totalConstMem/1024);
		printf(" compute capability : %d.%d\n", dev.major, dev.minor);
		printf(" alignment requirement for texture : %d\n", dev.textureAlignment);
		printf(" device overlap : %s\n", (dev.deviceOverlap ? "ok" : "not"));
		printf(" num. of multiprocessors : %d\n", dev.multiProcessorCount);
		printf(" kernel execution timeout : %s\n", (dev.kernelExecTimeoutEnabled ? "on" : "off"));
		printf(" integrated : %s\n", (dev.integrated ? "on" : "off"));
		printf(" host memory mapping : %s\n", (dev.canMapHostMemory ? "on" : "off"));
		exit(1);

/*
		for (	int iLoop=0;	iLoop<1920;	iLoop++	)
		{
			printf("AA %d %d %d \n",	iLoop,	iLoop%640,	(iLoop%640)/80	);
		}
		exit(1);
*/

		/*--------------------------*/
		/*	Initialize Random Seed	*/
		/*--------------------------*/
		init_genrand((unsigned)time(NULL));

		/*----------------*/
		/*	GPU Memory		*/
		/*----------------*/
		size_t size;
		cudaDeviceGetLimit(&size, cudaLimitMallocHeapSize);
		printf("Heap Size=%zd\n", size);
		cudaDeviceSetLimit(cudaLimitMallocHeapSize,	1500000000*1);
		cudaDeviceGetLimit(&size, cudaLimitMallocHeapSize);
		printf("Heap Size=%zd\n", size);


		/*----------------*/
		/*	About	cuRAND	*/
		/*----------------*/
    cudaMalloc(	(void**)&devMTGPStates,	64	*	sizeof(curandStateMtgp32)	);

		/* Allocate space MTGP kernel parameters */
		cudaMalloc(	(void**)&devKernelParams, sizeof(mtgp32_kernel_params)	);

		/*	Reformat from predefined parameter sets to kernel format		*/
		/*	and copy kernel parameters to device memory									*/
		curandMakeMTGP32Constants(	mtgp32dc_params_fast_11213,	devKernelParams	);

		/*	Initialize one state per thread block	*/
    curandMakeMTGP32KernelState(devMTGPStates,	mtgp32dc_params_fast_11213,	devKernelParams, 64, genrand_int32()	);

		/*	Generate and use pseudo-random	*/
/*
		generate_kernel<<<1, 10>>>(devMTGPStates);
*/

		/*----------------*/
		/*	Initialize		*/
		/*----------------*/
		FILE		*ID;

		int			i,j,k,n,m,	TempI[10]	=	{0};
		double	TempD[50]	=	{0},	J,	*Q0,	*lb,	*ub;
		float		TempF[50]	=	{0};
		char		TempC[255],	FileName[255];

		WIN32_FIND_DATA		win32fd;
		HANDLE						hFind;
		BOOL							Check_H;

		Func_Point	Func_Hand;




		STAR_Para		=	(STAR_Parameter*)malloc(	sizeof(STAR_Parameter)	*	(1)	);
		cudaMalloc(	(void**)&STAR_Para_d,				sizeof(STAR_Parameter)	*	(1)	);


		STAR_Partic		=	(STAR_Particle*)malloc(	sizeof(STAR_Particle)	*	(1024)	);
		cudaMalloc(	(void**)&STAR_Partic_d,			sizeof(STAR_Particle)	*	(1024)	);


		Cam_Para		=	(Camera_Parameter*)malloc(	sizeof(Camera_Parameter)	*	(10)	);
		cudaMalloc(	(void**)&Cam_Para_d,					sizeof(Camera_Parameter)	*	(10)	);

		/*====================*/
		/*	Get BasePara			*/
		/*====================*/
		Get_BasePara(			);
		printf("%d \n",	1111);

/*
		Get_BasePara_GPU(	);
		printf("%d \n",	1111);
*/

		/*----------------*/
		/*	Check nValue	*/
		/*	メモリー確保	*/
		/*----------------*/
		ID	=	NULL;
		while	(ID==NULL)
		ID	=	fopen(	"./CUDA_Input/Q_Parameter.Dat",	"rb"	);

		i	=	0;
		while	(	1	)
		{
			fseek(ID,	sizeof(double)*i,	SEEK_SET);
			fread(	TempD,	sizeof(double),	1,	ID);

			if	(	TempD[0]	==	pow(10,10)	)
			{
				break;
			}
			i	=	i	+	1;
		}
		fclose(ID);

		/*	メモリー確保	*/
		Q0	=	(double*)malloc(	sizeof(double)	*	(	i	+	40	)	);
		lb	=	(double*)malloc(	sizeof(double)	*	(	i	+	40	)	);
		ub	=	(double*)malloc(	sizeof(double)	*	(	i	+	40	)	);
		TempI[9]	=	i;

		/*----------------------------------*/
		/*	Q_Parameter											*/
		/*	配列の最後に番兵(DBL_MAX)がいる	*/
		/*----------------------------------*/
		ID	=	NULL;
		while	(ID==NULL)
		ID	=	fopen(	"./CUDA_Input/Q_Parameter.Dat",	"rb"	);

		i					=	0;
		for (	j=0;	j<3;	j++	)
		{
			TempI[0]	=	0;
			while	(	1	)
			{
				fseek(ID,	sizeof(double)*i,	SEEK_SET);
				fread(	TempD,	sizeof(double),	1,	ID);

				if	(	TempD[0]	==	pow(10,10)	)
				{
					if	(j==0)
					{	Q0[	TempI[0]	]	=	FLT_MAX;	}
					else if (j==1)
					{	lb[	TempI[0]	]	=	FLT_MAX;	}
					else if (j==2)
					{	ub[	TempI[0]	]	=	FLT_MAX;	}
					i	=	i	+	1;
					break;
				}

				if	(j==0)
				{	Q0[	TempI[0]	]	=	TempD[0];	}
				else if (j==1)
				{	lb[	TempI[0]	]	=	TempD[0];	}
				else if (j==2)
				{	ub[	TempI[0]	]	=	TempD[0];	}

				
				TempI[0]				=	TempI[0]	+	1;
				i								=	i	+	1;
			}
		}
		fclose(ID);

/*
		for (	i=0;	i<TempI[9]+1;	i++	)
		{
			if	(	i	<	TempI[9]	)
			{	printf("%d %f %f %f %f\n",i,Q0[i],lb[i],ub[i],	(ub[i]+lb[i])/2	);	}
			else
			{	printf("%d %le %le %le\n",i,Q0[i],lb[i],ub[i]);	}
		}
		exit(1);
*/

		OptPara[0].nPartical		=	1024;



		ParticalSwarm_Initialize(	lb,	ub,	Q0	);
		printf("%d \n",	OptPara[0].nValue);


		ParticalSwarm_Main(	Func_Hand	);











		return(0);
	}



/*======================================================================================================*/
/*	InternalFunction																																										*/
/*======================================================================================================*/
	void	Get_BasePara(	void	)
	{

		double	TempD[30]	=	{0};
		double	S[3],	C[3],	Rm_1[9],	Rm_2[9],	Rm_3[9];
		float		TempF[10]	=	{0.0};

		cudaError_t		ErrorID;
		FILE		*ID1;
		int	i,j,k,n,iCam,iSean,iKnown,	iImage,		nCam,	nImage,	nKeyPoint,	ObjeFlag,	LoadStep={0},	TempI[10]	=	{0},	iV,	iU;


		
		unsigned char				*LabelA;
		LabelA	=	(unsigned char*)malloc(sizeof(unsigned char) * ((1080*1920)+100)	);


		int	nValue={0},	nKnownTarm={0},	nUnknownTarm={0},	nValueMat[100][2]={0},	nSean[100]={0},	nSean_Main[100]={0};
		int	iU_8,	iV_8,	iU_Lo,	iV_Lo;




		char	FileName[255],	PathName[255],	LoadName[255];
		char	TempC[5];

		FileName[0]='\0';
		PathName[0]='\0';
		LoadName[0]='\0';

		strcat(	PathName,	"./CUDA_Input/"	);

/*
		printf("%s \n",PathName	);
		exit(1);
*/

		/*==================*/
		/*==================*/
		/*	STAR Parameter	*/
		/*==================*/
		/*==================*/
		STAR_Para[0].nVertex			=	6890;
		STAR_Para[0].nTriangle		=	13776;
		STAR_Para[0].nTriangle_32	=	13792;

		/*================*/
		/*	LandMark			*/
		/*================*/

		//--LandMark01_Vertex
		FileName[0]='\0';
		strcat(	FileName,	"LandMark01_Vertex.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		i					=	0;
		TempI[0]	=	0;
		while	(	1	)
		{
			fseek(	ID1,		sizeof(int)*i,	SEEK_SET			);
			fread(	TempI,	sizeof(int),		1,				ID1	);

			j	=	i	/	3;
			k	=	i	%	3;

//			printf(" %d %d %d %d \n",	i,j,k,TempI[0]-1		);
			if	(	TempI[0]	==	10000	)
			{
				break;
			}

			if			(k==0)
			{
				STAR_Para[0].LandMark01_Vertex[j].x	=	TempI[0]	-	1;
			}
			else if	(k==1)
			{
				STAR_Para[0].LandMark01_Vertex[j].y	=	TempI[0]	-	1;
			}
			else if	(k==2)
			{
				STAR_Para[0].LandMark01_Vertex[j].z	=	TempI[0]	-	1;
			}

			i	=	i	+	1;
		}
		fclose(ID1);



		//--LandMark02_Vertex
		FileName[0]='\0';
		strcat(	FileName,	"LandMark02_Vertex.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		i					=	0;
		TempI[0]	=	0;
		while	(	1	)
		{
			fseek(	ID1,		sizeof(int)*i,	SEEK_SET			);
			fread(	TempI,	sizeof(int),		1,				ID1	);

			j	=	i	/	3;	k	=	i	%	3;

//			printf(" %d %d %d %d \n",	i,j,k,TempI[0]-1		);

			if	(	TempI[0]	==	10000	)
			{
				break;
			}

			if			(k==0)
			{
				STAR_Para[0].LandMark02_Vertex[j].x	=	TempI[0]	-	1;
			}
			else if	(k==1)
			{
				STAR_Para[0].LandMark02_Vertex[j].y	=	TempI[0]	-	1;
			}
			else if	(k==2)
			{
				STAR_Para[0].LandMark02_Vertex[j].z	=	TempI[0]	-	1;
			}

			i	=	i	+	1;
		}
		fclose(ID1);


		//--LandMark01_Weight
		FileName[0]='\0';
		strcat(	FileName,	"LandMark01_Weight.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		i					=	0;
		while	(	1	)
		{
			fseek(	ID1,		sizeof(float)*i,	SEEK_SET			);
			fread(	TempF,	sizeof(float),		1,				ID1	);

			j	=	i	/	3;
			k	=	i	%	3;

//			printf(" %d %d %d %f \n",	i,j,k,TempF[0]		);
			if	(	TempF[0]	==	pow(10,10)	)
			{
				break;
			}

			if			(k==0)
			{
				STAR_Para[0].LandMark01_Weight[j].x	=	TempF[0];
			}
			else if	(k==1)
			{
				STAR_Para[0].LandMark01_Weight[j].y	=	TempF[0];
			}
			else if	(k==2)
			{
				STAR_Para[0].LandMark01_Weight[j].z	=	TempF[0];
			}

			i	=	i	+	1;
		}
		fclose(ID1);



		//--LandMark02_Weight
		FileName[0]='\0';
		strcat(	FileName,	"LandMark02_Weight.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		i					=	0;
		while	(	1	)
		{
			fseek(	ID1,		sizeof(float)*i,	SEEK_SET			);
			fread(	TempF,	sizeof(float),		1,				ID1	);

			j	=	i	/	3;
			k	=	i	%	3;

//			printf(" %d %d %d %f \n",	i,j,k,TempF[0]		);
			if	(	TempF[0]	==	pow(10,10)	)
			{
				break;
			}

			if			(k==0)
			{
				STAR_Para[0].LandMark02_Weight[j].x	=	TempF[0];
			}
			else if	(k==1)
			{
				STAR_Para[0].LandMark02_Weight[j].y	=	TempF[0];
			}
			else if	(k==2)
			{
				STAR_Para[0].LandMark02_Weight[j].z	=	TempF[0];
			}

			i	=	i	+	1;
		}
		fclose(ID1);

//		for (	i=0;	i<17;	i++	)
//		{
//			printf(" %d %d %d %d %f %f %f \n",	i,
//												STAR_Para[0].LandMark01_Vertex[i].x,	STAR_Para[0].LandMark01_Vertex[i].y,	STAR_Para[0].LandMark01_Vertex[i].z,
//												STAR_Para[0].LandMark01_Weight[i].x,	STAR_Para[0].LandMark01_Weight[i].y,	STAR_Para[0].LandMark01_Weight[i].z
//						);
//		}


//		for (	i=0;	i<17;	i++	)
//		{
//			printf(" %d %d %d %d %f %f %f \n",	i,
//												STAR_Para[0].LandMark02_Vertex[i].x,	STAR_Para[0].LandMark02_Vertex[i].y,	STAR_Para[0].LandMark02_Vertex[i].z,
//												STAR_Para[0].LandMark02_Weight[i].x,	STAR_Para[0].LandMark02_Weight[i].y,	STAR_Para[0].LandMark02_Weight[i].z
//						);
//		}
//		exit(1);

		/*==================*/
		/*	Shape_Dire			*/
		/*==================*/
//		TempI[0]	=	(	STAR_Para[0].nVertex*3*300	);
//
//		FileName[0]='\0';
//		strcat(	FileName,	"Shape_Dirs.Dat"	);
//
//		LoadName[0]='\0';
//		strcat(	LoadName,	PathName	);
//		strcat(	LoadName,	FileName	);
//
//		ID1	=	NULL;
//		while	(ID1==NULL)
//		ID1	=	fopen(	LoadName,	"rb"	);
//
//		/*------------------*/
//		/*	データ読み取り	*/
//		/*------------------*/
//		fread(	STAR_Para[0].Shape_Dirs,	sizeof(float),	TempI[0],	ID1);
//		fclose(ID1);
//
//		for (	i=0;	i<STAR_Para[0].nVertex*3*75;	i++	)
//		{
//			STAR_Para[0].Shape_Dirs4[i].x	=	STAR_Para[0].Shape_Dirs[(i*4)+0];
//			STAR_Para[0].Shape_Dirs4[i].y	=	STAR_Para[0].Shape_Dirs[(i*4)+1];
//			STAR_Para[0].Shape_Dirs4[i].z	=	STAR_Para[0].Shape_Dirs[(i*4)+2];
//			STAR_Para[0].Shape_Dirs4[i].w	=	STAR_Para[0].Shape_Dirs[(i*4)+3];
//		}
//

//		for (	i=0;	i<100;	i++	)
//		{
//			printf(" %d %.15f %.15f \n",	i,	STAR_Para[0].Shape_Dirs[i],	STAR_Para[0].Shape_Dirs[TempI[0]-1-i]	);
//		}
//		exit(1);


		/*==================*/
		/*	Shape_Dire			*/
		/*==================*/
		TempI[0]	=	6201600;

		FileName[0]='\0';
		strcat(	FileName,	"Shape_DirsVer3.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		/*------------------*/
		/*	データ読み取り	*/
		/*------------------*/
		fread(	STAR_Para[0].Shape_Dirs,	sizeof(float),	TempI[0],	ID1	);
		fclose(ID1);

		for (	i=0;	i<(TempI[0]/4);	i++	)
		{
			STAR_Para[0].Shape_Dirs4[i].x	=	STAR_Para[0].Shape_Dirs[(i*4)+0];
			STAR_Para[0].Shape_Dirs4[i].y	=	STAR_Para[0].Shape_Dirs[(i*4)+1];
			STAR_Para[0].Shape_Dirs4[i].z	=	STAR_Para[0].Shape_Dirs[(i*4)+2];
			STAR_Para[0].Shape_Dirs4[i].w	=	STAR_Para[0].Shape_Dirs[(i*4)+3];
		}

//		for (	i=0;	i<25;	i++	)
//		{
//			printf(" %d %.15f %.15f \n",	i*4+0,	STAR_Para[0].Shape_Dirs4[i].x,	STAR_Para[0].Shape_Dirs4[(TempI[0]/4)-1-i].w	);
//			printf(" %d %.15f %.15f \n",	i*4+1,	STAR_Para[0].Shape_Dirs4[i].y,	STAR_Para[0].Shape_Dirs4[(TempI[0]/4)-1-i].z	);
//			printf(" %d %.15f %.15f \n",	i*4+2,	STAR_Para[0].Shape_Dirs4[i].z,	STAR_Para[0].Shape_Dirs4[(TempI[0]/4)-1-i].y	);
//			printf(" %d %.15f %.15f \n",	i*4+3,	STAR_Para[0].Shape_Dirs4[i].w,	STAR_Para[0].Shape_Dirs4[(TempI[0]/4)-1-i].x	);
//		}
//		exit(1);



		/*==================*/
		/*	Shape_Dire			*/
		/*==================*/
		TempI[0]	=	6201600;

		FileName[0]='\0';
		strcat(	FileName,	"Shape_DirsVer2.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		/*------------------*/
		/*	データ読み取り	*/
		/*------------------*/
		fread(	STAR_Para[0].Shape_Dirs,	sizeof(float),	TempI[0],	ID1	);
		fclose(ID1);

//		for (	i=0;	i<100;	i++	)
//		{
//			printf(" %d %.15f %.15f \n",	i,	STAR_Para[0].Shape_Dirs[i],	STAR_Para[0].Shape_Dirs[TempI[0]-1-i]	);
//		}
//		exit(1);


		/*==================*/
		/*	V_Temp					*/
		/*==================*/
		TempI[0]	=	(	STAR_Para[0].nVertex*3	);

		FileName[0]='\0';
		strcat(	FileName,	"V_Temp.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		/*------------------*/
		/*	データ読み取り	*/
		/*------------------*/
		fread(	STAR_Para[0].V_Temp,	sizeof(float),	TempI[0],	ID1);
		fclose(ID1);

/*
		for (	i=0;	i<100;	i++	)
		{
			printf(" %d %.15f %.15f \n",	i,	STAR_Para[0].V_Temp[i],	STAR_Para[0].V_Temp[TempI[0]-1-i]	);
		}
		exit(1);
*/

		/*==================*/
		/*	J_Regressor			*/
		/*==================*/
		TempI[0]	=	(	STAR_Para[0].nVertex*24	);

		FileName[0]='\0';
		strcat(	FileName,	"J_Regressor.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		/*------------------*/
		/*	データ読み取り	*/
		/*------------------*/
		fread(	STAR_Para[0].J_Regressor,	sizeof(float),	TempI[0],	ID1);
		fclose(ID1);




		/*====================*/
		/*	J_Regressor_Ver2	*/
		/*====================*/

		/*--Coeff--*/
		TempI[0]	=	(	260	);

		FileName[0]='\0';
		strcat(	FileName,	"J_RegressorVer2_Co.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		/*------------------*/
		/*	データ読み取り	*/
		/*------------------*/
		fread(	STAR_Para[0].J_Reg_Co,	sizeof(float),	TempI[0],	ID1);
		fclose(ID1);


		/*---------*/
		/*--Joint--*/
		/*---------*/
		TempI[0]	=	(	260	);

		FileName[0]='\0';
		strcat(	FileName,	"J_RegressorVer2_Joint.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		/*	データ読み取り	*/
		fread(	STAR_Para[0].J_Reg_Joint,	sizeof(int),	TempI[0],	ID1);
		fclose(ID1);



		/*---------*/
		/*--Joint--*/
		/*---------*/
		TempI[0]	=	(	260	);

		FileName[0]='\0';
		strcat(	FileName,	"J_RegressorVer2_Vertex.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		/*	データ読み取り	*/
		fread(	STAR_Para[0].J_Reg_Vertex,	sizeof(int),	TempI[0],	ID1);
		fclose(ID1);



//		for (	i=0;	i<260;	i++	)
//		{
//			printf(" %d %d %d %.15f \n",	i,	STAR_Para[0].J_Reg_Joint[i],	STAR_Para[0].J_Reg_Vertex[i],	STAR_Para[0].J_Reg_Co[i]	);
//		}
//		exit(1);

/*
		for (	i=0;	i<100;	i++	)
		{
			printf(" %d %.15f %.15f \n",	i,	STAR_Para[0].J_Regressor[i],	STAR_Para[0].J_Regressor[TempI[0]-1-i]	);
		}
		exit(1);
*/

		/*==================*/
		/*	Pose_Dirs				*/
		/*==================*/
		TempI[0]	=	(	STAR_Para[0].nVertex*3*93	);

		FileName[0]='\0';
		strcat(	FileName,	"Pose_Dirs.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		/*------------------*/
		/*	データ読み取り	*/
		/*------------------*/
		fread(	STAR_Para[0].Pose_Dirs,	sizeof(float),	TempI[0],	ID1);
		fclose(ID1);

		for (	i=0;	i<STAR_Para[0].nVertex*3*(93/3);	i++	)
		{
			STAR_Para[0].Pose_Dirs3[i].x	=	STAR_Para[0].Pose_Dirs[(i*3)+0];
			STAR_Para[0].Pose_Dirs3[i].y	=	STAR_Para[0].Pose_Dirs[(i*3)+1];
			STAR_Para[0].Pose_Dirs3[i].z	=	STAR_Para[0].Pose_Dirs[(i*3)+2];
		}

/*
		for (	i=0;	i<100;	i++	)
		{
			printf(" %d %.15f %.15f \n",	i,	STAR_Para[0].Pose_Dirs[i],	STAR_Para[0].Pose_Dirs[TempI[0]-1-i]	);
		}
		exit(1);
*/



		/*==================*/
		/*	Pose Dire_Ver2	*/
		/*==================*/
		/*--Coeff--*/
		TempI[0]	=	(	490602+1	);

		FileName[0]='\0';
		strcat(	FileName,	"Pose_DirsVer2_Co.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		/*------------------*/
		/*	データ読み取り	*/
		/*------------------*/
		fread(	STAR_Para[0].Pose_DirsVer2_Co,	sizeof(float),	TempI[0],	ID1);
		fclose(ID1);

//		for (	i=0;	i<100;	i++	)
//		{
//			printf(" %d %.15f %.15f \n",	i,	STAR_Para[0].Pose_DirsVer2_Co[i],	STAR_Para[0].Pose_DirsVer2_Co[TempI[0]-1-i]	);
//		}
//		exit(1);



		/*---------*/
		/*--Joint--*/
		/*---------*/
		TempI[0]	=	(	163534	+	1	);

		FileName[0]='\0';
		strcat(	FileName,	"Pose_DirsVer2_FeatITI.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		/*	データ読み取り	*/
		fread(	STAR_Para[0].Pose_DirsVer2_FeatITI,	sizeof(unsigned short),	TempI[0],	ID1);
		fclose(ID1);


		/*---------*/
		/*--Joint--*/
		/*---------*/
		TempI[0]	=	(	163534	+	1	);

		FileName[0]='\0';
		strcat(	FileName,	"Pose_DirsVer2_Vertex.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		/*	データ読み取り	*/
		fread(	STAR_Para[0].Pose_DirsVer2_Vertex,	sizeof(unsigned short),	TempI[0],	ID1);
		fclose(ID1);


//		for (	i=0;	i<100;	i++	)
//		{
//			printf(" %d %d %d %d %d \n",	i,	STAR_Para[0].Pose_DirsVer2_Vertex[i],							STAR_Para[0].Pose_DirsVer2_FeatITI[i],
//																				STAR_Para[0].Pose_DirsVer2_Vertex[TempI[0]-1-i],	STAR_Para[0].Pose_DirsVer2_FeatITI[TempI[0]-1-i]	);
//		}
//		exit(1);
//

		/*---------------*/
		/*--Start_nFeat--*/
		/*---------------*/
		TempI[0]	=	(	(6890*2)+1	);

		FileName[0]='\0';
		strcat(	FileName,	"Pose_DirsVer2_Start_nFeat.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		/*	データ読み取り	*/
		fread(	STAR_Para[0].Pose_DirsVer2_Start_nFeat,	sizeof(int),	TempI[0],	ID1);
		fclose(ID1);


//		for (	i=0;	i<100;	i++	)
//		{
//			printf(" %d %d %d %d %d \n",	i,	STAR_Para[0].Pose_DirsVer2_Start_nFeat[i*2+0],				STAR_Para[0].Pose_DirsVer2_Start_nFeat[i*2+1],
//																				STAR_Para[0].Pose_DirsVer2_Start_nFeat[(6890-i)*2-2],	STAR_Para[0].Pose_DirsVer2_Start_nFeat[(6890-i)*2-1]	);
//		}
//		printf(" END %d\n",	STAR_Para[0].Pose_DirsVer2_Start_nFeat[6890*2]	);
//		exit(1);
//

		/*==================*/
		/*	Pose Dire_Ver2	*/
		/*==================*/
		/*--Coeff--*/
		TempI[0]	=	(	18948+1	);

		FileName[0]='\0';
		strcat(	FileName,	"WeightVer2_Co.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		/*------------------*/
		/*	データ読み取り	*/
		/*------------------*/
		fread(	STAR_Para[0].WeightsVer2_Co,	sizeof(float),	TempI[0],	ID1);
		fclose(ID1);

//		for (	i=0;	i<100;	i++	)
//		{
//			printf(" %d %.15f %.15f \n",	i,	STAR_Para[0].WeightsVer2_Co[i],	STAR_Para[0].WeightsVer2_Co[TempI[0]-1-i]	);
//		}
//		exit(1);

		/*---------*/
		/*--Joint--*/
		/*---------*/
		TempI[0]	=	(	18948+1	);

		FileName[0]='\0';
		strcat(	FileName,	"WeightVer2_JointITI.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		/*	データ読み取り	*/
		fread(	STAR_Para[0].WeightsVer2_JointITI,	sizeof(unsigned short),	TempI[0],	ID1);
		fclose(ID1);

//		for (	i=0;	i<100;	i++	)
//		{
//			printf(" %d %d %d \n",	i,	STAR_Para[0].WeightsVer2_JointITI[i],	STAR_Para[0].WeightsVer2_JointITI[TempI[0]-1-i]	);
//		}
//		exit(1);


		/*---------------*/
		/*--Start_nFeat--*/
		/*---------------*/
		TempI[0]	=	(	(6890*2)+1	);

		FileName[0]='\0';
		strcat(	FileName,	"WeightVer2_Start_nFeat.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		/*	データ読み取り	*/
		fread(	STAR_Para[0].WeightsVer2_Start_nFeat,	sizeof(int),	TempI[0],	ID1);
		fclose(ID1);


//		for (	i=0;	i<100;	i++	)
//		{
//			printf(" %d %d %d %d %d \n",	i,	STAR_Para[0].WeightsVer2_Start_nFeat[i*2+0],				STAR_Para[0].WeightsVer2_Start_nFeat[i*2+1],
//																				STAR_Para[0].WeightsVer2_Start_nFeat[(6890-i)*2-2],	STAR_Para[0].WeightsVer2_Start_nFeat[(6890-i)*2-1]	);
//		}
//		printf(" END %d\n",	STAR_Para[0].WeightsVer2_Start_nFeat[6890*2]	);
//		exit(1);

		/*==================*/
		/*	Weights					*/
		/*==================*/
		TempI[0]	=	(	STAR_Para[0].nVertex*24	);

		FileName[0]='\0';
		strcat(	FileName,	"Weights.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		/*------------------*/
		/*	データ読み取り	*/
		/*------------------*/
		fread(	STAR_Para[0].Weights,	sizeof(float),	TempI[0],	ID1);
		fclose(ID1);

		for (	i=0;	i<STAR_Para[0].nVertex*6;	i++	)
		{
			STAR_Para[0].Weights4[i].x	=	STAR_Para[0].Weights[(i*4)+0];
			STAR_Para[0].Weights4[i].y	=	STAR_Para[0].Weights[(i*4)+1];
			STAR_Para[0].Weights4[i].z	=	STAR_Para[0].Weights[(i*4)+2];
			STAR_Para[0].Weights4[i].w	=	STAR_Para[0].Weights[(i*4)+3];
		}


/*
		for (	i=0;	i<100;	i++	)
		{
			printf(" %d %.15f %.15f \n",	i,	STAR_Para[0].Weights[i],	STAR_Para[0].Weights[TempI[0]-1-i]	);
		}
		exit(1);
*/

		/*==================*/
		/*	V_Point					*/
		/*==================*/
		FileName[0]='\0';
		strcat(	FileName,	"V_Point.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		i					=	0;
		TempI[0]	=	0;
		while	(	1	)
		{
			fseek(	ID1,		sizeof(unsigned short int)*i,	SEEK_SET			);
			fread(	TempI,	sizeof(unsigned short int),		1,				ID1	);

			j	=	i	/	3;
			k	=	i	%	3;

			if	(	TempI[0]	==	20000	)
			{
				break;
			}

			if			(k==0)
			{
				STAR_Para[0].V_Point3[j].x	=	TempI[0];
			}
			else if	(k==1)
			{
				STAR_Para[0].V_Point3[j].y	=	TempI[0];
			}
			else if	(k==2)
			{
				STAR_Para[0].V_Point3[j].z	=	TempI[0];
			}

			i	=	i	+	1;
		}
		fclose(ID1);

//		printf(" %d %d \n",	i,j	);
//		exit(1);

/*
		for (	i=0;	i<100;	i++	)
		{
			printf(" %d %d %d %d %d %d \n",	STAR_Para[0].V_Point[i*3+0],	STAR_Para[0].V_Point[i*3+1],	STAR_Para[0].V_Point[i*3+2],
																			STAR_Para[0].V_Point3[i].x,		STAR_Para[0].V_Point3[i].y,		STAR_Para[0].V_Point3[i].z	);
		}
		exit(1);
*/

	/*==================*/
		/*	NewV_Point32		*/
		/*==================*/
		FileName[0]='\0';
		strcat(	FileName,	"V_Point_32.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		i					=	0;
		TempI[0]	=	0;
		while	(	1	)
		{
			fseek(	ID1,		sizeof(unsigned short int)*i,	SEEK_SET			);
			fread(	TempI,	sizeof(unsigned short int),		1,				ID1	);

			j	=	i	/	3;
			k	=	i	%	3;

			if	(	TempI[0]	==	20000	)
			{
				break;
			}

			if			(k==0)
			{
				STAR_Para[0].NewV_Point_32[j].x	=	TempI[0];
			}
			else if	(k==1)
			{
				STAR_Para[0].NewV_Point_32[j].y	=	TempI[0];
			}
			else if	(k==2)
			{
				STAR_Para[0].NewV_Point_32[j].z	=	TempI[0];
			}

			i	=	i	+	1;
		}
		fclose(ID1);

//		for (	i=0;	i<32;	i++	)
//		{
//			printf(" %d %d %d %d \n",	i,	STAR_Para[0].NewV_Point_32[i].x,	STAR_Para[0].NewV_Point_32[i].y,	STAR_Para[0].NewV_Point_32[i].z	);
//		}

//		for (	i=STAR_Para[0].nTriangle_32 - 32;	i<STAR_Para[0].nTriangle_32;	i++	)
//		{
//			printf(" %d %d %d %d \n",	i,	STAR_Para[0].NewV_Point_32[i].x,	STAR_Para[0].NewV_Point_32[i].y,	STAR_Para[0].NewV_Point_32[i].z	);
//		}
//		exit(1);

		/*==================*/
		/*	Kintree_Table		*/
		/*==================*/
		TempI[0]	=	(	24*2	);

		FileName[0]='\0';
		strcat(	FileName,	"Kintree_Table.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		/*------------------*/
		/*	データ読み取り	*/
		/*------------------*/
		fread(	STAR_Para[0].Kintree_Table,	sizeof(unsigned short int),	TempI[0],	ID1);
		fclose(ID1);

/*
		for (	i=0;	i<48;	i++	)
		{
			printf(" %d %d \n",	i,	STAR_Para[0].Kintree_Table[i]	);
		}
		exit(1);
*/


		/*==============*/
		/*	BasePara		*/
		/*==============*/
		FileName[0]='\0';
		strcat(	FileName,	"BasePara_Int.Dat"	);

		LoadName[0]='\0';
		strcat(	LoadName,	PathName	);
		strcat(	LoadName,	FileName	);

		ID1	=	NULL;
		while	(ID1==NULL)
		ID1	=	fopen(	LoadName,	"rb"	);

		/*------------------*/
		/*	データ読み取り	*/
		/*------------------*/
		fread(	&nCam,			sizeof(int),	1,	ID1);
		fread(	&nImage,		sizeof(int),	1,	ID1);
		fread(	&nKeyPoint,	sizeof(int),	1,	ID1);
		fread(	&ObjeFlag,	sizeof(int),	1,	ID1);

		for (	iCam=0;	iCam<nCam;	iCam++	)
		{
			Cam_Para[iCam].nCam				=	nCam;
			Cam_Para[iCam].nImage			=	nImage;
			Cam_Para[iCam].nKeyPoint	=	nKeyPoint;
			Cam_Para[iCam].ObjeFlag		=	ObjeFlag;

			fread(	TempI,		sizeof(int),	2,	ID1);

			Cam_Para[iCam].Pix_Tate	=	TempI[0];
			Cam_Para[iCam].Pix_Yoko	=	TempI[1];

//			printf("%d %d %d %d %d %d \n",	Cam_Para[iCam].nCam,	Cam_Para[iCam].nImage,	Cam_Para[iCam].nKeyPoint,	Cam_Para[iCam].ObjeFlag,	Cam_Para[iCam].Pix_Tate,	Cam_Para[iCam].Pix_Yoko	);
		}
		fclose(ID1);


		/*====================*/
		/*	Camera Parameter	*/
		/*====================*/
		for (	iCam=0;	iCam<Cam_Para[0].nCam;	iCam++	)
		{

			Cam_Para[iCam].Pix_Yoko_8	=	Cam_Para[iCam].Pix_Yoko/8;
			Cam_Para[iCam].Pix_Tate_8	=	Cam_Para[iCam].Pix_Tate/8;

			Cam_Para[iCam].OriUo		=	(Cam_Para[iCam].Pix_Yoko+1)/2.00;
			Cam_Para[iCam].OriVo		=	(Cam_Para[iCam].Pix_Tate+1)/2.00;

			/*	CameraParameter	*/
			FileName[0]='\0';
			strcat(	FileName,	"CamPara_Cam0.Dat"	);

			sprintf(TempC,	"%d",	iCam+1	);

			FileName[11]	=	TempC[0];
/*
			printf(" %s \n",	FileName	);
*/
			LoadName[0]='\0';
			strcat(	LoadName,	PathName	);
			strcat(	LoadName,	FileName	);

			ID1	=	NULL;
			while	(ID1==NULL)
			ID1	=	fopen(	LoadName,	"rb"	);

			/*------------------*/
			/*	データ読み取り	*/
			/*------------------*/
			fread(	Cam_Para[iCam].CamPara,	sizeof(double),	7,	ID1);
			fclose(ID1);

			/*	Focal	0		*/
			Cam_Para[iCam].F[0]	=	Cam_Para[iCam].CamPara[0];

			/*	CL		1:3	*/
			Cam_Para[iCam].CL[0]	=	Cam_Para[iCam].CamPara[1];
			Cam_Para[iCam].CL[1]	=	Cam_Para[iCam].CamPara[2];
			Cam_Para[iCam].CL[2]	=	Cam_Para[iCam].CamPara[3];

			/*	Angle	4:6	*/
			S[0]	=	sin(	Cam_Para[iCam].CamPara[4]	);
			S[1]	=	sin(	Cam_Para[iCam].CamPara[5]	);
			S[2]	=	sin(	Cam_Para[iCam].CamPara[6]	);

			C[0]	=	cos(	Cam_Para[iCam].CamPara[4]	);
			C[1]	=	cos(	Cam_Para[iCam].CamPara[5]	);
			C[2]	=	cos(	Cam_Para[iCam].CamPara[6]	);

			/*	JRm_1	*/
			Rm_1[0]	=	C[1];		Rm_1[1]	=	S[1];		Rm_1[2]	=	0;
			Rm_1[3]	=	-S[1];	Rm_1[4]	=	C[1];		Rm_1[5]	=	0;
			Rm_1[6]	=	0;			Rm_1[7]	=	0;			Rm_1[8]	=	1;

			/*	JRm_2	*/
			Rm_2[0]	=	1;			Rm_2[1]	=	0;			Rm_2[2]	=	0;
			Rm_2[3]	=	0;			Rm_2[4]	=	C[0];		Rm_2[5]	=	S[0];
			Rm_2[6]	=	0;			Rm_2[7]	=	-S[0];	Rm_2[8]	=	C[0];

			for (	i=0;	i	<	3;	i++	)
			{
				for (	j=0;	j	<	3;	j++	)
				{
					k	=	(i*3)	+	j;
					Rm_3[k]	=
						(	Rm_1[j]		*	Rm_2[(i*3)+0]	)
					+	(	Rm_1[j+3]	*	Rm_2[(i*3)+1]	)
					+	(	Rm_1[j+6]	*	Rm_2[(i*3)+2]	);
				}
			}

			/*	JRm_3	*/
			Rm_1[0]	=	C[2];		Rm_1[1]	=	0;	Rm_1[2]	=	-S[2];
			Rm_1[3]	=	0;			Rm_1[4]	=	1;	Rm_1[5]	=	0;
			Rm_1[6]	=	S[2];		Rm_1[7]	=	0;	Rm_1[8]	=	C[2];

			for (	i=0;	i	<	3;	i++	)
			{
				for (	j=0;	j	<	3;	j++	)
				{
					k	=	(i*3)	+	j;
					Rm_2[k]	=
						(	Rm_3[j]		*	Rm_1[(i*3)+0]	)
					+	(	Rm_3[j+3]	*	Rm_1[(i*3)+1]	)
					+	(	Rm_3[j+6]	*	Rm_1[(i*3)+2]	);
				}
			}

			Cam_Para[iCam].Rm[0]	=	-Rm_2[0];			Cam_Para[iCam].Rm[1]	=	-Rm_2[1];			Cam_Para[iCam].Rm[2]	=	-Rm_2[2];
			Cam_Para[iCam].Rm[3]	=	Rm_2[6];			Cam_Para[iCam].Rm[4]	=	Rm_2[7];			Cam_Para[iCam].Rm[5]	=	Rm_2[8];
			Cam_Para[iCam].Rm[6]	=	Rm_2[3];			Cam_Para[iCam].Rm[7]	=	Rm_2[4];			Cam_Para[iCam].Rm[8]	=	Rm_2[5];
		}

		/*======================*/
		/*	Label	and KeyPoint	*/
		/*======================*/
		for (	iCam=0;	iCam<Cam_Para[0].nCam;	iCam++	)
		{
			for (	iImage=0;	iImage<Cam_Para[0].nImage;	iImage++	)
			{

				/*	Label	*/
				FileName[0]='\0';
				strcat(	FileName,	"Label_Cam0_Image000.Dat"	);

				/*	iCam	*/
				sprintf(TempC,	"%d",	iCam+1	);
				FileName[9]	=	TempC[0];

				/*	iImage	*/
				sprintf(TempC,	"%03d",	iImage+1	);
				FileName[16]	=	TempC[0];
				FileName[17]	=	TempC[1];
				FileName[18]	=	TempC[2];

				LoadName[0]='\0';
				strcat(	LoadName,	PathName	);
				strcat(	LoadName,	FileName	);

				ID1	=	NULL;
				while	(ID1==NULL)
				ID1	=	fopen(	LoadName,	"rb"	);

				/*------------------*/
				/*	データ読み取り	*/
				/*------------------*/
				fread(	LabelA,	sizeof(unsigned char),	1080*1920,	ID1);
				fclose(ID1);

				//	Initialize
				for (	i=0;	i<(1080*1920/64);	i++	)
				{
					Cam_Para[iCam].Label[iImage][i]	=	0;
				}

				for (	iU=0;	iU<1920;	iU++	)
				{

					for (	iV=0;	iV<1080;	iV++	)
					{

						iU_8	=	iU/8;	iV_8	=	iV/8;
						iU_Lo	=	iU%8;	iV_Lo	=	iV%8;


						j	=	(iV_8)	+	(	iU_8*135	);
						k	=	(iV_Lo)	+	(	iU_Lo*8		);

						i	=	(iV)	+	(	(iU)*1080	);

						TempI[0]	=	(	i	/	BitSize	);
						TempI[1]	=	(	i	%	BitSize	);
						if	(	LabelA[i]	==	1	)
						{
							Cam_Para[iCam].Label[iImage][	j	]	=	Cam_Para[iCam].Label[iImage][	j	]	|	BitList[	k	];
						}
					}
				}

				//	Cam_Para[iCam].MinMax_All[iImage][0]	=	MaxU_All;
				//	Cam_Para[iCam].MinMax_All[iImage][1]	=	MinU_All;
				//	Cam_Para[iCam].MinMax_All[iImage][2]	=	MaxV_All;
				//	Cam_Para[iCam].MinMax_All[iImage][3]	=	MinV_All;
				//--MaxU_All
				for (	i=0;	i<(240*135);	i++	)
				{
					iU_8	=	239-(i/135);		iV_8	=	i%135;
					j			=	(iU_8*135)	+	iV_8;
					if	(	Cam_Para[iCam].Label[iImage][j]	!=	0)
					{
						Cam_Para[iCam].MinMax_All[iImage][0]	=	iU_8;
						break;
					}
				}

				//--MinU_All
				for (	i=0;	i<(240*135);	i++	)
				{
					iU_8	=	i/135;		iV_8	=	i%135;
					j			=	(iU_8*135)	+	iV_8;
					if	(	Cam_Para[iCam].Label[iImage][j]	!=	0)
					{
						Cam_Para[iCam].MinMax_All[iImage][1]	=	iU_8;
						break;
					}
				}

				//--MaxV_All
				for (	i=0;	i<(240*135);	i++	)
				{
					iU_8	=	i%240;		iV_8	=	134-(i/240);
					j			=	(iU_8*135)	+	iV_8;
					if	(	Cam_Para[iCam].Label[iImage][j]	!=	0)
					{
						Cam_Para[iCam].MinMax_All[iImage][2]	=	iV_8;
						break;
					}
				}

				//--MinV_All
				for (	i=0;	i<(240*135);	i++	)
				{
					iU_8	=	i%240;		iV_8	=	i/240;
					j			=	(iU_8*135)	+	iV_8;
					if	(	Cam_Para[iCam].Label[iImage][j]	!=	0)
					{
						Cam_Para[iCam].MinMax_All[iImage][3]	=	iV_8;
						break;
					}
				}

				/*------------------*/
				/*	データ読み取り	*/
				/*------------------*/
				FileName[0]='\0';
				strcat(	FileName,	"Keypoint_Cam0_Image000.Dat"	);

				/*	iCam	*/
				sprintf(TempC,	"%d",	iCam+1	);
				FileName[12]	=	TempC[0];

				/*	iImage	*/
				sprintf(TempC,	"%03d",	iImage+1	);
				FileName[19]	=	TempC[0];
				FileName[20]	=	TempC[1];
				FileName[21]	=	TempC[2];

				LoadName[0]='\0';
				strcat(	LoadName,	PathName	);
				strcat(	LoadName,	FileName	);

				ID1	=	NULL;
				while	(ID1==NULL)
				ID1	=	fopen(	LoadName,	"rb"	);

				/*------------------*/
				/*	データ読み取り	*/
				/*------------------*/
				fread(	Cam_Para[iCam].Keypoint[iImage],	sizeof(float),	35,	ID1);
				fclose(ID1);

//				for (	i=0;	i<17;	i++	)
//				{
//					printf("%d %d %d %f %f \n",	iCam,	iImage,	i,	Cam_Para[iCam].Keypoint[iImage][i*2+0],	Cam_Para[iCam].Keypoint[iImage][i*2+1]	);
//				}
//				printf("End %f \n",	Cam_Para[iCam].Keypoint[iImage][34]	);
			}
		}
//		exit(1);

//		for (	iCam=0;	iCam<8;	iCam++	)
//		{
//			for (	iImage=0;	iImage<Cam_Para[0].nImage;	iImage++	)
//			{
//
//				printf("%d %d %d %d %d %d \n",	iCam,	iImage,	Cam_Para[iCam].MinMax_All[iImage][0],	
//																											Cam_Para[iCam].MinMax_All[iImage][1],
//																											Cam_Para[iCam].MinMax_All[iImage][2],
//																											Cam_Para[iCam].MinMax_All[iImage][3]	);
//			}
//		}
//		exit(1);
//

//		for (	iCam=0;	iCam<nCam;	iCam++	)
//		{
//			for (	iImage=0;	iImage<Cam_Para[0].nImage;	iImage++	)
//			{
//				for (	i=0;	i<17;	i++	)
//				{
//					printf("%d %d %d %f %f \n",	iCam,	iImage,	i,	Cam_Para[iCam].Keypoint[iImage][i*2+0],	Cam_Para[iCam].Keypoint[iImage][i*2+1]	);
//				}
//				printf("End %f \n",	Cam_Para[iCam].Keypoint[iImage][34]	);
//			}
//		}
//		exit(1);




		/*------------------*/
		/*	GPUMemory copy	*/
		/*------------------*/
		/*	STAR Parameter	*/
		ErrorID	=	cudaMemcpy(	STAR_Para_d,	STAR_Para,	sizeof(STAR_Parameter)	*	1,	cudaMemcpyHostToDevice);
		if	(ErrorID	!=	cudaSuccess)
		{
			printf(	"\n   CudaMemoryCopy Error!!! 001"	);
			exit(1);
		}

		/*	Camera Parameter	*/
		ErrorID	=	cudaMemcpy(	Cam_Para_d,	Cam_Para,	sizeof(Camera_Parameter)	*	Cam_Para[0].nCam,	cudaMemcpyHostToDevice);
		if	(ErrorID	!=	cudaSuccess)
		{
			printf(	"\n   CudaMemoryCopy Error!!! 001"	);
			exit(1);
		}

		return;	
	}



