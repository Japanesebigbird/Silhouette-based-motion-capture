/*======================================================================================================*/
/*	InternalFunction																																										*/
/*	Initialize																																													*/
/*======================================================================================================*/
	void	ParticalSwarm_Initialize(	double	lb[],	double	ub[],	double	Q0[]	)
	{

		int	i,j,k;
		cudaMalloc(	(void**)&OptPara_d,	sizeof(OptP)	*	1	);

		i	=	0;
		while	(	lb[i]	!=	FLT_MAX	)
		i	=	i+1;

		OptPara[0].nValue			=	i;
		OptPara[0].nLocal			=	(int)(	0.01*OptPara[0].nPartical	);
		OptPara[0].Still			=	0;
		OptPara[0].Still02		=	0;


		if	(	OptPara[0].nLocal	<50	)
		OptPara[0].nLocal	=	50;

		i	=	OptPara[0].nValue+10;
		j	=	(	OptPara[0].nPartical	*	OptPara[0].nValue	)	+	10;
		k	=	(	OptPara[0].nPartical											)	+	10;

		/*	Host Memory	*/
		mb	=	(double*)malloc(	sizeof(double)	*	i	);
		rb	=	(double*)malloc(	sizeof(double)	*	i	);

		Part_Dis			=	(double*)malloc(	sizeof(double)	*	j	);
		Part_Dis_PB		=	(double*)malloc(	sizeof(double)	*	j	);
		Part_Vel			=	(double*)malloc(	sizeof(double)	*	j	);
		Part_PB				=	(double*)malloc(	sizeof(double)	*	k	);
		Part_Obje			=	(double*)malloc(	sizeof(double)	*	k	);
		Part_Rank			=	(int*)malloc(		sizeof(int)		*	k	);

		Part_NewDis		=	(double*)malloc(	sizeof(double)	*	j	);
		Part_NewVel		=	(double*)malloc(	sizeof(double)	*	j	);

		/*	Device Memory	*/
		cudaMalloc(	(void**)&mb_d,					(	sizeof(double)	*	i	)	);
		cudaMalloc(	(void**)&rb_d,					(	sizeof(double)	*	i	)	);

		cudaMalloc(	(void**)&Part_Dis_d,		(	sizeof(double)	*	j	)	);
		cudaMalloc(	(void**)&Part_Dis_PB_d,	(	sizeof(double)	*	j	)	);
		cudaMalloc(	(void**)&Part_Vel_d,		(	sizeof(double)	*	j	)	);
		cudaMalloc(	(void**)&Part_PB_d,			(	sizeof(double)	*	k	)	);
		cudaMalloc(	(void**)&Part_Obje_d,		(	sizeof(double)	*	k	)	);
		cudaMalloc(	(void**)&Part_Rank_d,		(	sizeof(double)	*	k	)	);


		cudaMalloc(	(void**)&Part_NewDis_d,	(	sizeof(double)	*	j	)	);
		cudaMalloc(	(void**)&Part_NewVel_d,	(	sizeof(double)	*	j	)	);


		/*	Search Range	*/
		for (	i=0;	i<(OptPara[0].nValue);	i++	)
		{
			mb[i]	=	(ub[i]+lb[i])	/	2;
			rb[i]	=	(ub[i]-mb[i]);
		}

		i	=	OptPara[0].nValue;
		ErrorID	=	cudaMemcpy(	mb_d,	mb,	sizeof(double)	*	i,	cudaMemcpyHostToDevice);
		if	(ErrorID	!=	cudaSuccess)
		{
			printf(	"\n   CudaMemoryCopy Error!!! 001"	);
			exit(1);
		}

		ErrorID	=	cudaMemcpy(	rb_d,	rb,	sizeof(double)	*	i,	cudaMemcpyHostToDevice);
		if	(ErrorID	!=	cudaSuccess)
		{
			printf(	"\n   CudaMemoryCopy Error!!! 001"	);
			exit(1);
		}

		/*--------------*/
		/*	Initialied	*/
		/*--------------*/
		for (	i=0;	i<OptPara[0].nPartical*OptPara[0].nValue;	i++	)
		{
			Part_Dis[i]			=	(	0.5	-	Randf()	)	*	2.0;
			if	(	Part_Dis[i]	<	-0.9999999	)
			{	Part_Dis[i]	=	-0.9999999;	}

			if	(	Part_Dis[i]	>	0.9999999	)
			{	Part_Dis[i]	=	0.9999999;	}

			Part_Dis_PB[i]	=	Part_Dis[i];
			Part_Vel[i]			=	(	0.5	-	Randf()	)	*	0.5;

			if	(	Part_Vel[i]	<	-0.9999999	)
			{	Part_Vel[i]	=	-0.9999999;	}

			if	(	Part_Vel[i]	>	0.9999999	)
			{	Part_Vel[i]	=	0.9999999;	}
		}

		for (	i=0;	i<OptPara[0].nValue;	i++	)
		{
			Part_Dis[i]			=	(	Q0[i]-mb[i]	)	/	rb[i];

			if	(	Part_Dis[i]	<	-0.9999999	)
			{	Part_Dis[i]	=	-0.9999999;	}

			if	(	Part_Dis[i]	>	0.9999999	)
			{	Part_Dis[i]	=	0.9999999;	}


			Part_Dis_PB[i]	=	Part_Dis[i];
			Part_Vel[i]			=	0.00;

			if	(	Part_Vel[i]	<	-0.9999999	)
			{	Part_Vel[i]	=	-0.9999999;	}

			if	(	Part_Vel[i]	>	0.9999999	)
			{	Part_Vel[i]	=	0.9999999;	}


//			printf(	"%d %f %f %f %f \n",i,Part_Dis[i],	Q0[i],ub[i],lb[i]		);
		}
//		exit(1);


		for (	i=0;	i<OptPara[0].nPartical;	i++	)
		{
			Part_PB[i]	=	FLT_MAX;
		}


/*
		for (	i=0;	i<OptPara[0].nValue;	i++	)
		{
			Part_Dis[i]			=	0;
			Part_Dis_PB[i]	=	Part_Dis[i];
		}
*/

		/*	Dis,Vel Host to Devece			*/
		i	=	OptPara[0].nPartical*OptPara[0].nValue;
		ErrorID	=	cudaMemcpy(	Part_Dis_d,	Part_Dis,					sizeof(double)	*	i,	cudaMemcpyHostToDevice);
		if	(ErrorID	!=	cudaSuccess)
		{
			printf(	"\n   CudaMemoryCopy Error!!! 001"	);
			exit(1);
		}

		ErrorID	=	cudaMemcpy(	Part_Dis_PB_d,	Part_Dis_PB,	sizeof(double)	*	i,	cudaMemcpyHostToDevice);
		if	(ErrorID	!=	cudaSuccess)
		{
			printf(	"\n   CudaMemoryCopy Error!!! 001"	);
			exit(1);
		}

		ErrorID	=	cudaMemcpy(	Part_Vel_d,	Part_Vel,					sizeof(double)	*	i,	cudaMemcpyHostToDevice);
		if	(ErrorID	!=	cudaSuccess)
		{
			printf(	"\n   CudaMemoryCopy Error!!! 001"	);
			exit(1);
		}

		/*	Personal Best Host to Devece			*/
		i	=	OptPara[0].nPartical;
		ErrorID	=	cudaMemcpy(	Part_PB_d,	Part_PB,					sizeof(double)	*	i,	cudaMemcpyHostToDevice);
		if	(ErrorID	!=	cudaSuccess)
		{
			printf(	"\n   CudaMemoryCopy Error!!! 001"	);
			exit(1);
		}

		ErrorID	=	cudaMemcpy(	OptPara_d,	OptPara,	sizeof(OptP)	*	1,	cudaMemcpyHostToDevice);
		if	(ErrorID	!=	cudaSuccess)
		{
			printf(	"\n   CudaMemoryCopy Error!!! 001"	);
			exit(1);
		}
		return;
	}


/*======================================================================================================*/
/*	InternalFunction																																										*/
/*	Initialize																																													*/
/*======================================================================================================*/

/*====================*/
/*	Child Function1		*/
/*	Next Displacement	*/
/*====================*/
	__global__
	void	Next_Displacement(	OptP		OptPara[],
														double	Part_Obje[],			int		Part_Rank[],
														double	Part_Dis[],				double	Part_Vel[],
														double	Part_NewDis[],		double	Part_NewVel[],
														double	Part_Dis_PB[],
														curandStateMtgp32 *state,	int	nPick	)
	{
		int	iThre;
		int	i,j,	TempI[5],	StateFlag[2],	FollowBird,	MyNumber,	GBestNumber;
		double	TempD[20],	FL,	Vec1,	Vec2,	Vec3;

		iThre	=	(blockIdx.x	*	blockDim.x)	+	threadIdx.x;

		/*---------*/
		/* Ranking */
		/*---------*/
		TempI[0]	=	0;
		for (	i=0;	i	<	OptPara[0].nPartical;	i++	)
		{
			TempI[1]	=	(	Part_Obje[iThre]	>	Part_Obje[	i	]	);
			if	(	TempI[1]	==	1	)
			{
				TempI[0]	=	TempI[0]+1;
			}
		}
		Part_Rank[	iThre	]	=	TempI[0];

		/*--------------------*/
		/* Define Grobal Best */
		/*--------------------*/
		GBestNumber	=	(	curand(&state[blockIdx.x]	)	%	(	OptPara[0].nPartical	)	);
		TempD[0]		=	Part_Rank[	GBestNumber	];
		for (	i=1;	i	<	nPick;	i++	)
		{
			TempI[0]	=	(	curand(&state[blockIdx.x]	)	%	(	OptPara[0].nPartical	)	);

			if	(	iThre	!=	TempI[0]	)
			{
				if	(	Part_Rank[	TempI[0]	]	<	TempD[0]	)
				{
					TempD[0]	=	Part_Rank[	TempI[0]	];
					GBestNumber	=	TempI[0];
				}
			}
		}

		/*--LocalBest Particle Number--*/
		GBestNumber	=	GBestNumber	*	OptPara[0].nValue;

		/*--Current Particle Number--*/
		MyNumber		=	(	iThre	*	OptPara[0].nValue	);

		/*--FollowBird Particle Number--*/
		FollowBird	=	(	curand(&state[blockIdx.x]	)	%	(	OptPara[0].nPartical	)	);
		FollowBird	=	(	FollowBird	*	OptPara[0].nValue	);

		/*----------------*/
		/*	Flight Length	*/
		/*----------------*/
		FL	=	0.5	+	(	curand_uniform_double(&state[blockIdx.x])	*	9.5);
		FL	=	(	FL	*	curand_uniform_double(	&state[blockIdx.x]	)	);

		/*--------------------*/
		/*	Local Best Coeff	*/
		/*--------------------*/
		TempD[5]	=	(	0.00	+	(	curand_uniform_double(&state[blockIdx.x])	*	(1.2)		)	);

		/*----------------------*/
		/*	Parsonal Best Coeff	*/
		/*----------------------*/
		TempD[6]	=	(	0.00	+	(	curand_uniform_double(&state[blockIdx.x])	*	(1.2)		)	);

		/*----------------*/
		/*	Inertia Coeff	*/
		/*----------------*/
		TempD[7]	=	(	0.10	+	(	curand_uniform_double(&state[blockIdx.x])	*	(0.85)	)	);

		/*----------------*/
		/*	ë¨ìxêßå¿óvëf	*/
		/*----------------*/
		TempD[8]	=	1.80;

		TempD[0]			=	curand_uniform_double(&state[blockIdx.x]);
		TempD[1]			=	0.1	+	(curand_uniform_double(&state[blockIdx.x])*0.5);
		StateFlag[0]	=	(	TempD[1]	<		TempD[0]	);
		StateFlag[1]	=	1	-	StateFlag[0];

		/*----------------------*/
		/*	Next Dispracement		*/
		/*----------------------*/
		for (	i=0;	i	<	OptPara[0].nValue;	i++	)
		{
			TempD[10]	=	Part_Dis[			GBestNumber	+	i	];

			TempD[11]	=	Part_Dis[			MyNumber		+	i	];
			TempD[12]	=	Part_Dis_PB[	MyNumber		+	i	];
			TempD[13]	=	Part_Vel[			MyNumber		+	i	];

			TempD[14]	=	Part_Dis_PB[	FollowBird	+	i	];

			/*	Vector From Current Particle To Local Best	*/
			Vec1			=	TempD[5]	*	(	TempD[10]	-	TempD[11]	);

			/*	Vector From Current Particle To Personal Best	OR	Vector From Current Particle To Hiding Position	*/
			Vec2			=	(	StateFlag[0]	*	(	TempD[6]	*	(	TempD[12]	-	TempD[11]	)	)	)	+	(	StateFlag[1]	*	(	TempD[6]	*	(	TempD[14]	-	TempD[11]	)	)	);

			/*	Velocity	*/
			Vec3			=	TempD[7]	*	TempD[13];

			/*	New Velocity	*/
			TempD[0]	=	(	Vec1	+	Vec2	+	Vec3	);

			if	(	TempD[0]	<	(-1*TempD[8])	)
			{	TempD[0]	=	-1*TempD[8];	}

			if	(	TempD[0]	>	(TempD[8])	)
			{	TempD[0]	=	TempD[8];		}


			TempD[0]	=	(	StateFlag[0]	*	TempD[0]	)	+	(	StateFlag[1]	*	(	FL	*	(	TempD[14]-TempD[11]	)	)	);

			/*	New Displacement	*/
			TempD[1]	=	TempD[11]	+	TempD[0];

			if	(	TempD[1]	<	-0.9999999	)
			{
				TempD[1]	=	-0.9999999;
				TempD[0]	=	0;
			}

			if	(	TempD[1]	>	0.9999999	)
			{
				TempD[1]	=	0.9999999;
				TempD[0]	=	0;
			}

			Part_NewDis[	MyNumber+i	]	=	TempD[1];
			Part_NewVel[	MyNumber+i	]	=	TempD[0];

		}

		return;
	}

/*==================*/
/*	Child Function2	*/
/*	GA							*/
/*==================*/
	__global__
	void	Next_Displacement_GA(OptP		OptPara[],
														double	Part_Dis[],				double	Part_Vel[],
														double	Part_NewDis[],		double	Part_NewVel[],
														int			Part_Rank[],
														curandStateMtgp32 *state	)
	{
		int	iThre;
		int	i,j,	TempI[20];
		double	TempD[25];
		double	TempF[25];

		iThre	=	(blockIdx.x	*	blockDim.x)	+	threadIdx.x;

		/*	Current Particle Number		*/
		TempI[1]	=	(	iThre	*	OptPara[0].nValue	);

		/*--------------*/
		/*	GA Probably	*/
		/*--------------*/
		TempI[15]	=				(	curand(&state[blockIdx.x]	)%100	);
		TempI[19]	=	40	+	(	curand(&state[blockIdx.x]	)%20	);

		/*	Mutation	*/
		TempI[14]	=	(	TempI[15]	<	0	);

		/*	CrossOver	*/
		TempI[13]	=	(	TempI[15]	>=	0	)+(	TempI[15]	<	TempI[19]	);
		TempI[13]	=	(	TempI[13]	==	2	);

		/*	Survive	*/
		TempI[12]	=	(	TempI[15]	>=	TempI[19]	);

/*
		TempI[14]	=	0;
		TempI[13]	=	0;
		TempI[12]	=	1;
*/
		/*----------------------------*/
		/*	Select CrossOver Particle	*/
		/*----------------------------*/
		TempI[5]	=	(	curand(&state[blockIdx.x]	)%256	);
		for (	i=1;	i	<	OptPara[0].nPartical;	i++	)
		{
			if	(	Part_Rank[	i	]	==	TempI[5]	)
			{
				TempI[2]	=	i;
				break;
			}
		}

		/*	Select Particle Number		*/
		TempI[2]	=	(	TempI[2]	*	OptPara[0].nValue	);
		for (	i=0;	i	<	(OptPara[0].nValue);	i++	)
		{
			/*	CrossOver Choice	*/
			TempI[3]	=	(	curand(&state[blockIdx.x]	)	);
			TempI[3]	=	(	TempI[3]%2	);
			TempI[3]	=	(	TempI[3]==0	);

			/*	Mutation Choice	*/
			TempI[4]	=	(	curand(&state[blockIdx.x]	)	);
			TempI[4]	=	(	TempI[4]%500	);
			TempI[4]	=	(	TempI[4]==0		);

			/*	Mutation Add	*/
			TempF[24]	=	(		0.1	-	0.2	*	curand_uniform_double(&state[blockIdx.x])	);

			/*	Savive		Ç‹ÇΩÇÕ	TempI[3]Ç™1Ç»ÇÁTempI[15]ÇÕ1		*/
			/*	CrossOver	Ç©Ç¬		TempI[3]Ç™1Ç»ÇÁTempI[16]ÇÕ1		*/
			/*	Mutation	Ç©Ç¬		TempI[4]Ç™1Ç»ÇÁTempI[17]ÇÕ1		*/
			TempI[15]	=	(	(	TempI[3]	+	TempI[13]	)	!=	2	);
			TempI[16]	=	(	(	TempI[3]	+	TempI[13]	)	==	2	);
			TempI[17]	=	(	(	TempI[4]	+	TempI[14]	)	==	2	);

			Part_Vel[	TempI[1]+i	]	=		(	TempI[15]	*	Part_NewVel[	TempI[1]+i	]	)
																+	(	TempI[16]	*	Part_NewVel[	TempI[2]+i	]	);

			TempF[23]								=		(	TempI[15]	*	Part_NewDis[	TempI[1]+i	]	)
																+	(	TempI[16]	*	Part_NewDis[	TempI[2]+i	]	)
																+	(	TempI[17]	*	TempF[24]	);

			if	(	TempF[23]	<	-0.9999999	)
			{
				TempF[23]	=	-0.9999999;
			}
			if	(	TempF[23]	>	0.9999999	)
			{
				TempF[23]	=	0.9999999;
			}
			Part_Dis[	TempI[1]+i	]	=	TempF[23];
		}

		return;
	}

/*==========================*/
/*	MainFunction						*/
/*==========================*/
	void	ParticalSwarm_Main(	Func_Point	Obje_Func	)
	{

		/*Define Val*/
		int			i,j,k,m,	iImage,	TempI[30],	Flag,	ExitFlag={0};
		double	TempD[50]={0},	B_GroBest;
		float		TempF[50]={0};
		long		T[50]={0};
		char		TempC[255],	FileName[255];

		FILE		*ID;

		dim3		block,	Threads;

		B_GroBest	=	-1000.00;

		double	*Partical_Dis_Save;
		Partical_Dis_Save	=	(double*)malloc(	sizeof(double)	*	(	OptPara[0].nValue	+	10	)	);

		/*--------------*/
		/*	Initialied	*/
		/*--------------*/

		Flag	=	0;
		m			=	0;


		while	(	1	)
		{

			/*========================*/
			/*========================*/
			/*	MainProgram						*/
			/*========================*/
			/*========================*/
			T[20]	=	clock();
			block			=	dim3(	OptPara[0].nPartical,		1,	1	);
			Threads		=	dim3(	32,											1,	1	);
			Get_Vertex_GPU1AAA	<<<	block,	Threads		>>>
											(	STAR_Para_d,	STAR_Partic_d,
												Cam_Para_d,		OptPara_d,
												Part_Dis_d,		Part_Dis_PB_d,
												Part_Obje_d,	Part_PB_d,
												rb_d,					mb_d	);
			cudaDeviceSynchronize();

			T[21]	=	clock();
			for (	iImage=0;	iImage<Cam_Para[0].nImage;	iImage++	)
			{


				T[22]	=	clock();

				block			=	dim3(	OptPara[0].nPartical/64,	1,	1	);
				Threads		=	dim3(	64,												1,	1	);
				Get_Vertex_GPU2AAA	<<<	block,	Threads		>>>
												(	STAR_Para_d,	STAR_Partic_d,
													Cam_Para_d,		OptPara_d,
													Part_Dis_d,		Part_Dis_PB_d,
													Part_Obje_d,	Part_PB_d,
													rb_d,					mb_d,				iImage	);
				cudaDeviceSynchronize();

				T[23]			=	clock();

				block			=	dim3(	OptPara[0].nPartical,		1,	1	);
				Threads		=	dim3(	32,											1,	1	);
				Get_Vertex_GPU3AAA	<<<	block,	Threads		>>>
												(	STAR_Para_d,	STAR_Partic_d,
													Cam_Para_d,		OptPara_d,
													Part_Dis_d,		Part_Dis_PB_d,
													Part_Obje_d,	Part_PB_d,
													rb_d,					mb_d,				iImage	);
				cudaDeviceSynchronize();


				T[24]			=	clock();

				block			=	dim3(	OptPara[0].nPartical,		1,	1	);
				Threads		=	dim3(	256,											1,	1	);
				Get_Vertex_GPU5AAA	<<<	block,	Threads		>>>
												(	STAR_Para_d,	STAR_Partic_d,
													Cam_Para_d,		OptPara_d,
													Part_Dis_d,		Part_Dis_PB_d,
													Part_Obje_d,	Part_PB_d,
													rb_d,					mb_d,				iImage	);
				cudaDeviceSynchronize();

				T[25]	=	clock();

				block			=	dim3(	(OptPara[0].nPartical)*Cam_Para[0].nCam,	1,	1	);
				Threads		=	dim3(	32,																				1,	1	);

				Objective_Function	<<<	block,	Threads		>>>
												(	STAR_Para_d,	STAR_Partic_d,
													Cam_Para_d,		OptPara_d,
													Part_Dis_d,		Part_Dis_PB_d,
													Part_Obje_d,	Part_PB_d,
													rb_d,					mb_d,	m	);
				cudaDeviceSynchronize();

				T[26]	=	clock();

				block			=	dim3(	OptPara[0].nPartical,	1,	1	);
				Threads		=	dim3(	256,									1,	1	);
				Obje_CalFunc	<<<	block,	Threads		>>>
												(	STAR_Partic_d,	OptPara_d,
													Cam_Para_d,
													Part_Dis_d,			Part_Dis_PB_d,
													Part_Obje_d,		Part_PB_d,
													1,						iImage	);
				cudaDeviceSynchronize();


				T[27]	=	clock();

//				printf(	"AAA %d \n",	T[21]-T[20]	);
//				printf(	"BBB %d \n",	T[23]-T[22]	);
//				printf(	"CCC %d \n",	T[24]-T[23]	);
//				printf(	"DDD %d \n",	T[25]-T[24]	);
//				printf(	"EEE %d \n",	T[26]-T[25]	);
//				printf(	"FFF %d \n",	T[27]-T[26]	);


			}

			T[41]	=	clock();
//			printf(	"AGAG %d \n",	T[41]-T[40]	);

			/*----------------------*/
			/*	Next Displacement		*/
			/*----------------------*/
			k	=	5	+	OptPara[0].Still;
			if	(	k	>=	OptPara[0].nPartical	)
			{
				k	=	OptPara[0].nPartical;
			}

			/*	Particle Swarm	*/
			block			=	dim3(	((OptPara[0].nPartical)/(32)),		1,	1	);
			Threads		=	dim3(	32,																1,	1	);
			i=0;
			Next_Displacement<<<	block,	Threads	>>>(	OptPara_d,
																									Part_Obje_d,			Part_Rank_d,
																									Part_Dis_d,				Part_Vel_d,
																									Part_NewDis_d,		Part_NewVel_d,
																									Part_Dis_PB_d,
																									devMTGPStates,		k	);
			cudaDeviceSynchronize();
			T[3]	=	clock();


			/*	Genetic Algorithm	*/
			block			=	dim3(	((OptPara[0].nPartical)/(32)),		1,	1	);
			Threads		=	dim3(	32,															1,	1	);
			Next_Displacement_GA<<<	block,	Threads	>>>(	OptPara_d,
																										Part_Dis_d,			Part_Vel_d,
																										Part_NewDis_d,	Part_NewVel_d,
																										Part_Rank_d,
																										devMTGPStates	);
			cudaDeviceSynchronize();
			T[4]	=	clock();

			/*----------------------*/
			/*	Device	Å®	Host		*/
			/*----------------------*/
			/*	Personal Best Host to Devece			*/
			i	=	OptPara[0].nPartical;
			ErrorID	=	cudaMemcpy(	Part_PB,		Part_PB_d,		sizeof(double)	*	i,	cudaMemcpyDeviceToHost);
			if	(ErrorID	!=	cudaSuccess)
			{
				printf(	"\n   CudaMemoryCopy Error!!! 001"	);
				exit(1);
			}

			i	=	OptPara[0].nPartical;
			ErrorID	=	cudaMemcpy(	Part_Obje,	Part_Obje_d,	sizeof(double)	*	i,	cudaMemcpyDeviceToHost);
			if	(ErrorID	!=	cudaSuccess)
			{
				printf(	"\n   CudaMemoryCopy Error!!! 001"	);
				exit(1);
			}
			T[5]	=	clock();

			/*--------------------------------------*/
			/*	Disp World Record and Mean Record		*/
			/*--------------------------------------*/

			TempI[0]	=	0;
			TempI[1]	=	0;
			TempF[0]	=	Part_PB[0];
			TempF[1]	=	0;
			Flag	=	0;
			for (	i=0;	i<OptPara[0].nPartical;	i++	)
			{
				TempF[1]	=	TempF[1]	+	Part_Obje[i];

				if	(	Part_PB[i]	<	TempF[0]	)
				{
					TempF[0]	=	Part_PB[i];
					TempI[1]	=	i;
					Flag			=	1;
				}
			}
			TempF[1]	=	TempF[1]	/	(OptPara[0].nPartical);


			if	(	B_GroBest	==	TempF[0]	)
			{
				OptPara[0].Still		=	OptPara[0].Still		+	1;
				OptPara[0].Still02	=	OptPara[0].Still02	+	1;
			}
			else
			{
				OptPara[0].Still		=	OptPara[0].Still	-	1;
				OptPara[0].Still02	=	0;

				if	(	OptPara[0].Still	<=	0	)
				{	OptPara[0].Still	=	0;	}

			}
			B_GroBest	=	TempF[0];

			T[6]	=	clock();

			T[14]	=	T[14]	+	(T[6]-T[5]);

			if	(	(m%1)==0	)
			{
				T[10]	=	0;
				T[11]	=	0;
				T[12]	=	0;
				T[13]	=	0;
				T[14]	=	0;

				printf("%d %.14f %.14f %d %d \n",m,TempF[0],TempF[1],OptPara[0].Still,OptPara[0].Still02	);
			}

			if			(	Cam_Para[0].ObjeFlag	==	1	)
			{
				if	(	OptPara[0].Still	>	150	&&	m	>	500	)
				{
					ExitFlag	=	1;
				}
			}
			else
			{
				if	(	m	>	500	)
				{
					ExitFlag	=	1;
				}
			}

//			if	(	(m%10)	==	0	&&	Flag	==	1	)
//			{

				if	(	ExitFlag	==	1	)
				{

				/*	Inport_PBDis	*/
				TempI[3]	=	(sizeof(double)	*	(	OptPara[0].nPartical	*	OptPara[0].nValue	)	);
				ErrorID		=	cudaMemcpy(	Part_Dis_PB,	Part_Dis_PB_d,	TempI[3],	cudaMemcpyDeviceToHost	);
				if	(ErrorID	!=	cudaSuccess)
				{
					printf(	"\n   CudaMemoryCopy Error!!! 002"	);
					exit(1);
				}

				/*	LocalBest Particle Number	*/
				TempI[1]	=	TempI[1]	*	OptPara[0].nValue;

				/*----------------------*/
				/*	Next Dispracement		*/
				/*----------------------*/
					for (	i=0;	i<OptPara[0].nValue;	i++	)
					{
						Partical_Dis_Save[i]	=	
									(	Part_Dis_PB[	TempI[1]+i	]	*	rb[i]	)	+	mb[i];
					}
					FileName[0]	=	'\0';
					strcat(	FileName,	"OptimizationData.Dat"	);

					/*	SaveData	*/
					ID	=	NULL;
					while	(ID==NULL)
					ID	=	fopen(	FileName,	"wb"	);

					fwrite(Partical_Dis_Save,	sizeof(double),	OptPara[0].nValue,	ID);
					fclose(ID);

					exit(1);
					Flag	=	0;
				}

//			}
			m	=	m	+	1;
		}

		return;
	}













