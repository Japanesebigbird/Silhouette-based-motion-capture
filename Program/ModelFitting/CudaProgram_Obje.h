/*================================================================*/
/* Objective Function																							*/
/*================================================================*/

/*--------------------*/
/* Internal Function	*/
/*--------------------*/
	__host__
	__device__
	void	Rodrigues(	double	Angle[],	float	Rm[]	)
	{
		double	N1,		N2,		N3,		Theta,	InvTheta,	C,		S,
						N1N1,	N2N2,	N3N3,	N1N2,		N1N3,			N2N3,	p1nC,
						N1_S,	N2_S,	N3_S;

		N1	=	Angle[0];
		N2	=	Angle[1];
		N3	=	Angle[2];

		Theta			=	sqrtf(	(N1*N1)	+	(N2*N2)	+	(N3*N3)	);
		InvTheta	=	1/Theta;

		if	(	Theta	<	1.0e-12	)
		{
			N1	=	N1;
			N2	=	N2;
			N3	=	N3;
		}
		else
		{
			N1	=	N1	*	InvTheta;
			N2	=	N2	*	InvTheta;
			N3	=	N3	*	InvTheta;
		}

		C			=	cos(Theta);
		S			=	sin(Theta);

		N1N1	=	N1	*	N1;
		N2N2	=	N2	*	N2;
		N3N3	=	N3	*	N3;

		N1N2	=	N1	*	N2;
		N1N3	=	N1	*	N3;
		N2N3	=	N2	*	N3;
		p1nC	=	(1-C);

		N1_S	=	N1	*	S;
		N2_S	=	N2	*	S;
		N3_S	=	N3	*	S;

		Rm[0]	=	C							+	(	N1N1*p1nC	);
		Rm[1]	=	(	N1N2*p1nC	)	+	(	N3_S			);
		Rm[2]	=	(	N1N3*p1nC	)	-	(	N2_S			);

		Rm[3]	=	(	N1N2*p1nC	)	-	(	N3_S			);
		Rm[4]	=	C							+	(	N2N2*p1nC	);
		Rm[5]	=	(	N2N3*p1nC	)	+	(	N1_S			);

		Rm[6]	=	(	N1N3*p1nC	)	+	(	N2_S			);
		Rm[7]	=	(	N2N3*p1nC	)	-	(	N1_S			);
		Rm[8]	=	C							+	(	N3N3*p1nC	);

		return;
	}


/*--------------------*/
/* Internal Function	*/
/*--------------------*/
	__device__
	void	Euler_Rodrigues(	double	Angle[]	)
	{
		int	i,j,k;
		double	TempF[5],	Rmx[3][3],	Rmy[3][3],	Rmz[3][3],	TempRm[3][3],	Rm[3][3],	S1,	S2,	S3,	C1,	C2,	C3,	Ang;

		S1	=	sin(	Angle[0]	);	S2	=	sin(	Angle[1]	);	S3	=	sin(	Angle[2]	);
		C1	=	cos(	Angle[0]	);	C2	=	cos(	Angle[1]	);	C3	=	cos(	Angle[2]	);

		Rmx[0][0]	=	1;		Rmx[0][1]	=	0;		Rmx[0][2]	=	0;
		Rmx[1][0]	=	0;		Rmx[1][1]	=	C1;		Rmx[1][2]	=	-S1;
		Rmx[2][0]	=	0;		Rmx[2][1]	=	S1;		Rmx[2][2]	=	C1;

		Rmy[0][0]	=	C2;		Rmy[0][1]	=	0;		Rmy[0][2]	=	S2;
		Rmy[1][0]	=	0;		Rmy[1][1]	=	1;		Rmy[1][2]	=	0;
		Rmy[2][0]	=	-S2;	Rmy[2][1]	=	0;		Rmy[2][2]	=	C2;

		Rmz[0][0]	=	C3;		Rmz[0][1]	=	-S3;	Rmz[0][2]	=	0;
		Rmz[1][0]	=	S3;		Rmz[1][1]	=	C3;		Rmz[1][2]	=	0;
		Rmz[2][0]	=	0;		Rmz[2][1]	=	0;		Rmz[2][2]	=	1;


		for(	i=0;	i<3;	i++	){
			for(	j=0;	j<3;	j++	){
				TempRm[i][j]	= (	Rmx[i][0] * Rmy[0][j]	)	+	(	Rmx[i][1] * Rmy[1][j]	)	+	(	Rmx[i][2] * Rmy[2][j]	);
			}
		}

		for(	i=0;	i<3;	i++	){
			for(	j=0;	j<3;	j++	){
				Rm[i][j]	= (	TempRm[i][0] * Rmz[0][j]	)	+	(	TempRm[i][1] * Rmz[1][j]	)	+	(	TempRm[i][2] * Rmz[2][j]	);
			}
		}

		TempF[0]	=	(	Rm[0][0]	+	Rm[1][1]	+	Rm[2][2]	-	1	)	/	2;
		Ang				=	acos(	TempF[0]	);

		if	(	sin(	Ang	)	<	0.000001	)
		{
			Angle[0]	=	Angle[0]	+	(	(M_PI_d/180)*0.1	);
			Angle[1]	=	Angle[1]	+	(	(M_PI_d/180)*0.1	);
			Angle[2]	=	Angle[2]	+	(	(M_PI_d/180)*0.1	);

			S1	=	sin(	Angle[0]	);	S2	=	sin(	Angle[1]	);	S3	=	sin(	Angle[2]	);
			C1	=	cos(	Angle[0]	);	C2	=	cos(	Angle[1]	);	C3	=	cos(	Angle[2]	);

			Rmx[0][0]	=	1;		Rmx[0][1]	=	0;		Rmx[0][2]	=	0;
			Rmx[1][0]	=	0;		Rmx[1][1]	=	C1;		Rmx[1][2]	=	-S1;
			Rmx[2][0]	=	0;		Rmx[2][1]	=	S1;		Rmx[2][2]	=	C1;

			Rmy[0][0]	=	C2;		Rmy[0][1]	=	0;		Rmy[0][2]	=	S2;
			Rmy[1][0]	=	0;		Rmy[1][1]	=	1;		Rmy[1][2]	=	0;
			Rmy[2][0]	=	-S2;	Rmy[2][1]	=	0;		Rmy[2][2]	=	C2;

			Rmz[0][0]	=	C3;		Rmz[0][1]	=	-S3;	Rmz[0][2]	=	0;
			Rmz[1][0]	=	S3;		Rmz[1][1]	=	C3;		Rmz[1][2]	=	0;
			Rmz[2][0]	=	0;		Rmz[2][1]	=	0;		Rmz[2][2]	=	1;

			for(	i=0;	i<3;	i++	){
				for(	j=0;	j<3;	j++	){
					TempRm[i][j]	= (	Rmx[i][0] * Rmy[0][j]	)	+	(	Rmx[i][1] * Rmy[1][j]	)	+	(	Rmx[i][2] * Rmy[2][j]	);
				}
			}

			for(	i=0;	i<3;	i++	){
				for(	j=0;	j<3;	j++	){
					Rm[i][j]	= (	TempRm[i][0] * Rmz[0][j]	)	+	(	TempRm[i][1] * Rmz[1][j]	)	+	(	TempRm[i][2] * Rmz[2][j]	);
				}
			}

			TempF[0]	=	(	Rm[0][0]	+	Rm[1][1]	+	Rm[2][2]	-	1	)	/	2;
			Ang				=	acosf(	TempF[0]	);
		}

/*
		printf(" %.14f %.14f \n",	Ang,	TempF[0]		);
		printf(" %.14f %.14f %.14f \n",	Rm[0][0],	Rm[0][1],	Rm[0][2]	);
		printf(" %.14f %.14f %.14f \n",	Rm[1][0],	Rm[1][1],	Rm[1][2]	);
		printf(" %.14f %.14f %.14f \n",	Rm[2][0],	Rm[2][1],	Rm[2][2]	);
		printf("\n"	);
*/

		TempF[0]	=	2*sin(Ang);
		TempF[1]	=	(	Rm[2][1]	-	Rm[1][2]	)	/	TempF[0];
		TempF[2]	=	(	Rm[0][2]	-	Rm[2][0]	)	/	TempF[0];
		TempF[3]	=	(	Rm[1][0]	-	Rm[0][1]	)	/	TempF[0];
/*
		printf(" %.14f %.14f %.14f \n",	TempF[1],	TempF[2],	TempF[3]	);
*/

		TempF[1]	=	TempF[1]*Ang;
		TempF[2]	=	TempF[2]*Ang;
		TempF[3]	=	TempF[3]*Ang;

/*
		printf(" %.14f %.14f %.14f \n",	Angle[0],	Angle[1],	Angle[2]	);
		printf(" %.14f %.14f %.14f \n",	sin(	Angle[0]),	sin(	Angle[1]),	sin(	Angle[2])	);
		printf(" %.14f %.14f %.14f \n",	cos(	Angle[0]),	cos(	Angle[1]),	cos(	Angle[2])	);
		printf(" \n"	);
*/

		Angle[0]	=	TempF[1];
		Angle[1]	=	TempF[2];
		Angle[2]	=	TempF[3];

/*
		printf(" %.14f %.14f %.14f \n",	Angle[0],	Angle[1],	Angle[2]	);
		printf("\n\n"	);
*/

		return;
	}



/*--------------------------------------------------*/
/*																									*/
/*--------------------------------------------------*/
	__global__
	void	Get_Vertex_GPU1AAA(	STAR_Parameter		STAR_Para[],	STAR_Particle		STAR_Partic[],
														Camera_Parameter	Cam_Para[],		OptP						OptPara[],
														double						Part_Dis[],		double					Part_Dis_PB[],
														double						Part_Obje[],	double					Part_PB[],
														double						rb[],					double					mb[]	)
	{

		int	i,j,k,	iVertex,	iVert900,	iVert3,	j300,	InITI,	Q_Posi,	Q_Posi01,	Q_Posi02,	Q_Posi03,	TempI[5];
		float	TempF	=	{0.00},	TempF2={0.00},	EXP,	Weight01[5],	Weight02[5];
		long		T[20]={0};

		__shared__	float4		Betas4[80],	Betas_Ratio4[90];
		__shared__	float			TempF_Shared[64],	Betas[320];
		int	iThre,	iThre_Lo;

		iThre			=	blockIdx.x;
		iThre_Lo	=	threadIdx.x;

//		printf(" %d %d %d \n",	iThre,iThre_Lo,blockDim.x	);



		if (	iThre	<	OptPara[0].nPartical	)
		{
			STAR_Partic[iThre].ReturnFlag	=	0;


			T[0]	=	clock();
			Q_Posi01	=	(	iThre								*	OptPara[0].nValue	);
			Q_Posi03	=	(	Cam_Para[0].nImage	*	75	);

			for (	i=iThre_Lo;	i<75;	i=i+blockDim.x	)
			{
				Betas_Ratio4[i].x	=	Part_Dis[	Q_Posi01	+	(Q_Posi03+(i*4)+0)	];
				Betas_Ratio4[i].y	=	Part_Dis[	Q_Posi01	+	(Q_Posi03+(i*4)+1)	];
				Betas_Ratio4[i].z	=	Part_Dis[	Q_Posi01	+	(Q_Posi03+(i*4)+2)	];
				Betas_Ratio4[i].w	=	Part_Dis[	Q_Posi01	+	(Q_Posi03+(i*4)+3)	];
			}
			__syncthreads;


			for (	i=iThre_Lo;	i<75;	i=i+blockDim.x	)
			{
				Betas4[i].x	=	(	Betas_Ratio4[i].x	*	rb[	0	+	(Q_Posi03+(i*4)+0)	]	)	+	mb[	0	+	(Q_Posi03+(i*4)+0)	];
				Betas4[i].y	=	(	Betas_Ratio4[i].y	*	rb[	0	+	(Q_Posi03+(i*4)+1)	]	)	+	mb[	0	+	(Q_Posi03+(i*4)+1)	];
				Betas4[i].z	=	(	Betas_Ratio4[i].z	*	rb[	0	+	(Q_Posi03+(i*4)+2)	]	)	+	mb[	0	+	(Q_Posi03+(i*4)+2)	];
				Betas4[i].w	=	(	Betas_Ratio4[i].w	*	rb[	0	+	(Q_Posi03+(i*4)+3)	]	)	+	mb[	0	+	(Q_Posi03+(i*4)+3)	];
			}
			__syncthreads;


			EXP	=	2.00;
			STAR_Partic[iThre].TempObje				=	0.00;
			STAR_Partic[iThre].TempObje_Betas	=	0.00;

			TempF		=	0.00;
			TempF2	=	1	/	299.000;
			for (	i=iThre_Lo;	i<75;	i=i+blockDim.x	)
			{
				Weight01[0]	=	(i*4+0)	*	TempF2;
				Weight01[1]	=	(i*4+1)	*	TempF2;
				Weight01[2]	=	(i*4+2)	*	TempF2;
				Weight01[3]	=	(i*4+3)	*	TempF2;

				Weight01[0]	=	0.009983305509182	*	(	Weight01[0]	*	Weight01[0]	);
				Weight01[1]	=	0.009983305509182	*	(	Weight01[1]	*	Weight01[1]	);
				Weight01[2]	=	0.009983305509182	*	(	Weight01[2]	*	Weight01[2]	);
				Weight01[3]	=	0.009983305509182	*	(	Weight01[3]	*	Weight01[3]	);

				TempF	=	TempF
								+	Weight01[0]	*	powf(	abs(Betas4[i].x/5.00),	EXP	)
								+	Weight01[1]	*	powf(	abs(Betas4[i].y/5.00),	EXP	)
								+	Weight01[2]	*	powf(	abs(Betas4[i].z/5.00),	EXP	)
								+	Weight01[3]	*	powf(	abs(Betas4[i].w/5.00),	EXP	);
			}
			TempF_Shared[iThre_Lo]	=	TempF;
			__syncthreads;


			if	(iThre_Lo	==	0)
			{
				TempF	=	0.00;
				for (	i=0;	i<32;	i=i+1	)
				{
					TempF	=	TempF	+	TempF_Shared[i];
				}
				STAR_Partic[iThre].TempObje_Betas	=	powf(	TempF,	1.0/EXP	);
			}
			__syncthreads;


			T[1]	=	clock();
			for (	i=0;	i<646;	i++	)
			{
				TempI[0]	=	i	*	(	blockDim.x*75	);
				TempI[1]	=	(	i	*	blockDim.x	)	+	iThre_Lo;

				TempF	=	0.00;
				k	=	iThre_Lo	+	TempI[0]	-	blockDim.x;
				for (	j=0;	j<75;	j++	)
				{

//				k	=	iThre_Lo	+	(	j	*	blockDim.x	)	+	TempI[0];
					k	=	k	+	blockDim.x;

					TempF	=	TempF
										+	(	STAR_Para[0].Shape_Dirs4[k].x	*	Betas4[j].x	)
										+	(	STAR_Para[0].Shape_Dirs4[k].y	*	Betas4[j].y	)
										+	(	STAR_Para[0].Shape_Dirs4[k].z	*	Betas4[j].z	)
										+	(	STAR_Para[0].Shape_Dirs4[k].w	*	Betas4[j].w	);
				}
				STAR_Partic[iThre].V_Pose_Ori[	TempI[1]	]	=	STAR_Para[0].V_Temp[	TempI[1]	]	+	TempF;

			}
			__syncthreads;


			T[2]	=	clock();
/*
			for (	iVertex=0;	iVertex<100;	iVertex++	)
			{
				printf(" %d %f %f %f \n",	iVertex,	STAR_Partic[iThre].V_Pose_Ori[(iVertex*3)+0],	STAR_Partic[iThre].V_Pose_Ori[(iVertex*3)+1],	STAR_Partic[iThre].V_Pose_Ori[(iVertex*3)+2]	);
			}

			for (	iVertex=6790;	iVertex<6890;	iVertex++	)
			{
				printf(" %d %f %f %f \n",	iVertex,	STAR_Partic[iThre].V_Pose_Ori[(iVertex*3)+0],	STAR_Partic[iThre].V_Pose_Ori[(iVertex*3)+1],	STAR_Partic[iThre].V_Pose_Ori[(iVertex*3)+2]	);
			}
*/


//			if	(iThre	==	0	&& iThre_Lo	==	0)
//			{
//				printf(	"22AA %d \n",	T[1]-T[0]	);
//				printf(	"22BB %d \n",	T[2]-T[1]	);
//			}



		}
		return;
	}



/*--------------------------------------------------*/
/*																									*/
/*--------------------------------------------------*/
	__global__
	void	Get_Vertex_GPU2AAA(	STAR_Parameter		STAR_Para[],	STAR_Particle		STAR_Partic[],
														Camera_Parameter	Cam_Para[],		OptP						OptPara[],
														double							Part_Dis[],		double						Part_Dis_PB[],
														double							Part_Obje[],	double						Part_PB[],
														double						rb[],					double					mb[],			int	iImage	)
	{

		int	i,j,k,	TempI[20],	iJoint,	iParent,	iVertex,	Q_Posi,	Q_Posi01,	Q_Posi02,	Q_Posi03;
		float	TempF[20]	=	{0.00},				Model_Joint[24*3+10]	=	{0.00},	TestModel_Joint[24*3+10]	=	{0.00},
					NormAngle[3],							Rm[9],
					Temp_HTrans[4][4]	=	{0},	HTrans[25][4][4]	=	{0},
					LoJoint[5]				=	{0},	HTrans2[25][4][4]	=	{0};

		long		T[20]={0};

		float			Trans[3];
		float			Betas[10]		=	{0.00};
		double		Pose[72]		=	{0.00};
		double		Angle[3];
		int	iThre;

		iThre	=	(blockIdx.x	*	blockDim.x)	+	threadIdx.x;


		if (	iThre	<	OptPara[0].nPartical	)
		{

			T[0]	=	clock();
			Q_Posi	=	(	iThre	*	OptPara[0].nValue	);
			Q_Posi01	=	(	iThre								*	OptPara[0].nValue	);
			Q_Posi02	=	(	iImage							*	75	);
			Q_Posi03	=	(	Cam_Para[0].nImage	*	75	);

			for (	i=0;	i<3;	i=i+1	)
			{
				Trans[i]	=	(	Part_Dis[	Q_Posi01	+	(	Q_Posi02	+	0	+	i	)	]	*	rb[	0	+	(	Q_Posi02	+	0	+	i	)	]	)	+	mb[	0	+	(	Q_Posi02	+	0	+	i	)	];
			}

			for (	i=0;	i<72;	i++	)
			{
				Pose[i]		=	(	Part_Dis[	Q_Posi01	+	(	Q_Posi02	+	3	+	i	)	]	*	rb[	0	+	(	Q_Posi02	+	3	+	i	)	]	)	+	mb[	0	+	(	Q_Posi02	+	3	+	i	)	];
			}

			for (	i=0;	i<2;	i++	)
			{
				Betas[i]	=	(	Part_Dis[	Q_Posi01	+	(	Q_Posi03	+	i	)	]	*	rb[	0	+	(	Q_Posi03	+	i	)	]	)	+	mb[	0	+	(	Q_Posi03	+	i	)	];
			}

			T[1]	=	clock();

			/*	Euler → Rodrigues	*/
			for (	iJoint=0;	iJoint<24;	iJoint++	)
			{
				i	=	(iJoint*3);
				Angle[0]	=	Pose[i+0];	Angle[1]	=	Pose[i+1];	Angle[2]	=	Pose[i+2];

				Euler_Rodrigues(	Angle	);
				Pose[i+0]	=	Angle[0];		Pose[i+1]	=	Angle[1];		Pose[i+2]	=	Angle[2];
			}
			T[2]	=	clock();

			/*	Feature	*/
			for (	i=0;	i<23;	i++	)
			{
				TempI[0]	=	(i*3)+3;
				TempI[1]	=	(i*4);

				Angle[0]	=	Pose[TempI[0]	+	0];
				Angle[1]	=	Pose[TempI[0]	+	1];
				Angle[2]	=	Pose[TempI[0]	+	2];

				TempF[0]	=	(Angle[0]*Angle[0])	+	(Angle[1]*Angle[1])	+	(Angle[2]*Angle[2]);
				TempF[0]	=	sqrtf(TempF[0]);
				if	(TempF[0]	<	1e-10)
				{
					TempF[0]	=	1e-10;
				}

				TempF[1]			=	1	/	TempF[0];
				NormAngle[0]	=	Angle[0]	*	TempF[1];
				NormAngle[1]	=	Angle[1]	*	TempF[1];
				NormAngle[2]	=	Angle[2]	*	TempF[1];

				TempF[1]	=	cos(	TempF[0]/2	);
				TempF[2]	=	sin(	TempF[0]/2	);

				STAR_Partic[iThre].Feature[	TempI[1]+0	]	=	NormAngle[0]	*	TempF[2];
				STAR_Partic[iThre].Feature[	TempI[1]+1	]	=	NormAngle[1]	*	TempF[2];
				STAR_Partic[iThre].Feature[	TempI[1]+2	]	=	NormAngle[2]	*	TempF[2];
				STAR_Partic[iThre].Feature[	TempI[1]+3	]	=	TempF[1]	-	1;
			}
			STAR_Partic[iThre].Feature[92]	=	Betas[1];


			T[3]	=	clock();

			for (	i=0;	i<259;	i++	)
			{

				j					=	STAR_Para[0].J_Reg_Joint[i];
				iVertex		=	STAR_Para[0].J_Reg_Vertex[i];
				TempF[10]	=	STAR_Para[0].J_Reg_Co[i];

				TempI[0]	=	(iVertex*3);
				TempF[0]	=	STAR_Partic[iThre].V_Pose_Ori[	TempI[0]+0	];
				TempF[1]	=	STAR_Partic[iThre].V_Pose_Ori[	TempI[0]+1	];
				TempF[2]	=	STAR_Partic[iThre].V_Pose_Ori[	TempI[0]+2	];

				TempI[0]	=	(j*3);
				Model_Joint[TempI[0]+0]	=	Model_Joint[TempI[0]+0]	+	(	TempF[10]	*	TempF[0]	);
				Model_Joint[TempI[0]+1]	=	Model_Joint[TempI[0]+1]	+	(	TempF[10]	*	TempF[1]	);
				Model_Joint[TempI[0]+2]	=	Model_Joint[TempI[0]+2]	+	(	TempF[10]	*	TempF[2]	);
			}

			T[4]	=	clock();

			for (	iJoint=0;	iJoint<24;	iJoint++	)
			{

				if	(	iJoint==0	)
				{
					TempI[0]	=	(iJoint*3);

					Angle[0]	=	Pose[	TempI[0]	+	0	];
					Angle[1]	=	Pose[	TempI[0]	+	1	];
					Angle[2]	=	Pose[	TempI[0]	+	2	];

					Rodrigues(	Angle,	Rm	);

					HTrans[iJoint][0][0]	=	Rm[0];	HTrans[iJoint][0][1]	=	Rm[3];	HTrans[iJoint][0][2]	=	Rm[6];	HTrans[iJoint][0][3]	=	Model_Joint[0];
					HTrans[iJoint][1][0]	=	Rm[1];	HTrans[iJoint][1][1]	=	Rm[4];	HTrans[iJoint][1][2]	=	Rm[7];	HTrans[iJoint][1][3]	=	Model_Joint[1];
					HTrans[iJoint][2][0]	=	Rm[2];	HTrans[iJoint][2][1]	=	Rm[5];	HTrans[iJoint][2][2]	=	Rm[8];	HTrans[iJoint][2][3]	=	Model_Joint[2];
					HTrans[iJoint][3][0]	=	0.00;		HTrans[iJoint][3][1]	=	0.00;		HTrans[iJoint][3][2]	=	0.00;		HTrans[iJoint][3][3]	=	1.00;		

					TempI[0]	=	(iJoint*3);
                                                                             
					STAR_Partic[iThre].GrobalJoint_3[	iJoint	].x	=	HTrans[iJoint][0][3]	+	Trans[0];
					STAR_Partic[iThre].GrobalJoint_3[	iJoint	].y	=	HTrans[iJoint][1][3]	+	Trans[1];
					STAR_Partic[iThre].GrobalJoint_3[	iJoint	].z	=	HTrans[iJoint][2][3]	+	Trans[2];


				}
				else
				{

					iParent	=	STAR_Para[0].Kintree_Table[iJoint*2];

					TempI[0]	=	(iJoint*3);
					TempI[1]	=	(iParent*3);
					LoJoint[0]	=	Model_Joint[TempI[0]+0]	-	Model_Joint[TempI[1]+0];
					LoJoint[1]	=	Model_Joint[TempI[0]+1]	-	Model_Joint[TempI[1]+1];
					LoJoint[2]	=	Model_Joint[TempI[0]+2]	-	Model_Joint[TempI[1]+2];


					Angle[0]	=	Pose[	TempI[0]	+	0	];
					Angle[1]	=	Pose[	TempI[0]	+	1	];
					Angle[2]	=	Pose[	TempI[0]	+	2	];

					Rodrigues(	Angle,	Rm	);

					Temp_HTrans[0][0]	=	Rm[0];	Temp_HTrans[0][1]	=	Rm[3];	Temp_HTrans[0][2]	=	Rm[6];	Temp_HTrans[0][3]	=	LoJoint[0];
					Temp_HTrans[1][0]	=	Rm[1];	Temp_HTrans[1][1]	=	Rm[4];	Temp_HTrans[1][2]	=	Rm[7];	Temp_HTrans[1][3]	=	LoJoint[1];
					Temp_HTrans[2][0]	=	Rm[2];	Temp_HTrans[2][1]	=	Rm[5];	Temp_HTrans[2][2]	=	Rm[8];	Temp_HTrans[2][3]	=	LoJoint[2];
					Temp_HTrans[3][0]	=	0.00;		Temp_HTrans[3][1]	=	0.00;		Temp_HTrans[3][2]	=	0.00;		Temp_HTrans[3][3]	=	1.00;		

					for (	i=0;	i<4;	i++	)
					{
						for (	j=0;	j<4;	j++	)
						{
							TempF[0]	=	0;
							for (	k=0;	k<4;	k++	)
							{
								TempF[0]	=	TempF[0]	+	HTrans[iParent][i][k]	*	Temp_HTrans[k][j];
							}
							HTrans[iJoint][i][j]	=	TempF[0];
						}
					}

					TempI[0]	=	(iJoint*3);
					STAR_Partic[iThre].GrobalJoint_3[	iJoint	].x	=	HTrans[iJoint][0][3]	+	Trans[0];
					STAR_Partic[iThre].GrobalJoint_3[	iJoint	].y	=	HTrans[iJoint][1][3]	+	Trans[1];
					STAR_Partic[iThre].GrobalJoint_3[	iJoint	].z	=	HTrans[iJoint][2][3]	+	Trans[2];


				}
			}
			T[5]	=	clock();

			for (	iJoint=0;	iJoint<24;	iJoint++	)
			{
				for (	i=0;	i<4;	i++	)
				{
					for (	j=0;	j<3;	j++	)
					{
						STAR_Partic[iThre].HTrans2[iJoint][i][j]	=	HTrans[iJoint][i][j];
					}
				}

				TempI[0]	=	iJoint*3;

				TempF[0]	=	Model_Joint[	TempI[0]+0	];
				TempF[1]	=	Model_Joint[	TempI[0]+1	];
				TempF[2]	=	Model_Joint[	TempI[0]+2	];
				TempF[3]	=	0;

				for (	i=0;	i<4;	i++	)
				{
					TempF[4]	=		(	HTrans[iJoint][i][0]	*	TempF[0]	)
											+	(	HTrans[iJoint][i][1]	*	TempF[1]	)
											+	(	HTrans[iJoint][i][2]	*	TempF[2]	)
											+	(	HTrans[iJoint][i][3]	*	TempF[3]	);

					STAR_Partic[iThre].HTrans2[iJoint][i][3]	=	HTrans[iJoint][i][3]	-	TempF[4];
				}
			}
			T[6]	=	clock();


//			if	(iThre	==	0)
//			{
//				printf(	"22AA %d \n",	T[1]-T[0]	);
//				printf(	"22BB %d \n",	T[2]-T[1]	);
//				printf(	"22CC %d \n",	T[3]-T[2]	);
//				printf(	"22DD %d \n",	T[4]-T[3]	);
//				printf(	"22EE %d \n",	T[5]-T[4]	);
//				printf(	"22FF %d \n",	T[6]-T[5]	);
//			}


		}
		return;
	}


/*--------------------------------------------------*/
/*																									*/
/*--------------------------------------------------*/
	__global__
	void	Get_Vertex_GPU3AAA(	STAR_Parameter		STAR_Para[],	STAR_Particle		STAR_Partic[],
														Camera_Parameter	Cam_Para[],		OptP						OptPara[],
														double							Part_Dis[],		double						Part_Dis_PB[],
														double							Part_Obje[],	double						Part_PB[],
														double						rb[],					double					mb[],			int	iImage	)
	{

		int	i,j,k,	TempI[20],	iJoint,	iVertex,	iVertex_B,	iFeature,	iU,iV,	iVert3,	iVert933,	j93,	Q_Posi,	Q_Posi01,	Q_Posi02,	Q_Posi03;
		float	TempF[20]	=	{0.00};

		long		T[20]={0};


		__shared__	float		Feature[93];
//		__shared__	float3	Feature3[33];
//		__shared__	float		HTrans2[25*4*4];
//		__shared__	float4	HTrans24[10*4*4];
		__shared__	float		TestHTrans2[24][4][4];


/*
		__shared__	float	Trans[3];
*/

		float	Trans[3];

		int	iThre,	iThre_Lo;

/*
		iThre	=	(blockIdx.x	*	blockDim.x)	+	threadIdx.x;
*/
		iThre			=	blockIdx.x;
		iThre_Lo	=	threadIdx.x;



		if (	iThre	<	OptPara[0].nPartical	)
		{

			T[0]	=	clock();

			Q_Posi	=	(	iThre	*	OptPara[0].nValue	);

			Q_Posi01	=	(	iThre								*	OptPara[0].nValue	);
			Q_Posi02	=	(	iImage							*	75	);
			Q_Posi03	=	(	Cam_Para[0].nImage	*	75	);

			for (	i=0;	i<3;	i=i+1	)
			{
				Trans[i]	=	(	Part_Dis[	Q_Posi01	+	(	Q_Posi02	+	0	+	i)	]	*	rb[	0	+	(	Q_Posi02	+	0	+	i	)	]	)	+	mb[	0	+	(	Q_Posi02	+	0	+	i	)	];
			}

			for (	i=iThre_Lo;	i<93;	i=i+blockDim.x	)
			{
				Feature[i]	=	STAR_Partic[iThre].Feature[i];
			}

			for (	i=iThre_Lo;	i<(24*4*4);	i=i+blockDim.x	)
			{
				iJoint		=	i	%	24;
				TempI[0]	=	i	/	24;
				j					=	TempI[0]%4;
				k					=	TempI[0]/4;
				TestHTrans2[iJoint][k][j]	=	STAR_Partic[iThre].HTrans2[iJoint][k][j];
			}
			__syncthreads;

			T[1]	=	clock();

			for (	iVertex=iThre_Lo;	iVertex<STAR_Para[0].nVertex;	iVertex=iVertex+blockDim.x	)
			{
				TempI[0]	=	STAR_Para[0].Pose_DirsVer2_Start_nFeat[iVertex*2+0];
				TempI[1]	=	STAR_Para[0].Pose_DirsVer2_Start_nFeat[iVertex*2+1]	+	TempI[0];

				TempF[0]	=	0;	TempF[1]	=	0;	TempF[2]	=	0;
				for (	i=TempI[0];	i<TempI[1];	i++	)
				{
					iFeature	=	STAR_Para[0].Pose_DirsVer2_FeatITI[i];

					TempF[3]	=	Feature[iFeature];
					TempF[0]	=	TempF[0]	+	(	STAR_Para[0].Pose_DirsVer2_Co[i*3+0]	*	TempF[3]	);
					TempF[1]	=	TempF[1]	+	(	STAR_Para[0].Pose_DirsVer2_Co[i*3+1]	*	TempF[3]	);
					TempF[2]	=	TempF[2]	+	(	STAR_Para[0].Pose_DirsVer2_Co[i*3+2]	*	TempF[3]	);
				}

				STAR_Partic[iThre].V_Pose[iVertex*3+0]	=	STAR_Partic[iThre].V_Pose_Ori[iVertex*3+0]	+	TempF[0];
				STAR_Partic[iThre].V_Pose[iVertex*3+1]	=	STAR_Partic[iThre].V_Pose_Ori[iVertex*3+1]	+	TempF[1];
				STAR_Partic[iThre].V_Pose[iVertex*3+2]	=	STAR_Partic[iThre].V_Pose_Ori[iVertex*3+2]	+	TempF[2];
			}
			__syncthreads;
			T[2]	=	clock();

			for (	iVertex=iThre_Lo;	iVertex<STAR_Para[0].nVertex;	iVertex=iVertex+blockDim.x	)
			{

				TempI[0]	=	STAR_Para[0].WeightsVer2_Start_nFeat[iVertex*2+0];
				TempI[1]	=	STAR_Para[0].WeightsVer2_Start_nFeat[iVertex*2+1]	+	TempI[0];

				for (	j=0;	j<3;	j++	)
				{

					TempF[0]	=	0;	TempF[1]	=	0;
					TempF[2]	=	0;	TempF[3]	=	0;
					for (	i=TempI[0];	i<TempI[1];	i++	)
					{
						iJoint		=	STAR_Para[0].WeightsVer2_JointITI[i];
						TempF[4]	=	STAR_Para[0].WeightsVer2_Co[i];

						TempF[0]	=	TempF[0]	+	TestHTrans2[iJoint][j][0]	*	TempF[4];
						TempF[1]	=	TempF[1]	+	TestHTrans2[iJoint][j][1]	*	TempF[4];
						TempF[2]	=	TempF[2]	+	TestHTrans2[iJoint][j][2]	*	TempF[4];
						TempF[3]	=	TempF[3]	+	TestHTrans2[iJoint][j][3]	*	TempF[4];
					}


					k	=	3;
					TempF[5+j]	=	TempF[k];
					for (	k=0;	k<3;	k++	)
					{
						TempF[5+j]	=	TempF[5+j]	+	(	TempF[k]	*	STAR_Partic[iThre].V_Pose[	k+(iVertex*3)	]	);
					}
				}
				STAR_Partic[iThre].Vertex[iVertex].x	=	TempF[5+0]	+	Trans[0];
				STAR_Partic[iThre].Vertex[iVertex].y	=	TempF[5+1]	+	Trans[1];
				STAR_Partic[iThre].Vertex[iVertex].z	=	TempF[5+2]	+	Trans[2];
			}

			if (	iThre_Lo	==	0	)
			{
				STAR_Partic[iThre].Vertex[6890].x	=	1000.000;
				STAR_Partic[iThre].Vertex[6890].y	=	1000.000;
				STAR_Partic[iThre].Vertex[6890].z	=	1000.000;
			}


			T[3]	=	clock();

//			if	(	iThre	==	0	&&	iThre_Lo	==	0	)
//			{
//				printf(	"22AA %d \n",	T[1]-T[0]	);
//				printf(	"22BB %d \n",	T[2]-T[1]	);
//				printf(	"22CC %d \n",	T[3]-T[2]	);
//			}
//

/*
			for (	iVertex=0;	iVertex<100;	iVertex++	)
			{
				printf(" %d %f %f %f \n",	iVertex,	STAR_Partic[iThre].Vertex[	(iVertex*4)+0	],	STAR_Partic[iThre].Vertex[	(iVertex*4)+1	],	STAR_Partic[iThre].Vertex[	(iVertex*4)+2	]	);
			}

			for (	iVertex=6790;	iVertex<6890;	iVertex++	)
			{
				printf(" %d %f %f %f \n",	iVertex,	STAR_Partic[iThre].Vertex[	(iVertex*4)+0	],	STAR_Partic[iThre].Vertex[	(iVertex*4)+1	],	STAR_Partic[iThre].Vertex[	(iVertex*4)+2	]	);
			}
*/

		}
		return;
	}

/*--------------------------------------------------*/
/*																									*/
/*--------------------------------------------------*/
	__global__
	void	Get_Vertex_GPU5AAA(	STAR_Parameter		STAR_Para[],	STAR_Particle		STAR_Partic[],
														Camera_Parameter	Cam_Para[],		OptP						OptPara[],
														double							Part_Dis[],		double						Part_Dis_PB[],
														double							Part_Obje[],	double						Part_PB[],
														double						rb[],					double					mb[],			int	iImage	)
	{

		int		i,j,k,	iGroup,	iPart,	iTriangle,	iCam,	iU,	TempI[5],	Rank[3],	P1,	P2,	P3;
		int		MinU_Temp,	MinU,	MaxU,	MinV,	MaxV,	MinU_8,	MaxU_8,	MinV_8,	MaxV_8,	Max_U_Gro,	Min_U_Gro,	Max_V_Gro,	Min_V_Gro,	CounterU_Gro,	CounterV_Gro;
		int		CounterU_8,	CounterV_8,	CounterDot_8,	Counter;

		unsigned int	TempUI,	TempUI2,	HideFlag,	ReturnFlag,	FlagC;
		float					TempF[5]	=	{0.00},	OriUo,	OriVo,	LineCo_X[3],	LineCo_Y[3];
		float3				Vec[3],	RealCood[3];
		ushort2				DegiCood[3];

		int	iThre_InPart_X,	iThre_InPart_Y,	iTri_Lo;


		__shared__	float3	CL[12],	Rm[12][3];
		__shared__	float		F[12];


		__shared__	unsigned char		LabelHideFlag[8][8][32],	LabelMinU_8[8][8][32],	LabelMaxU_8[8][8][32],	LabelMinV_8[8][8][32],	LabelMaxV_8[8][8][32],	MaxMin_All[8][4];


		unsigned long long	TempUL,	TempUL2;
		ulonglong2							DegiData[3][2];

		long	T[50];

		unsigned char	MaxU_All,	MaxV_All,	MinU_All,	MinV_All;

		if	(	threadIdx.x	<	8	)
		{

			Rm[threadIdx.x][0].x	=	Cam_Para[threadIdx.x].Rm[0];
			Rm[threadIdx.x][1].x	=	Cam_Para[threadIdx.x].Rm[3];
			Rm[threadIdx.x][2].x	=	Cam_Para[threadIdx.x].Rm[6];

			Rm[threadIdx.x][0].y	=	Cam_Para[threadIdx.x].Rm[1];
      Rm[threadIdx.x][1].y	=	Cam_Para[threadIdx.x].Rm[4];
      Rm[threadIdx.x][2].y	=	Cam_Para[threadIdx.x].Rm[7];

			Rm[threadIdx.x][0].z	=	Cam_Para[threadIdx.x].Rm[2];
      Rm[threadIdx.x][1].z	=	Cam_Para[threadIdx.x].Rm[5];
      Rm[threadIdx.x][2].z	=	Cam_Para[threadIdx.x].Rm[8];

			F[threadIdx.x]				=	Cam_Para[threadIdx.x].F[0];


			CL[threadIdx.x].x		=	Cam_Para[threadIdx.x].CL[0];
			CL[threadIdx.x].y		=	Cam_Para[threadIdx.x].CL[1];
			CL[threadIdx.x].z		=	Cam_Para[threadIdx.x].CL[2];
		}
		__syncthreads();

		OriUo		=	Cam_Para[0].OriUo	-	0.5;		OriVo		=	Cam_Para[0].OriVo	-	0.5;


		MaxU_All	=	0;		MaxV_All	=	0;
		MinU_All	=	255;	MinV_All	=	255;

		iPart						=	blockIdx.x	+	(	threadIdx.x	/	blockDim.x	);
		iThre_InPart_X	=	threadIdx.x	/		32;
		iThre_InPart_Y	=	threadIdx.x	%		32;
//		if	(	iThre_InPart_Y	==	0	&&	iThre_InPart_X	==	0	&&	iImage	==	0	)
//		{
//			STAR_Partic[iPart].ReturnFlag	=	0;
//		}

		//=====================
		//=====================
		//	About Vertex
		//=====================
		//=====================
		ReturnFlag	=	0;
		for (	iGroup=iThre_InPart_X;	iGroup<431;	iGroup=iGroup+8	)
		{

			for (	iTri_Lo=iThre_InPart_Y;	iTri_Lo<32;	iTri_Lo=iTri_Lo+32	)
			{

				iTriangle	=	(iGroup*32)	+	iTri_Lo;

				i	=	STAR_Para[0].NewV_Point_32[iTriangle].x;
				j	=	STAR_Para[0].NewV_Point_32[iTriangle].y;
				k	=	STAR_Para[0].NewV_Point_32[iTriangle].z;

				RealCood[0]	=	STAR_Partic[iPart].Vertex[i];
				RealCood[1]	=	STAR_Partic[iPart].Vertex[j];
				RealCood[2]	=	STAR_Partic[iPart].Vertex[k];

				Vec[1].x	=	RealCood[1].x	-	RealCood[0].x;
				Vec[1].y	=	RealCood[1].y	-	RealCood[0].y;
				Vec[1].z	=	RealCood[1].z	-	RealCood[0].z;

				Vec[2].x	=	RealCood[2].x	-	RealCood[0].x;
				Vec[2].y	=	RealCood[2].y	-	RealCood[0].y;
				Vec[2].z	=	RealCood[2].z	-	RealCood[0].z;

				Vec[0].x	=	(	Vec[1].y	*	Vec[2].z	)	-	(	Vec[1].z	*	Vec[2].y	);
				Vec[0].y	=	(	Vec[1].z	*	Vec[2].x	)	-	(	Vec[1].x	*	Vec[2].z	);
				Vec[0].z	=	(	Vec[1].x	*	Vec[2].y	)	-	(	Vec[1].y	*	Vec[2].x	);

				TempF[0]	=	(	Vec[0].x*Vec[0].x	)	+	(	Vec[0].y*Vec[0].y	)	+	(	Vec[0].z*Vec[0].z	);
				TempF[0]	=	sqrtf(	TempF[0]	);
				TempF[0]	=	1/TempF[0];

				Vec[0].x	=	Vec[0].x	*	TempF[0];
				Vec[0].y	=	Vec[0].y	*	TempF[0];
				Vec[0].z	=	Vec[0].z	*	TempF[0];

				for (	iCam=0;	iCam<8;	iCam++	)
				{
					//--0以下は角度が90度以上で無視(隠れている), 答えが1の時は，隠れていない.

					T[10]	=	clock(	);

					HideFlag	=	(	(	-1	*	(	(	Vec[0].x	*	Rm[iCam][2].x	)	+	(	Vec[0].y	*	Rm[iCam][2].y	)	+	(	Vec[0].z	*	Rm[iCam][2].z	)	)	)	>=	0	);

					T[11]	=	clock(	);

					TempUL	=	HideFlag;
					if	(	HideFlag	==	1	)
					{

						k	=	0;
						for (	i=0;	i<3;	i++	)
						{

							Vec[1].x	=	RealCood[i].x	-	CL[iCam].x;
							Vec[1].y	=	RealCood[i].y	-	CL[iCam].y;
							Vec[1].z	=	RealCood[i].z	-	CL[iCam].z;

							TempF[0]	=	(	Rm[iCam][0].x	*	Vec[1].x	)	+	(	Rm[iCam][0].y	*	Vec[1].y	)	+	(	Rm[iCam][0].z	*	Vec[1].z	);
							TempF[1]	=	(	Rm[iCam][1].x	*	Vec[1].x	)	+	(	Rm[iCam][1].y	*	Vec[1].y	)	+	(	Rm[iCam][1].z	*	Vec[1].z	);
							TempF[2]	=	(	Rm[iCam][2].x	*	Vec[1].x	)	+	(	Rm[iCam][2].y	*	Vec[1].y	)	+	(	Rm[iCam][2].z	*	Vec[1].z	);

							TempF[2]	=	-F[iCam]	/	TempF[2];

							TempI[0]	=	(int)(	(	TempF[2]*TempF[0]	)	+	OriUo	);
							TempI[1]	=	(int)(	(	TempF[2]*TempF[1]	)	+	OriVo	);

							//---------------------
							//--Part1
							//画面の外を排除
							//---------------------
							j	=	(TempI[0]<=0)	+	(TempI[0]>1919)	+	(TempI[1]<=0)	+	(TempI[1]>1079);
							if	(j	!=	0)
							{
								k	=	1;
								TempI[0]	=	0.00;
								TempI[1]	=	0.00;
							}

							DegiCood[i].x	=	TempI[0];
							DegiCood[i].y	=	TempI[1];

						}

						if	(	k	==	1)
						{
							HideFlag	=	0;
						}
					}

					T[12]	=	clock(	);

					//-----------------------------------------------------
					//--Part2
					//線になっているものを削除(	X or Y座標が3点全て同じ)
					//-----------------------------------------------------
					if	(	HideFlag	==	1	)
					{
						//--線になっているものは探索外
						P1	=	(	(	(	DegiCood[0].x	==	DegiCood[1].x	)	+	(	DegiCood[1].x	==	DegiCood[2].x	)	+	(	DegiCood[2].x	==	DegiCood[0].x	)	)	==	3	);
						P2	=	(	(	(	DegiCood[0].y	==	DegiCood[1].y	)	+	(	DegiCood[1].y	==	DegiCood[2].y	)	+	(	DegiCood[2].y	==	DegiCood[0].y	)	)	==	3	);
						P1	=	P1	+	P2;
						if	(	P1	!=	0	)
						{
							HideFlag	=	0;
						}
					}

					T[13]	=	clock(	);

					//---------------------------------------------------------
					//--Part3
					//線になっているものを削除(	3点の内2点のX and Y座標が同じ)
					//---------------------------------------------------------
					if	(	HideFlag	==	1	)
					{
						//--線になっているものは探索外_Part2
						P1	=	(	(	(	DegiCood[0].x	==	DegiCood[1].x	)	+	(	DegiCood[0].y	==	DegiCood[1].y	)	)	==	2	);
						P2	=	(	(	(	DegiCood[1].x	==	DegiCood[2].x	)	+	(	DegiCood[1].y	==	DegiCood[2].y	)	)	==	2	);
						P3	=	(	(	(	DegiCood[2].x	==	DegiCood[0].x	)	+	(	DegiCood[2].y	==	DegiCood[0].y	)	)	==	2	);

						P1	=	P1	+	P2	+	P3;
						if	(	P1	!=	0	)
						{
							HideFlag	=	0;
						}
					}

					T[14]	=	clock(	);

					//--------------------------------------
					//--Part4
					//	3線の傾きが同じ
					//--------------------------------------
					if	(	HideFlag	==	1	)
					{
						TempI[0]	=	(	DegiCood[0].y	<=	DegiCood[1].y	)	+	(	DegiCood[0].y	<=	DegiCood[2].y	);
						TempI[1]	=	(	DegiCood[1].y	<		DegiCood[0].y	)	+	(	DegiCood[1].y	<=	DegiCood[2].y	);
						TempI[2]	=	3-(TempI[0]+TempI[1]);

						//	Attention!!
						/*Rank[0] Min,	Rank[1] Max,	Rank[2] Middle*/
						Rank[0]	=	(	(TempI[0]==2)*0	)	+	(	(TempI[1]==2)*1	)	+	(	(TempI[2]==2)*2	);
						Rank[2]	=	(	(TempI[0]==1)*0	)	+	(	(TempI[1]==1)*1	)	+	(	(TempI[2]==1)*2	);
						Rank[1]	=	3-(	Rank[0]	+	Rank[2]	);

						MinV	=	DegiCood[	Rank[0]	].y;
						MaxV	=	DegiCood[	Rank[1]	].y;

						/*--------------*/
						/*	MinU,	MaxU	*/
						/*--------------*/
						TempI[0]	=	(	DegiCood[0].x	<=	DegiCood[1].x	)	+	(	DegiCood[0].x	<=	DegiCood[2].x	);
						TempI[1]	=	(	DegiCood[1].x	<		DegiCood[0].x	)	+	(	DegiCood[1].x	<=	DegiCood[2].x	);
						TempI[2]	=	3-(TempI[0]+TempI[1]);

						//	Attention!!
						/*	Rank[0] Min,	Rank[1] Max,	Rank[2] Middle*/
						Rank[0]	=	(	(TempI[0]==2)*0	)	+	(	(TempI[1]==2)*1	)	+	(	(TempI[2]==2)*2	);
						Rank[2]	=	(	(TempI[0]==1)*0	)	+	(	(TempI[1]==1)*1	)	+	(	(TempI[2]==1)*2	);
						Rank[1]	=	3-(	Rank[0]	+	Rank[2]	);

						//=============================================================
						// 0:Line_Min→Max,	1:Line_Min→Middle,	2:Line_Middle→Max
						//=============================================================

						//--------------------
						//--0:Line_Min→Max
						//--------------------
						TempI[0]		=	DegiCood[	Rank[1]	].y	-	DegiCood[	Rank[0]	].y;
						TempI[1]		=	DegiCood[	Rank[1]	].x	-	DegiCood[	Rank[0]	].x;
						P1					=	(	TempI[0]	==	0	);
						P2					=	(	TempI[1]	==	0	);
						P3					=	P1+P2;

						LineCo_X[0]	=	0.00;
						if	(	P2==1	)
						{
							LineCo_X[0]	=	99.00;
						}

						if	(	P3	==	0	)
						{
							LineCo_X[0]	=	(	(float)TempI[0]	/	(float)TempI[1]	);
						}

						if	(	LineCo_X[0]	>	100	)
						{
							HideFlag	=	0;
						}

						//--------------------
						//--1:LineMin→Middle
						//--------------------
						TempI[0]		=	DegiCood[	Rank[0]	].y	-	DegiCood[	Rank[2]	].y;
						TempI[1]		=	DegiCood[	Rank[0]	].x	-	DegiCood[	Rank[2]	].x;
						P1					=	(	TempI[0]	==	0	);
						P2					=	(	TempI[1]	==	0	);
						P3					=	P1+P2;

						LineCo_X[1]	=	0.00;
						if	(	P2==1	)
						{
							LineCo_X[1]	=	99.00;
						}

						if	(	P3	==	0	)
						{
							LineCo_X[1]	=	(	(float)TempI[0]	/	(float)TempI[1]	);
						}

						if	(	LineCo_X[1]	>	100	)
						{
							HideFlag	=	0;
						}

						//--------------------
						//--2:Line_Middle→Max
						//--------------------
						TempI[0]		=	DegiCood[	Rank[1]	].y	-	DegiCood[	Rank[2]	].y;
						TempI[1]		=	DegiCood[	Rank[1]	].x	-	DegiCood[	Rank[2]	].x;
						P1					=	(	TempI[0]	==	0	);
						P2					=	(	TempI[1]	==	0	);
						P3					=	P1+P2;

						LineCo_X[2]	=	0.00;
						if	(	P2==1	)
						{
							LineCo_X[2]	=	99.00;
						}

						if	(	P3	==	0	)
						{
							LineCo_X[2]	=	(	(float)TempI[0]	/	(float)TempI[1]	);
						}

						if	(	LineCo_X[2]	>	100	)
						{
							HideFlag	=	0;
						}


						MinU		=	DegiCood[	Rank[0]	].x;
						MaxU		=	DegiCood[	Rank[1]	].x;


						TempI[10]	=	(	DegiCood[	Rank[0]	].x	!=	DegiCood[	Rank[2]	].x	);
						MinU_Temp	=	MinU	+	TempI[10];

						if	(	MinU_Temp	==	MaxU	)
						{
							HideFlag=0;
						}


						MinU_8				=	(	MinU_Temp	>>	3	);	MaxU_8	=	(	MaxU	>>	3	);	CounterU_8	=	MaxU_8-MinU_8+1;
						MinV_8				=	(	MinV	>>	3	);			MaxV_8	=	(	MaxV	>>	3	);	CounterV_8	=	MaxV_8-MinV_8+1;
						CounterDot_8	=	CounterU_8	*	CounterV_8;
						if	(	CounterDot_8	>	25	)
						{
							ReturnFlag	=	1;
						}


						P1	=	(	(	LineCo_X[0]	==	LineCo_X[1]	)	+	(	LineCo_X[1]	==	LineCo_X[2]	)	+	(	LineCo_X[2]	==	LineCo_X[0]	)	==	3	);
						if	(	P1	==	1	)
						{
							HideFlag=0;
						}
					}




					if	(	HideFlag	==	0	)
					{
						MinU_8	=	255;	MaxU_8	=	0;
						MinV_8	=	255;	MaxV_8	=	0;
						CounterDot_8	=	0;
					}

					LabelHideFlag[iThre_InPart_X][iCam][iThre_InPart_Y		]	=	HideFlag;
					LabelMinU_8[iThre_InPart_X][iCam][iThre_InPart_Y			]	=	MinU_8;
					LabelMaxU_8[iThre_InPart_X][iCam][iThre_InPart_Y			]	=	MaxU_8;
					LabelMinV_8[iThre_InPart_X][iCam][iThre_InPart_Y			]	=	MinV_8;
					LabelMaxV_8[iThre_InPart_X][iCam][iThre_InPart_Y			]	=	MaxV_8;


					T[15]	=	clock(	);

					TempUL	=	0;
					if	(	HideFlag	==	1	)
					{

						//	0-10:MinU,		11-21:MinV,		22-29:MaxU,
						//	0-7:MaxV,	8-15:MinUのDegi_V,	16-23:MidUのDegi_U,	24-31:MidUのDegi_V

						//===============
						//--MinU and MinV
						//===============
						P1			=	MinU;
						P1			=	P1	&	2047;
						TempUL	=	P1;

						P1			=	MinV;
						P1			=	P1	&	2047;
						TempUL	=	TempUL	|	(	(unsigned long long)P1<<11		);

						//===============
						//--MaxU and MaxV
						//===============
						P1			=	(MaxU-MinU);
						P1			=	P1	&	255;
						TempUL	=	TempUL	|	(	(unsigned long long)P1<<22		);

						P1			=	(MaxV-MinV);
						P1			=	P1	&	255;
						TempUL	=	TempUL	|	(	(unsigned long long)P1<<30		);

						//================================================
						/*Rank[0] Min,	Rank[1] Max,	Rank[2] Middle*/
						//================================================
						//--(MinU_Point_V)
						//--DegiCood[	Rank[0]	]
						P1			=	DegiCood[	Rank[0]	].y	-	MinV;
						P1			=	P1	&	255;
						TempUL	=	TempUL	|	(	(unsigned long long)P1<<38	);

						//--(MidU_Point_U)
						//--DegiCood[	Rank[2]	]
						P1			=	DegiCood[	Rank[2]	].x	-	MinU;
						P1			=	P1	&	255;
						TempUL	=	TempUL	|	(	(unsigned long long)P1<<46	);

						//--(MidU_Point_V)
						//--DegiCood[	Rank[2]	]
						P1			=	DegiCood[	Rank[2]	].y	-	MinV;
						P1			=	P1	&	255;
						TempUL	=	TempUL	|	(	(unsigned long long)P1<<54	);
					}
					STAR_Partic[iPart].GroupData[iCam][iGroup].HideData[iTri_Lo]	=	TempUL;

					T[16]	=	clock(	);

					if	(	HideFlag	==	1	)
					{

						//=============================================================
						// 0:Line_Min→Max,	1:Line_Min→Middle,	2:Line_Middle→Max
						//=============================================================
						STAR_Partic[iPart].GroupData[iCam][iGroup].LineCo[iTri_Lo].x	=	LineCo_X[0];
						STAR_Partic[iPart].GroupData[iCam][iGroup].LineCo[iTri_Lo].y	=	LineCo_X[1];
						STAR_Partic[iPart].GroupData[iCam][iGroup].LineCo[iTri_Lo].z	=	LineCo_X[2];
					}
					T[17]	=	clock(	);



//					if	(	HideFlag	==	1	&&	iTri_Lo	==	0	&&	iPart	==	0	&&	iCam	==	0	)
//					{
//						printf(	"1:AA %d \n",	T[11]-T[10]	);
//						printf(	"1:BB %d \n",	T[12]-T[11]	);
//						printf(	"1:CC %d \n",	T[13]-T[12]	);
//						printf(	"1:DD %d \n",	T[14]-T[13]	);
//						printf(	"1:EE %d \n",	T[15]-T[14]	);
//						printf(	"1:FF %d \n",	T[16]-T[15]	);
//						printf(	"1:GG %d \n",	T[17]-T[16]	);
//					}

				}

			}
			__syncthreads();



			iCam			=	iThre_InPart_Y	/	4;
			TempI[1]	=	iThre_InPart_Y	%	4;
			if			(	TempI[1]==0	)
			{

				TempUI	=	0;
				for (	i=0;	i<32;	i++	)
				{
					if	(	LabelHideFlag[iThre_InPart_X][iCam][i]	==	1	)
					{
						TempUI	=	TempUI	|	(	(unsigned int)1<<i	);
					}
				}
				STAR_Partic[iPart].GroupData[iCam][iGroup].HideFlag_1	=	TempUI;


				//--MinU
				TempI[0]	=	LabelMinU_8[iThre_InPart_X][iCam][0];
				for (	i=1;	i<32;	i++	)
				{
					if	(	TempI[0]	>	LabelMinU_8[iThre_InPart_X][iCam][i]	)
					{
						TempI[0]	=	LabelMinU_8[iThre_InPart_X][iCam][i];
					}
				}
				LabelMinU_8[iThre_InPart_X][iCam][0]	=	TempI[0];

			}
			else if	(	TempI[1]==1	)
			{

				//--MinV
				TempI[0]	=	LabelMinV_8[iThre_InPart_X][iCam][0];
				for (	i=1;	i<32;	i++	)
				{
					if	(	TempI[0]	>	LabelMinV_8[iThre_InPart_X][iCam][i]	)
					{
						TempI[0]	=	LabelMinV_8[iThre_InPart_X][iCam][i];
					}
				}
				LabelMinV_8[iThre_InPart_X][iCam][0]	=	TempI[0];

			}
			else if	(	TempI[1]==2	)
			{

				//--MaxU
				TempI[0]	=	LabelMaxU_8[iThre_InPart_X][iCam][0];
				for (	i=1;	i<32;	i++	)
				{
					if	(	TempI[0]	<	LabelMaxU_8[iThre_InPart_X][iCam][i]	)
					{
						TempI[0]	=	LabelMaxU_8[iThre_InPart_X][iCam][i];
					}
				}
				LabelMaxU_8[iThre_InPart_X][iCam][0]	=	TempI[0];

			}
			else if	(	TempI[1]==3	)
			{
				//--MaxV
				TempI[0]	=	LabelMaxV_8[iThre_InPart_X][iCam][0];
				for (	i=1;	i<32;	i++	)
				{
					if	(	TempI[0]	<	LabelMaxV_8[iThre_InPart_X][iCam][i]	)
					{
						TempI[0]	=	LabelMaxV_8[iThre_InPart_X][iCam][i];
					}
				}
				LabelMaxV_8[iThre_InPart_X][iCam][0]	=	TempI[0];
			}
			__syncthreads();


			if	(	iThre_InPart_X	==	0	&&	iThre_InPart_Y	<	8	)
			{
				//--MaxU
				for (	i=0;	i<8;	i++	)
				{
					if	(	MaxU_All	<	LabelMaxU_8[i][iThre_InPart_Y][0]	)
					{
						MaxU_All	=		LabelMaxU_8[i][iThre_InPart_Y][0];
					}
				}

				//--MinU
				for (	i=0;	i<8;	i++	)
				{
					if	(	MinU_All	>	LabelMinU_8[i][iThre_InPart_Y][0]	)
					{
						MinU_All	=		LabelMinU_8[i][iThre_InPart_Y][0];
					}
				}

				//--MaxV
				for (	i=0;	i<8;	i++	)
				{
					if	(	MaxV_All	<	LabelMaxV_8[i][iThre_InPart_Y][0]	)
					{
						MaxV_All	=		LabelMaxV_8[i][iThre_InPart_Y][0];
					}
				}

				//--MinV
				for (	i=0;	i<8;	i++	)
				{
					if	(	MinV_All	>	LabelMinV_8[i][iThre_InPart_Y][0]	)
					{
						MinV_All	=		LabelMinV_8[i][iThre_InPart_Y][0];
					}
				}
			}


			if	(	iThre_InPart_Y	>=	8	&&	iThre_InPart_Y	<	16	)
			{
				Max_U_Gro			=	LabelMaxU_8[iThre_InPart_X][iThre_InPart_Y-8][0];
				Min_U_Gro			=	LabelMinU_8[iThre_InPart_X][iThre_InPart_Y-8][0];
				Max_V_Gro			=	LabelMaxV_8[iThre_InPart_X][iThre_InPart_Y-8][0];
				Min_V_Gro			=	LabelMinV_8[iThre_InPart_X][iThre_InPart_Y-8][0];
				CounterU_Gro	=	Max_U_Gro	-	Min_U_Gro	+	1;
				CounterV_Gro	=	Max_V_Gro	-	Min_V_Gro	+	1;

				if	(	Min_U_Gro	==	255	)
				{
					Max_U_Gro			=	0;		Min_U_Gro			=	0;
					Max_V_Gro			=	0;		Min_V_Gro			=	0;
					CounterU_Gro	=	0;		CounterV_Gro	=	0;
					CounterDot_8	=	0;
				}

				P1			=	Min_U_Gro;
				P1			=	P1	&	255;
				TempUI	=	P1;

				P1			=	Min_V_Gro;
				P1			=	P1	&	255;
				TempUI	=	TempUI	|	(	(unsigned int)P1<<8		);

				P1			=	CounterU_Gro;
				P1			=	P1	&	255;
				TempUI	=	TempUI	|	(	(unsigned int)P1<<16	);

				P1			=	CounterV_Gro;
				P1			=	P1	&	255;
				TempUI	=	TempUI	|	(	(unsigned int)P1<<24	);

				STAR_Partic[iPart].GroupData[iThre_InPart_Y-8][iGroup].MinMax_Gro	=	TempUI;
				if	(	(CounterU_Gro	*	CounterV_Gro)	>=	256	)
				{
					ReturnFlag	=	1;
				}
			}
			__syncthreads();


			LabelHideFlag[iThre_InPart_X][0][iThre_InPart_Y]	=	ReturnFlag;
			__syncthreads();

			TempUI	=	0;
			if	(	iThre_InPart_Y	<	8	)
			{
				TempUI	=	TempUI	|	LabelHideFlag[iThre_InPart_X][0][iThre_InPart_Y*4	+	0];
				TempUI	=	TempUI	|	LabelHideFlag[iThre_InPart_X][0][iThre_InPart_Y*4	+	1];
				TempUI	=	TempUI	|	LabelHideFlag[iThre_InPart_X][0][iThre_InPart_Y*4	+	2];
				TempUI	=	TempUI	|	LabelHideFlag[iThre_InPart_X][0][iThre_InPart_Y*4	+	3];
			}
			__syncthreads();
			LabelHideFlag[iThre_InPart_X][0][	iThre_InPart_Y	]	=	TempUI;
			__syncthreads();

			TempUI	=	0;
			if	(	iThre_InPart_Y	<	2	)
			{
				TempUI	=	TempUI	|	LabelHideFlag[iThre_InPart_X][0][iThre_InPart_Y*4	+	0];
				TempUI	=	TempUI	|	LabelHideFlag[iThre_InPart_X][0][iThre_InPart_Y*4	+	1];
				TempUI	=	TempUI	|	LabelHideFlag[iThre_InPart_X][0][iThre_InPart_Y*4	+	2];
				TempUI	=	TempUI	|	LabelHideFlag[iThre_InPart_X][0][iThre_InPart_Y*4	+	3];
			}
			__syncthreads();
			LabelHideFlag[iThre_InPart_X][0][	iThre_InPart_Y	]	=	TempUI;
			__syncthreads();


			if	(	iThre_InPart_Y	==	0	)
			{
				LabelHideFlag[iThre_InPart_X][0][	iThre_InPart_Y	]	=	LabelHideFlag[iThre_InPart_X][0][	iThre_InPart_Y	]	|	LabelHideFlag[iThre_InPart_X][0][	iThre_InPart_Y+1	];
			}
			__syncthreads();


			TempUI	=	0;
			if	(	iThre_InPart_Y	==	0	&&	iThre_InPart_X	<	2	)
			{
				TempUI	=	TempUI		|	LabelHideFlag[iThre_InPart_X*4	+	0][0][0];
				TempUI	=	TempUI		|	LabelHideFlag[iThre_InPart_X*4	+	1][0][0];
				TempUI	=	TempUI		|	LabelHideFlag[iThre_InPart_X*4	+	2][0][0];
				TempUI	=	TempUI		|	LabelHideFlag[iThre_InPart_X*4	+	3][0][0];
			}
			__syncthreads();
			LabelHideFlag[iThre_InPart_X][0][	iThre_InPart_Y	]	=	TempUI;
			__syncthreads();


			if	(	iThre_InPart_Y	==	0	&&	iThre_InPart_X	==	0	)
			{
				LabelHideFlag[iThre_InPart_X][0][0]	=	LabelHideFlag[iThre_InPart_X][0][0]	|	LabelHideFlag[iThre_InPart_X+1][0][0];
			}
			__syncthreads();


			if	(LabelHideFlag[0][0][0]	!=	0)
			{

				for (	i=iThre_InPart_Y;	i<(32400);	i=i+32	)
				{
					STAR_Partic[iPart].Label_Opt[iThre_InPart_X][i]	=	0;
				}

				if	(	iThre_InPart_Y	==	0	&&	iThre_InPart_X	==	0	)
				{
					STAR_Partic[iPart].ReturnFlag	=	1;
				}
				return;
			}

		}

		if	(	iThre_InPart_X	==	0	&&	iThre_InPart_Y	<	8	)
		{
			STAR_Partic[iPart].MinMax_All[iThre_InPart_Y][0]	=	MaxU_All;
			STAR_Partic[iPart].MinMax_All[iThre_InPart_Y][1]	=	MinU_All;
			STAR_Partic[iPart].MinMax_All[iThre_InPart_Y][2]	=	MaxV_All;
			STAR_Partic[iPart].MinMax_All[iThre_InPart_Y][3]	=	MinV_All;

		}
		__syncthreads();


//		if	(	iThre_InPart_X	==	0	&&	iThre_InPart_Y	<	8	&&	iPart	==	0	)
//		{
//			printf(" %d %d %d %d %d %d \n",	iPart,		iThre_InPart_Y,	STAR_Partic[iPart].MinMax_All[iThre_InPart_Y][0],
//																															STAR_Partic[iPart].MinMax_All[iThre_InPart_Y][1],
//																															STAR_Partic[iPart].MinMax_All[iThre_InPart_Y][2],
//																															STAR_Partic[iPart].MinMax_All[iThre_InPart_Y][3]		);
//		}




		//====================================
		//====================================
		//	About Keypoint
		//	iTraingleはiVertexを意味している
		//====================================
		//====================================
		OriUo		=	Cam_Para[0].OriUo;
		OriVo		=	Cam_Para[0].OriVo;

		iThre_InPart_X	=	threadIdx.x	/	32;
		iThre_InPart_Y	=	threadIdx.x	%	32;
		/*	Joint Cood	*/
		for (	iCam=iThre_InPart_X;	iCam<8;	iCam=iCam+8	)
		{

			for (	iTriangle=iThre_InPart_Y;	iTriangle<24;	iTriangle=iTriangle+32	)
			{
				Vec[0].x	=	STAR_Partic[iPart].GrobalJoint_3[iTriangle].x	-	CL[iCam].x;
				Vec[0].y	=	STAR_Partic[iPart].GrobalJoint_3[iTriangle].y	-	CL[iCam].y;
				Vec[0].z	=	STAR_Partic[iPart].GrobalJoint_3[iTriangle].z	-	CL[iCam].z;

				TempF[0]	=	(	Rm[iCam][0].x	*	Vec[0].x	)	+	(	Rm[iCam][0].y	*	Vec[0].y	)	+	(	Rm[iCam][0].z	*	Vec[0].z	);
				TempF[1]	=	(	Rm[iCam][1].x	*	Vec[0].x	)	+	(	Rm[iCam][1].y	*	Vec[0].y	)	+	(	Rm[iCam][1].z	*	Vec[0].z	);
				TempF[2]	=	(	Rm[iCam][2].x	*	Vec[0].x	)	+	(	Rm[iCam][2].y	*	Vec[0].y	)	+	(	Rm[iCam][2].z	*	Vec[0].z	);

				TempF[2]	=	1/TempF[2];

				STAR_Partic[iPart].GrobalJoint_Degi_2_Cam[iCam][	iTriangle	].x	=	(	-F[iCam]	*	(	TempF[2]*TempF[0]	)	)	+	(	OriUo	);
				STAR_Partic[iPart].GrobalJoint_Degi_2_Cam[iCam][	iTriangle	].y	=	(	-F[iCam]	*	(	TempF[2]*TempF[1]	)	)	+	(	OriVo	);
			}
			__syncthreads();


			/*	Keypoint	*/
			for (	iTriangle=iThre_InPart_Y;	iTriangle<17;	iTriangle=iTriangle+32	)
			{

				if	(	(	STAR_Para[0].LandMark01_Weight[iTriangle].x	+	STAR_Para[0].LandMark01_Weight[iTriangle].y	+	STAR_Para[0].LandMark01_Weight[iTriangle].z	)	!=	0.000	)
				{
					// XComp
					Vec[0].x	=		(	STAR_Partic[iPart].Vertex[	STAR_Para[0].LandMark01_Vertex[iTriangle].x	].x	*	STAR_Para[0].LandMark01_Weight[iTriangle].x	)
											+	(	STAR_Partic[iPart].Vertex[	STAR_Para[0].LandMark01_Vertex[iTriangle].y	].x	*	STAR_Para[0].LandMark01_Weight[iTriangle].y	)
											+	(	STAR_Partic[iPart].Vertex[	STAR_Para[0].LandMark01_Vertex[iTriangle].z	].x	*	STAR_Para[0].LandMark01_Weight[iTriangle].z	)
											+	(	STAR_Partic[iPart].Vertex[	STAR_Para[0].LandMark02_Vertex[iTriangle].x	].x	*	STAR_Para[0].LandMark02_Weight[iTriangle].x	)
											+	(	STAR_Partic[iPart].Vertex[	STAR_Para[0].LandMark02_Vertex[iTriangle].y	].x	*	STAR_Para[0].LandMark02_Weight[iTriangle].y	)
											+	(	STAR_Partic[iPart].Vertex[	STAR_Para[0].LandMark02_Vertex[iTriangle].z	].x	*	STAR_Para[0].LandMark02_Weight[iTriangle].z	);

					Vec[0].x	=	Vec[0].x	*	0.5;

					// YComp
					Vec[0].y	=		(	STAR_Partic[iPart].Vertex[	STAR_Para[0].LandMark01_Vertex[iTriangle].x	].y	*	STAR_Para[0].LandMark01_Weight[iTriangle].x	)
											+	(	STAR_Partic[iPart].Vertex[	STAR_Para[0].LandMark01_Vertex[iTriangle].y	].y	*	STAR_Para[0].LandMark01_Weight[iTriangle].y	)
											+	(	STAR_Partic[iPart].Vertex[	STAR_Para[0].LandMark01_Vertex[iTriangle].z	].y	*	STAR_Para[0].LandMark01_Weight[iTriangle].z	)
											+	(	STAR_Partic[iPart].Vertex[	STAR_Para[0].LandMark02_Vertex[iTriangle].x	].y	*	STAR_Para[0].LandMark02_Weight[iTriangle].x	)
											+	(	STAR_Partic[iPart].Vertex[	STAR_Para[0].LandMark02_Vertex[iTriangle].y	].y	*	STAR_Para[0].LandMark02_Weight[iTriangle].y	)
											+	(	STAR_Partic[iPart].Vertex[	STAR_Para[0].LandMark02_Vertex[iTriangle].z	].y	*	STAR_Para[0].LandMark02_Weight[iTriangle].z	);

					Vec[0].y	=	Vec[0].y	*	0.5;

					// ZComp
					Vec[0].z	=		(	STAR_Partic[iPart].Vertex[	STAR_Para[0].LandMark01_Vertex[iTriangle].x	].z	*	STAR_Para[0].LandMark01_Weight[iTriangle].x	)
											+	(	STAR_Partic[iPart].Vertex[	STAR_Para[0].LandMark01_Vertex[iTriangle].y	].z	*	STAR_Para[0].LandMark01_Weight[iTriangle].y	)
											+	(	STAR_Partic[iPart].Vertex[	STAR_Para[0].LandMark01_Vertex[iTriangle].z	].z	*	STAR_Para[0].LandMark01_Weight[iTriangle].z	)
											+	(	STAR_Partic[iPart].Vertex[	STAR_Para[0].LandMark02_Vertex[iTriangle].x	].z	*	STAR_Para[0].LandMark02_Weight[iTriangle].x	)
											+	(	STAR_Partic[iPart].Vertex[	STAR_Para[0].LandMark02_Vertex[iTriangle].y	].z	*	STAR_Para[0].LandMark02_Weight[iTriangle].y	)
											+	(	STAR_Partic[iPart].Vertex[	STAR_Para[0].LandMark02_Vertex[iTriangle].z	].z	*	STAR_Para[0].LandMark02_Weight[iTriangle].z	);

					Vec[0].z	=	Vec[0].z	*	0.5;

					Vec[0].x	=	Vec[0].x	-	CL[iCam].x;
					Vec[0].y	=	Vec[0].y	-	CL[iCam].y;
					Vec[0].z	=	Vec[0].z	-	CL[iCam].z;

					TempF[0]	=	(	Rm[iCam][0].x	*	Vec[0].x	)	+	(	Rm[iCam][0].y	*	Vec[0].y	)	+	(	Rm[iCam][0].z	*	Vec[0].z	);
					TempF[1]	=	(	Rm[iCam][1].x	*	Vec[0].x	)	+	(	Rm[iCam][1].y	*	Vec[0].y	)	+	(	Rm[iCam][1].z	*	Vec[0].z	);
					TempF[2]	=	(	Rm[iCam][2].x	*	Vec[0].x	)	+	(	Rm[iCam][2].y	*	Vec[0].y	)	+	(	Rm[iCam][2].z	*	Vec[0].z	);

					TempF[2]	=	1/TempF[2];

					STAR_Partic[iPart].Keypoint_Repro_2_Cam[iCam][	iTriangle	].x	=	(	-F[iCam]	*	(	TempF[2]*TempF[0]	)	)	+	(	OriUo	);
					STAR_Partic[iPart].Keypoint_Repro_2_Cam[iCam][	iTriangle	].y	=	(	-F[iCam]	*	(	TempF[2]*TempF[1]	)	)	+	(	OriVo	);
				}
				else
				{
					STAR_Partic[iPart].Keypoint_Repro_2_Cam[iCam][	iTriangle	].x	=	STAR_Partic[iPart].GrobalJoint_Degi_2_Cam[iCam][	(	STAR_Para[0].LandMark01_Vertex[iTriangle].x	-	7000	)	].x;
					STAR_Partic[iPart].Keypoint_Repro_2_Cam[iCam][	iTriangle	].y	=	STAR_Partic[iPart].GrobalJoint_Degi_2_Cam[iCam][	(	STAR_Para[0].LandMark01_Vertex[iTriangle].x	-	7000	)	].y;
				}
			}
		}

		return;
	}



/*-----------------------------------------*/
/*	Internal Function in ObjectiveFunction */
/*-----------------------------------------*/
	__device__
	void	Make_Local_Label(	unsigned int	MyCounter_25,
													int						MinU,					int				MaxU,					uchar4	Counter_Local,
													float3				LineCo[],			ushort2		DegiCood[],		unsigned long long	Local_Label_Shared[]	)
	{

		int	TempI[20],	P1,P2,P3,	iU,	iU_8,	iU_Lo,	iV_8,	iV_Lo,	FlagC;
		float	TempF[5];

		// 0:Line_Min→Max,	1:Line_Min→Middle,	2:Line_Middle→Max
		P1					=	MinU-1;
		LineCo[1].x	=	LineCo[0].x	*	(	P1								-	DegiCood[	0	].x	)	+	DegiCood[	0	].y	+	0.500;
		LineCo[1].y	=	LineCo[0].y	*	(	P1								-	DegiCood[	1	].x	)	+	DegiCood[	1	].y	+	0.500;
		LineCo[1].z	=	LineCo[0].z	*	(	(DegiCood[1].x-1)	-	DegiCood[	1	].x	)	+	DegiCood[	1	].y	+	0.500;

		//	FlagCに1が入ると，TempF[0]が上になる．
		if	(	DegiCood[0].x	==	DegiCood[1].x	)
		{	FlagC	=	(	DegiCood[0].y	>	DegiCood[1].y	);	}
		else
		{	FlagC	=	(	LineCo[0].x	>	LineCo[0].y	);	}


		for (	iU=MinU;	iU<MaxU;	iU++	)
		{
			// 0:Line_Min→Max,	1:Line_Min→Middle,	2:Line_Middle→Max
			//	0は常に採用する
			LineCo[1].x	+=	LineCo[0].x;
			TempF[0]		=		LineCo[1].x;

			//	((iU-LineCo[2].y)>=0)なら，2を採用する（　(iU>=LineCo[2].y)なら，2を採用する　）
			if	(	iU	<	DegiCood[	1	].x	)
			{
				LineCo[1].y	+=	LineCo[0].y;
				TempF[1]		=		LineCo[1].y;
			}
			else
			{
				LineCo[1].z	+=	LineCo[0].z;
				TempF[1]		=		LineCo[1].z;
			}

			//	FlagCに1が入ると，TempF[0]が上になる．
			//	TempI[0]の方が値が小さい，TempI[1]の方が値が大きい
			TempI[0]	=	(int)(	TempF[			FlagC	]	);
			TempI[1]	=	(int)(	TempF[	1	-	FlagC	]	);

			if	(	TempI[0]	!=	TempI[1]	)
			{
				//--Main
				TempI[3]	=	TempI[1]	%		8;
				TempI[2]	=	TempI[1]	>>	3;

				TempI[1]	=	TempI[0]	%		8;
				TempI[0]	=	TempI[0]	>>	3;

				iU_8			=	iU	>>	3;
				iU_Lo			=	iU	%		8;
				TempI[4]	=	(	(	iU_8	-	Counter_Local.y	)	*	Counter_Local.z	);

				for (	iV_8=TempI[0];	iV_8<TempI[2]+1;	iV_8++	)
				{
					//--Label
					P1	=	(iV_8==TempI[0])*TempI[1];
					P1	=	255	<<	P1;

					P2	=	(iV_8==TempI[2])*(7-TempI[3]);
					P2	=	255	>>	P2;

					P1	=	P1 & P2;
					P2	=	TempI[4]	+	(	iV_8	-	Counter_Local.w	);

					Local_Label_Shared[	MyCounter_25	+	P2	]	=	Local_Label_Shared[	MyCounter_25	+	P2	]	|	(unsigned long long)P1<<(8*iU_Lo);
				}
			}
		}

		return;
	}

/*-----------------------------------------*/
/*	Internal Function in ObjectiveFunction */
/*-----------------------------------------*/
	__device__
	void	Input_GrobalMemory_1(	int									iCam,
															int									iPart,
															int									Local_Label_Size,
															unsigned int				GroupCounter,
															uchar4							Counter_Gro,
															unsigned long long	Local_Label_Shared[],
															STAR_Particle				STAR_Partic[]					)
	{
		int	P1,	P2,	iU_8,	iV_8,	i,	j;
		unsigned long long	TempL;

		P1	=	(	Counter_Gro.x	*	Counter_Gro.z	);
		if	(	threadIdx.x	<	P1	)
		{
			iU_8			=	(	threadIdx.x	/	Counter_Gro.z	)	+	Counter_Gro.y;
			iV_8			=	(	threadIdx.x	%	Counter_Gro.z	)	+	Counter_Gro.w;
			TempL	=	0;
			P2	=	0;
			for (	i=0;	i<GroupCounter;	i=i+1	)
			{
				TempL	=	TempL	|	Local_Label_Shared[	P2	+	threadIdx.x	];
				P2		=	P2	+	Local_Label_Size;
			}

			if	(	TempL	!=	0	)
			{
				j	=	(	iU_8*135	)	+	iV_8;
				STAR_Partic[iPart].Label_Opt[iCam][j]	=	STAR_Partic[iPart].Label_Opt[iCam][j]	|	TempL;
			}
		}
		return;
	}


/*-----------------------------------------*/
/*	Internal Function in ObjectiveFunction */
/*-----------------------------------------*/
	__device__
	void	Input_GrobalMemory_2(	int									iCam,
															int									iPart,
															int									Local_Label_Size,
															unsigned int				GroupCounter,
															uchar4							Counter_Gro,
															uchar4							Counter_Shared[],
															unsigned long long	Local_Label_Shared[],
															STAR_Particle				STAR_Partic[]					)
	{
		int	P1,	P2,	P3,	iU_8,	iV_8,	i,	j,	k;
		unsigned long long	TempL;
		uchar4							Counter_Local;


		P1	=	(	Counter_Gro.x	*	Counter_Gro.z	);
		for (	i=threadIdx.x;	i<P1;	i=i+32	)
		{
			iU_8			=	(	i	/	Counter_Gro.z	)	+	Counter_Gro.y;
			iV_8			=	(	i	%	Counter_Gro.z	)	+	Counter_Gro.w;

			TempL	=	0;
			P3		=	0;
			for (	j=0;	j<GroupCounter;	j=j+1	)
			{
				Counter_Local	=	Counter_Shared[j];

				P2	=			(	iU_8	>=	Counter_Local.y	)
							&&	(	iU_8	<=	(Counter_Local.y+Counter_Local.x-1)	)
							&&	(	iV_8	>=	Counter_Local.w	)
							&&	(	iV_8	<=	(Counter_Local.z+Counter_Local.w-1)	);
				if	(	P2	)
				{
					k			=	(	(	iU_8	-	Counter_Local.y	)	*	Counter_Local.z	)	+	(	iV_8	-	Counter_Local.w	);
					TempL	=	TempL	|	Local_Label_Shared[	P3	+	k	];
				}
				P3	=	P3	+	Local_Label_Size;
			}

			if	(TempL	!=0	)
			{
				j	=	(	iU_8*135	)	+	iV_8;
				STAR_Partic[iPart].Label_Opt[iCam][j]	=	STAR_Partic[iPart].Label_Opt[iCam][j]	|	TempL;
			}
		}
		return;
	}


/*----------------------------------------------------------------------*/
/*	Internal Function																										*/
/*----------------------------------------------------------------------*/
	__global__
	void	Objective_Function(	STAR_Parameter		STAR_Para[],	STAR_Particle		STAR_Partic[],
														Camera_Parameter	Cam_Para[],		OptP						OptPara[],
														double						Part_Dis[],		double					Part_Dis_PB[],
														double						Part_Obje[],	double					Part_PB[],
														double						rb[],					double					mb[],
														int	m	)
	{


		int	TempI[20],	i,	j,	k,	n,	iGroup,	iTriangle,	iU,	iV,	MinU,	MaxU,	MinV,	MaxV,	iCam,
				iU_8,						iV_8,								iU_Lo,					iV_Lo,
				MinU_8,					MinV_8,							MaxU_8,					MaxV_8,
				MinU_8_Gro,			MaxU_8_Gro,					MinV_8_Gro,			MaxV_8_Gro,
				Counter_U_Gro,	Counter_V_Gro,			Counter_U,			Counter_V,

				P1,	P2,	P3,							Local_Label_Size	=	{25},	FlagC,	Rank[3],
				iPart,	iThre_InPart_X,	iThre_InPart_Y,	iThre_InPart_Z,	HideFlag;



		float		TempF[5]	=	{0.00};

		ushort2	DegiCood[3];
		float3	LineCo[2];


		unsigned long long	DegiCoodPack,	TempL={0},	TempL2={0};
		unsigned int				TempUI[5]={0},	KindFlag;
		__shared__	unsigned long long	Local_Label_Shared25[32*25];

		uchar4							Counter_Local,	Counter_Gro;
		__shared__	uchar4	Counter_Shared[32];



		unsigned int	GroupCounter,	MyCounter,	MyCounter_25;



		//	Counter_Local.x	=	Counter_U;	Counter_Gro.x	=	Counter_U_Gro;
		//	Counter_Local.y	=	MinU_8;			Counter_Gro.y	=	MinU_8_Gro;
		//	Counter_Local.z	=	Counter_V;	Counter_Gro.z	=	Counter_V_Gro;
		//	Counter_Local.w	=	MinV_8;			Counter_Gro.w	=	MinV_8_Gro;

		long	T[50];

		iPart					=	blockIdx.x	/	8;
		iCam					=	blockIdx.x	%	8;

//		printf(	"1:AA %d %d %d %d %d %d %d %d \n",	iPart,	iCam,	threadIdx.x,	blockDim.x,	blockIdx.x,	threadIdx.y,	blockDim.y,	blockIdx.y	);


		if	(	STAR_Partic[iPart].ReturnFlag	==	1	)
		{
			return;
		}


		for (	iGroup=0;	iGroup<431;	iGroup++	)
		{

			T[0]	=	clock(	);

			TempUI[0]		=	STAR_Partic[iPart].GroupData[iCam][iGroup].HideFlag_1;
			TempUI[1]		=	STAR_Partic[iPart].GroupData[iCam][iGroup].MinMax_Gro;

			LineCo[0]	=	STAR_Partic[iPart].GroupData[iCam][iGroup].LineCo[threadIdx.x];
			TempL			=	STAR_Partic[iPart].GroupData[iCam][iGroup].HideData[threadIdx.x];


			T[1]	=	clock(	);



			T[2]	=	clock(	);

			if	(	(	TempUI[0]	!=	0	)	==	1	)
			{
				T[10]	=	clock(	);


				//==================
				//--GrobalCounter
				//==================

				//	CounterU_Gro
				Counter_Gro.x		=	(	TempUI[1]>>16	);

				//	MinU_Gro
				Counter_Gro.y		=	(	TempUI[1]			);

				//	CounterV_Gro
				Counter_Gro.z		=	(	TempUI[1]>>24	);

				//	MinV_Gro
				Counter_Gro.w		=	(	TempUI[1]>>8	);

				KindFlag	=	(	(	Counter_Gro.x	*	Counter_Gro.z	)	<=	25	);
				if	(	KindFlag	)
				{	Local_Label_Size	=	(	Counter_Gro.x	*	Counter_Gro.z	);	}
				else
				{	Local_Label_Size	=	25;	}

				T[11]	=	clock(	);

				//--HideFlag
				HideFlag	=	(	(	TempUI[0]	>>	threadIdx.x	)	&	1	);


				//--HideFlag
				GroupCounter	=	TempUI[0];

				GroupCounter	=		(		GroupCounter						&	0b01010101010101010101010101010101	)
												+	(	(	GroupCounter	>>	1		)	&	0b01010101010101010101010101010101	);

				GroupCounter	=		(		GroupCounter						&	0b00110011001100110011001100110011	)
												+	(	(	GroupCounter	>>	2		)	&	0b00110011001100110011001100110011	);

				GroupCounter	=		(		GroupCounter						&	0b00001111000011110000111100001111	)
												+	(	(	GroupCounter	>>	4		)	&	0b00001111000011110000111100001111	);

				GroupCounter	=		(		GroupCounter						&	0b00000000111111110000000011111111	)
												+	(	(	GroupCounter	>>	8		)	&	0b00000000111111110000000011111111	);

				GroupCounter	=		(		GroupCounter						&	0b00000000000000001111111111111111	)
												+	(	(	GroupCounter	>>	16	)	&	0b00000000000000001111111111111111	);

				T[12]	=	clock(	);

				if	(	HideFlag	==	1	)
				{

					T[20]	=	clock(	);

					MyCounter			=	TempUI[0]	<<	(31-threadIdx.x);

					MyCounter			=			(	MyCounter								&	0b01010101010101010101010101010101	)
													+	(	(	MyCounter	>>	1		)			&	0b01010101010101010101010101010101	);

					MyCounter			=			(	MyCounter								&	0b00110011001100110011001100110011	)
													+	(	(	MyCounter	>>	2		)			&	0b00110011001100110011001100110011	);

					MyCounter			=			(	MyCounter								&	0b00001111000011110000111100001111	)
													+	(	(	MyCounter	>>	4		)			&	0b00001111000011110000111100001111	);

					MyCounter			=			(	MyCounter								&	0b00000000111111110000000011111111	)
													+	(	(	MyCounter	>>	8		)			&	0b00000000111111110000000011111111	);

					MyCounter			=			(	MyCounter								&	0b00000000000000001111111111111111	)
													+	(	(	MyCounter	>>	16	)			&	0b00000000000000001111111111111111	);

					MyCounter			=	MyCounter	-	1;
					MyCounter_25	=	MyCounter	*	Local_Label_Size;

					T[21]	=	clock(	);

					//================
					//--Initialize
					//================
					for (	i=0;	i<Local_Label_Size;	i++	)
					{
						Local_Label_Shared25[MyCounter_25	+	i	]	=	0;
					}


					//========================
					//	Main Degitize Cood
					//========================
					//--MinU
					T[22]	=	clock(	);

					MinU	=	TempL	&	2047;
					T[23]	=	clock(	);

					//--MinV
					MinV	=	(	TempL	>>	11)	&	2047;

					//--MaxU
					MaxU	=	MinU	+	(	(	TempL	>>	22	)	&	255	);

					//--MaxV
					MaxV	=	MinV	+	(	(	TempL	>>	30	)	&	255	);


					//--MinU_Point(Total)
					DegiCood[	0	].x	=	MinU;
					DegiCood[	0	].y	=	MinV	+	(	(	TempL	>>	38	)	&	255	);


					//	MidU_Point
					DegiCood[	1	].x	=	MinU	+	(	(	TempL	>>	46	)	&	255	);
					DegiCood[	1	].y	=	MinV	+	(	(	TempL	>>	54	)	&	255	);

					TempI[10]		=	(	DegiCood[	0	].x	!=	DegiCood[	1	].x	);
					MinU				=	DegiCood[	0	].x	+	TempI[10];
					T[24]	=	clock(	);

					//--Counter_Local.x	=	Counter_U;	Counter_Gro.x	=	Counter_U_Gro;
					//--Counter_Local.y	=	MinU_8;			Counter_Gro.y	=	MinU_8_Gro;
					//--Counter_Local.z	=	Counter_V;	Counter_Gro.z	=	Counter_V_Gro;
					//--Counter_Local.w	=	MinV_8;			Counter_Gro.w	=	MinV_8_Gro;

					if	(	KindFlag	!=	0	)
					{
						Counter_Local	=	Counter_Gro;
					}
					else
					{
						Counter_Local.y	=	(	MinU	>>	3	);
						Counter_Local.w	=	(	MinV	>>	3	);
						Counter_Local.x	=	(	MaxU	>>	3	)	-	Counter_Local.y	+	1;
						Counter_Local.z	=	(	MaxV	>>	3	)	-	Counter_Local.w	+	1;

						Counter_Shared[MyCounter]	=	Counter_Local;
					}

					T[25]	=	clock();
					Make_Local_Label(	MyCounter_25,	MinU,	MaxU,	Counter_Local,	LineCo,	DegiCood,	Local_Label_Shared25	);
					T[27]	=	clock(	);
				}
				__syncthreads();

				T[30]	=	clock(	);

				if			(	KindFlag	==	0	)
				{
					Input_GrobalMemory_2(	iCam,	iPart,	Local_Label_Size,	GroupCounter,	Counter_Gro,	Counter_Shared,		Local_Label_Shared25,	STAR_Partic		);
				}
				else if	(	KindFlag	==	1	)
				{
					Input_GrobalMemory_1(	iCam,	iPart,	Local_Label_Size,	GroupCounter,	Counter_Gro,	Local_Label_Shared25,	STAR_Partic		);
				}
				T[31]	=	clock();



				if	(	iPart	==	0	&&	threadIdx.x	==	0	&&	iCam	==	0	&&	Counter_Gro.x	*	Counter_Gro.z	*	GroupCounter<=800	)
				{
//					printf(	"1:AA01 %d \n",	T[1]-T[0]	);
//					printf(	"1:AA02 %d \n",	T[2]-T[1]	);
//					printf(	"1:BB %d \n",	T[11]-T[10]	);
//					printf(	"1:CC %d \n",	T[12]-T[11]	);
//
//					printf(	"1:DD %d \n",	T[21]-T[20]	);
//					printf(	"1:EE %d \n",	T[22]-T[21]	);
//					printf(	"1:FF %d \n",	T[23]-T[22]	);
//					printf(	"1:GG %d \n",	T[24]-T[23]	);
//					printf(	"1:HH %d \n",	T[25]-T[24]	);
//					printf(	"1:II %d \n",	T[26]-T[25]	);
//					printf(	"1:JJ %d \n",	T[27]-T[26]	);
//					printf(	"1:KK %d %d %d %d \n",	T[31]-T[30],	Counter_Gro.x	*	Counter_Gro.z,	GroupCounter,	Counter_Gro.x	*	Counter_Gro.z	*	GroupCounter	);
				}

			}
		}

		return;
	}



/*----------------------------------------------------------------------*/
/*	Internal Function																										*/
/*----------------------------------------------------------------------*/
	__global__
	void	Obje_CalFunc(	STAR_Particle		STAR_Partic[],	OptP						OptPara[],
											Camera_Parameter	Cam_Para[],
											double			Part_Dis[],						double						Part_Dis_PB[],
											double			Part_Obje[],					double						Part_PB[],
											int				iCam,										int	iImage	)
	{


		int	iU_Max,	iU_Min,	iV_Max,	iV_Min,	iU_8,	iV_8,	iThre_X,	iThre_Y;
		int	i,	j,	TempI,	iBit;
		unsigned int	TempUI;

		float	TempF[3]	=	{0.00};

		int	iPart,	iPart_Lo;

		iPart			=	blockIdx.x;
		iPart_Lo	=	threadIdx.x;
		long	T[20];
		float	TempObje	=	{0.00};

		unsigned long long	TempULL[5];
		ulonglong4					TempUL4;

/*
		__shared__	ulonglong2	TempUL_Shared[128];
*/
		__shared__	ulonglong3	TempUL_Shared[256];


		__shared__	ulonglong3	TempUL_Shared_Comf[256];


		if	(	STAR_Partic[iPart].ReturnFlag	==	1	)
		{

			if	(	iPart_Lo	==	0	)
			{
				Part_Obje[iPart]	=	10.00;
			}
			return;
		}



		TempObje	=	0.00;
		for (	iCam=0;	iCam<8;	iCam++	)
		{
			iU_Max	=	STAR_Partic[iPart].MinMax_All[iCam][0];
			iU_Min	=	STAR_Partic[iPart].MinMax_All[iCam][1];
			iV_Max	=	STAR_Partic[iPart].MinMax_All[iCam][2];
			iV_Min	=	STAR_Partic[iPart].MinMax_All[iCam][3];

			if	(	iU_Max	<	Cam_Para[iCam].MinMax_All[iImage][0]	)
			{
				iU_Max	=	Cam_Para[iCam].MinMax_All[iImage][0];
			}

			if	(	iU_Min	>	Cam_Para[iCam].MinMax_All[iImage][1]	)
			{
				iU_Min	=	Cam_Para[iCam].MinMax_All[iImage][1];
			}

			if	(	iV_Max	<	Cam_Para[iCam].MinMax_All[iImage][2]	)
			{
				iV_Max	=	Cam_Para[iCam].MinMax_All[iImage][2];
			}

			if	(	iV_Min	>	Cam_Para[iCam].MinMax_All[iImage][3]	)
			{
				iV_Min	=	Cam_Para[iCam].MinMax_All[iImage][3];
			}

			T[0]	=	clock();
			TempUL_Shared[iPart_Lo].x	=	0;	TempUL_Shared[iPart_Lo].y	=	0;	TempUL_Shared[iPart_Lo].z	=	0;

			iThre_X	=	threadIdx.x	%	16;
			iThre_Y	=	threadIdx.x	/	16;
			for (	iU_8=iU_Min	+	iThre_Y;	iU_8<iU_Max+1;	iU_8=iU_8+16	)
			{
				for (	iV_8=iV_Min	+	iThre_X;	iV_8<iV_Max+1;	iV_8=iV_8+16	)
				{
					i	=	(iU_8*135)	+	iV_8;

					TempULL[0]	=	STAR_Partic[iPart].Label_Opt[iCam][i];
					TempULL[1]	=	Cam_Para[iCam].Label[iImage][i];

					TempUL4.x	=	TempULL[0]	&	TempULL[1];
					TempUL4.y	=	TempUL4.x	^	TempULL[0];
					TempUL4.z	=	TempUL4.x	^	TempULL[1];

					if	(	TempULL[0]	!=	0	)
					{
						STAR_Partic[iPart].Label_Opt[iCam][i]	=	0;
					}

					if	(	TempUL4.z	!=	0	)
					{
						TempUL4.z	=			(	TempUL4.z						&	0b0101010101010101010101010101010101010101010101010101010101010101	)
												+	(	(	TempUL4.z	>>	1		)	&	0b0101010101010101010101010101010101010101010101010101010101010101	);

						TempUL4.z	=			(	TempUL4.z						&	0b0011001100110011001100110011001100110011001100110011001100110011	)
												+	(	(	TempUL4.z	>>	2		)	&	0b0011001100110011001100110011001100110011001100110011001100110011	);

						TempUL4.z	=			(	TempUL4.z						&	0b0000111100001111000011110000111100001111000011110000111100001111	)
												+	(	(	TempUL4.z	>>	4		)	&	0b0000111100001111000011110000111100001111000011110000111100001111	);

						TempUL4.z	=			(	TempUL4.z						&	0b0000000011111111000000001111111100000000111111110000000011111111	)
												+	(	(	TempUL4.z	>>	8		)	&	0b0000000011111111000000001111111100000000111111110000000011111111	);

						TempUL4.z	=			(	TempUL4.z						&	0b0000000000000000111111111111111100000000000000001111111111111111	)
												+	(	(	TempUL4.z	>>	16	)	&	0b0000000000000000111111111111111100000000000000001111111111111111	);

						TempUL4.z	=			(	TempUL4.z						&	0b0000000000000000000000000000000011111111111111111111111111111111	)
												+	(	(	TempUL4.z	>>	32	)	&	0b0000000000000000000000000000000011111111111111111111111111111111	);

						TempUL_Shared[iPart_Lo].z	+=	(	TempUL4.z	);
					}

					if	(	TempUL4.y	!=	0	)
					{
						TempUL4.y	=			(	TempUL4.y						&	0b0101010101010101010101010101010101010101010101010101010101010101	)
												+	(	(	TempUL4.y	>>	1		)	&	0b0101010101010101010101010101010101010101010101010101010101010101	);

						TempUL4.y	=			(	TempUL4.y						&	0b0011001100110011001100110011001100110011001100110011001100110011	)
												+	(	(	TempUL4.y	>>	2		)	&	0b0011001100110011001100110011001100110011001100110011001100110011	);

						TempUL4.y	=			(	TempUL4.y						&	0b0000111100001111000011110000111100001111000011110000111100001111	)
												+	(	(	TempUL4.y	>>	4		)	&	0b0000111100001111000011110000111100001111000011110000111100001111	);

						TempUL4.y	=			(	TempUL4.y						&	0b0000000011111111000000001111111100000000111111110000000011111111	)
												+	(	(	TempUL4.y	>>	8		)	&	0b0000000011111111000000001111111100000000111111110000000011111111	);

						TempUL4.y	=			(	TempUL4.y						&	0b0000000000000000111111111111111100000000000000001111111111111111	)
												+	(	(	TempUL4.y	>>	16	)	&	0b0000000000000000111111111111111100000000000000001111111111111111	);

						TempUL4.y	=			(	TempUL4.y						&	0b0000000000000000000000000000000011111111111111111111111111111111	)
												+	(	(	TempUL4.y	>>	32	)	&	0b0000000000000000000000000000000011111111111111111111111111111111	);

						TempUL_Shared[iPart_Lo].y	+=	(	TempUL4.y	);
					}

					if	(	TempUL4.x	!=	0	)
					{
						TempUL4.x	=			(	TempUL4.x						&	0b0101010101010101010101010101010101010101010101010101010101010101	)
												+	(	(	TempUL4.x	>>	1		)	&	0b0101010101010101010101010101010101010101010101010101010101010101	);

						TempUL4.x	=			(	TempUL4.x						&	0b0011001100110011001100110011001100110011001100110011001100110011	)
												+	(	(	TempUL4.x	>>	2		)	&	0b0011001100110011001100110011001100110011001100110011001100110011	);

						TempUL4.x	=			(	TempUL4.x						&	0b0000111100001111000011110000111100001111000011110000111100001111	)
												+	(	(	TempUL4.x	>>	4		)	&	0b0000111100001111000011110000111100001111000011110000111100001111	);

						TempUL4.x	=			(	TempUL4.x						&	0b0000000011111111000000001111111100000000111111110000000011111111	)
												+	(	(	TempUL4.x	>>	8		)	&	0b0000000011111111000000001111111100000000111111110000000011111111	);

						TempUL4.x	=			(	TempUL4.x						&	0b0000000000000000111111111111111100000000000000001111111111111111	)
												+	(	(	TempUL4.x	>>	16	)	&	0b0000000000000000111111111111111100000000000000001111111111111111	);

						TempUL4.x	=			(	TempUL4.x						&	0b0000000000000000000000000000000011111111111111111111111111111111	)
												+	(	(	TempUL4.x	>>	32	)	&	0b0000000000000000000000000000000011111111111111111111111111111111	);

						TempUL_Shared[iPart_Lo].x	+=	(	2*TempUL4.x	);
					}
				}
			}
			__syncthreads();
			T[1]	=	clock();

			TempUL4.x	=	0;		TempUL4.y	=	0;		TempUL4.z	=	0;
			if	(iPart_Lo	<	64)
			{
				TempUL4.x	=	TempUL4.x	+	TempUL_Shared[iPart_Lo*4+0].x;
				TempUL4.x	=	TempUL4.x	+	TempUL_Shared[iPart_Lo*4+1].x;
				TempUL4.x	=	TempUL4.x	+	TempUL_Shared[iPart_Lo*4+2].x;
				TempUL4.x	=	TempUL4.x	+	TempUL_Shared[iPart_Lo*4+3].x;

				TempUL4.y	=	TempUL4.y	+	TempUL_Shared[iPart_Lo*4+0].y;
				TempUL4.y	=	TempUL4.y	+	TempUL_Shared[iPart_Lo*4+1].y;
				TempUL4.y	=	TempUL4.y	+	TempUL_Shared[iPart_Lo*4+2].y;
				TempUL4.y	=	TempUL4.y	+	TempUL_Shared[iPart_Lo*4+3].y;

				TempUL4.z	=	TempUL4.z	+	TempUL_Shared[iPart_Lo*4+0].z;
				TempUL4.z	=	TempUL4.z	+	TempUL_Shared[iPart_Lo*4+1].z;
				TempUL4.z	=	TempUL4.z	+	TempUL_Shared[iPart_Lo*4+2].z;
				TempUL4.z	=	TempUL4.z	+	TempUL_Shared[iPart_Lo*4+3].z;
			}
			__syncthreads();
			TempUL_Shared[iPart_Lo].x	=	TempUL4.x;
			TempUL_Shared[iPart_Lo].y	=	TempUL4.y;
			TempUL_Shared[iPart_Lo].z	=	TempUL4.z;
			__syncthreads();

			TempUL4.x	=	0;		TempUL4.y	=	0;		TempUL4.z	=	0;
			if	(iPart_Lo	<	16)
			{
				TempUL4.x	=	TempUL4.x	+	TempUL_Shared[iPart_Lo*4+0].x;
				TempUL4.x	=	TempUL4.x	+	TempUL_Shared[iPart_Lo*4+1].x;
				TempUL4.x	=	TempUL4.x	+	TempUL_Shared[iPart_Lo*4+2].x;
				TempUL4.x	=	TempUL4.x	+	TempUL_Shared[iPart_Lo*4+3].x;

				TempUL4.y	=	TempUL4.y	+	TempUL_Shared[iPart_Lo*4+0].y;
				TempUL4.y	=	TempUL4.y	+	TempUL_Shared[iPart_Lo*4+1].y;
				TempUL4.y	=	TempUL4.y	+	TempUL_Shared[iPart_Lo*4+2].y;
				TempUL4.y	=	TempUL4.y	+	TempUL_Shared[iPart_Lo*4+3].y;

				TempUL4.z	=	TempUL4.z	+	TempUL_Shared[iPart_Lo*4+0].z;
				TempUL4.z	=	TempUL4.z	+	TempUL_Shared[iPart_Lo*4+1].z;
				TempUL4.z	=	TempUL4.z	+	TempUL_Shared[iPart_Lo*4+2].z;
				TempUL4.z	=	TempUL4.z	+	TempUL_Shared[iPart_Lo*4+3].z;
			}
			__syncthreads();
			TempUL_Shared[iPart_Lo].x	=	TempUL4.x;
			TempUL_Shared[iPart_Lo].y	=	TempUL4.y;
			TempUL_Shared[iPart_Lo].z	=	TempUL4.z;
			__syncthreads();

			TempUL4.x	=	0;		TempUL4.y	=	0;		TempUL4.z	=	0;
			if	(iPart_Lo	<	4)
			{
				TempUL4.x	=	TempUL4.x	+	TempUL_Shared[iPart_Lo*4+0].x;
				TempUL4.x	=	TempUL4.x	+	TempUL_Shared[iPart_Lo*4+1].x;
				TempUL4.x	=	TempUL4.x	+	TempUL_Shared[iPart_Lo*4+2].x;
				TempUL4.x	=	TempUL4.x	+	TempUL_Shared[iPart_Lo*4+3].x;

				TempUL4.y	=	TempUL4.y	+	TempUL_Shared[iPart_Lo*4+0].y;
				TempUL4.y	=	TempUL4.y	+	TempUL_Shared[iPart_Lo*4+1].y;
				TempUL4.y	=	TempUL4.y	+	TempUL_Shared[iPart_Lo*4+2].y;
				TempUL4.y	=	TempUL4.y	+	TempUL_Shared[iPart_Lo*4+3].y;

				TempUL4.z	=	TempUL4.z	+	TempUL_Shared[iPart_Lo*4+0].z;
				TempUL4.z	=	TempUL4.z	+	TempUL_Shared[iPart_Lo*4+1].z;
				TempUL4.z	=	TempUL4.z	+	TempUL_Shared[iPart_Lo*4+2].z;
				TempUL4.z	=	TempUL4.z	+	TempUL_Shared[iPart_Lo*4+3].z;
			}
			__syncthreads();
			TempUL_Shared[iPart_Lo].x	=	TempUL4.x;
			TempUL_Shared[iPart_Lo].y	=	TempUL4.y;
			TempUL_Shared[iPart_Lo].z	=	TempUL4.z;
			__syncthreads();

			TempUL4.x	=	0;		TempUL4.y	=	0;		TempUL4.z	=	0;
			if	(iPart_Lo	<	1)
			{
				TempUL4.x	=	TempUL4.x	+	TempUL_Shared[iPart_Lo*4+0].x;
				TempUL4.x	=	TempUL4.x	+	TempUL_Shared[iPart_Lo*4+1].x;
				TempUL4.x	=	TempUL4.x	+	TempUL_Shared[iPart_Lo*4+2].x;
				TempUL4.x	=	TempUL4.x	+	TempUL_Shared[iPart_Lo*4+3].x;

				TempUL4.y	=	TempUL4.y	+	TempUL_Shared[iPart_Lo*4+0].y;
				TempUL4.y	=	TempUL4.y	+	TempUL_Shared[iPart_Lo*4+1].y;
				TempUL4.y	=	TempUL4.y	+	TempUL_Shared[iPart_Lo*4+2].y;
				TempUL4.y	=	TempUL4.y	+	TempUL_Shared[iPart_Lo*4+3].y;

				TempUL4.z	=	TempUL4.z	+	TempUL_Shared[iPart_Lo*4+0].z;
				TempUL4.z	=	TempUL4.z	+	TempUL_Shared[iPart_Lo*4+1].z;
				TempUL4.z	=	TempUL4.z	+	TempUL_Shared[iPart_Lo*4+2].z;
				TempUL4.z	=	TempUL4.z	+	TempUL_Shared[iPart_Lo*4+3].z;
			}
			__syncthreads();
			TempUL_Shared[iPart_Lo].x	=	TempUL4.x;
			TempUL_Shared[iPart_Lo].y	=	TempUL4.y;
			TempUL_Shared[iPart_Lo].z	=	TempUL4.z;
			__syncthreads();

			T[2]	=	clock();
			if	(iPart_Lo	==	0)
			{

				TempUL4.x	=	0;		TempUL4.y	=	0;		TempUL4.z	=	0;

				TempUL4.x	=	TempUL_Shared[iPart_Lo].x;
				TempUL4.y	=	TempUL_Shared[iPart_Lo].y;
				TempUL4.z	=	TempUL_Shared[iPart_Lo].z;

				TempF[2]	=	0.00;
				for (	i=0;	i<17;	i++	)
				{
					if	(	Cam_Para[iCam].Keypoint[iImage][i*2+0]	!=	-1	)
					{
						TempF[0]	=	Cam_Para[iCam].Keypoint[iImage][i*2+0]	-	STAR_Partic[iPart].Keypoint_Repro_2_Cam[iCam][	i	].x;
						TempF[1]	=	Cam_Para[iCam].Keypoint[iImage][i*2+1]	-	STAR_Partic[iPart].Keypoint_Repro_2_Cam[iCam][	i	].y;
						TempF[2]	=	TempF[2]	+	(	(TempF[0]*TempF[0])	+	(TempF[1]*TempF[1]))*3.14;
					}
				}
				TempF[2]	=	TempF[2]/17.0;
				TempF[2]	=	(	(	(double)(1*TempUL4.y)	+	(double)TempUL4.z	+	(double)TempF[2]	)	/	(TempUL4.x+TempUL4.y+TempUL4.z)	);

				STAR_Partic[iPart].TempObje	=	STAR_Partic[iPart].TempObje	+	TempF[2];
			}
			__syncthreads();

			T[3]	=	clock();
		}

		if	(	iPart_Lo	==	0	&&	iImage	==	(	Cam_Para[0].nImage-1	)	)
		{

			STAR_Partic[iPart].TempObje		=	(	STAR_Partic[iPart].TempObje	/	(	Cam_Para[0].nCam	*	Cam_Para[0].nImage	)	);
			TempF[0]											=	0.05;
			TempF[1]											=	STAR_Partic[iPart].TempObje_Betas;
//			TempF[1]											=	TempF[1]	*	TempF[1];
			Part_Obje[iPart]							=	STAR_Partic[iPart].TempObje	+	(	TempF[1]*TempF[0]	);

			if	(	Part_PB[iPart]	>	Part_Obje[iPart]	)
			{
				Part_PB[iPart]	=	Part_Obje[iPart];

				TempI	=	(	iPart	*	OptPara[0].nValue	);
				for (	j=0;	j<OptPara[0].nValue;	j++	)
				{
					Part_Dis_PB[	TempI+j	]		=	Part_Dis[	TempI+j	];
				}
			}
		}


		return;
	}
























//		for (	i=threadIdx.x;	i<STAR_Para[0].nVertex;	i=i+blockDim.x	)
//		{
//			RealCood[0]	=	STAR_Partic[iPart].Vertex[i];
//
//			TempUL	=	0;	TempUL2	=	0;
//			P3			=	0;	Counter	=	0;
//			for (	iCam=0;	iCam<8;	iCam++	)
//			{
//				Vec[1].x	=	RealCood[0].x	-	CL[iCam].x;
//				Vec[1].y	=	RealCood[0].y	-	CL[iCam].y;
//				Vec[1].z	=	RealCood[0].z	-	CL[iCam].z;
//
//				TempF[0]	=	(	Rm[iCam][0].x	*	Vec[1].x	)	+	(	Rm[iCam][0].y	*	Vec[1].y	)	+	(	Rm[iCam][0].z	*	Vec[1].z	);
//				TempF[1]	=	(	Rm[iCam][1].x	*	Vec[1].x	)	+	(	Rm[iCam][1].y	*	Vec[1].y	)	+	(	Rm[iCam][1].z	*	Vec[1].z	);
//				TempF[2]	=	(	Rm[iCam][2].x	*	Vec[1].x	)	+	(	Rm[iCam][2].y	*	Vec[1].y	)	+	(	Rm[iCam][2].z	*	Vec[1].z	);
//
//				TempF[2]	=	-F[iCam]	/	TempF[2];
//
//				TempI[0]	=	(int)(	(	TempF[2]*TempF[0]	)	+	OriUo	);
//				TempI[1]	=	(int)(	(	TempF[2]*TempF[1]	)	+	OriVo	);
//
//				//---------------------
//				//--Part1
//				//画面の外は
//				//---------------------
//				j	=	(TempI[0]<=0)	+	(TempI[0]>1919)	+	(TempI[1]<=0)	+	(TempI[1]>1079);
//				if	(j	!=	0)
//				{
//					TempI[0]	=	0;
//					TempI[1]	=	0;
//				}
//
//				P1			=	TempI[0];
//				P1			=	P1	&	2047;
//				TempUL	=	TempUL	|	(	(unsigned long long)P1<<(P3*11)	);
//
//				P2			=	TempI[1];
//				P2			=	P2	&	2047;
//				TempUL2	=	TempUL2	|	(	(unsigned long long)P2<<(P3*11)	);
//
//				P3++;
//				if	(	P3==5	)
//				{
//					STAR_Partic[iPart].Vertex_Degi[i][Counter].x	=	TempUL;
//					STAR_Partic[iPart].Vertex_Degi[i][Counter].y	=	TempUL2;
//					P3	=	0;			Counter++;
//					TempUL	=	0;	TempUL2	=	0;
//				}
//			}
//			STAR_Partic[iPart].Vertex_Degi[i][Counter].x	=	TempUL;
//			STAR_Partic[iPart].Vertex_Degi[i][Counter].y	=	TempUL2;
//
//
//		}
//		__syncthreads();



