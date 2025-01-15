function	FitOpt();

close all
clear all

	global	Shape_Dirs	V_Temp	J_Regressor		Pose_Dirs		Kintree_Table		Weights		Cam	BasePara		V_Point	Input	nImage	nCam	NewV_Point_32	LandMark_Ori R1_Opt R2_Opt

	PathName	=	'../ModelFile/';
	load(	[	PathName 'J_Regressor.mat'		]	)
	load(	[	PathName 'Kintree_Table.mat'	]	)
	load(	[	PathName 'Pose_Dirs.mat'			]	)
	load(	[	PathName 'Shape_Dirs.mat'			]	)
	load(	[	PathName 'V_Temp.mat'					]	)
	load(	[	PathName 'Weights.mat'				]	)
	load(	[	PathName 'V_Point.mat'				]	)
	load(	[	PathName 'NewV_Point.mat'			]	)

	SubName			=	'DemoTry';
	TryPathName	=	'../../DemoData/';

	CUDA_InputPathName	=	['./CUDA_Input/'];
	LandMark_Ori				=	readmatrix(	[	TryPathName 'DemoLandMark.csv'	]	);


	R1_Stick	=	[	1,	2,	3,	4,	5,	6,	7,			9,			11,			13,			15,	16,			18,			20,			22,	23,			25,			27,			29,	30,	31,			33,			35,			37,	38	];
	R2_Stick	=	[	1,	2,	3,	4,	5,	6,			8,			10,			12,			14,	15,			17,			19,			21,	22,			24,			26,			28,	29,	30,			32,			34,			36,	37,	38	];

	R1_Opt	=	[			2,	3,	4,	5,	6,					9,			11,			13,					16,			18,			20,					23,			25,											31,			33											];
	R2_Opt	=	[			2,	3,	4,	5,	6,							10,			12,			14,					17,			19,			21,					24,			26,											32,			34									];


	%-=-=-=-=-=-=-=-=-=-=-=
	%--LandMark
	%-=-=-=-=-=-=-=-=-=-=-=

	%-------------
	%--LandMark01
	%-------------
	SaveInt	=	LandMark_Ori(R1_Opt,1:3);
	SaveInt	=	[	SaveInt(1:11,:);	[	7003,	7003,	7003	];	SaveInt(12:13,:);	[	7002,	7002,	7002	];	SaveInt(14:15,:)	];

	[Tate,Yoko]	=	size(	SaveInt	);
	SaveInt			=	reshape(	SaveInt',	[Tate*Yoko,1]	);
	SaveInt			=	[SaveInt;10000];

	FileName	=	[	CUDA_InputPathName	'LandMark01_Vertex.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveInt,'int');
	fclose(ID);

	%-------------
	%--LandMark02
	%-------------
	SaveInt	=	LandMark_Ori(R2_Opt,1:3);
	SaveInt	=	[	SaveInt(1:11,:);	[	7003,	7003,	7003	];	SaveInt(12:13,:);	[	7002,	7002,	7002	];	SaveInt(14:15,:)	];

	[Tate,Yoko]	=	size(	SaveInt	);
	SaveInt			=	reshape(	SaveInt',	[Tate*Yoko,1]	);
	SaveInt			=	[SaveInt;10000];

	FileName	=	[	CUDA_InputPathName	'LandMark02_Vertex.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveInt,'int');
	fclose(ID);


	%--------------------
	%--LandMark_Weight01
	%--------------------
	SaveSingle	=	LandMark_Ori(R1_Opt,4:6);
	SaveSingle	=	[	SaveSingle(1:11,:);	[	0.0,	0.0,	0.0	];	SaveSingle(12:13,:);	[	0.0,	0.0,	0.0	];	SaveSingle(14:15,:)	];

	[Tate,Yoko]	=	size(	SaveSingle	);
	SaveSingle	=	reshape(	SaveSingle',	[Tate*Yoko,1]	);
	SaveSingle	=	[SaveSingle;10^10];

	FileName	=	[	CUDA_InputPathName	'LandMark01_Weight.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveSingle,'single');
	fclose(ID);

	%--------------------
	%--LandMark_Weight02
	%--------------------
	SaveSingle	=	LandMark_Ori(R2_Opt,4:6);
	SaveSingle	=	[	SaveSingle(1:11,:);	[	0.0,	0.0,	0.0	];	SaveSingle(12:13,:);	[	0.0,	0.0,	0.0	];	SaveSingle(14:15,:)	];

	[Tate,Yoko]	=	size(	SaveSingle	);
	SaveSingle	=	reshape(	SaveSingle',	[Tate*Yoko,1]	);
	SaveSingle	=	[SaveSingle;10^10];


	FileName	=	[	CUDA_InputPathName	'LandMark02_Weight.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveSingle,'single');
	fclose(ID);

	%-=-=-=-=-=-=-=-=-=-=-=
	%--Shape_Dirs
	%-=-=-=-=-=-=-=-=-=-=-=
	SaveSingle	=	zeros(	6890*3*300,	1	);
	Counter	=	0;
	for	j	=	1:6890
		for	i	=	1:3
			R	=	(	Counter*300	)+1	:	(	Counter*300	)+300;
			SaveSingle(R)	=	Shape_Dirs(j,i,:);
			Counter	=	Counter	+	1;
		end
	end

	SaveSingle	=	[	SaveSingle;	10^10	];

	FileName	=	[	CUDA_InputPathName	'Shape_Dirs.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveSingle,'single');
	fclose(ID);

	%-=-=-=-=-=-=-=-=-=-=-=
	%--Shape_Dirs_Ver2
	%-=-=-=-=-=-=-=-=-=-=-=
	nVertex					=	6890;
	nVertex3				=	nVertex		*	3;
	nVertex3_4			=	nVertex		*	3	*	4;
	nVertex3_300		=	nVertex3	*	300;
	nVertex3_300_4	=	nVertex3	*	300	*	4;

	nThre			=	32;
	nLoop			=	fix(	nVertex3	/	nThre	)	+	1;

	nOneVal		=	(	nLoop	*	nThre	*	4	);
	nAllVal		=	(	nLoop	*	nThre	*	4	)	*	75;
	nThre_300	=	nThre*300;

	R0	=	1	:	nVertex;

	R01	=	1		:	12	:	(	nVertex3_4	-	11	);
	R02	=	2		:	12	:	(	nVertex3_4	-	10	);
	R03	=	3		:	12	:	(	nVertex3_4	-	9		);
	R04	=	4		:	12	:	(	nVertex3_4	-	8		);

	R05	=	5		:	12	:	(	nVertex3_4	-	7		);
	R06	=	6		:	12	:	(	nVertex3_4	-	6		);
	R07	=	7		:	12	:	(	nVertex3_4	-	5		);
	R08	=	8		:	12	:	(	nVertex3_4	-	4		);

	R09	=	9		:	12	:	(	nVertex3_4	-	3		);
	R10	=	10	:	12	:	(	nVertex3_4	-	2		);
	R11	=	11	:	12	:	(	nVertex3_4	-	1		);
	R12	=	12	:	12	:	(	nVertex3_4	-	0		);

	TempData	=	zeros(	nAllVal,	1	);
	for	i	=	1:75
		R5	=	((i-1)*nOneVal)+1	:	(i*nOneVal);

		OneData					=	zeros(	nOneVal,	1	);

		OneData(	R01	)	=	Shape_Dirs(	R0,	1,	(i-1)*4+1	);
		OneData(	R02	)	=	Shape_Dirs(	R0,	1,	(i-1)*4+2	);
		OneData(	R03	)	=	Shape_Dirs(	R0,	1,	(i-1)*4+3	);
		OneData(	R04	)	=	Shape_Dirs(	R0,	1,	(i-1)*4+4	);

		OneData(	R05	)	=	Shape_Dirs(	R0,	2,	(i-1)*4+1	);
		OneData(	R06	)	=	Shape_Dirs(	R0,	2,	(i-1)*4+2	);
		OneData(	R07	)	=	Shape_Dirs(	R0,	2,	(i-1)*4+3	);
		OneData(	R08	)	=	Shape_Dirs(	R0,	2,	(i-1)*4+4	);

		OneData(	R09	)	=	Shape_Dirs(	R0,	3,	(i-1)*4+1	);
		OneData(	R10	)	=	Shape_Dirs(	R0,	3,	(i-1)*4+2	);
		OneData(	R11	)	=	Shape_Dirs(	R0,	3,	(i-1)*4+3	);
		OneData(	R12	)	=	Shape_Dirs(	R0,	3,	(i-1)*4+4	);

		TempData(	R5	)	=	OneData;
	end


	for	i	=	1:nAllVal
		iVal		=	fix(	(i-1)	/	nOneVal	);
		Temp1		=	rem(	(i-1),	nOneVal	);

		iVertex	=	fix(	Temp1	/	12	);
		Temp2		=	rem(	Temp1,	12	);


		iComp		=	fix(	Temp2	/		4	);
		Val_Add	=	rem(	Temp2,		4	);

		iVal		=	(iVal*4)	+	1	+	Val_Add;
		iVertex	=	iVertex		+	1;
		iComp		=	iComp			+	1;

		if	(	iVertex	<=	nVertex		)
			if	(	TempData(i)	~=	Shape_Dirs(	iVertex,	iComp,	iVal	)	)
				'Miss'
				stop
			end
		else
			if	(	TempData(i)	~=	0	)
				'Miss'
				stop
			end
		end
	end


	%In Data
	AllData	=	zeros(	nAllVal,	1	);
	for	i	=	1:nLoop
		S_1	=	(i-1)	*	(nThre*4)	+	1;
		E_1	=	i			*	(nThre*4);

		Add_2	=	(i-1)	*	nThre_300;

		for	j	=	1:75
			Add_1	=	nOneVal*(j-1);
			R_1		=	(	S_1	+	Add_1	)	:	(	E_1	+	Add_1	);


			S_2	=	(j-1)	*	(nThre*4)	+	1;
			E_2	=	j			*	(nThre*4);
			R_2	=	(	S_2	+	Add_2	)	:	(	E_2	+	Add_2	);

			AllData(R_2)	=	TempData(R_1);
		end
	end

	%Comf
	for	i	=	1:nAllVal
		iLoop				=	fix(	(i-1)	/	nThre_300	);
		Temp1				=	rem(	(i-1),	nThre_300	);

		Temp_iVal		=	fix(	Temp1	/		(nThre*4)	);
		Temp2				=	rem(	Temp1,		(nThre*4)	);


		iVertex_Lo	=	fix(	Temp2	/		4	);
		iVal_Add		=	rem(	Temp2,		4	);

		TempiVertex	=	(	iLoop*nThre)	+	iVertex_Lo;

		iVertex			=	fix(	TempiVertex	/		3	);
		iComp				=	rem(	TempiVertex,		3	);


		iVal			=	(Temp_iVal*4)	+	1	+	iVal_Add;
		iVertex		=	iVertex	+	1;
		iComp			=	iComp	+	1;

		if	(	iVertex	<=	nVertex		)
			if	(	AllData(i)	~=	Shape_Dirs(	iVertex,	iComp,	iVal	)	)
				'Miss'
				stop
			end
		else
			if	(	AllData(i)	~=	0	)
				'Miss'
				stop
			end
		end
	end

	nData				=	length(AllData);
	SaveSingle	=	AllData;
	SaveSingle	=	[	SaveSingle;	10^10	];

	FileName	=	[	CUDA_InputPathName	'Shape_DirsVer3.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveSingle,'single');
	fclose(ID);

	nVertex				=	6890;
	nVertex3			=	nVertex		*	3;
	nVertex3_300	=	nVertex3	*	300;

	nThre			=	32;
	nLoop			=	fix(	nVertex3	/	nThre	)	+	1;

	nOneVal		=	(nLoop*nThre);
	nAllVal		=	(nLoop*nThre)	*	300;
	nThre_300	=	nThre*300;

	R0	=	1	:	nVertex;


	R1	=	1:3:(	nVertex3	-	2	);
	R2	=	2:3:(	nVertex3	-	1	);
	R3	=	3:3:(	nVertex3	-	0	);


	TempData	=	zeros(	nAllVal,	1	);
	for	i	=	1:300
		R5	=	((i-1)*nOneVal)+1	:	(i*nOneVal);

		OneData					=	zeros(	nOneVal,	1	);
		OneData(	R1	)	=	Shape_Dirs(R0,1,i);
		OneData(	R2	)	=	Shape_Dirs(R0,2,i);
		OneData(	R3	)	=	Shape_Dirs(R0,3,i);
		TempData(	R5	)	=	OneData;
	end

	for	i	=	1:nAllVal
		iVal		=	fix(	(i-1)	/	nOneVal	);
		Temp		=	rem(	(i-1),	nOneVal	);
		iVertex	=	fix(	Temp	/		3	);
		iComp		=	rem(	Temp,			3	);

		iVal		=	iVal		+	1;
		iVertex	=	iVertex	+	1;
		iComp		=	iComp		+	1;

		if	(	iVertex	<=	nVertex		)
			if	(	TempData(i)	~=	Shape_Dirs(	iVertex,	iComp,	iVal	)	)
				'Miss'
				stop
			end
		else
			if	(	TempData(i)	~=	0	)
				'Miss'
				stop
			end
		end

	end

	%In Data
	AllData	=	zeros(	nAllVal,	1	);
	for	i	=	1:nLoop
		S_1	=	(i-1)	*	nThre	+	1;
		E_1	=	i			*	nThre;

		Add_2	=	(i-1)	*	nThre_300;


		for	j	=	1:300
			Add_1	=	nOneVal*(j-1);
			R_1		=	(	S_1	+	Add_1	)	:	(	E_1	+	Add_1	);


			S_2	=	(j-1)	*	nThre	+	1;
			E_2	=	j			*	nThre;
			R_2	=	(	S_2	+	Add_2	)	:	(	E_2	+	Add_2	);

			AllData(R_2)	=	TempData(R_1);
		end
	end

	%Comf
	for	i	=	1:nAllVal
		iLoop				=	fix(	(i-1)	/	nThre_300	);
		Temp				=	rem(	(i-1),	nThre_300	);
		iVal				=	fix(	Temp	/		nThre	);
		iVertex_Lo	=	rem(	Temp,			nThre	);
		TempiVertex	=	(	iLoop*nThre)	+	iVertex_Lo;

		iVertex			=	fix(	TempiVertex	/		3	);
		iComp				=	rem(	TempiVertex,		3	);

		iVal		=	iVal		+	1;
		iVertex	=	iVertex	+	1;
		iComp		=	iComp		+	1;

		if	(	iVertex	<=	nVertex		)
			if	(	AllData(i)	~=	Shape_Dirs(	iVertex,	iComp,	iVal	)	)
				'Miss'
				stop
			end
		else
			if	(	AllData(i)	~=	0	)
				'Miss'
				stop
			end
		end
	end

	nData				=	length(AllData);
	SaveSingle	=	AllData;
	SaveSingle	=	[	SaveSingle;	10^10	];

	FileName	=	[	CUDA_InputPathName	'Shape_DirsVer2.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveSingle,'single');
	fclose(ID);

	%-=-=-=-=-=-=-=-=-=-=-=
	%--V_Temp
	%-=-=-=-=-=-=-=-=-=-=-=
	SaveSingle	=	zeros(	6890*3,	1	);
	Counter	=	0;
	for	j	=	1:6890
		R	=	(	Counter*3	)+1	:	(	Counter*3	)+3;
		SaveSingle(R)	=	V_Temp(j,:);
		Counter	=	Counter	+	1;
	end
	SaveSingle	=	[	SaveSingle;	10^10	];

	FileName	=	[	CUDA_InputPathName	'V_Temp.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveSingle,'single');
	fclose(ID);


	%-=-=-=-=-=-=-=-=-=-=-=
	%--J_Regressor
	%-=-=-=-=-=-=-=-=-=-=-=
	J_Reg_Joint		=	[];
	J_Reg_Vertex	=	[];
	J_Reg_Co	=	[];
	for	i	=	1:24
		ITI	=	find(J_Regressor(:,i)~=0);

		J_Reg_Joint		=	[	J_Reg_Joint;		repmat((i-1),[length(ITI),1]	)	];
		J_Reg_Vertex	=	[	J_Reg_Vertex;		ITI-1														];
		J_Reg_Co			=	[	J_Reg_Co;				J_Regressor(	ITI,	i	)					];
	end

	SaveSingle	=	zeros(	259,	1	);
	SaveSingle	=	J_Reg_Co;
	SaveSingle	=	[	SaveSingle;	10^10	];

	FileName	=	[	CUDA_InputPathName	'J_RegressorVer2_Co.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveSingle,'single');
	fclose(ID);


	SaveInt	=	zeros(	259,	1	);
	SaveInt	=	J_Reg_Joint;
	SaveInt	=	[	SaveInt;	10000	];

	FileName	=	[	CUDA_InputPathName	'J_RegressorVer2_Joint.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveInt,'uint16');
	fclose(ID);

	SaveInt	=	zeros(	259,	1	);
	SaveInt	=	J_Reg_Vertex;
	SaveInt	=	[	SaveInt;	10000	];

	FileName	=	[	CUDA_InputPathName	'J_RegressorVer2_Vertex.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveInt,'uint16');
	fclose(ID);



	SaveSingle	=	zeros(	6890*24,	1	);
	Counter	=	0;
	for	j	=	1:6890
		R	=	(	Counter*24	)+1	:	(	Counter*24	)+24;
		SaveSingle(R)	=	J_Regressor(j,:);
		Counter	=	Counter	+	1;
	end

	SaveSingle	=	[	SaveSingle;	10^10	];

	FileName	=	[	CUDA_InputPathName	'J_Regressor.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveSingle,'single');
	fclose(ID);


	%-=-=-=-=-=-=-=-=-=-=-=
	%--Pose_Dirs
	%-=-=-=-=-=-=-=-=-=-=-=
	VerMat	=	[];
	ITIMat	=	[];
	CoMat_X	=	[];
	CoMat_Y	=	[];
	CoMat_Z	=	[];
	nFeatMat	=	[];
	for	i	=	1:6890

		XComp			=	zeros(93,1);
		XComp(:)	=	Pose_Dirs(i,1,:);
		ITI				=	find(	XComp	~=	0	);
		ITIMat		=	[	ITIMat;	(ITI-1)	];

		VerMat	=	[	VerMat;	repmat(	(i-1),	[length(ITI),	1	]	)	];


		nFeatMat	=	[	nFeatMat;	length(ITI)	];


		Temp_X		=	zeros(	length(	ITI	),	1	);
		Temp_X(:)	=	Pose_Dirs(	i,	1,	ITI	);


		Temp_Y		=	zeros(	length(	ITI	),	1	);
		Temp_Y(:)	=	Pose_Dirs(	i,	2,	ITI	);


		Temp_Z		=	zeros(	length(	ITI	),	1	);
		Temp_Z(:)	=	Pose_Dirs(	i,	3,	ITI	);

		CoMat_X	=	[	CoMat_X;	Temp_X	];
		CoMat_Y	=	[	CoMat_Y;	Temp_Y	];
		CoMat_Z	=	[	CoMat_Z;	Temp_Z	];
	end

	StartMat	=	[0];
	for	i	=	1:6889
		StartMat	=	[	StartMat;	sum(	nFeatMat(1:i)	)	];
	end

	nData											=	length(CoMat_X)*3;
	Coeff											=	zeros(	nData,	1	);
	Coeff(	1	:	3	:	nData-2	)	=	CoMat_X;
	Coeff(	2	:	3	:	nData-1	)	=	CoMat_Y;
	Coeff(	3	:	3	:	nData-0	)	=	CoMat_Z;


	SaveSingle	=	Coeff;
	SaveSingle	=	[	SaveSingle;	10^10	];

	FileName	=	[	CUDA_InputPathName	'Pose_DirsVer2_Co.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveSingle,'single');
	fclose(ID);


	SaveInt	=	ITIMat;
	SaveInt	=	[	SaveInt;	10000	];

	FileName	=	[	CUDA_InputPathName	'Pose_DirsVer2_FeatITI.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveInt,'uint16');
	fclose(ID);



	SaveInt	=	VerMat;
	SaveInt	=	[	SaveInt;	10000	];

	FileName	=	[	CUDA_InputPathName	'Pose_DirsVer2_Vertex.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveInt,'uint16');
	fclose(ID);

	SaveInt													=	zeros(6890*2,1);
	SaveInt(	1	:	2	:	(6890*2)-1	)	=	StartMat;
	SaveInt(	2	:	2	:	(6890*2)-0	)	=	nFeatMat;
	SaveInt													=	[	SaveInt;	10000	];

	FileName	=	[	CUDA_InputPathName	'Pose_DirsVer2_Start_nFeat.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveInt,'uint32');
	fclose(ID);


	SaveSingle	=	zeros(	6890*3*93,	1	);
	Counter	=	0;
	for	j	=	1:6890
		for	i	=	1:3
			R	=	(	Counter*93	)+1	:	(	Counter*93	)+93;
			SaveSingle(R)	=	Pose_Dirs(j,i,:);
			Counter	=	Counter	+	1;
		end
	end
	SaveSingle	=	[	SaveSingle;	10^10	];

	FileName	=	[	CUDA_InputPathName	'Pose_Dirs.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveSingle,'single');
	fclose(ID);

	%-=-=-=-=-=-=-=-=-=-=-=
	%--Weights
	%-=-=-=-=-=-=-=-=-=-=-=
	SaveSingle	=	zeros(	6890*24,	1	);


	ITIMat		=	[];
	nFeatMat	=	[];
	CoMat			=	[];
	for	i	=	1:6890
		ITI				=	find(	Weights(i,:)	~=	0	);
		nFeatMat	=	[	nFeatMat;	length(	ITI	)	];

		Temp_X		=	Weights(	i,	ITI	);
		CoMat			=	[	CoMat;	Temp_X'	];
		ITIMat		=	[	ITIMat;	(ITI-1)'	];
	end


	StartMat	=	[0];
	for	i	=	1:6889
		StartMat	=	[	StartMat;	sum(	nFeatMat(1:i)	)	];
	end


	SaveSingle	=	CoMat;
	SaveSingle	=	[	SaveSingle;	10^10	];

	FileName	=	[	CUDA_InputPathName	'WeightVer2_Co.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveSingle,'single');
	fclose(ID);

	SaveInt	=	ITIMat;
	SaveInt	=	[	SaveInt;	10000	];

	FileName	=	[	CUDA_InputPathName	'WeightVer2_JointITI.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveInt,'uint16');
	fclose(ID);


	SaveInt													=	zeros(6890*2,1);
	SaveInt(	1	:	2	:	(6890*2)-1	)	=	StartMat;
	SaveInt(	2	:	2	:	(6890*2)-0	)	=	nFeatMat;
	SaveInt													=	[	SaveInt;	10000	];

	FileName	=	[	CUDA_InputPathName	'WeightVer2_Start_nFeat.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveInt,'uint32');
	fclose(ID);

	Counter	=	0;
	for	j	=	1:6890
		R	=	(	Counter*24	)+1	:	(	Counter*24	)+24;
		SaveSingle(R)	=	Weights(j,:);
		Counter	=	Counter	+	1;
	end
	SaveSingle	=	[	SaveSingle;	10^10	];

	FileName	=	[	CUDA_InputPathName	'Weights.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveSingle,'single');
	fclose(ID);


	%-=-=-=-=-=-=-=-=-=-=-=
	%--V_Point
	%-=-=-=-=-=-=-=-=-=-=-=
	SaveSingle	=	zeros(	13776*3,	1	);
	Counter	=	0;
	for	j	=	1:13776
		R	=	(	Counter*3	)+1	:	(	Counter*3	)+3;
		SaveSingle(R)	=	V_Point(j,:);
		Counter	=	Counter	+	1;
	end
	SaveSingle	=	[	SaveSingle;	20000	];

	FileName	=	[	CUDA_InputPathName	'V_Point.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveSingle,'uint16');
	fclose(ID);


	%-=-=-=-=-=-=-=-=-=-=-=
	%--NewV_Point
	%-=-=-=-=-=-=-=-=-=-=-=
	%----------------
	% NewV_Point_32
	%----------------
	[Tate,Yoko]	=	size(NewV_Point_32);
	SaveSingle	=	zeros(	Tate*Yoko,	1	);
	Counter	=	0;
	for	j	=	1:Tate
		R	=	(	Counter*Yoko	)+1	:	(	Counter*Yoko	)+Yoko;
		SaveSingle(R)	=	NewV_Point_32(j,:);
		Counter	=	Counter	+	1;
	end
	SaveSingle	=	[	SaveSingle;	20000	];

	FileName	=	[	CUDA_InputPathName	'V_Point_32.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveSingle,'uint16');
	fclose(ID);

	%-=-=-=-=-=-=-=-=-=-=-=
	%--Kintree_Table
	%-=-=-=-=-=-=-=-=-=-=-=
	SaveSingle	=	zeros(	24*2,	1	);
	Counter	=	0;
	for	j	=	1:24
		R	=	(	Counter*2	)+1	:	(	Counter*2	)+2;
		SaveSingle(R)	=	Kintree_Table(j,:);
		Counter	=	Counter	+	1;
	end
	SaveSingle	=	[	SaveSingle;	255	];
	SaveSingle(1)	=	255;

	FileName	=	[	CUDA_InputPathName	'Kintree_Table.Dat'];
	ID	=	0;
	while	ID	<=	2
		ID				=	fopen(FileName,'w');
	end
	fwrite(ID,SaveSingle,'uint16');
	fclose(ID);

	V_Point				=	V_Point					+	1;
	NewV_Point_32	=	NewV_Point_32		+	1;

	%===================
	%Camera Parameter
	%===================
	Cam(1).Ref_RealCood	=	[	[0;0;0],	[2;0;0],	[0;1;0],	[2;1;0],	[0;2;0],	[2;2;0],	[0;3;0],	[2;3;0],	[0;4;0],	[2;4;0]	];
	nCam		=	8;
	nImage	=	1;

	%--------------
	%--Load Data
	%--------------
	for	iCam	=	1:nCam
		MoviePathName					=	[	TryPathName	'Original_Movie/'	];
		MovieFileName					=	[	SubName	'_Cam'	num2str(iCam)	'.mp4'	];
		LoadName							=	[	MoviePathName MovieFileName	];
		Input(iCam).MovieData = VideoReader(	LoadName	);

		PosePathName					=	[	TryPathName 'Detection/Cam'	num2str(iCam)	'/'	];
		PoseFileName					=	[	SubName	'_Cam'	num2str(iCam)	'_Pose.txt'	];
		PoseLoadName					=	[	PosePathName	PoseFileName	];
		Temp									= readmatrix(	PoseLoadName	);
		Input(iCam).PoseData	= Temp(:,1:17*3);

		DetectPathName				=	[	TryPathName 'Detection/Cam'	num2str(iCam)	'/'	];
		DetectFileName				=	[	SubName	'_Cam' num2str(iCam)	'_Detection.txt'	];
		DetectLoadName				=	[	DetectPathName	DetectFileName	];
		List									= readmatrix(	DetectLoadName	);
		Input(iCam).List			=	List;
		Input(iCam).nList			=	length(	List(:,1)	);
	end

	%---------------------
	%--Camera Parameter
	%---------------------
	for	iCam	=	1:nCam
		PathName					=	[	TryPathName 'Detection/Cam' num2str(iCam) '/'	];
		FileName					=	[	SubName	'_Cam'	num2str(iCam)	'_CamPara.csv'	];
		LoadName					=	[	PathName	FileName	];
		Cam(iCam).CamPara	=	dlmread(	LoadName	);

		SaveSingle	=	Cam(iCam).CamPara;
		FileName	=	[	CUDA_InputPathName	'CamPara_Cam'	num2str(iCam)	'.Dat'];
		ID	=	0;
		while	ID	<=	2
			ID				=	fopen(FileName,'w');
		end
		fwrite(ID,SaveSingle,'double');
		fclose(ID);
	end

	Interbal				=	fix(	Input(1).nList	/	nImage	);
	PickUpList_Ori	=	Interbal	*	(	0:(nImage-1)	)	+	1;

	%------------------
	%Renge of Search
	%------------------
	[Joint_Low,Joint_Up]	=	GetRangeOfMotion(	);
	Joint_Low							=	Joint_Low.*(pi/180);
	Joint_Up							=	Joint_Up.*(pi/180);


	lb_Posture	=	[];
	ub_Posture	=	[];
	Q0_Posture	=	[];
	for	iImage	=	1:nImage
		Trans		=	[	1.0;	2;	1.0	];

		%Rotate
		Pose		=	zeros(	72,		1	);
		Pose(1)	=	pi/2;
		Pose(2)	=	pi*1.0;
		Pose(3)	=	0;

		Pose(4)	=	0;


		BodyRot_Low	=	Pose(1:3)	-	pi/2;
		BodyRot_Up	=	Pose(1:3)	+	pi/2;

		Trans_Low		=	Trans	-	[	3;	2;	2	];
		Trans_Up		=	Trans	+	[	3;	2;	2	];

		lb_Posture	=	[	lb_Posture;	Trans_Low;	BodyRot_Low;	Joint_Low	];
		ub_Posture	=	[	ub_Posture;	Trans_Up;		BodyRot_Up;		Joint_Up	];
		Q0_Posture	=	[	Q0_Posture;	Trans;			Pose										];
	end

	Q0_Betas		=	zeros(	300,	1	);
	Range_Betas	=	[	repmat(	5,		[5,1]		);
									repmat(	5.0,	[5,1]		);
									repmat(	5.0,	[90,1]	);
									repmat(	5.0,	[100,1]	);
									repmat(	5.0,	[100,1]	);		];


	lb_Betas	=	-1	*	Range_Betas;
	ub_Betas	=	1		*	Range_Betas;
	lb	=	[	lb_Posture;	lb_Betas	];
	ub	=	[	ub_Posture;	ub_Betas	];
	Q0	=	[	Q0_Posture;	Q0_Betas	];

	for	III	=	1:Input(1).MovieData.NumFrames

		PickUpList	=	PickUpList_Ori	+	(III-1);

		%-=-=-=-=-=-=-=-=-=-=-
		%	Q_Parameter
		%-=-=-=-=-=-=-=-=-=-=-
		SaveData=...
		[	
			Q0;
			10^10;
			lb;
			10^10;
			ub
			10^10;
		];

		FileName	=	[	CUDA_InputPathName	'Q_Parameter.Dat'	];
		ID	=	0;
		while	ID	<=	2
			ID				=	fopen(FileName,'w');
		end
		fwrite(ID,SaveData,'double');
		fclose(ID);

		for	iCam	=	1:nCam
			Input(iCam).Label					=	zeros(	nImage,	Input(iCam).MovieData.Height,	Input(iCam).MovieData.Width			);
			Input(iCam).img_b					=	zeros(	nImage,	Input(iCam).MovieData.Height,	Input(iCam).MovieData.Width,	3	);
			Input(iCam).PoseData_Curr	=	[];

			for	iCounter	=	1:nImage;
				iList	=	PickUpList(iCounter);

				MattingPathName01				=	[	TryPathName 'Detection/Cam'	num2str(iCam)	'/'	];
				MattingPathName02				=	[	SubName	'_Cam'	num2str(iCam) '_Matting/'	];
				MattingFileName					=	[	'DemoTry_Cam'	num2str(iCam) '_' num2str(iList,'%05.0f')	'_alpha.png'	];
				MattingLoadName					=	[	MattingPathName01 MattingPathName02 MattingFileName	];
				Input(iCam).MattingData	=	imread(	MattingLoadName	);

				Input(iCam).img_b(iCounter,:,:,:)	=	read(	Input(iCam).MovieData,	iList	);

				Thre	=	0.9;
				Flag	=	Input(iCam).MattingData	>	fix(	Thre	*	255		);

				%-------------------------
				%--Check Object Detection
				%-------------------------
				ITI				=	find(	Input(iCam).List	==	iList	);
				SizeData	=	(	Input(iCam).List(ITI,5)	.*	Input(iCam).List(ITI,6)	);
				[~,Jyun]	=	max(SizeData);
				ITI				=	ITI(	Jyun(1)	);

				Temp		=	(	(Input(iCam).List(	ITI,	5	)	*	Input(iCam).MovieData.Width)	/	2	);
				Temp(2)	=	(	(Input(iCam).List(	ITI,	6	)	*	Input(iCam).MovieData.Height)	/	2	);
				Range		=	max(	Temp	);

				A	=	fix(	Input(iCam).List(	ITI,	3	)	*	Input(iCam).MovieData.Width		-	(	Range*1.10	)	);
				B	=	fix(	Input(iCam).List(	ITI,	3	)	*	Input(iCam).MovieData.Width		+	(	Range*1.10	)	);

				C	=	fix(	Input(iCam).List(	ITI,	4	)	*	Input(iCam).MovieData.Height	-	(	Range*1.10	)	);
				D	=	fix(	Input(iCam).List(	ITI,	4	)	*	Input(iCam).MovieData.Height	+	(	Range*1.10	)	);

				if	A	<	1
					A	=	1;
				end

				if	B	>	Input(iCam).MovieData.Width
					B	=	Input(iCam).MovieData.Width;
				end

				if	C	<	1
					C	=	1;
				end

				if	D	>	Input(iCam).MovieData.Height
					D	=	Input(iCam).MovieData.Height;
				end

				TempPoseData(	1,	:					)	=	Input(iCam).PoseData(	iList,	:	);
				TempPoseData(	1,	1:3:end-2	)	=	TempPoseData(	1,	1:3:end-2	)	+	A;
				TempPoseData(	1,	2:3:end-1	)	=	TempPoseData(	1,	2:3:end-1	)	+	C;

				R				=	(	[	0,1,2,3,4,9,10,15,16	]	*	3	);
				R				=	(	[	0,1,2,3,4,	5,6,	7,8,	9,10,	11,12,	13,14,	15,16	]	*	3	);
				Flag01	=	TempPoseData(	1,	R+3	)	<		0.9;
				Flag02	=	TempPoseData(	1,	R+3	)	>=	0.9;

				TempPoseData(	1,	R+1	)	=	(	Flag02.*TempPoseData(	1,	R+1	)	)	+	(	Flag01*(-1.0)	);
				TempPoseData(	1,	R+2	)	=	(	Flag02.*TempPoseData(	1,	R+2	)	)	+	(	Flag01*(-1.0)	);
				Input(iCam).PoseData_Curr	=	[
																			Input(iCam).PoseData_Curr;
																			TempPoseData(	1,	R+1	);
																			TempPoseData(	1,	R+2	)
																		];

				Input(iCam).Label(iCounter,	C:D,	A:B	)	=	Flag*1;
			end
			Input(iCam).Label					=	uint8(Input(iCam).Label);
			Input(iCam).img_b					=	uint8(Input(iCam).img_b);

			%	1,nose;		2,LEye;		3,REye;		4,Leyer;		5,Reyer;
			%	6,Lsh;		7,Rsh;		8,LEl;		9,REl;			10,LWr;		11,RWr;	
			%	12,Lhip;	13,Rhip;	14,LKnee;	15,RKnee;		16,LAn;		17,RAn;	

%			Input(iCam).PoseData_Curr(:,[12,13])	=	-1.000;
			Input(iCam).PoseData_Curr	=	Input(iCam).PoseData_Curr(:,[	1,	3,2,	5,4,	7,9,11,	6,8,10,	13,15,17,	12,14,16	]	);
		end
%		Comf_Image(	nCam,	nImage,	Input	)

		%---------------------
		%--SaveBaseParameter
		%---------------------
		nKeyPoint	=	17;
		if	III	==	1
			ObjeFlag	=	1;
		else
			ObjeFlag	=	0;
		end

		SaveInt	=	[	nCam;	nImage;	nKeyPoint;	ObjeFlag	];
		for	iCam	=	1:nCam
			SaveInt	=...
						[	SaveInt;
							Input(iCam).MovieData.Height;
							Input(iCam).MovieData.Width
						];
		end
		FileName	=	[	CUDA_InputPathName	'BasePara_Int.Dat'];
		ID	=	0;
		while	ID	<=	2
			ID				=	fopen(FileName,'w');
		end
		fwrite(ID,SaveInt,'int');
		fclose(ID);

		%================
		%	KeyPoint_Cood
		%================
		nKeyPoint	=	17;
		for	iCam	=	1:nCam
			for	iImage	=	1:nImage;

				SaveSingle												=	zeros(nKeyPoint*2,1);
				SaveSingle(1:2:(nKeyPoint*2-1),1)	=	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	:	)';
				SaveSingle(2:2:(nKeyPoint*2-0),1)	=	Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	:	)';
				SaveSingle												=	[	SaveSingle;	10^10	];
				FileName	=	[	CUDA_InputPathName	'Keypoint_Cam' num2str(iCam)	'_Image' num2str(iImage,'%03.0f')   '.Dat'	];
				ID	=	0;
				while	ID	<=	2
					ID				=	fopen(FileName,'w');
				end
				fwrite(ID,SaveSingle,'single');
				fclose(ID);
			end
		end

		%================
		%	Silhouette
		%================
		for	iCam	=	1:nCam;
			for	iImage	=	1:nImage;

				SaveSingle	=	zeros(	1920*1080,	1	);
				Counter	=	0;
				for	j	=	1:1920
					R	=	(	Counter*1080	)+1	:	(	Counter*1080	)+1080;
					SaveSingle(R)	=	Input(iCam).Label(	iImage,	:,j	);
					Counter	=	Counter	+	1;
				end
				SaveSingle	=	[	SaveSingle	];

				FileName	=	[	CUDA_InputPathName	'Label_Cam' num2str(iCam)	'_Image'	num2str(iImage,'%03.0f') '.Dat'];
				ID	=	0;
				while	ID	<=	2
					ID				=	fopen(FileName,'w');
				end
				fwrite(ID,SaveSingle,'uint8');
				fclose(ID);
			end
		end
		Cam	=	Get_CamPara(	Input(iCam).MovieData.Width,	Input(iCam).MovieData.Height,	nCam,	Cam	);

		%Ç±ÇÍÇé¿çsÇ∑ÇÈÇ∆çƒìäâeÇµÇΩêlëÃÉÇÉfÉãÇå©ÇÈÇ±Ç∆Ç™Ç≈Ç´Ç‹Ç∑
%		Comf_Reprojection(	Q0	);

		%-----------------------
		%Ç±Ç±Ç≈CUDAÇégÇ¢Ç‹Ç∑
		%-----------------------
		!CudaProgram


		tic;
		FileName	=	[	'OptimizationData.Dat'	];
		ID	=	0;
		Flag	=	0;
		while	ID	<=	2
			A	=	toc;
			if	A	>	180
				Flag	=	1;
				A
				break;
			end
			if	A	>	180
				A;
			end
			ID				=	fopen(FileName);
		end

		if	Flag	==	0
			pause(0.5)
			AnsMat	=	fread(ID,'double');
			fclose(ID);
			FileName	=	[	'OptimizationData.Dat'];
			delete(FileName);

			FileName	=	[	'OptimizationData_Stage1_' num2str(III) '.Dat'	];
			SaveName	=	[	'.\Stage1_OutputFile\' FileName];
			ID	=	0;
			while	ID	<=	2
				ID				=	fopen(SaveName,'w');
			end
			fwrite(ID,AnsMat,'double');
			fclose(ID);
			pause(0.5)
		end

		ID	=	0;
		while	ID	<=	2
			ID				=	fopen(SaveName);
		end
		FFF	=	fread(ID,'double');
		fclose(ID);
		Q0	=	FFF;

		Cam	=	Get_CamPara(	Input(iCam).MovieData.Width,	Input(iCam).MovieData.Height,	nCam,	Cam	);

		%Ç±ÇÍÇé¿çsÇ∑ÇÈÇ∆çƒìäâeÇµÇΩêlëÃÉÇÉfÉãÇå©ÇÈÇ±Ç∆Ç™Ç≈Ç´Ç‹Ç∑
%		Comf_Reprojection(	Q0	);

		Q0_Posture=	Q0(	1	:	nImage*75		);
		Q0_Betas	=	Q0(	nImage*75+1:end	);

		Range_Posture	=	[];
		for	iImage	=	1:nImage
			%Translate
			Trans		=	[	0.1;	0.1;	0.1	];

			%Rotate
			Pose		=	zeros(	72,		1	);
			Pose(:)	=	pi/18;

			Range_Posture	=	[	Range_Posture;	Trans;	Pose	];
		end

		NewRange_Betas	=	Range_Betas		*	0.10;

		lb_Betas		=	Q0_Betas		-	NewRange_Betas;
		ub_Betas		=	Q0_Betas		+	NewRange_Betas;

		lb_Posture	=	Q0_Posture	-	Range_Posture;
		ub_Posture	=	Q0_Posture	+	Range_Posture;

		lb_Posture(7:75)	=	(	lb_Posture(7:75)	<	Joint_Low	).*Joint_Low	+	(	lb_Posture(7:75)	>=	Joint_Low	).*lb_Posture(7:75);
		ub_Posture(7:75)	=	(	ub_Posture(7:75)	>	Joint_Up	).*Joint_Up		+	(	ub_Posture(7:75)	<=	Joint_Up	).*ub_Posture(7:75);

		lb_Betas		=	(	(lb_Betas<-5).*(-5)	)	+	(	(lb_Betas>=-5).*lb_Betas);
		ub_Betas		=	(	(ub_Betas>5).*(5)	)		+	(	(ub_Betas<=5).*ub_Betas);

		lb	=	[	lb_Posture;	lb_Betas	];
		ub	=	[	ub_Posture;	ub_Betas	];
		Q0	=	[	Q0_Posture;	Q0_Betas	];
	end











return;


%=============================================================================
%Internal Function(Camera Cood)
%=============================================================================
function	Comf_Reprojection(	Q	);


	global	Shape_Dirs	V_Temp	J_Regressor		Pose_Dirs		Kintree_Table		Weights	Cam	BasePara	V_Point	Input	nImage	nCam	NewV_Point_32 LandMark_Ori R1_Opt R2_Opt

	Betas	=	Q(	(nImage)*75+1	:	(nImage)*75+300	);



	LandMark01	=	LandMark_Ori(	R1_Opt	,	:	);
	LandMark02	=	LandMark_Ori(	R2_Opt	,	:	);

	for	iImage	=	1:nImage;
		R	=	(iImage-1)*75+1	:	(iImage-1)*75+75;
		Q_iImage	=	Q(	R	);

		Trans	=	Q_iImage(1:3);
		Pose	=	Q_iImage(4:75);

		%	EulerAngle Å® Rodrigues Rotation
		Pose	=	Euler_Rodrigues(	Pose	);

		[Vertex,	GrobalJoint	]	=	Get_Vertex(	Betas,	Pose,	Trans,...
																					Shape_Dirs,...
																					V_Temp,...
																					J_Regressor,...
																					Pose_Dirs,...
																					Kintree_Table,...
																					Weights	);


		Vertex	=	[	Vertex;	[0,0,0]	];
		nTriang		=	length(NewV_Point_32);

		R		=	1:nTriang;
		P1	=	Vertex(	NewV_Point_32(	R,	1	),	:	);
		P2	=	Vertex(	NewV_Point_32(	R,	2	),	:	);
		P3	=	Vertex(	NewV_Point_32(	R,	3	),	:	);

		Vec1	=	P2-P1;
		Vec2	=	P3-P1;

		NormVec	=	cross(Vec1',Vec2');
		NormVec	=	NormVec';

		Leng	=	sum(	(NormVec.^2),2	).^(1/2);
		uNormVec	=	(NormVec./Leng);


		ITI						=	find(	(	Vec1	==	0	)	);
		uNormVec(ITI)	=	0;

		LandMark_Cood			=	[];
		JointCenter_Cood	=	[];
		for	iLand	=	1:length(	LandMark02(:,1)	)

			P11	=	LandMark01(iLand,1);
			P12	=	LandMark01(iLand,2);
			P13	=	LandMark01(iLand,3);

			R11	=	LandMark01(iLand,4);
			R12	=	LandMark01(iLand,5);
			R13	=	LandMark01(iLand,6);

			P21	=	LandMark02(iLand,1);
			P22	=	LandMark02(iLand,2);
			P23	=	LandMark02(iLand,3);

			R21	=	LandMark02(iLand,4);
			R22	=	LandMark02(iLand,5);
			R23	=	LandMark02(iLand,6);

			Cood_Land1	=	(	Vertex(	P11,	:	)*R11	)	+	(	Vertex(	P12,	:	)*R12	)	+	(	Vertex(	P13,	:	)*R13	);
			Cood_Land2	=	(	Vertex(	P21,	:	)*R21	)	+	(	Vertex(	P22,	:	)*R22	)	+	(	Vertex(	P23,	:	)*R23	);

			LandMark_Cood			=	[	LandMark_Cood;			Cood_Land1;Cood_Land2				];
			JointCenter_Cood	=	[	JointCenter_Cood;		(Cood_Land1+Cood_Land2)./2	];
		end

		JointCenter_Cood	=	[
													JointCenter_Cood(	1:11,		:	);
													GrobalJoint(			3,			:	);
													JointCenter_Cood(	12:13,	:	);
													GrobalJoint(			2,			:	);
													JointCenter_Cood(	14:15,	:	);
												];

		AAA	=	[	Vertex;	JointCenter_Cood	];
		[Tate,Yoko]	=	size(AAA);
		AAA					=	reshape(AAA',	[Tate*Yoko,1]	);
		StickCood		=	AAA';

		F_h			=	figure(	'position',	[0, 0, 640*2.5,360*2.5]	);
		A_h(1)	=	axes(		'Parent',F_h,'Position',[	0/3,		0/3,	1/3,	1/3	]	);
		A_h(2)	=	axes(		'Parent',F_h,'Position',[	1/3,		0/3,	1/3,	1/3	]	);
		A_h(3)	=	axes(		'Parent',F_h,'Position',[	2/3,		0/3,	1/3,	1/3	]	);

		A_h(4)	=	axes(		'Parent',F_h,'Position',[	0/3,		1/3	1/3,	1/3	]	);
		A_h(5)	=	axes(		'Parent',F_h,'Position',[	1/3,		1/3	1/3,	1/3	]	);
		A_h(6)	=	axes(		'Parent',F_h,'Position',[	2/3,		1/3	1/3,	1/3	]	);

		A_h(7)	=	axes(		'Parent',F_h,'Position',[	0/3,		2/3	1/3,	1/3	]	);
		A_h(8)	=	axes(		'Parent',F_h,'Position',[	1/3,		2/3	1/3,	1/3	]	);
		A_h(9)	=	axes(		'Parent',F_h,'Position',[	2/3,		2/3	1/3,	1/3	]	);

		for	iCam	=	1:8

			CL						=	Cam(iCam).CL;
			CamPosture_Rm	=	Cam(iCam).CamPosture_Rm;
			F							=	Cam(iCam).F;
			UoVo					=	[	Cam(iCam).OriUo;	Cam(iCam).OriVo	];

			%0à»â∫ÇÕäpìxÇ™90ìxà»è„Ç≈ñ≥éã(âBÇÍÇƒÇ¢ÇÈ),0ÇÊÇËëÂÇ´Ç≠0.707388à»â∫ÇÕÅCäpìxÇ™90ÇÊÇËè¨Ç≥Ç≠45ìxà»è„Ç≈ÇµÇ¡Ç©ÇËí≤Ç◊ÇÈÅC0.707388à»è„ÇÕäpìxÇ™45ìxà»â∫Ç≈Ç¥Ç¡Ç≠ÇËí≤Ç◊ÇÈ
			Angle_TriOpti	=	(	uNormVec(:,1)	.*	-CamPosture_Rm(7)	)	+	(	uNormVec(:,2)	.*	-CamPosture_Rm(8)	)	+	(	uNormVec(:,3)	.*	-CamPosture_Rm(9)	);
			Angle_TriOpti(	end-30:end	)	=	-1;

			%=============================================
			%Real_Cood
			%=============================================
			Vec_CL_RealPoint		=	Vertex'	-	CL;
			XAxis								=	reshape(CamPosture_Rm(1:3,:),[3*Cam(iCam).nSean,1]);
			YAxis								=	reshape(CamPosture_Rm(4:6,:),[3*Cam(iCam).nSean,1]);
			ZAxis								=	reshape(CamPosture_Rm(7:9,:),[3*Cam(iCam).nSean,1]);

			%-----------------
			%RightHand_FValue
			%-----------------
			Temp				=	ZAxis	.*	Vec_CL_RealPoint;
			LocalCood_Z	=	Temp(1:3:Cam(iCam).nSean*3-2,:)	+	Temp(2:3:Cam(iCam).nSean*3-1,:)	+	Temp(3:3:Cam(iCam).nSean*3-0,:);

			Temp				=	XAxis	.*	Vec_CL_RealPoint;
			LocalCood_X	=	Temp(1:3:Cam(iCam).nSean*3-2,:)	+	Temp(2:3:Cam(iCam).nSean*3-1,:)	+	Temp(3:3:Cam(iCam).nSean*3-0,:);

			Temp				=	YAxis	.*	Vec_CL_RealPoint;
			LocalCood_Y	=	Temp(1:3:Cam(iCam).nSean*3-2,:)	+	Temp(2:3:Cam(iCam).nSean*3-1,:)	+	Temp(3:3:Cam(iCam).nSean*3-0,:);

			%=============================================
			%Reprojection	and Objective_Function
			%=============================================
			ReProU	=	(	-F*(LocalCood_X./LocalCood_Z)	)	+	UoVo(	1	);
			ReProV	=	(	-F*(LocalCood_Y./LocalCood_Z)	)	+	UoVo(	2	);
			ReProU(end)	=	0.00;
			ReProV(end)	=	0.00;

			%=============================================
			%Real_Cood
			%=============================================
			Vec_CL_RealPoint		=	JointCenter_Cood'	-	CL;

			XAxis								=	reshape(CamPosture_Rm(1:3,:),[3*Cam(iCam).nSean,1]);
			YAxis								=	reshape(CamPosture_Rm(4:6,:),[3*Cam(iCam).nSean,1]);
			ZAxis								=	reshape(CamPosture_Rm(7:9,:),[3*Cam(iCam).nSean,1]);

			%-----------------
			%RightHand_FValue
			%-----------------
			Temp				=	ZAxis	.*	Vec_CL_RealPoint;
			LocalCood_Z	=	Temp(1:3:Cam(iCam).nSean*3-2,:)	+	Temp(2:3:Cam(iCam).nSean*3-1,:)	+	Temp(3:3:Cam(iCam).nSean*3-0,:);

			Temp				=	XAxis	.*	Vec_CL_RealPoint;
			LocalCood_X	=	Temp(1:3:Cam(iCam).nSean*3-2,:)	+	Temp(2:3:Cam(iCam).nSean*3-1,:)	+	Temp(3:3:Cam(iCam).nSean*3-0,:);

			Temp				=	YAxis	.*	Vec_CL_RealPoint;
			LocalCood_Y	=	Temp(1:3:Cam(iCam).nSean*3-2,:)	+	Temp(2:3:Cam(iCam).nSean*3-1,:)	+	Temp(3:3:Cam(iCam).nSean*3-0,:);

			%=============================================
			%Reprojection	and Objective_Function
			%=============================================
			ReProU_Joint	=	(	-F*(LocalCood_X./LocalCood_Z)	)	+	UoVo(	1	);
			ReProV_Joint	=	(	-F*(LocalCood_Y./LocalCood_Z)	)	+	UoVo(	2	);

			%=============================================
			%Real_Cood
			%=============================================
			Vec_CL_RealPoint		=	Cam(1).Ref_RealCood	-	CL;

			XAxis								=	reshape(CamPosture_Rm(1:3,:),[3*Cam(iCam).nSean,1]);
			YAxis								=	reshape(CamPosture_Rm(4:6,:),[3*Cam(iCam).nSean,1]);
			ZAxis								=	reshape(CamPosture_Rm(7:9,:),[3*Cam(iCam).nSean,1]);

			%-----------------
			%RightHand_FValue
			%-----------------
			Temp				=	ZAxis	.*	Vec_CL_RealPoint;
			LocalCood_Z	=	Temp(1:3:Cam(iCam).nSean*3-2,:)	+	Temp(2:3:Cam(iCam).nSean*3-1,:)	+	Temp(3:3:Cam(iCam).nSean*3-0,:);

			Temp				=	XAxis	.*	Vec_CL_RealPoint;
			LocalCood_X	=	Temp(1:3:Cam(iCam).nSean*3-2,:)	+	Temp(2:3:Cam(iCam).nSean*3-1,:)	+	Temp(3:3:Cam(iCam).nSean*3-0,:);

			Temp				=	YAxis	.*	Vec_CL_RealPoint;
			LocalCood_Y	=	Temp(1:3:Cam(iCam).nSean*3-2,:)	+	Temp(2:3:Cam(iCam).nSean*3-1,:)	+	Temp(3:3:Cam(iCam).nSean*3-0,:);	

			%=============================================
			%Reprojection	and Objective_Function
			%=============================================
			ReProU_Ref	=	(	-F*(LocalCood_X./LocalCood_Z)	)	+	UoVo(	1	);
			ReProV_Ref	=	(	-F*(LocalCood_Y./LocalCood_Z)	)	+	UoVo(	2	);

			%----------------
			%About BoundOK1
			%----------------
			nTriang		=	length(V_Point);
			BoundOK1	=	zeros(	1080,	1920	);
			BoundOK2	=	zeros(	1080,	1920	);
			BoundOK3	=	zeros(	1080,	1920	);

			P_Cood	=	zeros(2,3);
			MinU	=	zeros(1,1);		MaxU	=	zeros(1,1);
			MinV	=	zeros(1,1);		MaxV	=	zeros(1,1);
			TempD	=	zeros(10,1);	TempI	=	zeros(10,1);


			%32
			nGroup	=	length(	NewV_Point_32	)	/	32;
			for	iGroup	=	1:nGroup

				A1	=	[];	A2	=	[];
				B1	=	[];	B2	=	[];
				for	i	=	1:32


					PPP	=	NewV_Point_32(	(iGroup-1)*32+i,	:	);

					P_Cood			=	[];
					P_Cood(1,:)	=	ReProU(PPP);
					P_Cood(2,:)	=	ReProV(PPP);

					if	(	sum(	[	P_Cood(1,:),	P_Cood(2,:)	]	)	~=	0	)
						[BoundOK1]	=	GetShilette(	P_Cood,	BoundOK1	);

						A1	=	[A1,	P_Cood(1,:)	];
						B1	=	[B1,	P_Cood(2,:)	];
					end
				end
				Diff(iGroup,:)	=	[	max(A1)	-	min(A1),	max(B1)	-	min(B1)	];
			end

			Dot1	=	zeros(	1920*1080,	2		);
			Count	=	1;
			for	iU	=	1:1920
				for	iV	=	1:1080
					if	BoundOK1(	iV,iU	)	==	1;
						Dot1(Count,:)=	[iU,iV];
						Count	=	Count	+	1;
					end
				end
			end

			Dot3	=	zeros(	1920*1080,	2		);
			Count	=	1;
			for	iU	=	1:1920
				for	iV	=	1:1080
					if	BoundOK3(	iV,iU	)	==	1;
						Dot3(Count,:)=	[iU,iV];
						Count	=	Count	+	1;
					end
				end
			end

			Input_Label				=	[	];
			Input_Label(:,:)	=	Input(iCam).Label(iImage,:,:);


			Input_Label	=	uint8(Input_Label);
			BoundOK1		=	uint8(BoundOK1);
			Flag	=	abs(	BoundOK1	-	Input_Label	);

			Count_CG				=	sum(	sum(	BoundOK1		)	);
			Count_CG_Label	=	sum(	sum(	Input_Label	)	);
			CG				=	zeros(2,1);
			CG_Label	=	zeros(2,1);

			LabelPlot	=	zeros(	1920*1080,	2	);
			Count	=	1;
			for	iU	=	1:1920
				for	iV	=	1:1080
					if	BoundOK1(	iV,iU	)	==	1;
						CG(1)	=	CG(1)	+	(iU/Count_CG);
						CG(2)	=	CG(2)	+	(iV/Count_CG);
					end

					if	Input_Label(	iV,iU	)	==	1;
						CG_Label(1)	=	CG_Label(1)	+	(iU/Count_CG_Label);
						CG_Label(2)	=	CG_Label(2)	+	(iV/Count_CG_Label);
						LabelPlot(Count,:)	=	[iU,iV];
						Count	=	Count	+	1;
					end

				end
			end

			CData(:,:,:)	=	Input(iCam).img_b(	iImage,	:,:,:);
			CData					=	uint8(	CData	);

			image(	CData,	'Parent',	A_h(iCam)	)
			for	III	=	1:length(	NewV_Point_32(:,1)	)
				if	Angle_TriOpti(III)	<=	0.0
					line(	ReProU(	[	NewV_Point_32(III,:),	NewV_Point_32(III,1)	]	),...
								ReProV(	[	NewV_Point_32(III,:),	NewV_Point_32(III,1)	]	),...
															'LineStyle',	':',	'Color',					'c',	'LineWidth',			1.0,...
															'Marker',			'none',		'MarkerEdgeColor','k',	'MarkerFaceColor','y','MarkerSize',2,	'Parent',A_h(iCam)	);
				end
			end
			set(A_h(iCam),	'XLim',[1 Cam(iCam).Pix_Yoko],	'YLim',[1 Cam(iCam).Pix_Tate],	'YDir','reverse')

		end
	end
	waitfor(F_h);

return;

%-----------------------------------------------------------------------------------
%	Internal Function
%-----------------------------------------------------------------------------------
function	[BoundOK1]	=	GetShilette(	P_Cood,	BoundOK1	);


	Co(1,1)	=	(	P_Cood(2,1)	-	P_Cood(2,2)	)	/	(	P_Cood(1,1)	-	P_Cood(1,2)	);
	Co(2,1)	=	P_Cood(2,1)	-	(	Co(1,1)	*	P_Cood(1,1)	);

	Co(1,2)	=	(	P_Cood(2,2)	-	P_Cood(2,3)	)	/	(	P_Cood(1,2)	-	P_Cood(1,3)	);
	Co(2,2)	=	P_Cood(2,2)	-	(	Co(1,2)	*	P_Cood(1,2)	);

	Co(1,3)	=	(	P_Cood(2,3)	-	P_Cood(2,1)	)	/	(	P_Cood(1,3)	-	P_Cood(1,1)	);
	Co(2,3)	=	P_Cood(2,3)	-	(	Co(1,3)	*	P_Cood(1,3)	);

	Flag1	=	(	Co(1,1)	*	P_Cood(1,3)	)	+	Co(2,1);
	Flag1	=	Flag1	>	P_Cood(2,3);

	Flag2	=	(	Co(1,2)	*	P_Cood(1,1)	)	+	Co(2,2);
	Flag2	=	Flag2	>	P_Cood(2,1);

	Flag3	=	(	Co(1,3)	*	P_Cood(1,2)	)	+	Co(2,3);
	Flag3	=	Flag3	>	P_Cood(2,2);

	MaxU	=	floor(	max(	P_Cood(1,:)	)	)	+	1;
	MinU	=	floor(	min(	P_Cood(1,:)	)	);

	MaxV	=	floor(	max(	P_Cood(2,:)	)	)	+	1;
	MinV	=	floor(	min(	P_Cood(2,:)	)	);

	RU	=	MinU	:	MaxU;
	RV	=	MinV	:	MaxV;

	UMat	=	repmat(	RU,	[	length(RV),	1						]	);
	VMat	=	repmat(	RV',	[	1,				length(RU)	]	);

	AAA	=	Co(1,1)	*	UMat	+	Co(2,1);
	BBB	=	Co(1,2)	*	UMat	+	Co(2,2);
	CCC	=	Co(1,3)	*	UMat	+	Co(2,3);

	if	Flag1	==	1
		G	=	VMat	<	AAA;
	else
		G	=	VMat	>	AAA;
	end

	if	Flag2	==	1
		F	=	VMat	<	BBB;
	else
		F	=	VMat	>	BBB;
	end

	if	Flag3	==	1
		C	=	VMat	<	CCC;
	else
		C	=	VMat	>	CCC;
	end
	H	=	G+F+C;

	%Max U
	TempD(1)	=	P_Cood(1,1);	TempD(2)	=	P_Cood(1,1);
	TempD(3)	=	P_Cood(2,1);	TempD(4)	=	P_Cood(2,1);

	TempI(1)	=	1;				TempI(2)	=	1;
	TempI(3)	=	1;				TempI(4)	=	1;
	for	j	=	2:3
		if	(	TempD(1)	>	P_Cood(1,j)	)
			TempD(1)	=	P_Cood(1,j);
			TempI(1)	=	j;
		end

		if	(	TempD(2)	<	P_Cood(1,j)	)
			TempD(2)	=	P_Cood(1,j);
			TempI(2)	=	j;
		end

		if	(	TempD(3)	>	P_Cood(2,j)	)
			TempD(3)	=	P_Cood(2,j);
			TempI(3)	=	j;
		end

		if	(	TempD(4)	<	P_Cood(2,j)	)
			TempD(4)	=	P_Cood(2,j);
			TempI(4)	=	j;
		end
	end

	%P1 MinU
	P1	=	TempI(1);

	%P2 MaxU
	P2	=	TempI(2);

	%P3 Other
	P3	=	6-(P1+P2);

	TempD(11)	=	(	P_Cood(	2,	P1	)	-	P_Cood(	2,	P2	)	)	/	(	P_Cood(	1,	P1	)	-	P_Cood(	1,	P2	)	);
	TempD(12)	=		P_Cood(	2,	P1	)	-	(	TempD(11)	*	P_Cood(	1,	P1	)	);

	TempD(13)	=	(	P_Cood(	2,	P1	)	-	P_Cood(	2,	P3	)	)	/	(	P_Cood(	1,	P1	)	-	P_Cood(	1,	P3	)	);
	TempD(14)	=		P_Cood(	2,	P1	)	-	(	TempD(13)	*	P_Cood(	1,	P1	)	);

	TempD(15)	=	(	P_Cood(	2,	P2	)	-	P_Cood(	2,	P3	)	)	/	(	P_Cood(	1,	P2	)	-	P_Cood(	1,	P3	)	);
	TempD(16)	=		P_Cood(	2,	P2	)	-	(	TempD(15)	*	P_Cood(	1,	P2	)	);

	X1	=	floor(P_Cood(	1,	P1	))+1	:	floor(P_Cood(	1,	P2	)	);

	X2	=	floor(P_Cood(	1,	P1	))+1	:	floor(P_Cood(	1,	P3	)	);

	X3	=	floor(P_Cood(	1,	P3	))+1	:	floor(P_Cood(	1,	P2	)	);

	XY1	=	[];
	XY2	=	[];
	XY3	=	[];
	if	(	isempty(X1)~=1	)
		for	i	=	X1(1):X1(end)
			A	=	TempD(11)	.*	i	+	TempD(12);

			XY1	=	[	XY1,	[i;A]	];

			if	i	<=	floor(P_Cood(	1,	P3	)	)
				B	=	TempD(13)	.*	i	+	TempD(14);
				XY2	=	[	XY2,	[i;A]	];

			else
				B	=	TempD(15)	.*	i	+	TempD(16);
				XY2	=	[	XY2,	[i;A]	];
			end

			AA	=	(	A	<=	B	)	*	A	+	(	A	>	B		)	*	B;
			BB	=	(	A	>	B		)	*	A	+	(	A	<=	B	)	*	B;

			for	j	=	floor(AA)+1	:	floor(BB)
				if	(	i>=1	&&	i<=1920	&&	j>=1	&&	j<=1080		)
					BoundOK1(	j,i	)	=	1;
				end
			end
		end
	end














return;



%-----------------------------------------------------------------------------------
%	Internal Function
%-----------------------------------------------------------------------------------
function	[Flag]	=	Determin_1(	P1u,P1v,	P2u,P2v,	P3u,P3v,	P4u,P4v,	Flag	)


	TempI(1)	=	(	P1u	<	P2u	)	+	(	P1u	<	P3u	);
	TempI(2)	=	(	P2u	<	P1u	)	+	(	P2u	<	P3u	);
	TempI(3)	=	(	P_Cood(1,3)	<	P_Cood(1,1)	)	+	(	P_Cood(1,3)	<	P_Cood(1,2)	);

	%/*P1 Min,	P2 Middle,	P3 Max*/
	P1	=	(	(TempI(1)==2)*1	)	+	(	(TempI(2)==2)*2	)	+	(	(TempI(3)==2)*3	);
	P2	=	(	(TempI(1)==1)*1	)	+	(	(TempI(2)==1)*2	)	+	(	(TempI(3)==1)*3	);
	P3	=	(	(TempI(1)==0)*1	)	+	(	(TempI(2)==0)*2	)	+	(	(TempI(3)==0)*3	);

	MaxU	=	P_Cood(1,P3);
	MinU	=	P_Cood(1,P1);

	TempI(1)	=	(	P_Cood(2,1)	<	P_Cood(2,2)	)	+	(	P_Cood(2,1)	<	P_Cood(2,3)	);
	TempI(2)	=	(	P_Cood(2,2)	<	P_Cood(2,1)	)	+	(	P_Cood(2,2)	<	P_Cood(2,3)	);
	TempI(3)	=	(	P_Cood(2,3)	<	P_Cood(2,1)	)	+	(	P_Cood(2,3)	<	P_Cood(2,2)	);

	%/*P1 Min,	P2 Middle,	P3 Max*/
	P1	=	(	(TempI(1)==2)*1	)	+	(	(TempI(2)==2)*2	)	+	(	(TempI(3)==2)*3	);
	P2	=	(	(TempI(1)==1)*1	)	+	(	(TempI(2)==1)*2	)	+	(	(TempI(3)==1)*3	);
	P3	=	(	(TempI(1)==0)*1	)	+	(	(TempI(2)==0)*2	)	+	(	(TempI(3)==0)*3	);

	MaxV	=	P_Cood(2,P3);
	MinV	=	P_Cood(2,P1);

	Flag(1)	=	(	TempD(21)	<	MaxU	&&	TempD(21)	>	MinU	&&	TempD(22)	<	MaxV	&&	TempD(22)	>	MinV	);

return;





%---------------------------------------------
%InternalFunction
%---------------------------------------------
function	[Low,Up]	=	GetRangeOfMotion();


	Low	=	...
		[
			-90;		-90;		-90;			%	1		Lhip			(	7,	8,	9		)
			-90;		-90;		-90;			%	2		Rhip			(	10,	11,	12	)
			-90;		-90;		-90;			%	3		RibCenter	(	13,	14,	15	)
			-10;		-15;		-15;			%	4		LKnee			(	16,	17,	18	)
			-10;		-15;		-15;			%	5		RKnee			(	19,	20,	21	)
			-0.01;	-0.01;	-0.01;		%	6		Ç›ÇºÇ®Çø	(	22,	23,	24	)
			-45;		-15;		-45;			%	7		LAnkle		(	25,	26,	27	)
			-45;		-15;		-45;			%	8		RAnkle		(	28,	29,	30	)
			-0.01;	-0.01;	-0.01;		%	9		Ç›ÇºÇ®Çøè„(	31,	32,	33	)
			-90;		-15;		-15;			%	10	LMP				(	34,	35,	36	)
			-90;		-15;		-15;			%	11	RMP				(	37,	38,	39	)
			-90;		-90;		-90;			%	12	Neckâ∫		(	40,	41,	42	)
			-0.01;	-90;		-90;			%	13	Lå®çbçú		(	43,	44,	45	)
			-0.01;	-90;		-90;			%	14	Rå®çbçú		(	46,	47,	48	)
			-90;		-90;		-90;			%	12	Neckâ∫		(	49,	50,	51	)
			-90;		-90;		-90;			%	16	LShoulder	(	52,	53,	54	)
			-90;		-90;		-10;			%	17	RSholder	(	55,	56,	57	)
			-90;		-150;		-15;			%	18	LElbow		(	58,	59,	60	)
			-90;		-10;		-15;			%	19	RElbow		(	61,	62,	63	)
			-15;			-15;	-90;			%	20	LWrist		(	64,	65,	66	)
			-15;			-15;	-90;			%	21	RWrist		(	67,	68,	69	)
			-15;			-15;	-45;			%	22	LHand			(	70,	71,	72	)
			-15;			-15;	-45;			%	23	RHand			(	73,	74,	75	)
		];


	Up	=	...
		[
			90;			90;			90;				%	1		Lhip			(	7,	8,	9		)
			90;			90;			90;				%	2		Rhip			(	10,	11,	12	)
			90;			90;			90;				%	3		RibCenter	(	13,	14,	15	)
			140;		15;			15;				%	4		LKnee			(	16,	17,	18	)
			140;		15;			15;				%	5		RKnee			(	19,	20,	21	)
			0.01;		0.01;		0.01;			%	6		Ç›ÇºÇ®Çø	(	22,	23,	24	)
			90;			15;			45;				%	7		LAnkle		(	25,	26,	27	)
			90;			15;			45;				%	8		RAnkle		(	28,	29,	30	)
			0.01;		0.01;		0.01;			%	9		Ç›ÇºÇ®Çøè„(	31,	32,	33	)
			90;			15;			15;				%	10	LMP				(	34,	35,	36	)
			90;			15;			15;				%	11	RMP				(	37,	38,	39	)
			90;			90;			90;				%	12	Neckâ∫		(	40,	41,	42	)
			0.01;		90;			90;				%	13	Lå®çbçú		(	43,	44,	45	)
			0.01;		90;			90;				%	14	Rå®çbçú		(	46,	47,	48	)
			90;			90;			90;				%	12	Neckâ∫		(	49,	50,	51	)
			90;			90;			10;				%	16	LShoulder	(	52,	53,	54	)
			90;			90;			90;				%	17	RSholder	(	55,	56,	57	)
			90;			10;			15;				%	18	LElbow		(	58,	59,	60	)
			90;			150;		15;				%	19	RElbow		(	61,	62,	63	)
			15;			15;			90;				%	20	LWrist		(	64,	65,	66	)
			15;			15;			90;				%	21	RWrist		(	67,	68,	69	)
			15;			15;			45;				%	22	LHand			(	70,	71,	72	)
			15;			15;			45;				%	23	RHand			(	73,	74,	75	)
		];

return;

%=============================================================================
%Internal Function(Camera Cood)
%=============================================================================
function	Comf_Image(	nCam,	nImage,	Input	)

	for	iImage	=	1:nImage
		F_h(iImage)		=	figure(	'position',	[480, 240, 640*2,360*2]	);
		A_h(iImage,1)	=	axes(		'Parent',F_h(iImage),	'Position',[	0,		0,		0.5,	0.5	]	);
		A_h(iImage,2)	=	axes(		'Parent',F_h(iImage),	'Position',[	0,		0.5,	0.5,	0.5	]	);
		A_h(iImage,3)	=	axes(		'Parent',F_h(iImage),	'Position',[	0.5,	0,		0.5,	0.5	]	);
		A_h(iImage,4)	=	axes(		'Parent',F_h(iImage),	'Position',[	0.5,	0.5,	0.5,	0.5	]	);
	end

	for	iCam	=	1:nCam

		for	iImage	=	1:nImage

			Gray			=	uint8(	200.*(Input(iCam).Label(	iImage,	:,:)==0)	);

			Input(iCam).img_b(	iImage,	:,:,1)	=	Input(iCam).img_b(	iImage,	:,:,1).*Input(iCam).Label(	iImage,	:,:)	+	Gray;
			Input(iCam).img_b(	iImage,	:,:,2)	=	Input(iCam).img_b(	iImage,	:,:,2).*Input(iCam).Label(	iImage,	:,:)	+	Gray;
			Input(iCam).img_b(	iImage,	:,:,3)	=	Input(iCam).img_b(	iImage,	:,:,3).*Input(iCam).Label(	iImage,	:,:)	+	Gray;

			CData(:,:,:)	=	Input(iCam).img_b(	iImage,	:,:,:);
			image(	CData,	'Parent',	A_h(iImage,iCam)	)

			line(	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	1	),...
						Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	1	),...
													'LineStyle',	'none',	'MarkerSize',	8,...
													'Marker',			'o',		'MarkerEdgeColor','k',	'MarkerFaceColor',[1,0,0],	'Parent',	A_h(iImage,iCam)	);

			line(	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	2	),...
						Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	2	),...
													'LineStyle',	'none',	'MarkerSize',	8,...
													'Marker',			'o',		'MarkerEdgeColor','k',	'MarkerFaceColor',[1,1,0],	'Parent',	A_h(iImage,iCam)	);

			line(	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	3	),...
						Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	3	),...
													'LineStyle',	'none',	'MarkerSize',	8,...
													'Marker',			'o',		'MarkerEdgeColor','k',	'MarkerFaceColor',[1,0,1],	'Parent',	A_h(iImage,iCam)	);

			line(	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	4	),...
						Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	4	),...
													'LineStyle',	'none',	'MarkerSize',	8,...
													'Marker',			'o',		'MarkerEdgeColor','k',	'MarkerFaceColor',[1,0.5,0],	'Parent',	A_h(iImage,iCam)	);

			line(	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	5	),...
						Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	5	),...
													'LineStyle',	'none',	'MarkerSize',	8,...
													'Marker',			'o',		'MarkerEdgeColor','k',	'MarkerFaceColor',[1,0,0.5],	'Parent',	A_h(iImage,iCam)	);


			line(	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	6	),...
						Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	6	),...
													'LineStyle',	'none',	'MarkerSize',	8,...
													'Marker',			'o',		'MarkerEdgeColor','k',	'MarkerFaceColor',[1,0,0],	'Parent',	A_h(iImage,iCam)	);

			line(	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	7	),...
						Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	7	),...
													'LineStyle',	'none',	'MarkerSize',	8,...
													'Marker',			'o',		'MarkerEdgeColor','k',	'MarkerFaceColor',[0,1,0],	'Parent',	A_h(iImage,iCam)	);

			line(	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	8	),...
						Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	8	),...
													'LineStyle',	'none',	'MarkerSize',	8,...
													'Marker',			'o',		'MarkerEdgeColor','k',	'MarkerFaceColor',[0,0,1],	'Parent',	A_h(iImage,iCam)	);


			line(	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	9	),...
						Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	9	),...
													'LineStyle',	'none',	'MarkerSize',	8,...
													'Marker',			'^',		'MarkerEdgeColor','k',	'MarkerFaceColor',[1,0,0],	'Parent',	A_h(iImage,iCam)	);

			line(	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	10	),...
						Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	10	),...
													'LineStyle',	'none',	'MarkerSize',	8,...
													'Marker',			'^',		'MarkerEdgeColor','k',	'MarkerFaceColor',[0,1,0],	'Parent',	A_h(iImage,iCam)	);

			line(	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	11	),...
						Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	11	),...
													'LineStyle',	'none',	'MarkerSize',	8,...
													'Marker',			'^',		'MarkerEdgeColor','k',	'MarkerFaceColor',[0,0,1],	'Parent',	A_h(iImage,iCam)	);



			line(	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	12	),...
						Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	12	),...
													'LineStyle',	'none',	'MarkerSize',	8,...
													'Marker',			'square',		'MarkerEdgeColor','k',	'MarkerFaceColor',[1,0,0],	'Parent',	A_h(iImage,iCam)	);

			line(	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	13	),...
						Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	13	),...
													'LineStyle',	'none',	'MarkerSize',	8,...
													'Marker',			'square',		'MarkerEdgeColor','k',	'MarkerFaceColor',[0,1,0],	'Parent',	A_h(iImage,iCam)	);

			line(	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	14	),...
						Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	14	),...
													'LineStyle',	'none',	'MarkerSize',	8,...
													'Marker',			'square',		'MarkerEdgeColor','k',	'MarkerFaceColor',[0,0,1],	'Parent',	A_h(iImage,iCam)	);


			line(	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	15	),...
						Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	15	),...
													'LineStyle',	'none',	'MarkerSize',	8,...
													'Marker',			'diamond',		'MarkerEdgeColor','k',	'MarkerFaceColor',[1,0,0],	'Parent',	A_h(iImage,iCam)	);

			line(	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	16	),...
						Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	16	),...
													'LineStyle',	'none',	'MarkerSize',	8,...
													'Marker',			'diamond',		'MarkerEdgeColor','k',	'MarkerFaceColor',[0,1,0],	'Parent',	A_h(iImage,iCam)	);

			line(	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	17	),...
						Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	17	),...
													'LineStyle',	'none',	'MarkerSize',	8,...
													'Marker',			'diamond',		'MarkerEdgeColor','k',	'MarkerFaceColor',[0,0,1],	'Parent',	A_h(iImage,iCam)	);


		end
	end


return;


%=============================================================================
%Internal Function(Camera Cood)
%=============================================================================
function	[Vertex,	GrobalJoint	]	=	Get_Vertex(	Betas,	Pose,	Trans,...
																								Shape_Dirs,...
																								V_Temp,...
																								J_Regressor,...
																								Pose_Dirs,...
																								Kintree_Table,...
																								Weights	)

	%------------
	%--Body Type
	%------------
	%-Shape

	AddComp	=	zeros(	6890,	3	);
	for	i	=	1:3
		Co										=	zeros(	6890,	300	);
		Co(	1:6890,	1:300	)		=	Shape_Dirs(	1:6890,	i,	1:300	);
		AddComp(	1:6890,	i	)	=	Co	*	Betas;
	end
	V_Shaped	=	AddComp	+	V_Temp;

%	V_Shaped(1:100,:)
%	V_Shaped(end-100:end-0,:)

	for	i	=	1:23
		R1	=	(	(i-1)*3+1:(i-1)*3+3	)	+	3;
		R2	=	(	(i-1)*4+1:(i-1)*4+4	);

		Angle	=	Pose(R1);
		Leng	=	sqrt(	sum(	Angle.*Angle	)	);
		if	Leng	<	1e-10
			Leng	=	1e-10;
		end

		NormAng	=	Angle	./	Leng;

		C_Leng	=	cos(	Leng/2	);
		S_Leng	=	sin(	Leng/2	);

		QX			=	NormAng(1)	*	S_Leng;
		QY			=	NormAng(2)	*	S_Leng;
		QZ			=	NormAng(3)	*	S_Leng;
		QW			=	C_Leng	-	1;

		Quat_Ang(R2)	=	[	QX;	QY;	QZ;	QW	];
	end

	%-------------------------
	%--Change Shape by pose
	%-------------------------
	Shape_Feat	=	Betas(2);
	Feat				=	[Quat_Ang,	Shape_Feat]';

	AddComp	=	zeros(	6890,	3	);
	for	i	=	1:3
		Co										=	zeros(	6890,	93	);
		Co(	1:6890,	1:93	)		=	Pose_Dirs(	1:6890,	i,	1:93	);
		AddComp(	1:6890,	i	)	=	Co	*	Feat;
	end
	V_Pose	=	AddComp	+	V_Shaped;

%	V_Pose(1:100,:)
%	V_Pose(end-100:end-0,:)

	%-Joint
	Joint	=	J_Regressor'	*	V_Shaped;


	nJoint		=	length(	Kintree_Table(:,1)	);
	ParentID	=	Kintree_Table(2:end,1);
	ParentID	=	ParentID	+	1;

	JointID		=	Kintree_Table(1:end,2);
	JointID		=	JointID		+	1;

	%--Origin Joint
	i				=	1;
	R				=	((i-1)*3)+1:((i-1)*3)+3;
	Vecter	=	Pose(R);
	Rm			=	Rodrigues(	Vecter	);

	HTrans(1).Mat				=	[	Rm	Joint(1,:)'	];
	HTrans(1).Mat(4,4)	=	1;
	GrobalJoint					=	HTrans(1).Mat(1:3,4)';

	for	i	=	2:nJoint
		JointITI	=	i;
		ParentITI	=	ParentID(i-1);
%		[	JointITI,	ParentITI	]
		LoJoint	=	Joint(	JointITI,	:	)	-	Joint(	ParentITI,	:	);

		R				=	(	(JointITI-1)*3	)	+	1	:	(	(JointITI-1)*3	)	+	3;

		Vecter	=	Pose(R);
		Rm			=	Rodrigues(	Vecter	);

		Temp_HTrans				=	[	Rm	LoJoint(1,:)'	];
		Temp_HTrans(4,4)	=	1;

		HTrans(JointITI).Mat	=	HTrans(ParentITI).Mat	*	Temp_HTrans;

		GrobalJoint	=	[	GrobalJoint;	HTrans(JointITI).Mat(1:3,4)'];
	end



	HTransMat2	=	zeros(	4,	4,	nJoint	);
	for	i	=	1:nJoint
		Temp							=	HTrans(i).Mat	*	[	Joint(i,:)';	0	];
		Temp							=	[zeros(4,3),	Temp];
		HTrans(i).Mat2		=	HTrans(i).Mat	-	Temp;
		HTransMat2(:,:,i)	=	HTrans(i).Mat2;
	end

	Mody_Weights	=	zeros(	4,	4,	6890	);
	for	i	=	1:4
		Co									=	zeros(4,24);
		Co(:,:)							=	HTransMat2(	i,	:,	:	);
		Mody_Weights(i,:,:)	=	(	Co	*	Weights'	);
	end


	V_Pose(	:,	4	)	=	1;
	Pick_Weights		=	zeros(	6890,	1	);
	for	i	=	1:4
		j	=	1;
		Pick_Weights(:,1)	=	Mody_Weights(	i,	j,	:	);
		Vertex(:,i)				=	Pick_Weights(	:,	1	)	.*	V_Pose(	:,	j	);

		for	j	=	2:4
			Pick_Weights(:,1)	=	Mody_Weights(	i,	j,	:	);
			Vertex(:,i)				=	Vertex(:,i)	+	(	Pick_Weights(	:,	1	)	.*	V_Pose(	:,	j	)	);
		end
	end


	Vertex	=	Vertex(:,1:3);
	Vertex(:,1)	=	Vertex(:,1)	+	Trans(1);
	Vertex(:,2)	=	Vertex(:,2)	+	Trans(2);
	Vertex(:,3)	=	Vertex(:,3)	+	Trans(3);


	GrobalJoint(:,1)	=	GrobalJoint(:,1)	+	Trans(1);
	GrobalJoint(:,2)	=	GrobalJoint(:,2)	+	Trans(2);
	GrobalJoint(:,3)	=	GrobalJoint(:,3)	+	Trans(3);

return;

%=============================================================================
%Internal Function(Camera Cood)
%=============================================================================
function	Rm	=	Rodrigues(	Vecter	);

	N1	=	Vecter(1);
	N2	=	Vecter(2);
	N3	=	Vecter(3);

	Theta	=	(	(N1^2)	+	(N2^2)	+	(N3^2)	).^(1/2);

	if	Theta	<	1.0e-12
		N1	=	N1;
		N2	=	N2;
		N3	=	N3;
	else
		N1	=	N1/Theta;
		N2	=	N2/Theta;
		N3	=	N3/Theta;
	end

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

	Rm			=	zeros(3,3);
	Rm(1,1)	=	C							+	(	N1N1*p1nC	);
	Rm(2,1)	=	(	N1N2*p1nC	)	+	(	N3*S			);
	Rm(3,1)	=	(	N1N3*p1nC	)	-	(	N2*S			);

	Rm(1,2)	=	(	N1N2*p1nC	)	-	(	N3*S			);
	Rm(2,2)	=	C							+	(	N2N2*p1nC	);
	Rm(3,2)	=	(	N2N3*p1nC	)	+	(	N1*S			);

	Rm(1,3)	=	(	N1N3*p1nC	)	+	(	N2*S			);
	Rm(2,3)	=	(	N2N3*p1nC	)	-	(	N1*S			);
	Rm(3,3)	=	C							+	(	N3N3*p1nC	);



return;


%=============================================================================
%Internal Function(Camera Cood)
%=============================================================================
function	Pose	=	Euler_Rodrigues(	Pose	)

	for	i	=	1:24
		R	=	(i-1)*3+1:(i-1)*3+3;
		TempD(1)	=	Pose(	R(1)	);
		TempD(2)	=	Pose(	R(2)	);
		TempD(3)	=	Pose(	R(3)	);

		Rmx(1,1)	=	1;									Rmx(1,2)	=	0;									Rmx(1,3)	=	0;
		Rmx(2,1)	=	0;									Rmx(2,2)	=	cos(	TempD(1)	);	Rmx(2,3)	=	-sin(	TempD(1)	);
		Rmx(3,1)	=	0;									Rmx(3,2)	=	sin(	TempD(1)	);	Rmx(3,3)	=	cos(	TempD(1)	);

		Rmy(1,1)	=	cos(	TempD(2)	);	Rmy(1,2)	=	0;									Rmy(1,3)	=	sin(	TempD(2)	);
		Rmy(2,1)	=	0;									Rmy(2,2)	=	1;									Rmy(2,3)	=	0;
		Rmy(3,1)	=	-sin(	TempD(2)	);	Rmy(3,2)	=	0;									Rmy(3,3)	=	cos(	TempD(2)	);

		Rmz(1,1)	=	cos(	TempD(3)	);	Rmz(1,2)	=	-sin(	TempD(3)	);	Rmz(1,3)	=	0;
		Rmz(2,1)	=	sin(	TempD(3)	);	Rmz(2,2)	=	cos(	TempD(3)	);	Rmz(2,3)	=	0;
		Rmz(3,1)	=	0;									Rmz(3,2)	=	0;									Rmz(3,3)	=	1;

		Rm	=	Rmx	*	Rmy	*	Rmz;

		TempD(1)	=	(	Rm(1,1)	+	Rm(2,2)	+	Rm(3,3)	-	1	)	/	2;
		Ang				=	acos(	TempD(1)	);

		if	sin(	Ang	)	<	0.000001
			R	=	(i-1)*3+1:(i-1)*3+3;
			TempD(1)	=	Pose(	R(1)	)	+	(	(pi/180)*0.1	);
			TempD(2)	=	Pose(	R(2)	)	+	(	(pi/180)*0.1	);
			TempD(3)	=	Pose(	R(3)	)	+	(	(pi/180)*0.1	);

			Rmx(1,1)	=	1;									Rmx(1,2)	=	0;									Rmx(1,3)	=	0;
			Rmx(2,1)	=	0;									Rmx(2,2)	=	cos(	TempD(1)	);	Rmx(2,3)	=	-sin(	TempD(1)	);
			Rmx(3,1)	=	0;									Rmx(3,2)	=	sin(	TempD(1)	);	Rmx(3,3)	=	cos(	TempD(1)	);

			Rmy(1,1)	=	cos(	TempD(2)	);	Rmy(1,2)	=	0;									Rmy(1,3)	=	sin(	TempD(2)	);
			Rmy(2,1)	=	0;									Rmy(2,2)	=	1;									Rmy(2,3)	=	0;
			Rmy(3,1)	=	-sin(	TempD(2)	);	Rmy(3,2)	=	0;									Rmy(3,3)	=	cos(	TempD(2)	);

			Rmz(1,1)	=	cos(	TempD(3)	);	Rmz(1,2)	=	-sin(	TempD(3)	);	Rmz(1,3)	=	0;
			Rmz(2,1)	=	sin(	TempD(3)	);	Rmz(2,2)	=	cos(	TempD(3)	);	Rmz(2,3)	=	0;
			Rmz(3,1)	=	0;									Rmz(3,2)	=	0;									Rmz(3,3)	=	1;

			Rm	=	Rmx	*	Rmy	*	Rmz;
			TempD(1)	=	(	Rm(1,1)	+	Rm(2,2)	+	Rm(3,3)	-	1	)	/	2;
			Ang				=	acos(	TempD(1)	);
		end

		N1	=	(	Rm(3,2)	-	Rm(2,3)	)	/	(	2*sin(Ang)	);
		N2	=	(	Rm(1,3)	-	Rm(3,1)	)	/	(	2*sin(Ang)	);
		N3	=	(	Rm(2,1)	-	Rm(1,2)	)	/	(	2*sin(Ang)	);

		N1	=	N1	*	Ang;
		N2	=	N2	*	Ang;
		N3	=	N3	*	Ang;

		R	=	(i-1)*3+1:(i-1)*3+3;
		Pose(	R(1)	)	=	N1;
		Pose(	R(2)	)	=	N2;
		Pose(	R(3)	)	=	N3;
	end

return;






function	Cam	=	Get_CamPara(	Width,	Height,	nCam,	Cam	);

	for	iCam	=	1:nCam;
		Cam(iCam).nSean					=	1;
		Cam(iCam).Pix_Yoko			=	Width;
		Cam(iCam).Pix_Tate			=	Height;
		Cam(iCam).OriUo					=	(Cam(iCam).Pix_Yoko+1)	/	2;
		Cam(iCam).OriVo					=	(Cam(iCam).Pix_Tate+1)	/	2;

		%================================
		%TriPod_Position and LocalCood
		%================================
		Cam(iCam).TriPod_Position		=	Cam(iCam).CamPara(2:4);
		Cam(iCam).TriPod_Lo_J1_J2		=	[	0.00;	0.00;	0.00	];
		Cam(iCam).TriPod_Lo_J2_J3		=	[	0.00;	0.00;	0.00	];
		Cam(iCam).TriPod_Lo_J3_CL		=	[	0.00;	0.00;	0.00	];
		Cam(iCam).LocalCoodMat	=...
					[
						Cam(iCam).TriPod_Position;
						Cam(iCam).TriPod_Lo_J1_J2;
						Cam(iCam).TriPod_Lo_J2_J3;
						Cam(iCam).TriPod_Lo_J3_CL
					];

		%==================
		%	CamPosture
		%==================
		%--TriPod_Angle
		Cam(iCam).TriPod_Angle	=	[	0;	0	];
		Cam(iCam).S_Tripod			=	sin(	Cam(iCam).TriPod_Angle	);
		Cam(iCam).C_Tripod			=	cos(	Cam(iCam).TriPod_Angle	);

		%--OpticAxis
		Cam(iCam).OpticAxis_Angle	=	Cam(iCam).CamPara(7);
		Cam(iCam).S_OpticAxis			=	sin(	Cam(iCam).OpticAxis_Angle	);
		Cam(iCam).C_OpticAxis			=	cos(	Cam(iCam).OpticAxis_Angle	);

		%--FValue
		Cam(iCam).F		=	Cam(iCam).CamPara(1);

		Cam(iCam).Angle	=	Cam(iCam).CamPara(5:6);

		Cam(iCam).S			=	sin(	Cam(iCam).Angle	);
		Cam(iCam).C			=	cos(	Cam(iCam).Angle	);

		%============
		%Tripod_X Rm
		%============
		Cam(iCam).Tripod_X_Rm	=...
					[
						1;
						0;
						0;
						0;
						Cam(iCam).C_Tripod(1);
						Cam(iCam).S_Tripod(1);
						0
						-1*Cam(iCam).S_Tripod(1);
						Cam(iCam).C_Tripod(1);
					];

		%============
		%Tripod_Y Rm
		%============
		Cam(iCam).Tripod_Y_Rm	=...
					[
						Cam(iCam).C_Tripod(2);
						0;
						-1*Cam(iCam).S_Tripod(2);
						0;
						1;
						0;
						Cam(iCam).S_Tripod(2);
						0;
						Cam(iCam).C_Tripod(2);
					];

		%==========
		%Joint1 Rm
		%==========
		Cam(iCam).J1_Rm	=...
					[
						Cam(iCam).C(2:2:end-0)';
						Cam(iCam).S(2:2:end-0)';
						zeros(	1,	Cam(iCam).nSean	);
						-Cam(iCam).S(2:2:end-0)';
						Cam(iCam).C(2:2:end-0)';
						zeros(	1,	Cam(iCam).nSean	);
						zeros(	1,	Cam(iCam).nSean	);
						zeros(	1,	Cam(iCam).nSean	);
						ones(		1,	Cam(iCam).nSean	);
					];

		%==========
		%Joint2 Rm
		%==========
		Cam(iCam).J2_Rm	=...
						[
							ones(		1,	Cam(iCam).nSean	);
							zeros(	1,	Cam(iCam).nSean	);
							zeros(	1,	Cam(iCam).nSean	);
							zeros(	1,	Cam(iCam).nSean	);
							Cam(iCam).C(1:2:end-1)';
							Cam(iCam).S(1:2:end-1)';
							zeros(	1,	Cam(iCam).nSean	);
							-Cam(iCam).S(1:2:end-1)';
							Cam(iCam).C(1:2:end-1)';
						];

		%==========
		%Joint3 Rm
		%==========
		Cam(iCam).J3_Rm	=...
						[
							Cam(iCam).C_OpticAxis(1);
							0;
							-Cam(iCam).S_OpticAxis(1);
							0;
							1;
							0;
							Cam(iCam).S_OpticAxis(1);
							0;
							Cam(iCam).C_OpticAxis(1);
						];

		%================
		%Tripod Posture
		%================
		for	i	=	1:3
			for	j	=	1:3
				k	=	(i-1)*3	+	j;
				Cam(iCam).Gro_Tripod_Rm(k,:)	=...
									(	Cam(iCam).Tripod_X_Rm(	j,:	)		.*	Cam(iCam).Tripod_Y_Rm(	(i-1)*3+1,:	)	)...
								+	(	Cam(iCam).Tripod_X_Rm(	j+3,:	)	.*	Cam(iCam).Tripod_Y_Rm(	(i-1)*3+2,:	)	)...
								+	(	Cam(iCam).Tripod_X_Rm(	j+6,:	)	.*	Cam(iCam).Tripod_Y_Rm(	(i-1)*3+3,:	)	);
			end
		end

		%================
		%Segment1 Posture
		%================
		for	i	=	1:3
			for	j	=	1:3
				k	=	(i-1)*3	+	j;
				Cam(iCam).Gro_S1_Rm(k,:)	=...
									(	Cam(iCam).Gro_Tripod_Rm(	j,:	)		.*	Cam(iCam).J1_Rm(	(i-1)*3+1,:	)	)...
								+	(	Cam(iCam).Gro_Tripod_Rm(	j+3,:	)	.*	Cam(iCam).J1_Rm(	(i-1)*3+2,:	)	)...
								+	(	Cam(iCam).Gro_Tripod_Rm(	j+6,:	)	.*	Cam(iCam).J1_Rm(	(i-1)*3+3,:	)	);
			end
		end

		%================
		%Segment2 Posture
		%================
		for	i	=	1:3
			for	j	=	1:3
				k	=	(i-1)*3	+	j;
				Cam(iCam).Gro_S2_Rm(k,:)	=...
									(	Cam(iCam).Gro_S1_Rm(	j,:	)		.*	Cam(iCam).J2_Rm(	(i-1)*3+1,:	)	)...
								+	(	Cam(iCam).Gro_S1_Rm(	j+3,:	)	.*	Cam(iCam).J2_Rm(	(i-1)*3+2,:	)	)...
								+	(	Cam(iCam).Gro_S1_Rm(	j+6,:	)	.*	Cam(iCam).J2_Rm(	(i-1)*3+3,:	)	);
			end
		end

		%================
		%Segment3 Posture
		%================
		for	i	=	1:3
			for	j	=	1:3
				k	=	(i-1)*3	+	j;
				Cam(iCam).Gro_S3_Rm(k,:)	=...
								(	Cam(iCam).Gro_S2_Rm(	j,:	)		.*	Cam(iCam).J3_Rm(	(i-1)*3+1,:	)	)...
							+	(	Cam(iCam).Gro_S2_Rm(	j+3,:	)	.*	Cam(iCam).J3_Rm(	(i-1)*3+2,:	)	)...
							+	(	Cam(iCam).Gro_S2_Rm(	j+6,:	)	.*	Cam(iCam).J3_Rm(	(i-1)*3+3,:	)	);
			end
		end
		Cam(iCam).CamPosture_Rm	=...
					[	-1*Cam(iCam).Gro_S3_Rm(1:3,:);	Cam(iCam).Gro_S3_Rm(7:9,:);	Cam(iCam).Gro_S3_Rm(4:6,:)	];

		for	i	=	1:3
			R	=	i	:	3	:	(	(Cam(iCam).nSean)*3-(3-i)	);

			Cam(iCam).CL(	R,	1	)	=...
				[		Cam(iCam).Gro_S1_Rm(i+0,:)	.*	Cam(iCam).TriPod_Lo_J1_J2(1,1)...
					+	Cam(iCam).Gro_S1_Rm(i+3,:)	.*	Cam(iCam).TriPod_Lo_J1_J2(2,1)...
					+	Cam(iCam).Gro_S1_Rm(i+6,:)	.*	Cam(iCam).TriPod_Lo_J1_J2(3,1)
				]';

			Cam(iCam).CL(	R,	1	)	=	Cam(iCam).CL(	R,	1	)...
			+	[		Cam(iCam).Gro_S2_Rm(i+0,:)	.*	Cam(iCam).TriPod_Lo_J2_J3(1,1)...
					+	Cam(iCam).Gro_S2_Rm(i+3,:)	.*	Cam(iCam).TriPod_Lo_J2_J3(2,1)...
					+	Cam(iCam).Gro_S2_Rm(i+6,:)	.*	Cam(iCam).TriPod_Lo_J2_J3(3,1)
				]';

			Cam(iCam).CL(	R,	1	)	=	Cam(iCam).CL(	R,	1	)...
			+	[		Cam(iCam).Gro_S3_Rm(i+0,:)	.*	Cam(iCam).TriPod_Lo_J3_CL(1,1)...
					+	Cam(iCam).Gro_S3_Rm(i+3,:)	.*	Cam(iCam).TriPod_Lo_J3_CL(2,1)...
					+	Cam(iCam).Gro_S3_Rm(i+6,:)	.*	Cam(iCam).TriPod_Lo_J3_CL(3,1)
				]';

			Cam(iCam).CL(	R,	1	)	=	Cam(iCam).CL(	R,	1	)	+	Cam(iCam).TriPod_Position(i);
		end
	end

return;