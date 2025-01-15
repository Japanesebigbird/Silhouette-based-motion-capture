function	FitOpt_Ver2();

close all
clear all

	global	Shape_Dirs	V_Temp	J_Regressor		Pose_Dirs		Kintree_Table		Weights		Cam	BasePara		V_Point	Input	nImage	nCam	NewV_Point_32

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

	VerMat		=	[];
	ITIMat		=	[];
	CoMat_X		=	[];
	CoMat_Y		=	[];
	CoMat_Z		=	[];
	nFeatMat	=	[];
	for	i	=	1:6890

		XComp			=	zeros(93,1);
		XComp(:)	=	Pose_Dirs(i,1,:);
		ITI				=	find(	XComp	~=	0	);
		ITIMat		=	[	ITIMat;	(ITI-1)	];

		VerMat		=	[	VerMat;	repmat(	(i-1),	[length(ITI),	1	]	)	];
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

	SaveInt		=	ITIMat;
	SaveInt		=	[	SaveInt;	10000	];

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
	[Tate,Yoko]	=	size(NewV_Point_32)
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


	V_Point				=	V_Point				+	1;
	NewV_Point_32	=	NewV_Point_32	+	1;


	%===================
	%Camera Parameter
	%===================
	Cam(1).Ref_RealCood	=	[	[0;0;0],	[2;0;0],	[0;1;0],	[2;1;0],	[0;2;0],	[2;2;0],	[0;3;0],	[2;3;0],	[0;4;0],	[2;4;0]	];
	nCam		=	8;

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

	%------------------
	%Renge of Search
	%------------------
	[Joint_Low,Joint_Up]	=	GetRangeOfMotion(	);
	Joint_Low							=	Joint_Low.*(pi/180);
	Joint_Up							=	Joint_Up.*(pi/180);

	nStage	=	2;
	nFr			=	Input(1).nList;
	PickUpList_Stage(2).PickUpList	=	[1 nFr];

	%------------------
	%	PickUpList
	%------------------
	List	=	[];
	for	iFr	=	1:nFr
		List	=	[	List;	[iFr,iFr]	];
	end
	PickUpList_Stage(1).PickUpList	=	List;


	Ver			=	2;
	B_Ver		=	Ver-1;
	nLoop	=	length(	PickUpList_Stage(B_Ver).PickUpList(:,1)	)
	for	iLoop	=	1:nLoop

		%------------
		%--PickUpList
		%------------
		List				=	PickUpList_Stage(B_Ver).PickUpList(iLoop,1)	:	PickUpList_Stage(B_Ver).PickUpList(iLoop,2);
		nImage_Temp	=	length(	List	);

		FileName	=	[	'OptimizationData_Stage' num2str(B_Ver) '_' num2str(iLoop) '.Dat'	];
		LoadName	=	[	'.\Stage' num2str(B_Ver) '_OutputFile\' FileName];

		ID	=	0;
		while	ID	<=	2
			ID				=	fopen(LoadName);
		end
		FFF	=	fread(ID,'double');
		fclose(ID);
		Q	=	FFF;

		Betas	=	Q(	(nImage_Temp)*75+1	:	(nImage_Temp)*75+300	);

		for	iImage	=	1:nImage_Temp
			iFr	=	List(iImage);

			R	=	(iImage-1)*75+1	:	(iImage-1)*75+75;
			Q_iImage	=	Q(	R	);
			QData(iFr).Q	=	[	Q_iImage;	Betas	];
		end
	end

	nLoop					=	length(	PickUpList_Stage(Ver).PickUpList(:,1)	);
	for	iLoop	=	1:nLoop

		%------------
		%--PickUpList
		%------------
		PickUpList	=	PickUpList_Stage(Ver).PickUpList(iLoop,1)	:	PickUpList_Stage(Ver).PickUpList(iLoop,2);
		nImage			=	length(PickUpList);
		nImage_Temp	=	length(PickUpList);


		%-----------------
		%--About Posture
		%-----------------
		lb_Posture	=	[];	ub_Posture	=	[];	Q0_Posture	=	[];
		for	iImage	=	1:length(PickUpList)
			iFr	=	PickUpList(iImage);
			Trans				=	QData(iFr).Q(1:3);
			Trans_Low		=	Trans	-	[	0.1;	0.1;	0.1	];
			Trans_Up		=	Trans	+	[	0.1;	0.1;	0.1	];

			%Rotate
			Pose		=	QData(iFr).Q(4:75);

			Pose_Low	=	Pose	-	pi/18;
			Pose_Up		=	Pose	+	pi/18;

			Pose_Low(4:72)	=	(	(	Pose_Low(4:72)	<	Joint_Low	)	.*	Joint_Low		)	+	(	(	Pose_Low(4:72)	>=	Joint_Low	)	.*	Pose_Low(4:72)	);
			Pose_Up(4:72)		=	(	(	Pose_Up(4:72)		>	Joint_Up	)	.*	Joint_Up		)	+	(	(	Pose_Up(4:72)		<=	Joint_Up	)	.*	Pose_Up(4:72)		);

			lb_Posture	=	[	lb_Posture;	Trans_Low;	Pose_Low	];
			ub_Posture	=	[	ub_Posture;	Trans_Up;		Pose_Up		];
			Q0_Posture	=	[	Q0_Posture;	Trans;			Pose			];
		end

		%---------------
		%--About Betas
		%---------------
		BetasMat	=	[];
		for	iImage	=	1:nImage_Temp
			iFr	=	PickUpList(iImage);
			R					=	(1*75)	+	1	:	(1*75)	+	300;
			BetasMat	=	[	BetasMat,	QData(iFr).Q(	R	)	];
		end
		Q0_Betas	=	mean(BetasMat,		2	);
		STD_Betas	=	std(BetasMat,	[],	2	);

		ub_Betas	=	Q0_Betas	+	STD_Betas;
		lb_Betas	=	Q0_Betas	-	STD_Betas;

		lb_Betas		=	(	(lb_Betas<-5)	.*	(-5)	)	+	(	(lb_Betas>=-5)	.*	lb_Betas	);
		ub_Betas		=	(	(ub_Betas>5)	.*	(5)		)	+	(	(lb_Betas<=5)		.*	ub_Betas	);

		lb	=	[	lb_Posture;	lb_Betas	];
		ub	=	[	ub_Posture;	ub_Betas	];
		Q0	=	[	Q0_Posture;	Q0_Betas	];

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
			Input(iCam).Label					=	zeros(	nImage_Temp,	Input(iCam).MovieData.Height,	Input(iCam).MovieData.Width			);
			Input(iCam).img_b					=	zeros(	nImage_Temp,	Input(iCam).MovieData.Height,	Input(iCam).MovieData.Width,	3	);
			Input(iCam).PoseData_Curr	=	[];

			for	iCounter	=	1:nImage_Temp
				[iCam,iCounter]
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


		%---------------------
		%--SaveBaseParameter
		%---------------------
		nKeyPoint	=	17;
		ObjeFlag	=	1;

		SaveInt	=	[	nCam;	nImage_Temp;	nKeyPoint;	ObjeFlag	];
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
			for	iImage	=	1:nImage_Temp;

				SaveSingle							=	zeros(nKeyPoint*2,1);
				SaveSingle(1:2:(nKeyPoint*2-1),1)	=	Input(iCam).PoseData_Curr(	(iImage-1)*2+1,	:	)';
				SaveSingle(2:2:(nKeyPoint*2-0),1)	=	Input(iCam).PoseData_Curr(	(iImage-1)*2+2,	:	)';

				SaveSingle							=	[	SaveSingle;	10^10	];
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
			for	iImage	=	1:nImage_Temp;

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

		%-----------------------
		%‚±‚±‚ÅCUDA‚ðŽg‚¢‚Ü‚·
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


			FileName	=	[	'OptimizationData_Stage' num2str(Ver) '_' num2str(iLoop) '.Dat'	];
			SaveName	=	[	'.\Stage' num2str(Ver) '_OutputFile\' FileName];
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
	end

return;

%-----------------------------------------------------------------------------------
%	Internal Function
%-----------------------------------------------------------------------------------
function	[Low,Up]	=	GetRangeOfMotion();

	Low	=	...
		[
			-90;		-90;		-90;			%	1		Lhip			(	7,	8,	9		)
			-90;		-90;		-90;			%	2		Rhip			(	10,	11,	12	)
			-90;		-90;		-90;			%	3		RibCenter	(	13,	14,	15	)
			-10;		-15;		-15;			%	4		LKnee			(	16,	17,	18	)
			-10;		-15;		-15;			%	5		RKnee			(	19,	20,	21	)
			-0.01;	-0.01;	-0.01;		%	6		‚Ý‚¼‚¨‚¿	(	22,	23,	24	)
			-45;		-15;		-45;			%	7		LAnkle		(	25,	26,	27	)
			-45;		-15;		-45;			%	8		RAnkle		(	28,	29,	30	)
			-0.01;	-0.01;	-0.01;		%	9		‚Ý‚¼‚¨‚¿ã(	31,	32,	33	)
			-90;		-15;		-15;				%	10	LMP				(	34,	35,	36	)
			-90;		-15;		-15;				%	11	RMP				(	37,	38,	39	)
			-90;		-90;		-90;			%	12	Neck‰º		(	40,	41,	42	)
			-0.01;	-90;		-90;			%	13	LŒ¨bœ		(	43,	44,	45	)
			-0.01;	-90;		-90;			%	14	RŒ¨bœ		(	46,	47,	48	)
			-90;		-90;		-90;			%	12	Neck‰º		(	49,	50,	51	)
			-90;		-90;		-90;			%	16	LShoulder	(	52,	53,	54	)
			-90;		-90;		-10;			%	17	RSholder	(	55,	56,	57	)
			-90;		-150;		-15;			%	18	LElbow		(	58,	59,	60	)
			-90;		-10;		-15;			%	19	RElbow		(	61,	62,	63	)
			-15;			-15;			-90;			%	20	LWrist		(	64,	65,	66	)
			-15;			-15;			-90;			%	21	RWrist		(	67,	68,	69	)
			-15;			-15;			-45;			%	22	LHand			(	70,	71,	72	)
			-15;			-15;			-45;			%	23	RHand			(	73,	74,	75	)
		];


	Up	=	...
		[
			90;			90;			90;				%	1		Lhip			(	7,	8,	9		)
			90;			90;			90;				%	2		Rhip			(	10,	11,	12	)
			90;			90;			90;				%	3		RibCenter	(	13,	14,	15	)
			140;		15;			15;				%	4		LKnee			(	16,	17,	18	)
			140;		15;			15;				%	5		RKnee			(	19,	20,	21	)
			0.01;		0.01;		0.01;			%	6		‚Ý‚¼‚¨‚¿	(	22,	23,	24	)
			90;			15;			45;				%	7		LAnkle		(	25,	26,	27	)
			90;			15;			45;				%	8		RAnkle		(	28,	29,	30	)
			0.01;		0.01;		0.01;			%	9		‚Ý‚¼‚¨‚¿ã(	31,	32,	33	)
			90;			15;			15;				%	10	LMP				(	34,	35,	36	)
			90;			15;			15;				%	11	RMP				(	37,	38,	39	)
			90;			90;			90;				%	12	Neck‰º		(	40,	41,	42	)
			0.01;		90;			90;				%	13	LŒ¨bœ		(	43,	44,	45	)
			0.01;		90;			90;				%	14	RŒ¨bœ		(	46,	47,	48	)
			90;			90;			90;				%	12	Neck‰º		(	49,	50,	51	)
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


