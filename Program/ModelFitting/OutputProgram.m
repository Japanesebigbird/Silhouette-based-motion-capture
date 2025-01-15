function	OutputProgram(	);

close all
clear all


	PathName	=	'../ModelFile/';
	load(	[	PathName 'J_Regressor.mat'		]	)
	load(	[	PathName 'Kintree_Table.mat'	]	)
	load(	[	PathName 'Pose_Dirs.mat'			]	)
	load(	[	PathName 'Shape_Dirs.mat'			]	)
	load(	[	PathName 'V_Temp.mat'					]	)
	load(	[	PathName 'Weights.mat'				]	)
	load(	[	PathName 'V_Point.mat'				]	)
	load(	[	PathName 'NewV_Point.mat'			]	)

	V_Point	=	V_Point	+	1;


	SubName				=	'DemoTry';
	TryPathName		=	'../../DemoData/';;
	StageNumber		=	1;


	%-=-=-=-=-=-=-=-=-=-=-=
	%--LandMark
	%-=-=-=-=-=-=-=-=-=-=-=
	LandMark_Ori	=	readmatrix(	[	TryPathName 'DemoLandMark.csv'	]	);
	LandMark_Ori	=...	
		[	LandMark_Ori;
			[	1349	3078	3080	0	1	0
				3506	3018	6476	1	0	0
			]
		];


	%-=-=-=-=-=-=-=-=-=-=-=-=-=
	%--Load Optimization Data
	%-=-=-=-=-=-=-=-=-=-=-=-=-=
	if			StageNumber	==	1
		Temp		=	dir(	'Stage1_OutputFile'	);
		nFr			=	length(Temp)-2;
		nImage	=	1;
		for	iFr	=	1:nFr
			%-------------
			%--Load File
			%-------------
			FileName	=	[	'OptimizationData_Stage1_' num2str(iFr) '.Dat'	];
			LoadName	=	[	'.\Stage1_OutputFile\' FileName];
			ID	=	0;
			while	ID	<=	2
				ID				=	fopen(LoadName);
			end
			FFF	=	fread(ID,'double');
			fclose(ID);
			Q_Data(:,iFr)	=	FFF;
		end

	elseif	StageNumber	==	2
		FileName	=	[	'OptimizationData_Stage2_1.Dat'	];
		LoadName	=	[	'.\Stage2_OutputFile\' FileName];
		ID	=	0;
		while	ID	<=	2
			ID				=	fopen(LoadName);
		end
		FFF	=	fread(ID,'double');
		fclose(ID);

		nFr	=	(	length(FFF)-300	)	/	75;

		Betas	=	FFF(	(nFr)*75+1	:	(nFr)*75+300	);

		for	iFr	=	1:nFr
			R	=	(iFr-1)*75+1	:	(iFr-1)*75+75;
			Q_Data(:,iFr)	=	[	FFF(R);	Betas	];
		end
	end


	for	iFr	=	1:nFr
		Q0		=	Q_Data(:,iFr);
		Betas	=	Q0(	76	:	375	);

		R	=	1:75;
		Q_iImage	=	Q0(	R	);

		Trans	=	Q_iImage(1:3);
		Pose	=	Q_iImage(4:75);

		%	EulerAngle → Rodrigues Rotation
		Pose	=	Euler_Rodrigues(	Pose	);

		[Vertex,	GrobalJoint	]	=	Get_Vertex(	Betas,	Pose,	Trans,...
																					Shape_Dirs,...
																					V_Temp,...
																					J_Regressor,...
																					Pose_Dirs,...
																					Kintree_Table,...
																					Weights	);

		%------------
		%--LandMark
		%------------
		LandMark_Cood	=	[	];
		for	iLand	=	1:length(	LandMark_Ori	)
			P11	=	LandMark_Ori(iLand,1);
			P12	=	LandMark_Ori(iLand,2);
			P13	=	LandMark_Ori(iLand,3);

			R11	=	LandMark_Ori(iLand,4);
			R12	=	LandMark_Ori(iLand,5);
			R13	=	LandMark_Ori(iLand,6);

			Cood_Land1		=	(	Vertex(	P11,	:	)*R11	)	+	(	Vertex(	P12,	:	)*R12	)	+	(	Vertex(	P13,	:	)*R13	);
			LandMark_Cood	=	[	LandMark_Cood;			Cood_Land1	];
		end
		LandMark_Cood	=	[	LandMark_Cood;
											GrobalJoint(			3,			:	);
											GrobalJoint(			2,			:	);
										];

		[Tate,Yoko]							=	size(LandMark_Cood);
		OutPutData(iFr,:)				=	reshape(LandMark_Cood',	[1,	Tate*Yoko	]	);

		[Tate,Yoko]							=	size(Vertex);
		Animation_Vertex(iFr,:)	=	reshape(Vertex',	[1,	Tate*Yoko	]	);
	end
	StickCood	=	[	Animation_Vertex,	OutPutData	];
	AnimationProgram(	StickCood,	V_Point	);

	%-------------------------------------------------
	%--出力データの中身(ランドマークの3次元座標値)
	%-------------------------------------------------
	%1:頭長,	2:鼻,	3:右目,	4:左目,	5:右耳,	6:左耳,	7:胸骨上縁,	8:胸骨上縁後ろ,
	%9:右肩前,	10:右肩後,	11:右肘外,	12:右肘内,	13:右手首内,	14右手首外:,	15:右手,
	%16:左肩前,	17:左肩後,	18:左肘外,	19:左肘内,	20:左手首内,	21左手首外:,	22:左手,
	%23:右膝外,	24:右膝内,	25:右足首外,	26:右足首内,	27:右MP外,	28右MP内:,	29:右つま先,	30:右踵
	%31:右膝外,	32:右膝内,	33:右足首外,	34:右足首内,	35:右MP外,	36右MP内:,	37:右つま先,	38:右踵
	%39:みぞおち前,	40:みぞおち後,	41:右股関節,	42:左股関節
	dlmwrite(	'OutputData.csv',		OutPutData	)


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










%================================================================================================
%================================================================================================
%================================================================================================
%================================================================================================
%	AnimationProgram
%================================================================================================
%================================================================================================
%================================================================================================
%================================================================================================
function	AnimationProgram(	StickCood, TempV	);


	global	V_Point

	V_Point	=		TempV;


	[frame,Dum]	=	size(StickCood);
	point	=	Dum/3;
	
	%===============
	%Get_PointList 
	%===============
	Point_List		=	Get_PointList(point);

	%===============
	%Get_SegmentList 
	%===============
	Segment_List	=	Get_SegmentList(point);
	if		Segment_List	==	-1
		return

	end

	%===============
	%Get_ArrowList 
	%===============
	Arrow_List			=	Get_ArrowList(point);
	if	Arrow_List	==	-1
		return

	end

	%===============
	%Get_FloorCood
	%===============
	[FloorCood,Obje_Cood,CamCood]	=	Get_FloorCood();

	List.Arrow		=	Arrow_List;
	List.Cylinder	=	Segment_List;
	List.Sphere		=	Point_List;
	List.V_Point	=	V_Point;


	Obje.Floor		=	FloorCood;
	Obje.Obje			=	Obje_Cood;
	Obje.Cam			=	CamCood;

	STVIEW_3Dim(StickCood,List,Obje,250,3);




return

%=========================================================
%Internal Function
%=========================================================
function	Arrow_List	=	Get_ArrowList(point);

		%Dis	Pro	Color(1),	Color(3),	Color(2),		Alpha,	Radi,		EdgeWidth,	EdgeAlpha,		Vertex
	Arrow_List	=	[];


return

%=========================================================
%Internal Function
%=========================================================
function	Point_List	=	Get_PointList(point);

	for	iPoint	=	1:42
		Point_List(iPoint,:)	=...
			[	iPoint+6890,	1,		0,1,0,		1,		1,					0.3,				10];
	end


return



%=========================================================
%Internal Function
%=========================================================
function	Segment_List=	Get_SegmentList(point);

	Segment_List	=	[	];

return


%=========================================================
%Internal Function
%=========================================================
function		[FloorCood,Obje_Cood,CamCood]	=	Get_FloorCood();


%			XMin		XMax	YMin	YMax	ZMin	YMax
	FloorCood	=	...
		[	0 		2 		0 	4		-0.011	0.5];

	%======================
	%Object_Cood
	%======================

	%Experiment1
	%Surface
	%		XMin			XMax		YMin		YMax		ZMin		ZMax		L_Wid		LineColor			FaceColor

	%StartDash
	Obje_Cood	=...
		[	
			-10.0,		10.00,		-10,	10,		-0.012,	-0.012,			1,			0,	0,	0,		0.7/2,	0.3/2,	0.3/2;
			-10.0,		10.00,		-10,	10,		-0.06,	-0.06,			1,			0,	0,	0,		0.7/2,	0.3/2,	0.3/2;
			-10.0,		-10.0,		-10,	10,		-0.06,	-0.012,			1,			0,	0,	0,		0.7/2,	0.3/2,	0.3/2;
			10.00,		10.00,		-10,	10,		-0.06,	-0.012,			1,			0,	0,	0,		0.7/2,	0.3/2,	0.3/2;
			-10.0,		10.00,		-10,	-10,	-0.06,	-0.012,			1,			0,	0,	0,		0.7/2,	0.3/2,	0.3/2;
			-10.0,		10.00,		10,		10,		-0.06,	-0.012,			1,			0,	0,	0,		0.7/2,	0.3/2,	0.3/2;
			0,				2.00,		0,			4.00,	-0.01,	-0.01,			1.5,		0,	0,	0,		0.2, 		0.5,	1.0;
		];




	%======================
	%CamCood
	%======================
%	[Name,Path]	=	uigetfile('.mat');
Name	=	0;
	if			Name	==	0
		PostureMatrix	=	[];
	elseif	Name	==	1
		load([Path Name]);
	end

	[nCam,~]	=	size(PostureMatrix);
	if	nCam	==	0;
		CamCood(1).Cood	=	[];
	else

		for	iCam	=	1:nCam
			Rotation	=	[3,2,1];
			Angle			=	PostureMatrix(iCam,4:6);
			Ori				=	PostureMatrix(iCam,1:3)';

			%-----X→Y→Z
			for	i	=1:3
				S(i)	=	sin(Angle(1,i));	C(i)	=	cos(Angle(1,i));
				Axis(i)	=	Rotation(i);
			end

			%========Euler Angle
			E(1).Rm	=	[	1,	0,	0;
									0,	C(1)	-S(1);
									0,	S(1)	C(1)];

			E(2).Rm	=	[	C(2),		0,	S(2);
									0,			1,	0;
									-S(2),	0,	C(2)];

			E(3).Rm	=	[	C(3)	-S(3)	0;
									S(3)	C(3)	0;
									0,		0,		1];

			Rm	=	E(Axis(3)).Rm	*	E(Axis(2)).Rm	*	E(Axis(1)).Rm;

			Local_CamCood	=	...
			[	[0.15;	0.20;		0.30]...
				[-0.15;	0.20;		0.30]...
				[0.15;	-0.20;	0.30]...
				[-0.15;	-0.20;	0.30]...
				[0.15;	0;			0]...
				[0;			0.15;		0]...
				[0;			0;			0.15]	];

			CamOri(iCam,1:3)		=	Ori;
			OriMat							=	repmat(Ori,[1,7]);
			TempCamCood					=	(Rm*Local_CamCood)+OriMat;
%			CamCood(iCam).Cood	=	[OriMat;TempCamCood];
			CamCood(iCam).Cood	=	PostureMatrix(iCam,:);
			CamCood(iCam).LocalCood	=	Local_CamCood;
		end
	end
return



%====================================================================
%Draw StickPicture
%====================================================================
function	STVIEW_3Dim(C_data,List,Obje,fps,dim)


	backCol			=	'K';
	[nFr,Yoko]	=	size(C_data);
	point				= Yoko/dim;

	%-------
	%Value
	%-------
	XY	=	[];
	dXY	=	[];

	%=================================================
	%make drawing window
	%=================================================
	WindowHandle(1).fig	=	figure;

	set(	WindowHandle(1).fig,...
				'name',									'Stick Picture Viewing Program',...
				'numbertitle',					'off',...
				'doublebuffer',					'on',...
				'closerequestfcn',			'closereq',...
				'Toolbar',							'none',...
				'Menubar',							'none',...
				'Position',							[50 120 1200,800],...
				'Resize',								'off',...
				'tag',									'fig_h',...
				'WindowButtonDownFcn',	@MouseDownFunc,...
				'WindowButtonUpFcn',		@MouseUpFunc,...
				'KeyPressFcn',					'',...
				'KeyReleaseFcn',				'',...
				'WindowScrollWheelFcn',	@FcnWheelScroll,...
				'userdata',							{XY dXY},...
				'HitTest',							'Off');

	WindowHandle(1).Color			=	get(WindowHandle(1).fig,'Color');



	WindowHandle(2).fig	=	figure;

	set(	WindowHandle(2).fig,...
				'name',									'Slider',...
				'numbertitle',					'off',...
				'doublebuffer',					'on',...
				'closerequestfcn',			'closereq',...
				'Toolbar',							'none',...
				'Menubar',							'none',...
				'Position',							[1250 120 500,850],...
				'Resize',								'off',...
				'tag',									'fig_h1',...
				'visible',							'off',...
				'HitTest',							'Off');

	WindowHandle(2).Color			=	get(WindowHandle(2).fig,'Color');



	Min	=	-50;
	Max	=	50;
	R		=	Max-Min;
	TateMax	=	850-40;

	for	i	=	0:74
		FIX	=	fix(	i/25	);
		REM	=	rem(	i,	25	);
		TateITI	=	TateMax	-	(	(REM)*33	);
		YokoITI	=	20			+	(	(FIX)*120	);
			uicontrol(	WindowHandle(2).fig,...
									'style',								'slider',...
									'position',							[YokoITI,TateITI,80,20],...
									'value',								0,...
									'min',									Min,...
									'max',									Max,...
									'enable',								'on',...
									'sliderstep',						[1/(R),10/(R)],...
									'tag',									['ParaSlier' num2str(i+1)	],...
									'callback',							@Slier_callback	);

		uicontrol(	WindowHandle(2).fig,...
								'style',								'text',...
								'position',							[YokoITI+80,TateITI,30,20],...
								'string',								num2str(0),...
								'tag',									['ParaSlierTxt' num2str(i+1)	],...
								'FontSize',							12,...
								'FontName',							'Times New Roman',...
								'HorizontalAlignment',	'right',...
								'BackgroundColor',			WindowHandle(1).Color*0.5);




	end


	%================
	%Make Panel
	%================
	WindowHandle(1).Panel			=...
			uipanel(	WindowHandle(1).fig,...
								'Position',						[0.0,0.0,1.0,1.0],...
								'BorderType',					'none',...
								'BackgroundColor',		backCol,...
								'HitTest',						'Off');


	WindowHandle(1).Panel_2		=...
			uipanel(	WindowHandle(1).fig,...
								'Position',						[0.0,0.0,1.0,0.1],...
								'BorderType',					'none',...
								'BackgroundColor',		WindowHandle(1).Color,...
								'HitTest',						'Off');

	WindowHandle(1).PanelColor=	get(WindowHandle(1).Panel,'BackgroundColor');

	WindowHandle(1).PanelAxes	=	axes(	'Parent',WindowHandle(1).Panel,...
																		'Position',[0,0,1,1],...
																		'Color',WindowHandle(1).PanelColor);



	%==================
	%FPS Slider
	%==================
	fps_txt			=...
		uicontrol(	WindowHandle(1).fig,...
								'style',								'text',...
								'position',							[5,1,80,40],...
								'Visible',							'off',...
								'string',								fps(1),...
								'tag',									'fps_txt',...
								'FontSize',							24,...
								'FontName',							'Times New Roman',...
								'HorizontalAlignment',	'right',...
								'BackgroundColor',			WindowHandle(1).Color);

	fps_txt2		=...
		uicontrol(	WindowHandle(1).fig,...
								'style',								'text',...
								'position',							[85,1,60,40],...
								'Visible',							'off',...
								'string',								'fps',...
								'tag',									'fps_txt2',...
								'FontSize',							24,...
								'FontName',							'Times New Roman',...
								'HorizontalAlignment',	'left',...
								'BackgroundColor',			WindowHandle(1).Color);

	fps_slider	=...
		uicontrol(	WindowHandle(1).fig,...
								'style',								'slider',...
								'position',							[145,1,240,40],...
								'Visible',							'off',...
								'value',								fps(1),...
								'min',									1,...
								'max',									fps(1),...
								'sliderstep',						[1/(fps(1)-1),10/(fps(1)-1)],...
								'enable',								'on',...
								'callback',							@slider_move,...
								'tag',									'fps_s');

		uicontrol(	WindowHandle(1).fig,...
								'style',								'text',...
								'position',							[5,45,380,40],...
								'Visible',							'off',...
								'string',								'Replay Speed',...
								'FontSize',							24,...
								'FontName',							'Times New Roman',...
								'HorizontalAlignment',	'Center',...
								'BackgroundColor',			WindowHandle(1).Color);

	%==================
	%play	Button
	%==================
	start_button	=...
		uicontrol(	WindowHandle(1).fig,...
								'style',								'pushbutton',...
								'position',							[385,1,80,40],...
								'Visible',							'off',...
								'string',								'=>',...
								'callback',							@bstart_callback,...
								'tag',									'start_b',...
								'userdata',							{nFr});

		uicontrol(	WindowHandle(1).fig,...
								'style',								'text',...
								'position',							[385,45,80,40],...
								'Visible',							'off',...
								'string',								'play',...
								'FontSize',							24,...
								'FontName',							'Times New Roman',...
								'HorizontalAlignment',	'Center',...
								'BackgroundColor',			WindowHandle(1).Color);

	%==================
	%Pause	Button
	%==================
	pause_button	=...
		uicontrol(	WindowHandle(1).fig,...
								'style',								'pushbutton',...
								'position',							[465,1,80,40],...
								'Visible',							'off',...
								'string',								'■',...
								'enable',								'off',...
								'callback',							@bpause_callback,...
								'tag',									'pause_b');

		uicontrol(	WindowHandle(1).fig,...
								'style',								'text',...
								'position',							[465,45,80,40],...
								'Visible',							'off',...
								'string',								'Stop',...
								'FontSize',							24,...
								'FontName',							'Times New Roman',...
								'HorizontalAlignment',	'Center',...
								'BackgroundColor',			WindowHandle(1).Color);

	%==================
	%Rec	Button
 	%==================
	rec_button		=...
		uicontrol(	WindowHandle(1).fig,...
								'style',								'pushbutton',...
								'position',							[545,1,80,40],...
								'Visible',							'off',...
								'string',								'●',...
								'enable',								'on',...
								'callback',							@brec_callback,...
								'tag',									'rec_b');

		uicontrol(	WindowHandle(1).fig,...
								'style',								'text',...
								'position',							[545,45,80,40],...
								'Visible',							'off',...
								'string',								'Rec',...
								'FontSize',							24,...
								'FontName',							'Times New Roman',...
								'HorizontalAlignment',	'Center',...
								'BackgroundColor',			WindowHandle(1).Color);

	%==================
	%Frame	Slider
	%==================
	frm_txt				=...
		uicontrol(	WindowHandle(1).fig,...
								'style',								'text',...
								'position',							[625,1,90,40],...
								'string',								1,...
								'tag',									'frm_txt',...
								'FontSize',							24,...
								'FontName',							'Times New Roman',...
								'HorizontalAlignment',	'right',...
								'BackgroundColor',			WindowHandle(1).Color);

	frm_slider		=...
		uicontrol(	WindowHandle(1).fig,...
								'style',								'slider',...
								'position',							[725,1,240,40],...
								'value',								1,...
								'min',									1,...
								'max',									nFr,...
								'enable',								'on',...
								'sliderstep',						[1/(nFr-1),10/(nFr-1)],...
								'callback',							@slider_move2,...
								'tag',									'frm_s');

		uicontrol(	WindowHandle(1).fig,...
								'style',								'text',...
								'position',							[725,45,240,40],...
								'string',								'Frame Number',...
								'FontSize',							24,...
								'FontName',							'Times New Roman',...
								'HorizontalAlignment',	'Center',...
								'BackgroundColor',			WindowHandle(1).Color);


	%==================
	%End	Button
	%==================
	end_button		=...
		uicontrol(	WindowHandle(1).fig,...
								'style',								'pushbutton',...
								'position',							[965,1,80,40],...
								'string',								'quit',...
								'value',								1,...
								'callback',							@bend_callback,...
								'tag',									'end_b');


	%==================
	%OutPut CameraPara
	%==================
	CamPara_button	=...
		uicontrol(	WindowHandle(1).fig,...
								'style',								'pushbutton',...
								'Visible',							'off',...
								'position',							[1045,1,120,40],...
								'string',								'Out CameraPara',...
								'value',								1,...
								'callback',							@Out_CamPara_callback);





	[StickCood]	=	GetStick_Cood(C_data,List);


	%Initial_Stick
	[Handle]	=	Draw_Stick(StickCood,List,WindowHandle,1);

	[ObjeHandle]	=	Draw_Objection(Obje,WindowHandle);


	set(	WindowHandle(1).PanelAxes,'View',[-37.5, 30]);
	axis(WindowHandle(1).PanelAxes,'off','equal');

	set(	WindowHandle(1).PanelAxes,...
				'CameraPositionMode',				'manual',...
				'CameraViewAngleMode',			'manual',...
				'CameraPositionMode',				'manual',...
				'CameraTargetMode',					'manual');

	L1	=	light(	WindowHandle(1).PanelAxes,...
								'position',			[3,10,8]);

	L2	=	light(	WindowHandle(1).PanelAxes,...
								'position',			[-3,10,8]);

	L3	=	light(	WindowHandle(1).PanelAxes,...
								'position',			[3,-10,8]);
	L4	=	light(	WindowHandle(1).PanelAxes,...
								'position',			[-3,-10,8]);

	time_range		=	[0 0];
	frm_seq				=	[1:nFr];
	tim_seq				=	(frm_seq-1)	/	fps;
	time_range(1)	=	min([time_range(1) min(tim_seq)]);
	time_range(2)	=	max([time_range(2) max(tim_seq)]);

	set(fps_txt,'userdata',{frm_seq tim_seq time_range});
	set(frm_txt,'userdata',{StickCood Handle List Obje WindowHandle});
	set(WindowHandle(1).fig,'userdata',{StickCood Handle List Obje WindowHandle XY dXY ObjeHandle});


	%-------------------------------
	%	描画データプロパティ
	%-------------------------------
	pause on;
	tic;

	%------------------------------------
	% --- startボタンが押されるまで待機
	%------------------------------------
	waitfor(start_button,'enable','off');

	%------------------------------------
	% --- endボタンが押されるまで繰り返し
	%------------------------------------
	while get(end_button,'value')==1
		start_time	=	toc	+	time_range(1);
		set(fps_slider,'userdata',start_time);

		frame	=	1;
		iFr		=	1;

		%-------------------------------
		% --- Drawing Stick Picture
		%-------------------------------
		while frame(iFr)<nFr

			%--------------------------------------------------------
			% --- endボタンかradioボタンが押されたらループを抜ける
			%--------------------------------------------------------
			if get(end_button,'value')	==	0
				break;
			end

			%start_time→ここまでの時間
			time_inst	=	toc-start_time;
			frm_chk		=	tim_seq<=time_inst;
			frame			=	sum(frm_chk);
			if frame	==	0
				frame	=	1;
			end

			[nSphere,~]	=	size(List.Sphere);
			for	NNN	=	1:nSphere
				set(Handle.Sphere(NNN).Hand,...
					'xdata',StickCood.Sphere(NNN).P_SpXData(:,:,frame),...
					'ydata',StickCood.Sphere(NNN).P_SpYData(:,:,frame),...
					'zdata',StickCood.Sphere(NNN).P_SpZData(:,:,frame),...
					'Parent',WindowHandle(1).PanelAxes);
			end

			[nCylinder,~]	=	size(List.Cylinder);
			for	iList	=	1:nCylinder
				set(Handle.Cylinder(iList).Hand,...
					'xdata',StickCood.Cylinder(iList).P_XData(:,:,frame),...
					'ydata',StickCood.Cylinder(iList).P_YData(:,:,frame),...
					'zdata',StickCood.Cylinder(iList).P_ZData(:,:,frame),...
					'Parent',WindowHandle(1).PanelAxes);
			end

			[nArrow,~]	=	size(List.Arrow);
			for	iList	=	1:nArrow
				set(Handle.Arrow(iList).Hand,...
					'xdata',StickCood.Arrow(iList).P_XData(:,:,frame),...
					'ydata',StickCood.Arrow(iList).P_YData(:,:,frame),...
					'zdata',StickCood.Arrow(iList).P_ZData(:,:,frame),...
					'Parent',WindowHandle(1).PanelAxes);
			end
			set(frm_txt,'string',frame(iFr));

			drawnow;

			data				=	get(fps_txt,'userdata');
			tim_seq			=	data{2};
			time_range	=	data{3};
			start_time	=	get(fps_slider,'userdata');
		end
	end


	close all


return






%%%-------------------------------%%%
%%%       Button CallBack					%%%
%%%-------------------------------%%%
%=========================================
% OutPut_CameraParameter_Button
%=========================================
function	Out_CamPara_callback(Dum1,Dum2)

	TempData	=	get(gcf,'userdata');
	Obje			=	TempData{4};
	[nCam]		=	length(Obje.Cam);

	PostureMatrix	=	[];
	for		iCam	=	1:nCam
		Handle					=	findobj(gcf,'tag',['CamTag' num2str(iCam)]);
		TempData				=	get(Handle,'userdata');
		Dummy						=	TempData{1};
		Para						=	TempData{2};

		TempPara				=	[Para.Ori',Para.Angle];
		PostureMatrix		=	[PostureMatrix;	TempPara];
	end

	pause(0.1);
	[CamPara_Name,CamPara_Path]	=	uiputfile('*.mat','Save CameraParameter file');

	if	CamPara_Name	==	0
		return
	end

	save([CamPara_Path CamPara_Name],'PostureMatrix');

return


%=========================================
%	Recording Button
%=========================================
function brec_callback(Dum1,Dum2)

	%-------------------------------
	%	Movieファイル作成プログラム
	%-------------------------------
	fps_slider	=	findobj(gcf,'tag','fps_s');
	frm_slider	=	findobj(gcf,'tag','frm_s');
	fps_txt			=	findobj(gcf,'tag','fps_txt');
	fps_txt2		=	findobj(gcf,'tag','fps_txt2');
	frm_txt			=	findobj(gcf,'tag','frm_txt');
	button			=	findobj(gcf,'style','pushbutton');
	fig_h				=	findobj('tag','fig_h');

	fps					=	get(fps_slider,'value');

	Temp				=	get(frm_txt,'userdata');
	StickCood		=	Temp{1};
	Handle			=	Temp{2};
	List				=	Temp{3};
	Obje				=	Temp{4};
	WindowHandle=	Temp{5};

	CamPara.CameraPositionMode	=	get(WindowHandle(1).PanelAxes,'CameraPositionMode');
	CamPara.CameraPosition			=	get(WindowHandle(1).PanelAxes,'CameraPosition');
	CamPara.CameraTargetMode		=	get(WindowHandle(1).PanelAxes,'CameraTargetMode');
	CamPara.CameraTarget				=	get(WindowHandle(1).PanelAxes,'CameraTarget');
	CamPara.CameraUpVectorMode	=	get(WindowHandle(1).PanelAxes,'CameraUpVectorMode');
	CamPara.CameraUpVector			=	get(WindowHandle(1).PanelAxes,'CameraUpVector');
	CamPara.CameraViewAngleMode	=	get(WindowHandle(1).PanelAxes,'CameraViewAngleMode');
	CamPara.CameraViewAngle			=	get(WindowHandle(1).PanelAxes,'CameraViewAngle');
	CamPara.FigPosi							=	get(WindowHandle(1).fig,'Position');

	set(WindowHandle(1).fig,'Visible','off');

	fps_p	=	fps;

	Panel1_Posi		=	get(WindowHandle(1).Panel,'Position');
	Panel1_Color	=	get(WindowHandle(1).Panel,'BackgroundColor');
	[~,~,nFr]	=	size(StickCood.Cylinder(1).P_XData(1,1,:));
	for	iFr	=	1:nFr
		MovieHandle(1).fig	=	figure(	'name','Stick Picture Viewing Program',...
																'numbertitle','off','doublebuffer','on','Toolbar','figure',...
																'closerequestfcn','closereq','tag','fig_h2','Position',CamPara.FigPosi);
		set(	MovieHandle(1).fig,...
					'Toolbar','none',...
					'Menubar','none');

		MovieHandle(1).Panel_2		=	uipanel(	MovieHandle(1).fig,...
																					'Position',[0.0,0.0,1.0,0.105],...
																					'BorderType','none',...
																					'BackgroundColor',Panel1_Color);


		MovieHandle(1).Panel			=	uipanel(	MovieHandle(1).fig,...
																					'Position',Panel1_Posi,...
																					'BorderType','none',...
																					'BackgroundColor',Panel1_Color);

		MovieHandle(1).PanelAxes	=	axes(	'Parent',MovieHandle(1).Panel,...
																			'Position',[0,0,1,1],...
																			'Color',Panel1_Color);


		Draw_Objection(Obje,MovieHandle);
		Draw_Stick(StickCood,List,MovieHandle,iFr);
		set(gca,'CameraPositionMode',CamPara.CameraPositionMode);
		set(gca,'CameraPosition',CamPara.CameraPosition);
		set(gca,'CameraTargetMode',CamPara.CameraTargetMode);
		set(gca,'CameraTarget',CamPara.CameraTarget);
		set(gca,'CameraUpVectorMode',CamPara.CameraUpVectorMode);
		set(gca,'CameraUpVector',CamPara.CameraUpVector);
		set(gca,'CameraViewAngleMode',CamPara.CameraViewAngleMode);	
		set(gca,'CameraViewAngle',CamPara.CameraViewAngle);
		axis(MovieHandle(1).PanelAxes,'off','equal');


		pause(0.5)
		M(iFr)	=	getframe(MovieHandle(1).PanelAxes);
		close(MovieHandle(1).fig)
	end

	pause(1)
	[moviefile,moviepath]	=	uiputfile('*.avi','Save movie file');
	if moviefile==0
	  return;
	end

	disp('Now making movie file...')


	V	=	VideoWriter([moviepath moviefile]);
	V.FrameRate	=	fps_p;
	V.Quality		=	100;
	open(V)
	for	iFr	=	1:nFr
		writeVideo(V,M(iFr))
	end
	close(V)

	disp('COMPLETE!!');
	set(fig_h,'Visible','on');

	return;


%=========================================
%	endボタン
%=========================================
function bend_callback(Dum1,Dum2)

	e_button	=	findobj(gcf,'tag','start_b');
	set(e_button,'value',0);
	bstart_callback;

	return;

%=========================================
%CallBack Function	(Play Button)
%=========================================
function bstart_callback(Dum1,Dum2)

	fps_slider	=	findobj(gcf,'tag','fps_s');
	frm_slider	=	findobj(gcf,'tag','frm_s');
	fps_txt			=	findobj(gcf,'tag','fps_txt');
	frm_txt			=	findobj(gcf,'tag','frm_txt');
	s_button		=	findobj(gcf,'tag','start_b');
	p_button		=	findobj(gcf,'tag','pause_b');
	r_button		=	findobj(gcf,'tag','rec_b');


	tmp_frm			=	str2num(get(frm_txt,'string'));
	data				=	get(fps_txt,'userdata');
	frm_seq			=	data{1};
	tim_seq			=	data{2};

	set(fps_slider,'userdata',toc-tim_seq(tmp_frm));

	set(frm_slider,'enable','off');
	set(s_button,'enable','off');
	set(p_button,'enable','on');
	set(r_button,'enable','off');

return;

%=========================================
%CallBack Function	(Slider_RePlay Speed)
%=========================================
function slider_move(Dum1,Dum2);

	fps_slider	=	findobj(gcf,'tag','fps_s');
	fps_txt			=	findobj(gcf,'tag','fps_txt');
	frm_txt			=	findobj(gcf,'tag','frm_txt');

	data				=	get(fps_txt,'userdata');
	start_time	=	get(fps_slider,'userdata');
	tmp_frm			=	str2num(get(frm_txt,'string'));
	frm_seq			=	data{1};
	tim_seq			=	data{2};
	fig					=	1;

	set(fps_txt,'string',round(get(fps_slider,'value')));
	fps	=	str2num(get(fps_txt,'string'));
	time_range		=	[0 0];
	tim_seq				=	(frm_seq)/fps;

	time_range(1)	=	min([time_range(1) min(tim_seq)]);
	time_range(2)	=	max([time_range(2) max(tim_seq)]);
	set(fps_txt,'userdata',{frm_seq tim_seq time_range});
	set(fps_slider,'userdata',toc-tim_seq(tmp_frm));

return;


%=========================================
%CallBack Function	(Slider_FrameNumber)
%=========================================
function slider_move2(Dum1,Dum2);

	fps_slider	=	findobj(gcf,'tag','fps_s');
	frm_slider	=	findobj(gcf,'tag','frm_s');
	fps_txt			=	findobj(gcf,'style','text');
	frm_txt			=	findobj(gcf,'tag','frm_txt');
	set(frm_txt,'string',round(get(frm_slider,'value')))

	Temp				=	get(frm_txt,'userdata');
	StickCood		=	Temp{1};
	Handle			=	Temp{2};
	List				=	Temp{3};

	frame				=	str2num(get(frm_txt,'string'));


	set(	Handle.Vertex(1).Hand,...
				'Vertices',	[	StickCood.C_data_X(:,frame),StickCood.C_data_Y(:,frame),StickCood.C_data_Z(:,frame)	]	);




	[nSphere,~]	=	size(List.Sphere);
	for	NNN	=	1:nSphere
		set(Handle.Sphere(NNN).Hand,...
			'xdata',StickCood.Sphere(NNN).P_SpXData(:,:,frame),...
			'ydata',StickCood.Sphere(NNN).P_SpYData(:,:,frame),...
			'zdata',StickCood.Sphere(NNN).P_SpZData(:,:,frame));
	end

	[nCylinder,~]	=	size(List.Cylinder);
	for	iList	=	1:nCylinder
		set(Handle.Cylinder(iList).Hand,...
			'xdata',StickCood.Cylinder(iList).P_XData(:,:,frame),...
			'ydata',StickCood.Cylinder(iList).P_YData(:,:,frame),...
			'zdata',StickCood.Cylinder(iList).P_ZData(:,:,frame));
	end

	[nArrow,~]	=	size(List.Arrow);
	for	iList	=	1:nArrow
		set(Handle.Arrow(iList).Hand,...
			'xdata',StickCood.Arrow(iList).P_XData(:,:,frame),...
			'ydata',StickCood.Arrow(iList).P_YData(:,:,frame),...
			'zdata',StickCood.Arrow(iList).P_ZData(:,:,frame));
	end

	drawnow;
	return;


%=========================================
%	pauseボタン
%=========================================
function bpause_callback(Dum1,Dum2)

	fps_slider	=	findobj(gcf,'tag','fps_s');
	frm_slider	=	findobj(gcf,'tag','frm_s');
	frm_txt			=	findobj(gcf,'tag','frm_txt');
	s_button		=	findobj(gcf,'tag','start_b');
	p_button		=	findobj(gcf,'tag','pause_b');
	r_button		=	findobj(gcf,'tag','rec_b');
	set(frm_slider,'enable','on','value',str2num(get(frm_txt,'string')));
	set(s_button,'enable','on');
	set(p_button,'enable','off');
	set(r_button,'enable','on');
	waitfor(p_button,'enable','on');

	return;

%%%-------------------------------%%%
%%%     MouseAction CallBack			%%%
%%%-------------------------------%%%
function	Slier_callback(Dum1,Dum2)

	global	Shape_Dirs	V_Temp	J_Regressor	Pose_Dirs	Kintree_Table	Weights	V_Point



	Temp				=	get(gcf,'userdata');
	StickCood		=	Temp{1};
	Handle			=	Temp{2};
	List				=	Temp{3};
	Obje				=	Temp{4};
	WindowHandle=	Temp{5};


	for	i	=	1:25
		Hundle_Slider(i)			=	findobj(	gcf,	'tag',	[	'ParaSlier' num2str(i)		]	);
		Hundle_SliderText(i)	=	findobj(	gcf,	'tag',	[	'ParaSlierTxt' num2str(i)	]	);

		Parameter(i)	=	(Hundle_Slider(i).Value/10);
		set(	Hundle_SliderText(i),	'String',	num2str(	Parameter(i),'%.1f'		)	);
	end


	Count	=	1;
	for	i	=	26:75
		Hundle_Slider(i)			=	findobj(	gcf,	'tag',	[	'ParaSlier' num2str(i)		]	);
		Hundle_SliderText(i)	=	findobj(	gcf,	'tag',	[	'ParaSlierTxt' num2str(i)	]	);

		PosePara(Count)	=	(Hundle_Slider(i).Value/20);
		set(	Hundle_SliderText(i),	'String',	num2str(	PosePara(Count),'%.1f'		)	);
		Count	=	Count	+	1;

	end


	Pose	=	zeros(	72,		1	);
	Betas	=	zeros(	300,	1	);

	Betas(1:25)	=	Parameter;
	Pose(1)			=	pi/2;

	Pose(4:53)	=	PosePara;

	[Vertex,	GrobalJoint	]	=	Get_Vertex_Ver2(	Betas,	Pose,...
																				Shape_Dirs,...
																				V_Temp,...
																				J_Regressor,...
																				Pose_Dirs,...
																				Kintree_Table,...
																				Weights	);



	AAA	=	[	Vertex;	GrobalJoint	];
	[Tate,Yoko]	=	size(AAA);
	AAA					=	reshape(AAA',	[Tate*Yoko,1]	);
	C_data			=	AAA';

	C_data_X(:,1)	=	C_data(	1,	1:3:end-2	);
	C_data_Y(:,1)	=	C_data(	1,	2:3:end-1	);
	C_data_Z(:,1)	=	C_data(	1,	3:3:end-0	);

	DiffX	=	C_data_X(	6891:6914,	1	)	-	StickCood.C_data_X(	6891:6914,	1	);
	DiffY	=	C_data_Y(	6891:6914,	1	)	-	StickCood.C_data_Y(	6891:6914,	1	);
	DiffZ	=	C_data_Z(	6891:6914,	1	)	-	StickCood.C_data_Z(	6891:6914,	1	);

	StickCood.C_data_X	=	C_data_X;
	StickCood.C_data_Y	=	C_data_Y;
	StickCood.C_data_Z	=	C_data_Z;

	[nSphere,~]	=	size(List.Sphere);
	for	NNN	=	1:nSphere
		set(Handle.Sphere(NNN).Hand,...
			'xdata',Handle.Sphere(NNN).Hand.XData	+	DiffX(NNN),...
			'ydata',Handle.Sphere(NNN).Hand.YData	+	DiffY(NNN),...
			'zdata',Handle.Sphere(NNN).Hand.ZData	+	DiffZ(NNN)	);
	end

	set(	Handle.Vertex(1).Hand,...
				'Vertices',	Vertex	);

	set(WindowHandle(2).fig,'userdata',{StickCood Handle List Obje WindowHandle});

return

%=====================================================
%Callback Function (Mause Up)
%=====================================================
function	MouseUpFunc(hObject,EventData)

	Last_ClickType	=	get(gcf,'SelectionType');
	set(gcf,'WindowButtonMotionFcn','');

return

%=====================================================
%Callback Function (Mause Down)
%=====================================================
function	MouseDownFunc(hObject,EventData)

	MainFigure		=	findobj(gcf,'tag','fig_h');
	data					=	get(MainFigure,'userdata');

	StickCood			=	data{1};
	Handle				=	data{2};
	List					=	data{3};
	Obje					=	data{4};
	WindowHandle	=	data{5};
	XY						=	data{6};
	dXY						=	data{7};
	ObjeHandle		=	data{8};

	XY						=	get(groot,'PointerLocation');
	set(MainFigure,'userdata',{StickCood Handle List Obje WindowHandle XY dXY ObjeHandle});

	%===================
	%Check Click Type
	%===================
	ClickType		=	get(WindowHandle(1).fig,'SelectionType');
	%Left Click
	if	strcmp(ClickType,'normal');
		set(gcf,'WindowButtonMotionFcn',@MouseMoveFunc);
	end

	%Right Click
	if	strcmp(ClickType,'alt')
		set(gcf,'WindowButtonMotionFcn',@MouseMoveFunc);
	end

	%Buth Click
	if	strcmp(ClickType,'extend')
		set(gcf,'WindowButtonMotionFcn',@MouseMoveFunc);
	end

return

%=====================================================
%Callback Function (Mause Move)
%=====================================================
function	MouseMoveFunc(hObject,EventData)


	MainFigure		=	findobj(gcf,'tag','fig_h');
	data					=	get(MainFigure,'userdata');
	StickCood			=	data{1};
	Handle				=	data{2};
	List					=	data{3};
	Obje					=	data{4};
	WindowHandle	=	data{5};
	XY						=	data{6};
	dXY						=	data{7};
	ObjeHandle		=	data{8};

	%================
	%
	%================
	ClickType			=	get(WindowHandle(1).fig,'SelectionType');
	NewXY					=	get(groot,'PointerLocation');
	dXY						=	NewXY	-	XY;
	XY						=	NewXY;
	set(MainFigure,'userdata',{StickCood Handle List Obje WindowHandle XY dXY ObjeHandle});

	%Both Click
	if	strcmp(ClickType,'extend')
		camdolly(WindowHandle(1).PanelAxes,-dXY(1),-dXY(2),0,'movetarget','pixels');

	end

	%Left_Click	Zoom
	if	strcmp(ClickType,'normal');
		ThetaX	=	dXY(1).*-0.5;
		ThetaY	=	dXY(2).*-0.5;
		camorbit(WindowHandle(1).PanelAxes,ThetaX(1),ThetaY(1),'coordsys');
		set(WindowHandle(1).PanelAxes,'CameraUpVector',[0 0 1]);
	end

	%Right_Click Zoom
	if	strcmp(ClickType,'alt');
		ScaleY	=	1+(dXY(2).*0.01);
		if	ScaleY	>	1.5
			ScaleY	=	1.5;
		elseif	ScaleY	<	0.5
			ScaleY	=	0.5;
		end
		camzoom(WindowHandle(1).PanelAxes,ScaleY);
	end

	set(WindowHandle(1).fig,'Color',get(WindowHandle(1).fig,'Color'));
	set(WindowHandle(1).PanelAxes,'Color',get(WindowHandle(1).PanelAxes,'Color'));
	axis (WindowHandle(1).PanelAxes,'off','equal');


return



%===============================================
%CallBack Function
%===============================================
function	FcnWheelScroll(hObject,EventData)

	fps_slider			=	findobj(gcf,'tag','fps_s');
	frm_slider			=	findobj(gcf,'tag','frm_s');
	fps_txt					=	findobj(gcf,'style','text');
	frm_txt					=	findobj(gcf,'tag','frm_txt');

	MAX							=	get(frm_slider,'max');
	MIN							=	get(frm_slider,'min');
	TempSliderFrame	=	round(get(frm_slider,'value'));

	SliderFrame			=	TempSliderFrame	+	EventData.VerticalScrollCount;
	if	SliderFrame	>	MAX	|	SliderFrame	<	MIN
		return
	end
	set(frm_txt,'string',SliderFrame);
	set(frm_slider,'value',SliderFrame)

	Temp				=	get(frm_txt,'userdata');
	StickCood		=	Temp{1};
	Handle			=	Temp{2};
	List				=	Temp{3};

	frame				=	str2num(get(frm_txt,'string'));

	set(	Handle.Vertex(1).Hand,...
				'Vertices',	[	StickCood.C_data_X(:,frame),StickCood.C_data_Y(:,frame),StickCood.C_data_Z(:,frame)	]	);


	[nSphere,~]	=	size(List.Sphere);
	for	iList	=	1:nSphere
		set(Handle.Sphere(iList).Hand,...
			'xdata',StickCood.Sphere(iList).P_SpXData(:,:,frame),...
			'ydata',StickCood.Sphere(iList).P_SpYData(:,:,frame),...
			'zdata',StickCood.Sphere(iList).P_SpZData(:,:,frame));
	end

	[nCylinder,~]	=	size(List.Cylinder);
	for	iList	=	1:nCylinder
		set(Handle.Cylinder(iList).Hand,...
			'xdata',StickCood.Cylinder(iList).P_XData(:,:,frame),...
			'ydata',StickCood.Cylinder(iList).P_YData(:,:,frame),...
			'zdata',StickCood.Cylinder(iList).P_ZData(:,:,frame));
	end

	[nArrow,~]	=	size(List.Arrow);
	for	iList	=	1:nArrow
		set(Handle.Arrow(iList).Hand,...
			'xdata',StickCood.Arrow(iList).P_XData(:,:,frame),...
			'ydata',StickCood.Arrow(iList).P_YData(:,:,frame),...
			'zdata',StickCood.Arrow(iList).P_ZData(:,:,frame));
	end

	MainFigure		=	findobj(gcf,'tag','fig_h');
	data					=	get(MainFigure,'userdata');
	StickCood			=	data{1};
	Handle				=	data{2};
	List					=	data{3};
	Obje					=	data{4};
	WindowHandle	=	data{5};
	XY						=	data{6};
	dXY						=	data{7};
	ObjeHandle		=	data{8};

return




%========================================================================
%Internal Function
%========================================================================
function	[Handle]	=	Draw_Stick(StickCood,List,WindowHandle,iFr);



	Handle.Vertex(1).Hand	=...
							trisurf(	List.V_Point,...
												StickCood.C_data_X(	:,	iFr	),...
												StickCood.C_data_Y(	:,	iFr	),...
												StickCood.C_data_Z(	:,	iFr	),...
												'FaceColor',	[0.8,0.8,0.8],...
												'EdgeColor',	[0.6,0.6,0.6],...
												'Parent',			WindowHandle(1).PanelAxes,...
												'HitTest',		'Off',...	
												'FaceAlpha',	0.5	);

	%=====================
	%Initial_StickPicure
	%=====================
	[nSphere,~]	=	size(List.Sphere);
	for	NNN	=	1:nSphere
		RGP(1)		=	List.Sphere(NNN,3);
		RGP(2)		=	List.Sphere(NNN,4);
		RGP(3)		=	List.Sphere(NNN,5);

		F_Alpha		=	List.Sphere(NNN,6);
		EdgeWidth	=	List.Sphere(NNN,7);
		EdgeAlpha	=	List.Sphere(NNN,8);

		Handle.Sphere(NNN).Hand	=	...
			surface(	StickCood.Sphere(NNN).P_SpXData(:,:,iFr),...
								StickCood.Sphere(NNN).P_SpYData(:,:,iFr),...
								StickCood.Sphere(NNN).P_SpZData(:,:,iFr),...
								'LineStyle',				'none',...
								'LineWidth',				EdgeWidth,...
								'EdgeAlpha',				EdgeAlpha,...
								'FaceColor',				[RGP(1) RGP(2) RGP(3)],...
								'FaceAlpha',				F_Alpha,...
								'FaceLighting',			'gouraud',...
								'AmbientStrength',	0.5,...
								'SpecularExponent',	5,...
								'SpecularStrength',	0.7,...
								'Parent',						WindowHandle(1).PanelAxes,...
								'HitTest',					'Off');
	end

	[nCylinder,~]	=	size(List.Cylinder);
	for	iList	=	1:nCylinder
		Tip							=	List.Cylinder(iList,1);
		Tail						=	List.Cylinder(iList,2);
		RGB(1)					=	List.Cylinder(iList,3);
		RGB(2)					=	List.Cylinder(iList,4);
		RGB(3)					=	List.Cylinder(iList,5);
		Cylinder_Alpha	=	List.Cylinder(iList,6);
		Cylinder_Width	=	List.Cylinder(iList,7);
		Edge_Width			=	List.Cylinder(iList,8);
		Edge_Alpha			=	List.Cylinder(iList,9);

		Handle.Cylinder(iList).Hand	=	...
			surface(	StickCood.Cylinder(iList).P_XData(:,:,iFr),...
								StickCood.Cylinder(iList).P_YData(:,:,iFr),...
								StickCood.Cylinder(iList).P_ZData(:,:,iFr),...
								'LineStyle',			'none',...
								'LineWidth',			Edge_Width,...
								'EdgeAlpha',			Edge_Alpha,...
								'FaceColor',			[RGB(1) RGB(2) RGB(3)],...
								'FaceAlpha',			Cylinder_Alpha,...
								'FaceLighting',		'gouraud',...
								'AmbientStrength',0.50,...
								'SpecularExponent',10,...
								'SpecularStrength',0.7,...
								'Parent',					WindowHandle(1).PanelAxes,...
								'HitTest',				'Off');

	end

	[nArrow,~]	=	size(List.Arrow);
	for	iList	=	1:nArrow
		Tip							=	List.Arrow(iList,1);
		Tail						=	List.Arrow(iList,2);
		RGB(1)					=	List.Arrow(iList,3);
		RGB(2)					=	List.Arrow(iList,4);
		RGB(3)					=	List.Arrow(iList,5);
		Arrow_Alpha			=	List.Arrow(iList,6);
		Arrow_Width			=	List.Arrow(iList,7);
		Edge_Width			=	List.Arrow(iList,8);
		Edge_Alpha			=	List.Arrow(iList,9);

		Handle.Arrow(iList).Hand	=	...
			surface(	StickCood.Arrow(iList).P_XData(:,:,iFr),...
								StickCood.Arrow(iList).P_YData(:,:,iFr),...
								StickCood.Arrow(iList).P_ZData(:,:,iFr),...
								'LineStyle',				'none',...
								'LineWidth',				Edge_Width,...
								'EdgeAlpha',				Edge_Alpha,...
								'EdgeColor',				[1,1,1],...
								'FaceColor',				[RGB(1) RGB(2) RGB(3)],...
								'FaceAlpha',				Arrow_Alpha,...
								'FaceLighting',			'gouraud',...
								'AmbientStrength',	0.50,...
								'SpecularExponent',	10,...
								'SpecularStrength',	0.7,...
								'Parent',						WindowHandle(1).PanelAxes,...
								'HitTest',					'Off');

	end



return


%====================================================
%Internal Function (Calu_StickCood)
%====================================================
function	[StickCood]	=	GetStick_Cood(C_data,List)

	[nFr,~]		=	size(C_data);
	StickCood	=	[];


	%==============
	%Sphere Cood
	%==============
	[OriSphere_X,OriSphere_Y,OriSphere_Z]	=	sphere(10);
	[nSpherePoint,~]	=	size(List.Sphere);
	for	NNN	=	1:nSpherePoint
		iPoint			=	List.Sphere(NNN,1);
		SphereSize	=	List.Sphere(NNN,2);
		nVertex			=	List.Sphere(NNN,9);

		[OriSphere_X,OriSphere_Y,OriSphere_Z]	=	sphere(nVertex);
		PointRange	=	(iPoint-1)*3+1:(iPoint-1)*3+3;

		Sphere_X	=	OriSphere_X.*(SphereSize/100);
		Sphere_Y	=	OriSphere_Y.*(SphereSize/100);
		Sphere_Z	=	OriSphere_Z.*(SphereSize/100);
		for	iFr	=	1:nFr
			Sphere(NNN).P_SpXData(:,:,iFr)	=	Sphere_X+C_data(iFr,PointRange(1));
			Sphere(NNN).P_SpYData(:,:,iFr)	=	Sphere_Y+C_data(iFr,PointRange(2));
			Sphere(NNN).P_SpZData(:,:,iFr)	=	Sphere_Z+C_data(iFr,PointRange(3));
		end

		StickCood.Sphere(NNN)	=	Sphere(NNN);
	end

	%==============
	%Cylinder Cood
	%==============
	%	Dis	Pro	Color(1),	Color(2),	Color(3),		Alpha,	Radi,		EdgeWidth,	EdgeAlpha
	[nCylinder,~]	=	size(List.Cylinder);
	for	iList	=	1:nCylinder
		Tip							=	List.Cylinder(iList,1);
		Tail						=	List.Cylinder(iList,2);
		RGB(1)					=	List.Cylinder(iList,3);
		RGB(2)					=	List.Cylinder(iList,4);
		RGB(3)					=	List.Cylinder(iList,5);
		Cylinder_Alpha	=	List.Cylinder(iList,6);
		Cylinder_Width	=	List.Cylinder(iList,7);
		Edge_Width			=	List.Cylinder(iList,8);
		Edge_Alpha			=	List.Cylinder(iList,9);
		nVertex					=	List.Cylinder(iList,10);

		[Cylinder_X,Cylinder_Y,Cylinder_Z]	=	cylinder([0,Cylinder_Width,Cylinder_Width,0],nVertex);
		Cylinder_Z(2,:)	=	0.0;
		Cylinder_Z(3,:)	=	1.0;

		TipRange	=	(Tip-1)*3+1	:	(Tip-1)*3+3;
		TailRange	=	(Tail-1)*3+1:	(Tail-1)*3+3;

		TempZ	=	C_data(:,TipRange)	-	C_data(:,TailRange);
		SP		=	repmat([1,1,1],[nFr,1]);
		TempY	=	cross(TempZ,SP);
		TempX	=	cross(TempY,TempZ);

		LengthX	=	(TempX(:,1).^2	+	TempX(:,2).^2	+	TempX(:,3).^2).^(1/2);
		X(:,1)	=	TempX(:,1)./LengthX;
		X(:,2)	=	TempX(:,2)./LengthX;
		X(:,3)	=	TempX(:,3)./LengthX;

		LengthY	=	(TempY(:,1).^2	+	TempY(:,2).^2	+	TempY(:,3).^2).^(1/2);
		Y(:,1)	=	TempY(:,1)./LengthY;
		Y(:,2)	=	TempY(:,2)./LengthY;
		Y(:,3)	=	TempY(:,3)./LengthY;


		LengthZ	=	(TempZ(:,1).^2	+	TempZ(:,2).^2	+	TempZ(:,3).^2).^(1/2);
		Z(:,1)	=	TempZ(:,1)./LengthZ;
		Z(:,2)	=	TempZ(:,2)./LengthZ;
		Z(:,3)	=	TempZ(:,3)./LengthZ;

		P_X					=	Cylinder_X;
		P_Y					=	Cylinder_Y;
		[Tate,Yoko]	=	size(P_X);

		for	iTate	=	1:Tate
			Range						=	(iTate-1)*Yoko+1:(iTate-1)*Yoko+Yoko;
			DotMat(1,Range)	=	P_X(iTate,:);
			DotMat(2,Range)	=	P_Y(iTate,:);
		end

		for	iFr	=	1:nFr
			P_Z		=	Cylinder_Z.*LengthZ(iFr);

			for	iTate	=	1:Tate
				Range						=	(iTate-1)*Yoko+1:(iTate-1)*Yoko+Yoko;
				DotMat(3,Range)	=	P_Z(iTate,:);
			end

			Temp_Cylinder_P_X01	=	[X(iFr,1),Y(iFr,1),Z(iFr,1)]	*		DotMat;
			Temp_Cylinder_P_Y01	=	[X(iFr,2),Y(iFr,2),Z(iFr,2)]	*		DotMat;
			Temp_Cylinder_P_Z01	=	[X(iFr,3),Y(iFr,3),Z(iFr,3)]	*		DotMat;


			for	iTate	=	1:Tate
				Range													=	(iTate-1)*Yoko+1:(iTate-1)*Yoko+Yoko;
				Cylinder_P_X011(iTate,1:Yoko)	=	Temp_Cylinder_P_X01(1,Range);
				Cylinder_P_Y011(iTate,1:Yoko)	=	Temp_Cylinder_P_Y01(1,Range);
				Cylinder_P_Z011(iTate,1:Yoko)	=	Temp_Cylinder_P_Z01(1,Range);	
			end

			Cylinder(iList).P_XData(:,:,iFr)	=	Cylinder_P_X011+C_data(iFr,TailRange(1));
			Cylinder(iList).P_YData(:,:,iFr)	=	Cylinder_P_Y011+C_data(iFr,TailRange(2));
			Cylinder(iList).P_ZData(:,:,iFr)	=	Cylinder_P_Z011+C_data(iFr,TailRange(3));

		end
		StickCood.Cylinder(iList)	=	Cylinder(iList);
	end

	%==============
	%Arrow Cood
	%==============
	%	Dis	Pro	Color(1),	Color(2),	Color(3),		Alpha,	Radi,		EdgeWidth,	EdgeAlpha
	[nArrow,~]	=	size(List.Arrow);
	for	iList	=	1:nArrow
		Tip							=	List.Arrow(iList,1);
		Tail						=	List.Arrow(iList,2);
		RGB(1)					=	List.Arrow(iList,3);
		RGB(2)					=	List.Arrow(iList,4);
		RGB(3)					=	List.Arrow(iList,5);
		Arrow_Alpha			=	List.Arrow(iList,6);
		Arrow_Width			=	List.Arrow(iList,7);
		Edge_Width			=	List.Arrow(iList,8);
		Edge_Alpha			=	List.Arrow(iList,9);
		nVertex					=	List.Arrow(iList,10);

		[Arrow_X,Arrow_Y,Arrow_Z]	=	cylinder([0,Arrow_Width*2,Arrow_Width,Arrow_Width,0],nVertex);
		Arrow_Z(2,:)	=	0.5;
		Arrow_Z(3,:)	=	Arrow_Z(2,:);
		Arrow_Z(4,:)	=	Arrow_Z(5,:);

		TipRange	=	(Tip-1)*3+1	:	(Tip-1)*3+3;
		TailRange	=	(Tail-1)*3+1:	(Tail-1)*3+3;

		TempZ		=	C_data(:,TipRange)	-	C_data(:,TailRange);
		SP			=	repmat([1,1,1],[nFr,1]);
		TempY		=	cross(TempZ,SP);
		TempX		=	cross(TempY,TempZ);

		LengthX	=	(TempX(:,1).^2	+	TempX(:,2).^2	+	TempX(:,3).^2).^(1/2);
		X(:,1)	=	TempX(:,1)./LengthX;
		X(:,2)	=	TempX(:,2)./LengthX;
		X(:,3)	=	TempX(:,3)./LengthX;

		LengthY	=	(TempY(:,1).^2	+	TempY(:,2).^2	+	TempY(:,3).^2).^(1/2);
		Y(:,1)	=	TempY(:,1)./LengthY;
		Y(:,2)	=	TempY(:,2)./LengthY;
		Y(:,3)	=	TempY(:,3)./LengthY;


		LengthZ	=	(TempZ(:,1).^2	+	TempZ(:,2).^2	+	TempZ(:,3).^2).^(1/2);
		Z(:,1)	=	TempZ(:,1)./LengthZ;
		Z(:,2)	=	TempZ(:,2)./LengthZ;
		Z(:,3)	=	TempZ(:,3)./LengthZ;

		P_X					=	Arrow_X;
		P_Y					=	Arrow_Y;
		[Tate,Yoko]	=	size(P_X);

		DotMat_Arrow	=	[];
		for	iTate	=	1:Tate
			Range									=	(iTate-1)*Yoko+1:(iTate-1)*Yoko+Yoko;
			DotMat_Arrow(1,Range)	=	P_X(iTate,:);
			DotMat_Arrow(2,Range)	=	P_Y(iTate,:);
		end


		for	iFr	=	1:nFr
			P_Z		=	Arrow_Z.*LengthZ(iFr);

			for	iTate	=	1:Tate
				Range									=	(iTate-1)*Yoko+1:(iTate-1)*Yoko+Yoko;
				DotMat_Arrow(3,Range)	=	P_Z(iTate,:);
			end

			Temp_Arrow_P_X01	=	[X(iFr,1),Y(iFr,1),Z(iFr,1)]	*		DotMat_Arrow;
			Temp_Arrow_P_Y01	=	[X(iFr,2),Y(iFr,2),Z(iFr,2)]	*		DotMat_Arrow;
			Temp_Arrow_P_Z01	=	[X(iFr,3),Y(iFr,3),Z(iFr,3)]	*		DotMat_Arrow;


			for	iTate	=	1:Tate
				Range													=	(iTate-1)*Yoko+1:(iTate-1)*Yoko+Yoko;
				Arrow_P_X011(iTate,1:Yoko)	=	Temp_Arrow_P_X01(1,Range);
				Arrow_P_Y011(iTate,1:Yoko)	=	Temp_Arrow_P_Y01(1,Range);
				Arrow_P_Z011(iTate,1:Yoko)	=	Temp_Arrow_P_Z01(1,Range);	
			end

			Arrow(iList).P_XData(:,:,iFr)	=	Arrow_P_X011+C_data(iFr,TailRange(1));
			Arrow(iList).P_YData(:,:,iFr)	=	Arrow_P_Y011+C_data(iFr,TailRange(2));
			Arrow(iList).P_ZData(:,:,iFr)	=	Arrow_P_Z011+C_data(iFr,TailRange(3));


		end
		StickCood.Arrow(iList)	=	Arrow(iList);
	end


	for	iFr	=	1:nFr
		StickCood.C_data_X(	:,	iFr	)	=	C_data(iFr,1:3:end-2);
		StickCood.C_data_Y(	:,	iFr	)	=	C_data(iFr,2:3:end-1);
		StickCood.C_data_Z(	:,	iFr	)	=	C_data(iFr,3:3:end-0);
	end

return

%===========================================
%Internal Function (Draw Obje)
%===========================================
function	[ObjeHandle]	=	Draw_Objection(Obje,WindowHandle)

	[nObje,~]	=	size(Obje.Obje);
	ObjeHandle.Obje	=	[];
	for	iObje	=	1:nObje
		XMin			=	Obje.Obje(iObje,1);
		XMax			=	Obje.Obje(iObje,2);
		YMin			=	Obje.Obje(iObje,3);
		YMax			=	Obje.Obje(iObje,4);
		ZMin			=	Obje.Obje(iObje,5);
		ZMax			=	Obje.Obje(iObje,6);
		LineWidth	=	Obje.Obje(iObje,7);
		LineRGP		=	Obje.Obje(iObje,8:10);
		FaceRGP		=	Obje.Obje(iObje,11:13);

		if			XMin	==	XMax
			[YY,ZZ]				=	meshgrid([YMin,YMax],[ZMin,ZMax]);
			[Tate,Yoko]		=	size(YY);
			XX						=	repmat(XMin,[Tate,Yoko]);

		elseif	YMin	==	YMax
			[XX,ZZ]				=	meshgrid([XMin,XMax],[ZMin,ZMax]);
			[Tate,Yoko]		=	size(XX);
			YY						=	repmat(YMin,[Tate,Yoko]);

		elseif	ZMin	==	ZMax
			[XX,YY]				=	meshgrid([XMin,XMax],[YMin,YMax]);
			[Tate,Yoko]		=	size(XX);
			ZZ						=	repmat(ZMin,[Tate,Yoko]);

		end

		if	LineWidth	==	0
			ObjeHandle.Obje(iObje)	=...
				surface(	XX,	YY,	ZZ,...
									'FaceColor',								FaceRGP,...
									'Facelighting',							'gouraud',...
									'Parent',										WindowHandle(1).PanelAxes,...
									'SpecularStrength',					0,...
									'SpecularColorReflectance',	0,...
									'AmbientStrength',					0,...
									'SpecularExponent',					20,...
									'HitTest',									'Off');

		else
			ObjeHandle.Obje(iObje)	=...
				surface(	XX,	YY,	ZZ,...
									'LineWidth',								LineWidth,...
									'EdgeColor',								LineRGP,...
									'FaceColor',								FaceRGP,...
									'Facelighting',							'gouraud',...
									'Parent',										WindowHandle(1).PanelAxes,...
									'SpecularStrength',					0,...
									'SpecularColorReflectance',	0,...
									'AmbientStrength',					0,...
									'SpecularExponent',					20,...
									'HitTest',									'Off');
		end
	end


	%Floor
	MeshX	=	Obje.Floor(1):0.25:Obje.Floor(2);
	MeshY	=	Obje.Floor(3):0.25:Obje.Floor(4);

	for	NNN	=	1:length(MeshX)
		XX	=	MeshX(NNN);
		if	mod(NNN-1,4)	==	0
			line(	[XX,						XX],...
						[MeshY(1),			MeshY(end)],...
						[Obje.Floor(5),	Obje.Floor(5)],...
						'LineWidth',		0.5,...
						'Color',				[0.8 0.8 0.8],...
						'Parent',				WindowHandle(1).PanelAxes,...
						'HitTest',			'Off');

		else
			line(	[XX,						XX],...
						[MeshY(1),			MeshY(end)],...
						[Obje.Floor(5),	Obje.Floor(5)],...
						'LineWidth',		0.5,...
						'Color',				[0.1 0.1 0.1],...
						'Parent',				WindowHandle(1).PanelAxes,...
						'HitTest',			'Off');

		end
	end

	for	NNN	=	1:length(MeshY)
		YY	=	MeshY(NNN);
		if	mod(NNN-1,4)	==	0

			line(	[MeshX(1),			MeshX(end)],...
						[YY,						YY],...
						[Obje.Floor(5),	Obje.Floor(5)],...
						'LineWidth',		0.5,...
						'Color',				[0.8 0.8 0.8],...
						'Parent',				WindowHandle(1).PanelAxes,...
						'HitTest',			'Off');

		else

			line(	[MeshX(1),			MeshX(end)],...
						[YY,						YY],...
						[Obje.Floor(5),	Obje.Floor(5)],...
						'LineWidth',		0.5,...
						'Color',				[0.1 0.1 0.1],...
						'Parent',				WindowHandle(1).PanelAxes,...
						'HitTest',			'Off');
		end
	end

	[nCam]	=	length(Obje.Cam);

	if	isempty(Obje.Cam(1).Cood)	==	1
		nCam	=	0;
	end

	for	iCam	=	1:nCam
		Ori												=	Obje.Cam(iCam).Cood(1,1:3)';
		Angle											=	Obje.Cam(iCam).Cood(1,4:6);
		Local_CamCood							=	Obje.Cam(iCam).LocalCood;

		CameraPara.Ori						=	Ori;
		CameraPara.Angle					=	Angle;
		CameraPara.Local_CamCood	=	Local_CamCood;


		[Camera_Cood]	=	Get_Grobal_CameraCood(Ori,Angle,Local_CamCood);

		F_Name	=	['D:\ProF\STVIEW_3D\Number_Image\Number' num2str(iCam) '.jpeg'];
		A				=	imread(F_Name);
		B(:,:,1)	=	A(:,:,1)';
		B(:,:,2)	=	A(:,:,2)';
		B(:,:,3)	=	A(:,:,3)';

		CamNamber_Image(:,:,1)=B(:,:,1);
		CamNamber_Image(:,:,2)=B(:,:,2);
		CamNamber_Image(:,:,3)=B(:,:,3);



		%CamLine
		for	NNN	=	1:4
			ObjeHandle.Camera(iCam).Line(NNN)	=	...
			line(	[Camera_Cood(1,NNN),	Camera_Cood(4,NNN)],...
						[Camera_Cood(2,NNN),	Camera_Cood(5,NNN)],...
						[Camera_Cood(3,NNN),	Camera_Cood(6,NNN)],...
						'LineWidth',					1,...
						'Color',							[1 1 1],...
						'Parent',							WindowHandle(1).PanelAxes,...
						'HitTest',						'Off');
		end

		%Xaxis
		NNN	=	5;
		ObjeHandle.Camera(iCam).Line(NNN)	=	...
		line(	[Camera_Cood(1,NNN),	Camera_Cood(4,NNN)],...
					[Camera_Cood(2,NNN),	Camera_Cood(5,NNN)],...
					[Camera_Cood(3,NNN),	Camera_Cood(6,NNN)],...
					'LineWidth',					3,...
					'Color',							[1 0 0],...
					'Parent',							WindowHandle(1).PanelAxes,...
					'HitTest',						'Off');

		%Yaxis
		NNN	=	6;
		ObjeHandle.Camera(iCam).Line(NNN)	=	...
		line(	[Camera_Cood(1,NNN),	Camera_Cood(4,NNN)],...
					[Camera_Cood(2,NNN),	Camera_Cood(5,NNN)],...
					[Camera_Cood(3,NNN),	Camera_Cood(6,NNN)],...
					'LineWidth',					3,...
					'Color',							[0 1 0],...
					'Parent',							WindowHandle(1).PanelAxes,...
					'HitTest',						'Off');

		%Zaxis
		NNN	=	7;
		ObjeHandle.Camera(iCam).Line(NNN)	=	...
		line(	[Camera_Cood(1,NNN),	Camera_Cood(4,NNN)],...
					[Camera_Cood(2,NNN),	Camera_Cood(5,NNN)],...
					[Camera_Cood(3,NNN),	Camera_Cood(6,NNN)],...
					'LineWidth',					3,...
					'Color',							[0 0 1],...
					'Parent',							WindowHandle(1).PanelAxes,...
					'HitTest',						'Off');

		ObjeHandle.Camera(iCam).Surface	=...
		surface(	[Camera_Cood(4,1) Camera_Cood(4,2);	Camera_Cood(4,3) Camera_Cood(4,4)],...
							[Camera_Cood(5,1) Camera_Cood(5,2);	Camera_Cood(5,3) Camera_Cood(5,4)],...
							[Camera_Cood(6,1) Camera_Cood(6,2);	Camera_Cood(6,3) Camera_Cood(6,4)],...
							CamNamber_Image,...
							'FaceColor',					'texturemap',...
							'CDataMapping',				'direct',...
							'FaceAlpha',					0.7,...
							'LineStyle',					'-',...
							'LineWidth',					1,...
							'EdgeAlpha',					0.8,...
							'EdgeColor',					[1 1 1],...
							'Parent',							WindowHandle(1).PanelAxes,...
							'HitTest',						'On',...
							'ButtonDownFcn',			{@HitFunction_Cam nCam},...
							'userdata',						{iCam CameraPara},...
							'tag',								['CamTag' num2str(iCam)]);

	end

return


%=============================================================================
%Internal Function(Camera Cood)
%=============================================================================
function		[Camera_Cood]	=	Get_Grobal_CameraCood(Ori,Angle,Local_CamCood)

	Rotation	=	[3,2,1];
	%-----X→Y→Z
	for	i	=1:3
		S(i)		=	sin(Angle(1,i));	C(i)	=	cos(Angle(1,i));
		Axis(i)	=	Rotation(i);
	end

	%========Euler Angle
	E(1).Rm	=	[	1,	0,	0;
							0,	C(1)	-S(1);
							0,	S(1)	C(1)];

	E(2).Rm	=	[	C(2),		0,	S(2);
							0,			1,	0;
							-S(2),	0,	C(2)];

	E(3).Rm	=	[	C(3)	-S(3)	0;
							S(3)	C(3)	0;
							0,		0,		1];

	Rm	=	E(Axis(3)).Rm	*	E(Axis(2)).Rm	*	E(Axis(1)).Rm;

	OriMat							=	repmat(Ori,[1,7]);
	TempCamCood					=	(Rm*Local_CamCood)+OriMat;
	Camera_Cood					=	[OriMat;TempCamCood];

return




%=============================================================================
%Internal Function(Camera Cood)
%=============================================================================
function	[Vertex,	GrobalJoint	]	=	Get_Vertex_Ver2(	Betas,	Pose,...
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
	Rm			=	Rodrigues_Ver2(	Vecter	);

	HTrans(1).Mat				=	[	Rm	Joint(1,:)'	];
	HTrans(1).Mat(4,4)	=	1;
	GrobalJoint					=	HTrans(1).Mat(1:3,4)';

	for	i	=	2:nJoint
		JointITI	=	i;
		ParentITI	=	ParentID(i-1);
		LoJoint	=	Joint(	JointITI,	:	)	-	Joint(	ParentITI,	:	);

		R				=	(	(JointITI-1)*3	)	+	1	:	(	(JointITI-1)*3	)	+	3;
		Vecter	=	Pose(R);
		Rm			=	Rodrigues_Ver2(	Vecter	);

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

return;

%=============================================================================
%Internal Function(Camera Cood)
%=============================================================================
function	Rm	=	Rodrigues_Ver2(	Vecter	);

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

