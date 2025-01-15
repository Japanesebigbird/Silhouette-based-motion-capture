function	HitTest();

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

	V_Point	=		V_Point	+1;
	Pose	=	zeros(	3,		1	);
	Betas	=	zeros(	300,	1	);

	%ëSëÃ
	Pose(1)	=	pi/2;
	Pose(2)	=	0;
	Pose(3)	=	0;


	PoseList	=	...
	[
		0;		0;		30;			%	1		Lhip			(	7,	8,	9		)
		0;		0;		-30;		%	2		Rhip			(	10,	11,	12	)
		0;		0;		0;			%	3		RibCenter	(	13,	14,	15	)
		0;		0;		0;			%	4		LKnee			(	16,	17,	18	)
		0;		0;		0;			%	5		RKnee			(	19,	20,	21	)
		0;		0;		0;			%	6		Ç›ÇºÇ®Çø	(	22,	23,	24	)
		0;		0;		0;			%	7		LAnkle		(	25,	26,	27	)
		0;		0;		0;			%	8		RAnkle		(	28,	29,	30	)
		0;		0;		0;			%	9		Ç›ÇºÇ®Çøè„(	31,	32,	33	)
		0;		0;		0;			%	10	LMP				(	34,	35,	36	)
		0;		0;		0;			%	11	RMP				(	37,	38,	39	)
		0;		0;		0;			%	12	Neckâ∫		(	40,	41,	42	)
		0;		0;		-0;			%	13	Lå®çbçú		(	43,	44,	45	)
		0;		0;		0;			%	14	Rå®çbçú		(	46,	47,	48	)
		0;		0;		0;			%	12	Neckâ∫		(	49,	50,	51	)
		0;		0;		-0;			%	16	LShoulder	(	52,	53,	54	)
		0;		0;		0;			%	17	RSholder	(	55,	56,	57	)
		-0;		-20;		0;		%	18	LElbow		(	58,	59,	60	)
		-0;		20;		0;			%	19	RElbow		(	61,	62,	63	)
		0;		0;		0;			%	20	LWrist		(	64,	65,	66	)
		0;		0;		0;			%	21	RWrist		(	67,	68,	69	)
		0;		0;		0;			%	22	LHand			(	70,	71,	72	)
		0;		0;		0;			%	23	RHand			(	73,	74,	75	)
	];
	PoseList	=	pi.*(	PoseList./180	);
	Pose	=	[	Pose;	PoseList];

	Betas		=	zeros(300,1);
	[Vertex,	GrobalJoint	]	=	Get_Vertex(	Betas,	Pose,...
																				Shape_Dirs,...
																				V_Temp,...
																				J_Regressor,...
																				Pose_Dirs,...
																				Kintree_Table,...
																				Weights	);

	Temp				=	[	Vertex(:,1:3);	];
	[Tate,Yoko]	=	size(Temp);
	Temp				=	reshape(Temp',	[Tate*Yoko,1]	);
	StickCood		=	Temp';
	F_3DStick(	StickCood, V_Point		);

return;

%-----------------------------------------------------------------------
%InternalFunction
%-----------------------------------------------------------------------
function	[Vertex,	GrobalJoint	]	=	Get_Vertex(	Betas,	Pose,...
																								Shape_Dirs,...
																								V_Temp,...
																								J_Regressor,...
																								Pose_Dirs,...
																								Kintree_Table,...
																								Weights,...
																								V_Pose	)

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
		[	JointITI,	ParentITI	];
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

return;

%-----------------------------------------------------------------------
%InternalFunction
%-----------------------------------------------------------------------
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