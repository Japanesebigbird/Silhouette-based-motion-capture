function	GetModelFile();

clear all
close all

	%-----------
	%F(V_Point)
	%-----------
	Temp		=	readmatrix(	'F.txt'	);
	V_Point	=	Temp;
	save(	'V_Point.mat',	'V_Point'	)

	%-------------
	%J_regressor
	%-------------
	Temp				=	readmatrix(	'J_regressor.txt'	);
	Temp				=	Temp';
	J_Regressor	=	Temp;
	save(	'J_Regressor.mat',	'J_Regressor'	)

	%-------------
	%KinTree
	%-------------
	Temp					=	readmatrix(	'KinTree.txt'	);
	Temp					=	Temp';
	Kintree_Table	=	Temp;
	save(	'Kintree_Table.mat',	'Kintree_Table'	)

	%-------------
	%V_Temp
	%-------------
	Temp		=	readmatrix(	'V_Temp.txt'	);
	V_Temp	=	Temp;
	save(	'V_Temp.mat',	'V_Temp'	)

	%-------------
	%Weights
	%-------------
	Temp		=	readmatrix(	'Weights.txt'	);
	Weights	=	Temp;
	save(	'Weights.mat',	'Weights'	)

	%-------------
	%Pose_Dirs
	%-------------
	Temp	=	readmatrix(	'Pose_Dirs.txt'	);
	Temp1	=	zeros(6890,3,93);
	for	i	=	1:93
		R							=	(i-1)*3+1:(i-1)*3+3;
		Temp1(:,:,i)	=	Temp(:,R);
	end
	Pose_Dirs	=	Temp1;
	save(	'Pose_Dirs.mat',	'Pose_Dirs'	)

	%-------------
	%Shape_Dirs
	%-------------
	Temp	=	readmatrix(	'Shape_Dirs.txt'	);
	Temp1	=	zeros(6890,3,300);
	for	i	=	1:300
		R							=	(i-1)*3+1:(i-1)*3+3;
		Temp1(:,:,i)	=	Temp(:,R);
	end
	Shape_Dirs	=	Temp1;
	save(	'Shape_Dirs.mat',	'Shape_Dirs'	)


	%-------------
	%V_Point
	%-------------
	V_Point	=	V_Point	+1;
	Pair32	=	readmatrix(	'32Pair1_Tri.csv'				);
	nVertex	=	6890;

	NewV_Point_32	=	[];
	for	i	=	1:length(Pair32)
		NewV_Point_32	=	[	NewV_Point_32;	V_Point(	Pair32(i),	:	)	];
	end
	NewV_Point_32	=	NewV_Point_32	-	1;
	NewV_Point_32	=	[
										NewV_Point_32;	
										repmat(	((nVertex+1)-1),	[	31,	3	]		)
									];

	save(	'NewV_Point.mat',	'NewV_Point_32'	)


return;


