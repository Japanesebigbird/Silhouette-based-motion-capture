function	Make_NewTriMap();

close all
clear all

	%Select Directory

	TempDir	=	uigetdir(	cd,	'Select DepthFolder'	);
	if			TempDir(end)	==	'h'
		TempDir	=	TempDir(1:end-5);
	elseif	TempDir(end)	==	'p'
		TempDir	=	TempDir(1:end-6);
	elseif	TempDir(end)	==	'm'
		TempDir	=	TempDir(1:end-4);
	else
		return;
	end


	Dir_Depth			=	[	TempDir 'Depth'			];
	Dir_TriMap		=	[	TempDir 'TriMap'		];
	Dir_Color			=	[	TempDir 'Trim'			];
	Dir_NewTriMap	=	[	TempDir 'NewTriMap'	];


	ANS	=	exist(	Dir_NewTriMap	);

	if	ANS	==	7
		rmdir(Dir_NewTriMap,'s')
	end
	mkdir(	Dir_NewTriMap	)

	List	=	dir(	[Dir_Color	'/*.jpg']	);
	nFile	=	length(List(:,1));

	F_h	=	figure(	'position',	[480, 240, 640,640]		);
	A_h	=	axes(	'Parent',F_h,'Position',[	0,	0,	1,	1	]	);
	for	iFile	=	1:nFile

		CData			=	imread(	[	Dir_Color '/' List(iFile).name(1:end-4)	'.jpg'	]	);
		DepthData	=	imread(	[	Dir_Depth '/' List(iFile).name(1:end-4)	'.png'	]	);
		TriMap		=	imread(	[	Dir_TriMap '/' List(iFile).name(1:end-4) '_00.png'	]	);

		ITI					=	find(	TriMap~=0	);
		PickUpData	=	DepthData(	ITI	);

		STD		=	std(	double(PickUpData)	);
		Mean	=	mean(	double(PickUpData)	);

		P_3SD	=	Mean	+	(	3*STD	);
		M_3SD	=	Mean	-	(	3*STD	);

		PickUpData(	find(	PickUpData	>	P_3SD	)	)	=	65000;
		PickUpData(	find(	PickUpData	<	M_3SD	)	)	=	65000;

		DepthData(	ITI	)				=	PickUpData;

		TriMap(	find(	DepthData	==	65000	)	)	=	0;

		% LowThre
		ITI	=	find(	TriMap	==	128);
		Temp			=	CData(:,:,1);
		Temp(ITI)	=	(Temp(ITI)	*	0.50)	+	(255*0.50);
		CData(:,:,1)	=	Temp;

		Temp			=	CData(:,:,2);
		Temp(ITI)	=	(Temp(ITI)	*	0.50)	+	(255*0.50);
		CData(:,:,2)	=	Temp;


		% HighThre
		ITI	=	find(	TriMap	==	255);
		Temp			=	CData(:,:,3);
		Temp(ITI)	=	(Temp(ITI)	*	0.50)	+	(255*0.50);
		CData(:,:,3)	=	Temp;


		Temp			=	CData(:,:,2);
		Temp(ITI)	=	(Temp(ITI)	*	0.50)	+	(255*0.50);
		CData(:,:,2)	=	Temp;


		image(CData,	'Parent',A_h)
		drawnow

		SaveName	=	[	Dir_NewTriMap '/' List(iFile).name(1:end-4) '_00.png'	];
		imwrite(	TriMap,	SaveName		)
	end
	close(F_h);










return