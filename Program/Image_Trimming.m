function	Image_Triming(	);

close all
clear all


	[FileName_Movie,PathName_Movie]	=	uigetfile(	{'*.mp4;*.avi'},	'Select OriginalMovieFile'	);
	Temp01	=	split(FileName_Movie,".");

	[FileName_Text,PathName_Text]	=	uigetfile(	{	[	Temp01{1} '*.txt'	]	},	'Select DetectTextFile'	);
	Temp02	=	split(FileName_Text,".");


	Temp01	=	Temp01{1};
	Temp02	=	Temp02{1};
	Temp02	=	Temp02(1:end-10);

	if	Temp01	~=	Temp02
		return;
	end
	TrialName	=	Temp01;
	mkdir(	[	PathName_Text TrialName	'_Trim']	)


	SavePath	=	[	PathName_Text TrialName '_Trim/'	];
	List			= readmatrix(	[PathName_Text FileName_Text ]	);

	nFr		=	max(	List(:,1)	);
	for	iFr	=	1:nFr
		ITI				=	find(	List	==	iFr	);
		SizeData	=	(	List(ITI,5)	.*	List(ITI,6)	);
		[~,Jyun]	=	max(SizeData);
		ITI				=	ITI(	Jyun(1)	);

		NewList(iFr,:)	=	List(	ITI,	:	);
	end
	List	=	NewList;
	nList	=	length(	List(:,1)	);


	MovieData 					= VideoReader(	[	PathName_Movie	FileName_Movie	]	);
	BaseData.nFr				=	MovieData.NumFrames;
	BaseData.Ratio_Pic	=	(	MovieData.Height	/	MovieData.Width	);

	F_h	=	figure(	'position',	[480, 240, 640,640]		);
	A_h	=	axes(	'Parent',F_h,'Position',[	0,	0,	1,	1	]	);
	for	iList	=	1:nList;
		iFr					=	List(iList,1);
		img_b				=	read(	MovieData,	iFr	);


		Temp		=	(	(List(	iList,	5	)	*	MovieData.Width)/2	);
		Temp(2)	=	(	(List(	iList,	6	)	*	MovieData.Height)/2	);
		Range		=	max(	Temp	);

		A	=	fix(	List(	iList,	3	)	*	MovieData.Width		-	(	Range*1.10	)	);
		B	=	fix(	List(	iList,	3	)	*	MovieData.Width		+	(	Range*1.10	)	);

		C	=	fix(	List(	iList,	4	)	*	MovieData.Height	-	(	Range*1.10	)	);
		D	=	fix(	List(	iList,	4	)	*	MovieData.Height	+	(	Range*1.10	)	);

		if	A	<	1
			A	=	1;
		end

		if	B	>	MovieData.Width
			B	=	MovieData.Width;
		end

		if	C	<	1
			C	=	1;
		end

		if	D	>	MovieData.Height
			D	=	MovieData.Height;
		end

		Save	=	img_b(C:D,A:B,:);

		image(Save,	'Parent',A_h)
		drawnow
		imwrite(Save,[	SavePath	TrialName '_' num2str(iFr,'%05.0f') '.jpg' ])
	end
	close(F_h)

return;