function	F_3DStick(	StickCood, TempV	);


	global	V_Point

	V_Point	=		TempV;


	[frame,Dum]	=	size(StickCood);
	point	=	Dum/3


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
%	ArrowList	=	...
%		[26,	27,	1,				0,				0,					1.0,		0.01,		1.0,				0.2,					20];

	if	point	>=	23
		Arrow_List	=	[];
	else
		Arrow_List	=	[];
	end




return

%=========================================================
%Internal Function
%=========================================================
function	Point_List	=	Get_PointList(point);

	Point_List	=...
		[
			1,		0.2,		0,0,1,		1,		1,					0.3,				10
			1,		0.2,		1,0,0,		1,		1,					0.3,				10
			1,		0.2,		1,0,0,		1,		1,					0.3,				10
			1,		0.2,		1,0,0,		1,		1,					0.3,				10
		];

return

%=========================================================
%Internal Function
%=========================================================
function	Segment_List=	Get_SegmentList(point);
	Segment_List	=	[]
return


%=========================================================
%Internal Function
%=========================================================
function		[FloorCood,Obje_Cood,CamCood]	=	Get_FloorCood();

%			XMin		XMax	YMin	YMax	ZMin	YMax
	FloorCood	=	...
		[	-1 		1 		-1 	1		-1	-1];


	%======================
	%Object_Cood
	%======================

	%Experiment1
	%Surface
	%		XMin			XMax		YMin		YMax		ZMin		ZMax		L_Wid		LineColor			FaceColor

	%StartDash
	Obje_Cood	=...
		[	];




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

			%-----X¨Y¨Z
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
				'Position',							[50 120 1200,850],...
				'Resize',								'off',...
				'tag',									'fig_h',...
				'WindowButtonDownFcn',	@MouseDownFunc,...
				'WindowButtonUpFcn',		@MouseUpFunc,...
				'KeyPressFcn',					'',...
				'KeyReleaseFcn',				'',...
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
		[	FIX	REM	];
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



	[StickCood]	=	GetStick_Cood(C_data,List);
%	save('StickCood','StickCood')
%	load('StickCood')


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


	set(WindowHandle(2).fig,'userdata',{StickCood Handle List Obje WindowHandle});
	set(WindowHandle(1).fig,'userdata',{StickCood Handle List Obje WindowHandle XY dXY ObjeHandle});


	waitfor(WindowHandle(1).fig		)

	close all

return






%%%-------------------------------%%%
%%%       Button CallBack					%%%
%%%-------------------------------%%%






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

	[Vertex,	GrobalJoint	]	=	Get_Vertex(	Betas,	Pose,...
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
												'ButtonDownFcn',			{@HitFunction_Sphere -1	},...
												'HitTest',		'On',...	
												'FaceAlpha',	0.7,...
												'FaceColor',	[0.98,	0.88,	0.76],...
												'EdgeColor',	[	0,0,	0		]	);

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
								'ButtonDownFcn',			{@HitFunction_Sphere NNN},...
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
			Sphere(NNN).P_SpXData(:,:,iFr)	=	Sphere_X+[0];
			Sphere(NNN).P_SpYData(:,:,iFr)	=	Sphere_Y+[0];
			Sphere(NNN).P_SpZData(:,:,iFr)	=	Sphere_Z+[0];
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
		YY	=	MeshY(NNN);
		if	mod(NNN-1,4)	==	0
			line(	[XX,						XX],...
						[MeshY(1),			MeshY(end)],...
						[Obje.Floor(5),	Obje.Floor(6)],...
						'LineWidth',		0.5,...
						'Color',				[0.8 0.8 0.8],...
						'Parent',				WindowHandle(1).PanelAxes,...
						'HitTest',			'Off');

			line(	[MeshX(1),			MeshX(end)],...
						[YY,						YY],...
						[Obje.Floor(5),	Obje.Floor(6)],...
						'LineWidth',		0.5,...
						'Color',				[0.8 0.8 0.8],...
						'Parent',				WindowHandle(1).PanelAxes,...
						'HitTest',			'Off');

		else
			line(	[XX,						XX],...
						[MeshY(1),			MeshY(end)],...
						[Obje.Floor(5),	Obje.Floor(6)],...
						'LineWidth',		0.5,...
						'Color',				[0.1 0.1 0.1],...
						'Parent',				WindowHandle(1).PanelAxes,...
						'HitTest',			'Off');

			line(	[MeshX(1),			MeshX(end)],...
						[YY,						YY],...
						[Obje.Floor(5),	Obje.Floor(6)],...
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
	%-----X¨Y¨Z
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
function	[Vertex,	GrobalJoint	]	=	Get_Vertex(	Betas,	Pose,...
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




%==================================================
%Callback Function (Camera)
%==================================================
function	HitFunction_Sphere(	hObject,	EventData,	SphereNum	)

	global	V_Point

	if	EventData.Button	==	1

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


		InPoint	=	EventData.IntersectionPoint';

		iPoint			=	List.Sphere(1,1);
		SphereSize	=	List.Sphere(1,2);
		nVertex			=	List.Sphere(1,9);

		[OriSphere_X,OriSphere_Y,OriSphere_Z]	=	sphere(nVertex);
		Sphere_X															=	OriSphere_X.*(SphereSize/100);
		Sphere_Y															=	OriSphere_Y.*(SphereSize/100);
		Sphere_Z															=	OriSphere_Z.*(SphereSize/100);
		P_SpXData(:,:)	=	Sphere_X	+	InPoint(1);
		P_SpYData(:,:)	=	Sphere_Y	+	InPoint(2);
		P_SpZData(:,:)	=	Sphere_Z	+	InPoint(3);

		set(Handle.Sphere(1).Hand,	'xdata',P_SpXData,	'ydata',P_SpYData,	'zdata',P_SpZData	);

		Vec	=...
			[
				(	StickCood.C_data_X	-	EventData.IntersectionPoint(1)	)';
				(	StickCood.C_data_Y	-	EventData.IntersectionPoint(2)	)';
				(	StickCood.C_data_Z	-	EventData.IntersectionPoint(3)	)';
			];

		Leng		=	(	sum(Vec.^2)	).^(1/2);
		[~,ITI]	=	min(Leng);

		FindMat	=...
		[	
			find(	V_Point(:,1)	==	ITI	)
			find(	V_Point(:,2)	==	ITI	)
			find(	V_Point(:,3)	==	ITI	)
		];

		ITI	=	1;
		FlagMat	=	[];
		for	i	=	1:length(	FindMat(:,1)	)

			P1	=	V_Point(	FindMat(i),	1	);
			P2	=	V_Point(	FindMat(i),	2	);
			P3	=	V_Point(	FindMat(i),	3	);

			P1	=	[	StickCood.C_data_X(P1);	StickCood.C_data_Y(P1);	StickCood.C_data_Z(P1)	];
			P2	=	[	StickCood.C_data_X(P2);	StickCood.C_data_Y(P2);	StickCood.C_data_Z(P2)	];
			P3	=	[	StickCood.C_data_X(P3);	StickCood.C_data_Y(P3);	StickCood.C_data_Z(P3)	];

			C1	=	cross(	P2-P1,	P3-P1	);
			Vec	=	InPoint-P1;

			FlagMat	=	[	FlagMat;	abs(	dot(	C1,	Vec	)	)	];

%			C1	=	cross(	P2-P1,	InPoint-P2	);
%			C2	=	cross(	P3-P2,	InPoint-P3	);
%			C3	=	cross(	P1-P3,	InPoint-P1	);

%			C1	=	C1	/	(	(sum(C1.^2)).^(1/2)	);
%			C2	=	C2	/	(	(sum(C2.^2)).^(1/2)	);
%			C3	=	C3	/	(	(sum(C3.^2)).^(1/2)	);

%			Flag	=	sign(	dot(C1,C2)	)	+	sign(	dot(C2,C3)	)	+	sign(	dot(C3,C1)	);
%			if	Flag	==	3	||	Flag	==	-3
%				ITI	=	i;
%			end
		end


		[~,ITI]	=	min(FlagMat);

		P1	=	[	StickCood.C_data_X(	V_Point(	FindMat(ITI),	1	)	);	StickCood.C_data_Y(	V_Point(	FindMat(ITI),	1	)	);	StickCood.C_data_Z(	V_Point(	FindMat(ITI),	1	)	)	];
		P2	=	[	StickCood.C_data_X(	V_Point(	FindMat(ITI),	2	)	);	StickCood.C_data_Y(	V_Point(	FindMat(ITI),	2	)	);	StickCood.C_data_Z(	V_Point(	FindMat(ITI),	2	)	)	];
		P3	=	[	StickCood.C_data_X(	V_Point(	FindMat(ITI),	3	)	);	StickCood.C_data_Y(	V_Point(	FindMat(ITI),	3	)	);	StickCood.C_data_Z(	V_Point(	FindMat(ITI),	3	)	)	];

		P_SpXData(:,:)	=	Sphere_X	+	P1(1);
		P_SpYData(:,:)	=	Sphere_Y	+	P1(2);
		P_SpZData(:,:)	=	Sphere_Z	+	P1(3);
		set(Handle.Sphere(2).Hand,	'xdata',P_SpXData,	'ydata',P_SpYData,	'zdata',P_SpZData	);

		P_SpXData(:,:)	=	Sphere_X	+	P2(1);
		P_SpYData(:,:)	=	Sphere_Y	+	P2(2);
		P_SpZData(:,:)	=	Sphere_Z	+	P2(3);
		set(Handle.Sphere(3).Hand,	'xdata',P_SpXData,	'ydata',P_SpYData,	'zdata',P_SpZData	);

		P_SpXData(:,:)	=	Sphere_X	+	P3(1);
		P_SpYData(:,:)	=	Sphere_Y	+	P3(2);
		P_SpZData(:,:)	=	Sphere_Z	+	P3(3);
		set(Handle.Sphere(4).Hand,	'xdata',P_SpXData,	'ydata',P_SpYData,	'zdata',P_SpZData	);

		CoMat			=	[	[	P1,	P2,	P3	]	];
		RightHand	=	[	InPoint					];

		(	inv(	CoMat	)	*	RightHand	)'

%		V_Point(	FindMat(ITI),	:	)
		sprintf(	' %d %d %d ',	V_Point(	FindMat(ITI),	:	)	)

		set(MainFigure,'userdata',{StickCood Handle List Obje WindowHandle XY dXY ObjeHandle});
	end

		

return


