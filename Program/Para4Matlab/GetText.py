import numpy as np

fname				=	'model.npz'
model_dict	=	np.load(	fname,	allow_pickle=True	)

#------------------------
# Save Pose Dirs	(Dim3)
#------------------------
Pose_Dirs	=	model_dict['posedirs']
nTate			=	Pose_Dirs.shape[0];
nYoko			=	Pose_Dirs.shape[1];
nOku			=	Pose_Dirs.shape[2];

SaveData	=	np.zeros(	(	nTate,	nYoko*nOku	)	)
for	iOku	in	range(nOku):
	S	=	iOku*3;
	E	=	iOku*3+3;
	SaveData[:,	S:E]	=	Pose_Dirs[:,:,iOku];

np.savetxt('Pose_Dirs.txt',	SaveData,	fmt='%.30f'	)

#----------------------
# Save V Temp	(Dim2)
#----------------------
V_Temp	=	model_dict['v_template'];
np.savetxt('V_Temp.txt',	V_Temp,	fmt='%.30f'	)

#------------------------
# Save Shape Dirs (Dim3)
#------------------------
Shape_Dirs	=	model_dict['shapedirs'];
nTate				=	Shape_Dirs.shape[0];
nYoko				=	Shape_Dirs.shape[1];
nOku				=	Shape_Dirs.shape[2];

SaveData	=	np.zeros(	(	nTate,	nYoko*nOku	)	)
for	iOku	in	range(nOku):
	S	=	iOku*3;
	E	=	iOku*3+3;
	SaveData[:,	S:E]	=	Shape_Dirs[:,:,iOku];

np.savetxt('Shape_Dirs.txt',	SaveData,	fmt='%.30f'	)

#-------------------------
# Save J_regressor (Dim2)
#-------------------------
J_regressor	=	model_dict['J_regressor'];
np.savetxt('J_regressor.txt',	J_regressor,	fmt='%.30f'	)

#----------------------
# Save Weights (Dim2)
#----------------------
Weights	=	model_dict['weights'];
np.savetxt('Weights.txt',	Weights,	fmt='%.30f'	)

#----------------------------
# Save Kintree_Table (Dim2)
#----------------------------
KinTree	=	model_dict['kintree_table'];
np.savetxt('KinTree.txt',	KinTree,	fmt='%.30f'	)

#----------------------------
# Save F (Dim2)
#----------------------------
F	=	model_dict['f']
np.savetxt('F.txt',	F,	fmt='%.30f'	)

