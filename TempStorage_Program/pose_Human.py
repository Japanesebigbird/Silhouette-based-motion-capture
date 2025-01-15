import argparse
import time
from pathlib import Path

import	tkinter.filedialog
import	tkinter as tk


import os
import copy
import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
import numpy as np

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages,	letterbox
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path, save_one_box
from utils.plots import colors, plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized


def detect(opt):
	source, weights, view_img, save_txt, imgsz, save_txt_tidl, kpt_label = opt.source, opt.weights, opt.view_img, opt.save_txt, opt.img_size, opt.save_txt_tidl, opt.kpt_label
	save_img	=	not opt.nosave and not source.endswith('.txt')  # save inference images
	webcam		=	source.isnumeric() or source.endswith('.txt') or source.lower().startswith(	('rtsp://', 'rtmp://', 'http://', 'https://')	)


	Temp	=	source.split("/")[0:-1];
	Save_Dir	=	"";
	for	i	in	range(len(Temp)):
		Save_Dir	=	Save_Dir	+	Temp[i]	+	'/'


	Temp	=	source.split("/");


	SaveName	=	source[0:-5]	+	'_Pose.txt';


	# Directories
	#	increment run
	save_dir	=	increment_path(Path(opt.project) / opt.name, exist_ok=opt.exist_ok)

	# make dir
	(save_dir / 'labels' if (save_txt or save_txt_tidl) else save_dir).mkdir(parents=True, exist_ok=True)

	#	Initialize
	set_logging()

	device = select_device(opt.device)

	# half precision only supported on CUDA
	half = device.type != 'cpu' and not save_txt_tidl

	#	Load model
	#	load FP32 model
	model = attempt_load(weights, map_location=device)


	# model stride
	stride	=	int(	model.stride.max()	)
	if isinstance(imgsz, (list,tuple)):
		assert len(imgsz)	==	2; "height and width of image has to be specified"
		imgsz[0] = check_img_size(	imgsz[0],	s=stride	)
		imgsz[1] = check_img_size(	imgsz[1],	s=stride	)
	else:
		# check img_size
		imgsz	=	check_img_size(	imgsz,	s=stride	)

	#	get class names
	names	=	model.module.names if hasattr(model, 'module') else model.names

	if half:
		model.half()  # to FP16

	# Second-stage classifier
	classify = False
	if classify:
		# initialize
		modelc = load_classifier(name='resnet101', n=2)
		modelc.load_state_dict(torch.load('weights/resnet101.pt', map_location=device)['model']).to(device).eval()

	# Set Dataloader
	vid_path, vid_writer = None, None
	if webcam:
		view_img = check_imshow()
		cudnn.benchmark = True  # set True to speed up constant image size inference
		dataset = LoadStreams(source, img_size=imgsz, stride=stride)
	else:
		dataset = LoadImages(	source,	img_size=imgsz,	stride=stride	)

	Out_Cood	=	-1	*	np.ones(	(	dataset.nf+100,	(17*3)*10	)	);


	# Run inference
	if device.type != 'cpu':
		# run once
		model(	torch.zeros(	1, 3, imgsz, imgsz	).to(device).type_as(	next(	model.parameters()	)	)	)


	t0 = time.time()
	Counter	=	0;
	for path, img_dum, im0s, vid_cap in dataset:

		# Padded resize
		img = letterbox(im0s, new_shape=imgsz, stride=stride, auto=False)[0]

		# Convert
		img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
		img = np.ascontiguousarray(img)
		img	=	torch.from_numpy(img).to(device)

		# uint8 to fp16/32
		img	=	img.half() if half else img.float()
		img /= 255.0  # 0 - 255 to 0.0 - 1.0
		if img.ndimension() == 3:
			img	=	img.unsqueeze(0)

		#	Inference
		t1		=	time_synchronized()
		pred	=	model(	img, augment=opt.augment	)[0]

		# Apply NMS
		pred	=	non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms, kpt_label=kpt_label)


		t2		=	time_synchronized()
	
		# Apply Classifier
		if classify:
			pred = apply_classifier(pred, modelc, img, im0s)

		# Process detections
		for i, det in enumerate(pred):  # detections per image
			if webcam:  # batch_size >= 1
				p, s, im0, frame = path[i], '%g: ' % i, im0s[i].copy(), dataset.count
			else:
				p, s, im0, frame = path, '', im0s.copy(), getattr(dataset, 'frame', 0)

			# to Path
			p = Path(p)

			# img.jpg
			save_path = str(save_dir / p.name)

			# img.txt
			txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')

			# print string
			s += '%gx%g ' % img.shape[2:]

			# normalization gain whwh
			gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]
			if len(det):
				# Rescale boxes from img_size to im0 size
				scale_coords(	img.shape[2:],	det[:, :4],	im0.shape,	kpt_label=False								)
				scale_coords(	img.shape[2:],	det[:, 6:],	im0.shape,	kpt_label=kpt_label,	step=3	)

				# Print results
				for c in det[:, 5].unique():
				  # detections per class
					n = (det[:, 5] == c).sum()

					# add to string
					s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "

				# Write results
				for det_index, (*xyxy, conf, cls) in enumerate(reversed(det[:,:6])):

					if save_txt:  # Write to file
						xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh

						line = (cls, *xywh, conf) if opt.save_conf else (cls, *xywh)  # label format
						with open(txt_path + '.txt', 'a') as f:
							f.write(('%g ' * len(line)).rstrip() % line + '\n')

					if save_img or opt.save_crop or view_img:  # Add bbox to image

						# integer class
						c			=	int(cls)
						label	=	None if opt.hide_labels else (names[c] if opt.hide_conf else f'{names[c]} {conf:.2f}')
						kpts	=	det[det_index, 6:]
						R1											=	(det_index*17*3)+0;
						R2											=	(det_index*17*3)+17*3;
						Temp										=	kpts.tolist(	);
						Out_Cood[	Counter,	R1:R2	]	=	Temp;



						plot_one_box(xyxy, im0, label=label, color=colors(c, True), line_thickness=opt.line_thickness, kpt_label=kpt_label, kpts=kpts, steps=3, orig_shape=im0.shape[:2])
						if opt.save_crop:
							save_one_box(xyxy, im0s, file=save_dir / 'crops' / names[c] / f'{p.stem}.jpg', BGR=True)

				if save_txt_tidl:  # Write to file in tidl dump format
					for *xyxy, conf, cls in det_tidl:
						xyxy = torch.tensor(xyxy).view(-1).tolist()
						line = (conf, cls,  *xyxy) if opt.save_conf else (cls, *xyxy)  # label format
						with open(txt_path + '.txt', 'a') as f:
							f.write(('%g ' * len(line)).rstrip() % line + '\n')

			# Print time (inference + NMS)
			print(f'{s}Done. ({t2 - t1:.3f}s)')



			# Stream results
			cv2.imshow('Test', im0)
			cv2.waitKey(1)  # 1 millisecond

			# Save results (image with detections)
#			if save_img:
#				if dataset.mode == 'image':
#					cv2.imwrite(save_path, im0)
#				else:  # 'video' or 'stream'
#					if vid_path != save_path:  # new video
#						vid_path = save_path
#						if isinstance(vid_writer, cv2.VideoWriter):
#							vid_writer.release()  # release previous video writer
#						if vid_cap:  # video
#							fps = vid_cap.get(cv2.CAP_PROP_FPS)
#							w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
#							h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
#						else:  # stream
#							fps, w, h = 30, im0.shape[1], im0.shape[0]
#							save_path += '.mp4'
#						vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
#					vid_writer.write(im0)

		Counter	=	Counter	+	1;


	np.savetxt(	SaveName,	Out_Cood,	fmt='%.6e',	delimiter=','	)

	if save_txt or save_txt_tidl or save_img:
		s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt or save_txt_tidl else ''
		print(f"Results saved to {save_dir}{s}")





	print(f'Done. ({time.time() - t0:.3f}s)')


if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--weights',				nargs		=	'+',						type		=	str,						default	=	'yolov5s.pt',	help='model.pt path(s)'	)
	parser.add_argument('--source',					type		=	str,						default	=	'data/images',	help		=	'source'															)  # file/folder, 0 for webcam
	parser.add_argument('--img-size',				nargs		=	'+',						type		=	int,						default	=	640, help='inference size (pixels)'		)
	parser.add_argument('--conf-thres',			type		=	float,					default	=	0.25,						help		=	'object confidence threshold'					)
	parser.add_argument('--iou-thres',			type		=	float,					default	=	0.45,						help		=	'IOU threshold for NMS'								)
	parser.add_argument('--device',					default	=	'',							help		=	'cuda device, i.e. 0 or 0,1,2,3 or cpu'													)
	parser.add_argument('--view-img',				action	=	'store_true',		help		=	'display results'																								)
	parser.add_argument('--save-txt',				action	=	'store_true',		help		=	'save results to *.txt'																					)
	parser.add_argument('--save-txt-tidl',	action	=	'store_true',		help		=	'save results to *.txt in tidl format'													)
	parser.add_argument('--save-bin',				action	=	'store_true',		help		=	'save base n/w outputs in raw bin format'												)
	parser.add_argument('--save-conf',			action	=	'store_true',		help		=	'save confidences in --save-txt labels'													)
	parser.add_argument('--save-crop',			action	=	'store_true',		help		=	'save cropped prediction boxes'																	)
	parser.add_argument('--nosave',					action	=	'store_true',		help		=	'do not save images/videos'																			)
	parser.add_argument('--classes',				nargs		=	'+',						type		=	int, help='filter by class: --class 0, or --class 0 2 3'				)
	parser.add_argument('--agnostic-nms',		action	=	'store_true',		help		=	'class-agnostic NMS'																						)
	parser.add_argument('--augment',				action	=	'store_true',		help		=	'augmented inference'																						)
	parser.add_argument('--update',					action	=	'store_true',		help		=	'update all models'																							)
	parser.add_argument('--project',				default	=	'runs/detect',	help		=	'save results to project/name'																	)
	parser.add_argument('--name',						default	=	'exp',					help		=	'save results to project/name'																	)
	parser.add_argument('--exist-ok',				action	=	'store_true',		help		=	'existing project/name ok, do not increment'										)
	parser.add_argument('--line-thickness',	default	=	3, type=int,		help		=	'bounding box thickness (pixels)'																)
	parser.add_argument('--hide-labels',		default	=	False,					action	=	'store_true',		help	=	'hide labels'														)
	parser.add_argument('--hide-conf',			default	=	False,					action	=	'store_true',		help	=	'hide confidences'											)
	parser.add_argument('--kpt-label',			action	=	'store_true',		help		=	'use keypoint labels'																						)


	opt = parser.parse_args()


	root	=	tk.Tk()
	root.withdraw()
	iDir	=	os.path.abspath(	os.path.dirname(__file__)	)
	Temp	=	tkinter.filedialog.askdirectory(initialdir=iDir,	title="LoadFolder"	)
	opt.source			=	Temp;



	opt.img_size		=	640;
	opt.weights			=	['yolov7-w6-pose.pt']
	opt.hide_labels	=	True;
	opt.hide_conf		=	True;
	opt.kpt_label		=	True;
	opt.device			=	'0';

	check_requirements(exclude=('tensorboard', 'pycocotools', 'thop'))

	with torch.no_grad():
		if opt.update:  # update all models (to fix SourceChangeWarning)
			for opt.weights in ['yolov5s.pt', 'yolov5m.pt', 'yolov5l.pt', 'yolov5x.pt']:
				detect(opt=opt)
				strip_optimizer(opt.weights)
		else:
			detect(opt=opt)
