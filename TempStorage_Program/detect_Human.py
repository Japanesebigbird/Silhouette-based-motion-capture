from	pathlib	import	Path
from	numpy	import	random

import	os
import	tkinter.filedialog
import	tkinter as tk
import	argparse
import	time
import	cv2
import	torch
import	torch.backends.cudnn	as	cudnn
import	matplotlib.pyplot			as	plt



from	models.experimental	import	attempt_load
from	utils.datasets			import	LoadStreams,		LoadImages
from	utils.general				import	check_img_size,	check_requirements,	check_imshow,			non_max_suppression,	apply_classifier, \
																	scale_coords,		xyxy2xywh,					strip_optimizer,	set_logging,					increment_path

from	utils.plots					import	plot_one_box
from	utils.torch_utils		import	select_device, load_classifier, time_synchronized, TracedModel


def detect(save_img=False):

	source,	weights,	save_txt,	imgsz	=	opt.source, opt.weights, opt.save_txt, opt.img_size
	save_img	=	not opt.nosave and not source.endswith('.txt')  # save inference images

	# Directories

  #	increment run
#	save_dir = Path(increment_path(Path(opt.project) / opt.name, exist_ok=opt.exist_ok))

	#	make dir
#	(save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)

	#	Initialize
	set_logging()
	device = select_device(opt.device)

  # half precision only supported on CUDA
	half = device.type != 'cpu'

	#-------------
	# Load model
	#-------------
  # load FP32 model
	model		=	attempt_load(	weights,	map_location=device	)

  # model stride
	stride	=	int(model.stride.max())

  # check img_size
	imgsz		=	check_img_size(imgsz, s=stride)

	#	Set Half form Single
	if half:
		model.half()  # to FP16

	# Second-stage classifier


	#-------------
	# SelectFile
	#-------------
	root	=	tk.Tk()
	root.withdraw()
	fTyp				= [("MovieFile", "*.mp4;*.MP4")]
	iDir				=	os.path.abspath(os.path.dirname(__file__))
	target_file	=	tkinter.filedialog.askopenfilenames(filetypes=fTyp,	initialdir=iDir)

	if	len(target_file)	==	0:
		print("Error!! Select File")
		return;



	root	=	tk.Tk()
	root.withdraw()
	iDir				=	os.path.abspath(os.path.dirname(__file__))
	SaveFolder	=	tkinter.filedialog.askdirectory(initialdir=iDir,	title="SaveFolder"	)
	save_dir		=	SaveFolder

	for	source	in	target_file:

		#	Set Dataloader
		vid_path, vid_writer	=	None, None
		dataset	=	LoadImages(		source,	img_size	=	imgsz,	stride	=	stride	)

		# Get names and colors
		names		=	model.module.names if hasattr(model, 'module') else model.names
		colors	=	[	[random.randint(0, 255) for _ in range(3)] for _ in names	]

		# Run inference
		if device.type != 'cpu':
			#	run once
			model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))
		old_img_w	=	old_img_h = imgsz
		old_img_b	=	1

		t0 = time.time()
		WriteCounter	=	0;
		for path, img, im0s, vid_cap in dataset:

			img = torch.from_numpy(img).to(device)
			img = img.half() if half else img.float()		# uint8 to fp16/32
			img /= 255.0																# 0 - 255 to 0.0 - 1.0

			if img.ndimension() == 3:
				img = img.unsqueeze(0)


			# Warmup
			if device.type != 'cpu' and (old_img_b != img.shape[0] or old_img_h != img.shape[2] or old_img_w != img.shape[3]):
				old_img_b	=	img.shape[0]
				old_img_h	=	img.shape[2]
				old_img_w	=	img.shape[3]
				for i in range(3):
					model(img, augment=opt.augment)[0]

			# Inference
			t1 = time_synchronized()
			with torch.no_grad():	# Calculating gradients would cause a GPU memory leak
				pred = model(img, augment=opt.augment)[0]

			t2	=	time_synchronized()

			# Apply NMS
			pred	=	non_max_suppression(	pred,	opt.conf_thres,	opt.iou_thres,	classes=opt.classes,	agnostic=opt.agnostic_nms	)

			t3		=	time_synchronized(	)

			# Apply Classifier
#			if classify:
#				pred	=	apply_classifier(	pred,	modelc,	img,	im0s	)
#				print(pred.shape)


			#----------------------
			# Process detections
			#----------------------
			# detections per image
			for i, det in enumerate(pred):

				p, s, im0, frame = path, '', im0s, getattr(dataset, 'frame', 0)

				# to Path
				p	=	Path(p)

				ITI				=	p.name.find('.');
				SaveName	=	p.name[0:ITI]	+	'_ObDe'	+	p.name[ITI:]

				#	img.jpg
				save_path	=	save_dir + '/'	+	SaveName

				#	img.txt
#				txt_path	=	(save_dir + '/'	+	'labels' + '/'	+	p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')
				txt_path	=	(save_dir + '/'	+	p.stem)


				#	normalization gain whwh
				gn	=	torch.tensor(im0.shape)[	[1, 0, 1, 0]	]
				if len(det):
					# Rescale boxes from img_size to im0 size
					det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

					# Print results
					for c in det[:, -1].unique():

						# detections per class ( Count Object )
						n = (det[:, -1] == c).sum()
						s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string


					# Write results
					for *xyxy, conf, cls in reversed(det):

						#--Only Person
						if	cls	==	0:
							if save_txt:  # Write to file
								# normalized xywh
								xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()

								# label format
								line = (cls, *xywh, conf) if opt.save_conf else (cls, *xywh)

								if	WriteCounter	==	0:
									WriteCounter	=	1;
									with open( txt_path	+	'_Detection.txt', 'w') as f:
										f.write(	str(frame) + ' '	)
										f.write(	('%g ' * len(line)	).rstrip() % line + '\n'	)

								else:
									with open( txt_path	+	'_Detection.txt', 'a') as f:
										f.write(	str(frame) + ' '	)
										f.write(	('%g ' * len(line)	).rstrip() % line + '\n'	)


							if save_img or view_img:  # Add bbox to image
								label = f'{names[int(cls)]} {conf:.2f}'
								plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=2)

				# Print time (inference + NMS)
				print(f'{s}Done. ({(1E3 * (t2 - t1)):.1f}ms) Inference, ({(1E3 * (t3 - t2)):.1f}ms) NMS')


				# Save results (image with detections)
				if save_img:
					if dataset.mode == 'image':
						cv2.imwrite(save_path, im0)
						print(f" The image with the result is saved in: {save_path}")
					else:  # 'video' or 'stream'
						if vid_path != save_path:  # new video
							vid_path = save_path
							if isinstance(vid_writer, cv2.VideoWriter):
								vid_writer.release()  # release previous video writer
							if vid_cap:  # video
								fps	=	vid_cap.get(cv2.CAP_PROP_FPS)
								w		=	int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
								h		=	int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
							else:  # stream
								fps, w, h	=	30, im0.shape[1], im0.shape[0]
								save_path	+=	'.mp4'

							vid_writer	=	cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
						vid_writer.write(im0)


#		print(	save_txt	)
#		print(	save_img	)
#		if save_txt or save_img:
#			s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ''
#			#print(f"Results saved to {save_dir}{s}")

		print(f'Done. ({time.time() - t0:.3f}s)')


if __name__ == '__main__':

	parser = argparse.ArgumentParser()
	parser.add_argument('--weights', nargs='+', type=str, default='yolov7.pt', help='model.pt path(s)')
	parser.add_argument('--source', type=str, default='inference/images', help='source')  # file/folder, 0 for webcam
	parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
	parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
	parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
	parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
	parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
	parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
	parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
	parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
	parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
	parser.add_argument('--augment', action='store_true', help='augmented inference')
	parser.add_argument('--project', default='runs/detect', help='save results to project/name')
	parser.add_argument('--name', default='exp', help='save results to project/name')
	parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
	opt = parser.parse_args()


	opt.save_txt		=	True;
	opt.img_size		=	1280;
	opt.conf_thres	=	0.4;
	opt.weights			=	['yolov7-e6e.pt']
	opt.device			=	'0';
	#check_requirements(exclude=('pycocotools', 'thop'))
	with torch.no_grad():
		detect()
