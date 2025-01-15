import	argparse
import	time

from	pathlib	import Path

import	torch
import	cv2
import	yaml
from		torchvision	import	transforms

import	numpy	as	np

from	utils.datasets	import	letterbox
from	utils.general		import	non_max_suppression_mask_conf,	increment_path

from	detectron2.modeling.poolers		import	ROIPooler
from	detectron2.structures					import	Boxes

from	detectron2.utils.memory			import	retry_if_cuda_oom
from	detectron2.layers							import	paste_masks_in_image





def	processFrame(image):

 image = letterbox(	image,	640,	stride=64,	auto=True		)[0]

 image_ = image.copy()
 image  = transforms.ToTensor()(image)
 image  = torch.tensor(np.array(	[	image.numpy()	]	)	)
 image  = image.to(device)
 image  = image.half()	if	half	else	image.float()

 output = model(image)

 inf_out, train_out, attn, mask_iou, bases, sem_output = output['test'], output['bbox_and_cls'], output['attn'], output['mask_iou'], output['bases'], output['sem']

 bases = torch.cat( [bases, sem_output], dim=1 )

 nb, _, height, width = image.shape

 names  = model.names
 pooler = ROIPooler(output_size=hyp['mask_resolution'],	scales=(model.pooler_scale,),	sampling_ratio	=	1,	pooler_type='ROIAlignV2',	canonical_level=2)

 # non maxima suppression
 output, output_mask, output_mask_score, output_ac, output_ab = non_max_suppression_mask_conf( inf_out,                 attn,         bases,
                                                                                               pooler,                  hyp,          conf_thres=opt.conf_thres,
                                                                                               iou_thres=opt.iou_thres, merge=False,  mask_iou=None )

 pred, pred_masks = output[0], output_mask[0]
 base =	bases[0]


 if	pred	is not None:
  bboxes	=	Boxes(	pred[:,:4]	)
  original_pred_masks	=	pred_masks.view(-1,	hyp['mask_resolution'],	hyp['mask_resolution'])

  pred_masks	=	retry_if_cuda_oom(paste_masks_in_image)(original_pred_masks,	bboxes,	(height,width),	threshold	=	0.50)

  pred_masks_np	=	pred_masks.detach().cpu().numpy()
  pred_cls			=	pred[:,5].detach().cpu().numpy()
  pred_conf			=	pred[:,4].detach().cpu().numpy()
  nbboxes				=	bboxes.tensor.detach().cpu().numpy().astype(int)

  image_display	=	image[0].permute(1,2,0)*255
  image_display	=	image_display.cpu().numpy().astype(np.uint8)

  image_display	=	cv2.cvtColor(image_display,	cv2.COLOR_RGB2BGR)

  for	one_mask,	bbox,	cls,	conf	in	zip(pred_masks_np,	nbboxes,	pred_cls,	pred_conf):

   if	conf	<	opt.conf_thres:
    continue

   color	=	[	np.random.randint(255),	np.random.randint(255),	np.random.randint(255)	]

   image_display[one_mask]	=	image_display[one_mask]	*	0.5	+	np.array(color,	dtype	=	np.uint8)*0.5

   label	=	'%s %.3f'	%	(names[int(cls)],	conf)

   tf	=	max(	opt.thickness - 1,	1	)
   t_size	=	cv2.getTextSize(	label,	0,	fontScale=opt.thickness/3,	thickness	=	tf	)[0]
   c2	=	bbox[0]+t_size[0],	bbox[1]-t_size[1]-3

   if	not	opt.nobbox:
    cv2.rectangle(	image_display,	(bbox[0],	bbox[1]),	(bbox[2],	bbox[3]),	color,	thickness=opt.thickness,	lineType=cv2.LINE_AA	)

   if	not	opt.nolabel:
    cv2.rectangle(	image_display,	(bbox[0],	bbox[1]),	c2,	color,	-1,	cv2.LINE_AA	)
    cv2.putText(		image_display,	label,	(bbox[0],	bbox[1]-2),	0,	opt.thickness/3,	[255,255,255],	thickness=tf,	lineType=cv2.LINE_AA		)


   return image_display

 return image_





def	onImage():
 image	=	cv2.imread(opt.source)
 assert	image	is	not	None,	'Image Not Found '	+	opt.source

 image	=	cv2.cvtColor(image,	cv2.COLOR_BGR2RGB)
 img_size	=	image.shape

 image_display	=	processFrame(image)

 image_display	=	cv2.resize(image_display,	(img_size[1],	img_size[0])	)

 if	not	opt.nosave:
  cv2.imwrite(save_path,	image_display)
  print("Output saved: ",	save_path)

 if	opt.view_img:
  cv2.imshow("Result",	image_display)
  cv2.waitKey(0)




if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='yolov7-mask.pt', help='model.pt path(s)')
    parser.add_argument('--source', type=str, default='1.jpg', help='source')  # file/folder, 0 for webcam
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--project', default='runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')

    parser.add_argument('--hyp', type=str, default='data/hyp.scratch.mask.yaml', help='hyperparameter file for yolov7 mask')
    parser.add_argument('--seed', type=int, default=1, help='random seed to change color')
    parser.add_argument('--thickness', type=int, default=1, help='affects bbox and font size')
    parser.add_argument('--nobbox', action='store_true', help='hide bounding boxes')
    parser.add_argument('--nolabel', action='store_true', help='hide instance labels')
    parser.add_argument('--showfps', action='store_true', help='show fps on the videos/webcam')





    opt = parser.parse_args()
    print(opt)
    np.random.seed(opt.seed)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    half = device.type != "cpu"

    weights = torch.load(opt.weights)
    model		=	weights['model'].to(device)
    if	half:
     model	=	model.half();


    with	open(opt.hyp)	as	f:
     hyp	=	yaml.load(f,	Loader=yaml.FullLoader)


    if	not	opt.nosave:
     save_dir	=	Path(increment_path(Path(opt.project)	/	opt.name,	exist_ok=False))	#increment runs

     save_dir.mkdir(parents=True,	exist_ok=True)

     save_path	=	str(save_dir	/	opt.source)


    webcam	=	opt.source.isnumeric()

    if	webcam	and not opt.nosave:
     save_path	=	save_path	+	".mp4"

    img_formats	=	[	'bmp',	'jpg',	'jpeg',	'png',	'tiff',	'dng',	'webp',	'mpo'	]	#	acceptable image formats
    vid_formats	=	[	'mov',	'avi',	'mp4',	'mpg',	'm4v',	'wmv',	'mkv'					]	#	acceptable video formats


    with torch.no_grad():
     if		opt.source.split('.')[-1].lower()	in	img_formats:
      onImage()
     elif	opt.source.split('.')[-1].lower()	in	vid_formats	or	webcam:
      onVideo()
     else:
      print("Not a valid source...")
