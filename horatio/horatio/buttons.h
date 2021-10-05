/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */

const char      *get_base_name(void);
Hor_Image_Format get_image_format(void);

#ifdef _XtIntrinsic_h

void fill_complicated_panel ( XtAppContext app_con,
			      Widget panel, const char *root_name,
			      XtCallbackProc quit_proc );

#endif /* _XtIntrinsic_h */
