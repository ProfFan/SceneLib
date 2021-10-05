/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */

/*******************
*   @IMAGE_CODE, @CANNY_CODE, @LINE_FIT_CODE, @LINE_SEGMENT_CODE,
*   @PLESSEY_CORNER_CODE, @SMITH_CORNER_CODE, @WANG_CORNER_CODE,
*   @BOG_CORNER_MATCH_CODE, @BD_CORNER_MATCH_CODE,
*   @LINE_MATCH_CODE, @IMAGE_FLOW_CODE, @IMAGE_SEGMENT_CODE, @CORRELATION_CODE
*
*   Process type codes for grab, canny, Plessey corner, Smith corner,
*   corner matching, line fitting, line matching, image flow, image
*   segmentation and correlation process types.
********************/
#define IMAGE_CODE             0x0 /* bare image: no operation performed */
#define CANNY_CODE             0x1
#define LINE_FIT_CODE          0x2
#define LINE_SEGMENT_CODE      0x4
#define PLESSEY_CORNER_CODE    0x8
#define SMITH_CORNER_CODE     0x10
#define WANG_CORNER_CODE      0x20
#define BOG_CORNER_MATCH_CODE 0x40
#define BD_CORNER_MATCH_CODE  0x80
#define LINE_MATCH_CODE      0x100
#define IMAGE_FLOW_CODE      0x200
#define IMAGE_SEGMENT_CODE   0x400
#define CORRELATION_CODE     0x800

/*******************
*   @IMAGE_MASK, @CANNY_MASK, @LINE_FIT_MASK, @LINE_SEGMENT_MASK,
*   @PLESSEY_CORNER_MASK, @SMITH_CORNER_MASK, @WANG_CORNER_MASK,
*   @BOG_CORNER_MATCH_MASK, @BD_CORNER_MATCH_MASK,
*   @LINE_MATCH_MASK, @IMAGE_FLOW_MASK, @IMAGE_SEGMENT_MASK, @CORRELATION_MASK
*
*   Process type mask codes, i.e. 0xffffffff exclusive-or'd with the
*   corresponding process type codes.
********************/
#define CANNY_MASK            (0xffffffff ^            CANNY_CODE)
#define LINE_FIT_MASK         (0xffffffff ^         LINE_FIT_CODE)
#define LINE_SEGMENT_MASK     (0xffffffff ^     LINE_SEGMENT_CODE)
#define PLESSEY_CORNER_MASK   (0xffffffff ^   PLESSEY_CORNER_CODE)
#define SMITH_CORNER_MASK     (0xffffffff ^     SMITH_CORNER_CODE)
#define WANG_CORNER_MASK      (0xffffffff ^      WANG_CORNER_CODE)
#define BOG_CORNER_MATCH_MASK (0xffffffff ^ BOG_CORNER_MATCH_CODE)
#define BD_CORNER_MATCH_MASK  (0xffffffff ^  BD_CORNER_MATCH_CODE)
#define LINE_MATCH_MASK       (0xffffffff ^       LINE_MATCH_CODE)
#define IMAGE_FLOW_MASK       (0xffffffff ^       IMAGE_FLOW_CODE)
#define IMAGE_SEGMENT_MASK    (0xffffffff ^    IMAGE_SEGMENT_CODE)
#define CORRELATION_MASK      (0xffffffff ^      CORRELATION_CODE)

#ifdef _XtIntrinsic_h

Widget fill_ops_params_panel ( Widget parent, ... );

#endif /* _XtIntrinsic_h */

int get_ops_toggle_state(void);
