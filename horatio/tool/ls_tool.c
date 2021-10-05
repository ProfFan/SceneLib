/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdio.h>

#ifdef HOR_MSDOS
#include <X11/Intrnsc.h>
#else
#include <X11/Intrinsic.h>
#endif

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"
#include "horatio/process.h"
#include "horatio/tool.h"

static u_long   line_colour;
static Hor_Bool colours_set = HOR_FALSE;

void hor_set_line_segment_colours ( u_long loc_line_colour )
{
   line_colour = loc_line_colour;
   colours_set = HOR_TRUE;
}

Hor_Bool hor_get_line_segment_params ( Hor_LI_Output_Params *out_prms )
{
   if ( !colours_set )
      hor_error ( "colours not set (hor_get_line_segment_params)",
		  HOR_FATAL );

   out_prms->line_colour = line_colour;
   return HOR_TRUE;
}
