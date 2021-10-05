/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdio.h>
#include <stdlib.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"
#include "horatio/process.h"

static Hor_Assoc_Label gb_result_type_label  = HOR_ASSOC_ERROR;
static Hor_Assoc_Label gb_process_type_label = HOR_ASSOC_ERROR;
static Hor_Assoc_Label gb_process_label      = HOR_ASSOC_ERROR;

/*******************
*   void @hor_set_gb_result_type_label ( Hor_Assoc_Label result_type_label )
*   void @hor_set_gb_process_type_label ( Hor_Assoc_Label process_type_label )
*   Hor_Assoc_Label @hor_get_gb_result_type_label(void)
*   Hor_Assoc_Label @hor_get_gb_process_type_label(void)
*   Hor_Assoc_Label @hor_get_gb_process_label(void)
*   Hor_GB_Process_Data *@hor_gb_make_process_data ( const char *root_name )
*   void  @hor_gb_add_process         ( const char *root_name )
*   void *@hor_gb_execute             ( Hor_List  input_list,
*                                      void *gb_old_result, void *gb_data )
*   void  @hor_gb_output              ( void *gb_result, void *gb_data )
*   void  @hor_gb_update_process_data ( void *data )
*   void  @hor_gb_free_process_data   ( void *data )
*   void  @hor_gb_free_result         ( void *gb_result )
*
*   Grab "process" definition functions.
********************/
void hor_set_gb_result_type_label ( Hor_Assoc_Label result_type_label )
{
   gb_result_type_label = result_type_label;
}

void hor_set_gb_process_type_label ( Hor_Assoc_Label process_type_label )
{
   gb_process_type_label = process_type_label;
}

Hor_Assoc_Label hor_get_gb_result_type_label(void)
{
   return gb_result_type_label;
}

Hor_Assoc_Label hor_get_gb_process_type_label(void)
{
   return gb_process_type_label;
}

Hor_Assoc_Label hor_get_gb_process_label(void)
{
   return gb_process_label;
}

Hor_GB_Process_Data *hor_gb_make_process_data ( const char *root_name )
{
   Hor_GB_Process_Data *data;

   data = hor_malloc_type ( Hor_GB_Process_Data );
   strcpy ( data->root_name, root_name );
   data->image_count = 0;
   return data;
}

void hor_gb_add_process ( const char *root_name )
{
   if ( gb_process_type_label == HOR_ASSOC_ERROR )
      hor_error ( "grab process type not set (hor_gb_add_process)", HOR_FATAL);

   if ( gb_process_label != HOR_ASSOC_ERROR )
      hor_error("grab process already exists (hor_gb_add_process)", HOR_FATAL);

   gb_process_label = hor_add_process ( gb_process_type_label, NULL,
				        hor_gb_make_process_data ( root_name ),
				        NULL );
}

void *hor_gb_execute ( Hor_List input_list, void *gb_old_result, void *gb_data )
{
   Hor_Image           *old_result = (Hor_Image *)           gb_old_result;
   Hor_GB_Process_Data *data       = (Hor_GB_Process_Data *) gb_data;
   Hor_Image           *image;

   if ( input_list != NULL )
      hor_error ( "corrupted process input list (hor_gb_execute)", HOR_FATAL );

   image = hor_read_next_image ( data->root_name, data->image_count,
				 HOR_NON_FATAL);
   if ( image == NULL ) return NULL;

   if ( old_result != NULL )
      if ( image->width  != old_result->width ||
	   image->height != old_result->height ||
	   image->type   != old_result->type )
      {
	 hor_errno = HOR_PROCESS_IMAGES_INCOMPATIBLE;
	 hor_free_image ( image );
	 return NULL;
      }

   if ( data->image_count == 0 )
      hor_message ( "initialised horatio" );

   return ((void *) image);
}

void hor_gb_output ( void *gb_result, void *gb_data ) /* NULL */
{
   Hor_Image *result = (Hor_Image *) gb_result;

   if ( result == NULL ) return;
   hor_display_image ( result, 1, 0.0, 256.0 );
}

void hor_gb_update_process_data ( void *data )
{
   ((Hor_GB_Process_Data *) data)->image_count++;
}

void hor_gb_free_process_data ( void *data )
{
   hor_free ( (void *) data );
   gb_process_label = HOR_ASSOC_ERROR;
}

void hor_gb_free_result ( void *gb_result )
{
   Hor_Image *result = (Hor_Image *) gb_result;

   if ( result == NULL ) return;
   hor_free_image ( result );
}
