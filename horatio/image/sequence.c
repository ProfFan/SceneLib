/* Copyright 1994 Jason Merron (jasm@robots.oxford.ac.uk))
              and Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdio.h>
#include <stddef.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"

static char digit ( int number )
{
   switch ( number )
   {
      case 0: return '0';
      case 1: return '1';
      case 2: return '2';
      case 3: return '3';
      case 4: return '4';
      case 5: return '5';
      case 6: return '6';
      case 7: return '7';
      case 8: return '8';
      case 9: return '9';
      default: hor_error ( "illegal number (digit)", HOR_FATAL );
   }

   return -1;
}

/*******************
*   Hor_Count_String @hor_count_string ( int count )
*   Hor_Image *@hor_read_next_image ( const char *root_name, int count,
*                                    Hor_Error_Type error_type )
*   void @hor_write_next_image ( const char *root_name, int count, 
*                               Hor_Image *image )
*
*   Functions for reading and writing image sequence.
*
*   hor_count_string() returns a three-character string representation of a
*   number, e.g. "045" for 45.
*
*   hor_read_next_image() calls hor_read_image() to read an image from a
*   sequence, with given number in sequence. The error type is used when the
*   read fails. hor_write_next_image() calls hor_write_image() to output an
*   image of a sequence.
********************/
Hor_Count_String hor_count_string ( int count )
{
   Hor_Count_String cs;

   sprintf ( cs.string, "%c%c%c", digit((count/100) % 10),
                                  digit((count/10) % 10),
                                  digit(count % 10) );
   return cs;
}

Hor_Image *hor_read_next_image ( const char *root_name, int count,
				 Hor_Error_Type error_type )
{
   char             base_name[100];
   Hor_Image       *result;
   Hor_Count_String cstring = hor_count_string(count);

   sprintf ( base_name, "%s.%s", root_name, cstring.string );
   result = hor_read_image ( base_name );
   if ( result == NULL ) /* try two-digit name: base-name%c%c.<suffix> */
   {
      sprintf ( base_name, "%s%c%c", root_name,
	                             cstring.string[1], cstring.string[2] );
      result = hor_read_image ( base_name );
      if ( result == NULL )
      {
	 hor_perror ( "" );
	 hor_error ( "cannot read image %d of sequence %s", error_type,
		     count, root_name );
      }
      else
	 hor_message ( "read image %s", base_name );
   }
   else hor_message ( "read image %s", base_name );

   return result;
}

void hor_write_next_image ( const char *root_name, int count,
			    Hor_Image *image )
{
   char             base_name[100];
   Hor_Count_String cstring = hor_count_string(count);

   sprintf ( base_name, "%s.%s", root_name, cstring.string );
   hor_write_image ( base_name, image, 0.0, 256.0 );
}

/*******************
*   Hor_List @hor_read_image_sequence ( char *root_name,
*                                      int start, int finish, int *last_image)
*
*   Reads and returns a list of images with the given root name, between
*   numbers "start" and "finish".  The number of the final image read is
*   returned in the variable "last_image", or -1 if no images were read.
*   It can be passed as NULL in which case it is ignored.
*   Additionally, the function returns NULL if no images are read.
********************/
Hor_List hor_read_image_sequence ( char *root_name, int start, int finish,
				   int *last_image )
{ 
   Hor_List     seqptr = NULL;
   Hor_Image    *imptr;
   int          image_number;

   if (start > finish)
   {
      hor_error ("finish frame less than start frame (hor_read_image_sequence)", HOR_NON_FATAL);
      return NULL;
   }

   for (image_number = start; image_number <= finish; image_number++)
   {
      if ((imptr = hor_read_next_image (root_name, image_number,
					HOR_NON_FATAL)) == NULL )
	 break;
      
      seqptr = hor_insert (seqptr, imptr);
   }

   if ( last_image != NULL )
      if (image_number == start)  /* if no images were read, set "last image read"*/
	 *last_image = -1;         /*   to -1 to flag this condition */
      else
	 *last_image = image_number - 1;

   return (hor_reverse (seqptr));
}

/*******************
*   void @hor_write_image_sequence ( Hor_List sequence, char *root_name,
*                                   int start )
*
*   Writes a list of images to disk with given root name, starting the sequence
*   numbering with "start".
********************/
void hor_write_image_sequence (Hor_List sequence, char *root_name, int start)
{
   char       new_root_name[200];
   int        image_number;
   Hor_Image *current_imptr;
   Hor_List   list_ptr;

   strcpy(new_root_name, root_name);

   for ( list_ptr = sequence, image_number = start; 
	 list_ptr != NULL; list_ptr = list_ptr->next, image_number++ )
   {
      current_imptr = (Hor_Image *) list_ptr->contents;
      hor_write_next_image (new_root_name, image_number, current_imptr);
   }
}

/*******************
*   void hor_free_image_sequence (Hor_List sequence)
*
*   Frees a list of images.
********************/
void hor_free_image_sequence (Hor_List sequence)
{
  hor_free_list (sequence, (void (*)(void *)) hor_free_image);
}
