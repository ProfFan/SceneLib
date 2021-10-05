/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/sequence.h */

#ifndef HOR_REDUCED_LIB

/*******************
*   typedef struct hor_count_string
*   {
*      char string[10];
*   } @Hor_Count_String;
*
*   Definition of structure containing image sequence number string.
********************/
typedef struct hor_count_string
{
   char string[10];
} Hor_Count_String;

Hor_Count_String hor_count_string ( int count );
Hor_Image *hor_read_next_image  ( const char *root_name, int count,
				  Hor_Error_Type error_type );
void       hor_write_next_image ( const char *root_name, int count,
				  Hor_Image *image );

#ifdef _HOR_LIST_

Hor_List hor_read_image_sequence ( char *root_name, int start, int finish,
				   int *last_image );
void hor_write_image_sequence (Hor_List sequence, char *root_name, int start);
void hor_free_image_sequence (Hor_List sequence);

#endif /* _HOR_LIST_ */
#endif /* HOR_REDUCED_LIB */
