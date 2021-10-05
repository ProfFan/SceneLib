/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "horatio/global.h"

int hor_errno;

/*******************
*   Horatio error codes:
*
*   @HORATIO_OK: No error.
*
*   Global library:
*   @HOR_GLOBAL_READ_FAILED:     Read failed.
*   @HOR_GLOBAL_TALLOC_FAILED:   Call to hor_talloc() failed.
*   @HOR_GLOBAL_TALLOC_TOO_BIG:  Request for temporary allocation of a block
*                               larger than the maximum size.
*   @HOR_GLOBAL_TALLOC_OVERFLOW: Too many temporary blocks allocated.
*
*   Maths library:
*   @HOR_MATH_MATRIX_SINGULAR:           Singular matrix.
*   @HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS: Illegal matrix dimensions.
*   @HOR_MATH_MATRIX_INCOMPATIBLE:       Matrices incompatible.
*   @HOR_MATH_MATRIX_NOT_POS_DEFINITE:   Matrix not positive definite.
*   @HOR_MATH_MATRIX_EXTRA_ZEROES:       Augment matrix with extra zero rows.
*   @HOR_MATH_MATRIX_NOT_SQUARE:         Matrix not square.
*   @HOR_MATH_ILLEGAL_VALUE:             Illegal value.
*   @HOR_MATH_NO_CONVERGENCE:            No convergence achieved.
*   @HOR_MATH_NUMREC_BAD_PERMUTATION:    Bad lista permutation in Numerical
*                                       Recipes.
*   @HOR_MATH_ALLOCATION_FAILED:         Allocation failed.
*   @HOR_MATH_NULL_POINTER_ARGUMENT:     NULL pointer argument passed to
*                                       function.
*   @HOR_MATH_LIMIT_EXCEEDED:           Built in limit exceeded.
*
*   List library:
*   @HOR_LIST_ASSOCIATION_NOT_FOUND: Assocation not found for label in list.
*
*   Image library:
*   @HOR_IMAGE_CANNOT_READ_IMAGE_HEADER:     Can not read image header from
*                                           file.
*   @HOR_IMAGE_ILLEGAL_IMAGE_TYPE_IN_HEADER: Illegal image type in file header.
*   @HOR_IMAGE_ILLEGAL_PIXEL_SCALE:          Illegal pixel value scaling factor.
*   @HOR_IMAGE_ILLEGAL_BITS_PER_PIXEL:       Illegal bits-per-pixel.
*   @HOR_IMAGE_INVALID_IMAGE_REGION:         Invalid image region.
*   @HOR_IMAGE_IMAGE_TOO_SMALL:              Image too small.
*   @HOR_IMAGE_OPEN_FAILED_FOR_READ:         File open failed for read.
*   @HOR_IMAGE_OPEN_FAILED_FOR_WRITE:        File open failed for write.
*   @HOR_IMAGE_READ_FAILED:                  Read failed.
*   @HOR_IMAGE_WRITE_FAILED:                 Write failed.
*   @HOR_IMAGE_ALLOCATION_FAILED:            Allocation failed.
*   @HOR_IMAGE_NULL_POINTER_ARGUMENT:        NULL pointer argument passed.
*   @HOR_IMAGE_WRONG_TYPE_IMAGE:             Image has wrong type.
*   @HOR_IMAGE_BAD_MAGIC_NUMBER:             Illegal magic number in header.
*
*   Graphics library:
*   @HOR_GRAPHICS_DISPLAY_WINDOW_NOT_SET:      Display window not set.
*   @HOR_GRAPHICS_CANVAS_NOT_INITIALIZED:      Canvas not initialized.
*   @HOR_GRAPHICS_ILLEGAL_SUBSAMPLING_RATIO:   Illegal subsampling ratio.
*   @HOR_GRAPHICS_NON_EXISTENT_FONT_NAME:      Non-existent font.
*   @HOR_GRAPHICS_ILLEGAL_INTERNAL_DIMENSIONS: Illegal internal dimensions.
*   @HOR_GRAPHICS_X_DISPLAY_FUNCTION_FAILED:   X display function failed.
*   @HOR_GRAPHICS_INVALID_DISPLAY_DEPTH:       Invalid display depth.
*   @HOR_GRAPHICS_COLOUR_NOT_ALLOCATED:        Colour not allocated.
*   @HOR_GRAPHICS_NULL_POINTER_ARGUMENT:       NULL pointer argument passed.
*   @HOR_GRAPHICS_WRONG_TYPE_IMAGE:            Image has wrong type.
*   @HOR_GRAPHICS_NON_EXISTENT_IMAGE:          Cannot read image file.
*   @HOR_GRAPHICS_INCOMPATIBLE_IMAGE:          Image incompatible with user
*                                              dimensions.
*
*   Image processing library:
*   @HOR_IMPROC_REGION_TOO_SMALL:        Region too small.
*   @HOR_IMPROC_ILLEGAL_IMAGE_TYPE:      Illegal image type in image processing
*                                       operation.
*   @HOR_IMPROC_ILLEGAL_CORNER_POSITION: Illegal corner position.
*   @HOR_IMPROC_ILLEGAL_PARAMETERS:      Illegal parameter values passed to
*                                       function.
*   @HOR_IMPROC_IMAGES_INCOMPATIBLE:     Images incompatible with each other.
*   @HOR_IMPROC_NULL_POINTER_ARGUMENT:   NULL pointer argument passed.
*   @HOR_IMPROC_OPEN_FAILED_FOR_READ:    File open failed for read.
*   @HOR_IMPROC_OPEN_FAILED_FOR_WRITE:   File open failed for write.
*   @HOR_IMPROC_READ_FAILED:             Read failed.
*   @HOR_IMPROC_ALLOCATION_FAILED:       Allocation failed.
*
*   Process library:
*   @HOR_PROCESS_UNDEFINED_RESULT_TYPE:         Undefined result type.
*   @HOR_PROCESS_UNDEFINED_PROCESS:             Undefined process.
*   @HOR_PROCESS_UNDEFINED_PROCESS_TYPE:        Undefined process type.
*   @HOR_PROCESS_INCOMPATIBLE_RESULT_TYPES:     Incompatible result types.
*   @HOR_PROCESS_DIFFERENT_LENGTH_RESULT_LISTS: Different length result type
*                                              lists.
*   @HOR_PROCESS_DUPLICATED_RESULT_TYPES:       Duplicated result types.
*   @HOR_PROCESS_DUPLICATED_PROCESS_TYPES:      Duplicated process types.
*   @HOR_PROCESS_ILLEGAL_PROCESS_DATA:          Illegal non-null process data.
*   @HOR_PROCESS_ILLEGAL_OUTPUT_DATA:           Illegal non-null output data.
*   @HOR_PROCESS_IMAGES_INCOMPATIBLE:           Images incompatible with each
*                                              other.
*   @HOR_PROCESS_NULL_POINTER_ARGUMENT:         NULL pointer argument passed.
*
*   Tool library:
*   @HOR_TOOL_POPUP_PANEL_NOT_IN_USE:     Popup panel not in use.
*   @HOR_TOOL_POPUP_PANEL_ALREADY_IN_USE: Popup panel already in use.
*   @HOR_TOOL_POPUP_PANEL_NOT_REGISTERED: Popup panel not registered.
*   @HOR_TOOL_POPUP_PANEL_NOT_INIT:       Popup panel not initialised.
*   @HOR_TOOL_GRAPH_NEGATIVE_TIME_RANGE:  Negative graph time range specified.
*   @HOR_TOOL_GRAPH_NEGATIVE_F_RANGE:     Negative graph function range
*                                        specified.
*   @HOR_TOOL_GRAPH_ALREADY_SET_UP:       Graph already set up.
*   @HOR_TOOL_GRAPH_NOT_SET_UP:           Graph not set up.
*   @HOR_TOOL_GRAPH_TIME_REVERSAL:        Graph time-reversal.
*   @HOR_TOOL_GRAPH_NO_POINTS:            No points on graph.
*   @HOR_TOOL_OPEN_FAILED_FOR_WRITE:      File open failed for write.
*   @HOR_TOOL_3D_ITEM_NOT_FOUND:          3D item label not recognised.
*   @HOR_TOOL_ALLOCATION_FAILED:          Allocation failed.
********************/

#define HOR_GLOBAL_ERROR_BASE   1
#define HOR_MATH_ERROR_BASE     2
#define HOR_PIPE_ERROR_BASE     3
#define HOR_LIST_ERROR_BASE     4
#define HOR_IMAGE_ERROR_BASE    5
#define HOR_GRAPHICS_ERROR_BASE 6
#define HOR_IMPROC_ERROR_BASE   7
#define HOR_PROCESS_ERROR_BASE  8
#define HOR_TOOL_ERROR_BASE     9

/*******************
*   void @hor_perror ( const char *s )
*
*   Prints a Horatio error message, prefixed by the given string if one is
*   provided. Works like the normal hor_perror().
********************/
void hor_perror ( const char *s )
{
   char prefix[300], message[300], library[20];

   if ( s == NULL || strlen(s) == 0 )
      sprintf ( prefix, "Horatio:" );
   else
      sprintf ( prefix, "%s: Horatio:", s );

   if ( hor_errno == HORATIO_OK )
   {
      hor_print ( "%s No error\n", prefix );
      return;
   }

   switch ( hor_errno )
   {
      /* global library */
      case HOR_GLOBAL_READ_FAILED:
      strcpy ( message, "Write failed" ); break;

      case HOR_GLOBAL_TALLOC_FAILED:
      strcpy ( message, "Call to hor_talloc() failed" ); break;

      case HOR_GLOBAL_TALLOC_TOO_BIG:
      strcpy ( message, "Temporary allocation block size too large" ); break;

      case HOR_GLOBAL_TALLOC_OVERFLOW:
      strcpy ( message, "Too many temporary blocks allocated" ); break;

      case HOR_MATH_MATRIX_SINGULAR:
      strcpy ( message, "Singular matrix" ); break;

      case HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS:
      strcpy ( message, "Illegal matrix dimensions" ); break;

      case HOR_MATH_MATRIX_INCOMPATIBLE:
      strcpy ( message, "Matrices incompatible" ); break;

      case HOR_MATH_MATRIX_NOT_POS_DEFINITE:
      strcpy ( message, "Matrix not positive definite" ); break;

      case HOR_MATH_MATRIX_EXTRA_ZEROES:
      strcpy ( message, "Augment matrix with extra zero rows" ); break;

      case HOR_MATH_MATRIX_NOT_SQUARE:
      strcpy ( message, "Hor_Matrix not square" ); break;

      case HOR_MATH_ILLEGAL_VALUE:
      strcpy ( message, "Illegal value" ); break;

      case HOR_MATH_NO_CONVERGENCE:
      strcpy ( message, "No convergence achieved" ); break;

      case HOR_MATH_NUMREC_BAD_PERMUTATION:
      strcpy ( message, "Bad lista permutation in Numerical Recipes" ); break;

      case HOR_MATH_ALLOCATION_FAILED:
      strcpy ( message, "Allocation failed" ); break;

      case HOR_MATH_NULL_POINTER_ARGUMENT:
      strcpy ( message, "NULL pointer argument" ); break;

      case HOR_MATH_LIMIT_EXCEEDED:
      strcpy ( message, "Built-in limit exceeded" ); break;

      /* list library */
      case HOR_LIST_ASSOCIATION_NOT_FOUND:
      strcpy ( message, "Assocation not found for label in list" ); break;

      /* image library */
      case HOR_IMAGE_CANNOT_READ_IMAGE_HEADER:
      strcpy ( message, "Can not read image header from file" ); break;

      case HOR_IMAGE_ILLEGAL_IMAGE_TYPE_IN_HEADER:
      strcpy ( message, "Illegal image type in file header" ); break;

      case HOR_IMAGE_ILLEGAL_PIXEL_SCALE:
      strcpy ( message, "Illegal pixel value scaling factor" ); break;

      case HOR_IMAGE_ILLEGAL_BITS_PER_PIXEL:
      strcpy ( message, "Illegal bits-per-pixel" ); break;

      case HOR_IMAGE_INVALID_IMAGE_REGION:
      strcpy ( message, "Invalid image region" ); break;

      case HOR_IMAGE_IMAGE_TOO_SMALL:
      strcpy ( message, "Image too small" ); break;

      case HOR_IMAGE_OPEN_FAILED_FOR_READ:
      strcpy ( message, "File open failed for read" ); break;

      case HOR_IMAGE_OPEN_FAILED_FOR_WRITE:
      strcpy ( message, "File open failed for write" ); break;

      case HOR_IMAGE_READ_FAILED:
      strcpy ( message, "Write failed" ); break;

      case HOR_IMAGE_WRITE_FAILED:
      strcpy ( message, "Write failed" ); break;

      case HOR_IMAGE_ALLOCATION_FAILED:
      strcpy ( message, "Allocation failed" ); break;

      case HOR_IMAGE_NULL_POINTER_ARGUMENT:
      strcpy ( message, "NULL pointer argument passed" ); break;

      case HOR_IMAGE_WRONG_TYPE_IMAGE:
      strcpy ( message, "Image has wrong type" ); break;

      case HOR_IMAGE_BAD_MAGIC_NUMBER:
      strcpy ( message, "Illegal magic number in header" ); break;

      /* graphics library */
      case HOR_GRAPHICS_DISPLAY_WINDOW_NOT_SET:
      strcpy ( message, "Display window not set" ); break;

      case HOR_GRAPHICS_CANVAS_NOT_INITIALIZED:
      strcpy ( message, "Canvas not initialized" ); break;

      case HOR_GRAPHICS_ILLEGAL_SUBSAMPLING_RATIO:
      strcpy ( message, "Illegal subsampling ratio" ); break;

      case HOR_GRAPHICS_NON_EXISTENT_FONT_NAME:
      strcpy ( message, "Non-existent font" ); break;

      case HOR_GRAPHICS_ILLEGAL_INTERNAL_DIMENSIONS:
      strcpy ( message, "Illegal internal dimensions" ); break;

      case HOR_GRAPHICS_X_DISPLAY_FUNCTION_FAILED:
      strcpy ( message, "X display function failed" ); break;

      case HOR_GRAPHICS_INVALID_DISPLAY_DEPTH:
      strcpy ( message, "Invalid display depth" ); break;

      case HOR_GRAPHICS_COLOUR_NOT_ALLOCATED:
      strcpy ( message, "Colour not allocated" ); break;

      case HOR_GRAPHICS_NULL_POINTER_ARGUMENT:
      strcpy ( message, "NULL pointer argument passed" ); break;

      case HOR_GRAPHICS_WRONG_TYPE_IMAGE:
      strcpy ( message, "Image has wrong type" ); break;

      case HOR_GRAPHICS_NON_EXISTENT_IMAGE:
      strcpy ( message, "Cannot read image file" ); break;

      case HOR_GRAPHICS_INCOMPATIBLE_IMAGE:
      strcpy ( message, "Image incompatible with user dimensions" ); break;

      /* improc library */
      case HOR_IMPROC_REGION_TOO_SMALL:
      strcpy ( message, "Region too small" ); break;

      case HOR_IMPROC_ILLEGAL_IMAGE_TYPE:
      strcpy ( message, "Illegal image type in image processing operation" );
      break;

      case HOR_IMPROC_ILLEGAL_CORNER_POSITION:
      strcpy ( message, "Illegal corner position" );
      break;

      case HOR_IMPROC_ILLEGAL_PARAMETERS:
      strcpy ( message, "Illegal parameter values passed to function" ); break;

      case HOR_IMPROC_IMAGES_INCOMPATIBLE:
      strcpy ( message, "Images incompatible with each other" ); break;

      case HOR_IMPROC_NULL_POINTER_ARGUMENT:
      strcpy ( message, "NULL pointer argument passed" ); break;

      case HOR_IMPROC_OPEN_FAILED_FOR_READ:
      strcpy ( message, "File open failed for read" ); break;

      case HOR_IMPROC_OPEN_FAILED_FOR_WRITE:
      strcpy ( message, "File open failed for write" ); break;

      case HOR_IMPROC_READ_FAILED:
      strcpy ( message, "Write failed" ); break;

      case HOR_IMPROC_ALLOCATION_FAILED:
      strcpy ( message, "Allocation failed" ); break;

      /* process library */
      case HOR_PROCESS_UNDEFINED_RESULT_TYPE:
      strcpy ( message, "Undefined result type" ); break;

      case HOR_PROCESS_UNDEFINED_PROCESS:
      strcpy ( message, "Undefined process" ); break;

      case HOR_PROCESS_UNDEFINED_PROCESS_TYPE:
      strcpy ( message, "Undefined process type" ); break;

      case HOR_PROCESS_INCOMPATIBLE_RESULT_TYPES:
      strcpy ( message, "Incompatible result types" ); break;

      case HOR_PROCESS_DIFFERENT_LENGTH_RESULT_LISTS:
      strcpy ( message, "Different length result type lists" ); break;

      case HOR_PROCESS_DUPLICATED_RESULT_TYPES:
      strcpy ( message, "Duplicated result types" ); break;

      case HOR_PROCESS_DUPLICATED_PROCESS_TYPES:
      strcpy ( message, "Duplicated process types" ); break;

      case HOR_PROCESS_ILLEGAL_PROCESS_DATA:
      strcpy ( message, "Illegal non-null process data" ); break;

      case HOR_PROCESS_ILLEGAL_OUTPUT_DATA:
      strcpy ( message, "Illegal non-null output data" ); break;

      case HOR_PROCESS_IMAGES_INCOMPATIBLE:
      strcpy ( message, "Images incompatible with each other" ); break;

      case HOR_PROCESS_NULL_POINTER_ARGUMENT:
      strcpy ( message, "NULL pointer argument passed" ); break;

      /* tool library */
      case HOR_TOOL_POPUP_PANEL_NOT_IN_USE:
      strcpy ( message, "Popup panel not in use" ); break;

      case HOR_TOOL_POPUP_PANEL_ALREADY_IN_USE:
      strcpy ( message, "Popup panel already in use" ); break;

      case HOR_TOOL_POPUP_PANEL_NOT_REGISTERED:
      strcpy ( message, "Popup panel not registered" ); break;

      case HOR_TOOL_POPUP_PANEL_NOT_INIT:
      strcpy ( message, "Popup panel not initialised" ); break;

      case HOR_TOOL_GRAPH_NEGATIVE_TIME_RANGE:
      strcpy ( message, "Negative graph time range specified" ); break;

      case HOR_TOOL_GRAPH_NEGATIVE_F_RANGE:
      strcpy ( message, "Negative graph function range specified" ); break;

      case HOR_TOOL_GRAPH_ALREADY_SET_UP:
      strcpy ( message, "Graph already set up" ); break;

      case HOR_TOOL_GRAPH_NOT_SET_UP:
      strcpy ( message, "Graph not set up" ); break;

      case HOR_TOOL_GRAPH_TIME_REVERSAL:
      strcpy ( message, "Graph time-reversal" ); break;

      case HOR_TOOL_GRAPH_NO_POINTS:
      strcpy ( message, "No points on graph" ); break;

      case HOR_TOOL_OPEN_FAILED_FOR_WRITE:
      strcpy ( message, "File open failed for write" ); break;

      case HOR_TOOL_3D_ITEM_NOT_FOUND:
      strcpy ( message, "3D item label not recognised" ); break;

      case HOR_TOOL_ALLOCATION_FAILED:
      strcpy ( message, "Allocation failed" ); break;

      default:
      strcpy ( message, "Unrecognised error" ); break;
   }

   /* determine which Horatio library invoked the error */
   switch ( hor_errno/100 )
   {
      case HOR_GLOBAL_ERROR_BASE:   strcpy ( library, "global" );   break;
      case HOR_MATH_ERROR_BASE:     strcpy ( library, "math" );     break;
      case HOR_PIPE_ERROR_BASE:     strcpy ( library, "pipe" );     break;
      case HOR_LIST_ERROR_BASE:     strcpy ( library, "list" );     break;
      case HOR_IMAGE_ERROR_BASE:    strcpy ( library, "image" );    break;
      case HOR_GRAPHICS_ERROR_BASE: strcpy ( library, "graphics" ); break;
      case HOR_IMPROC_ERROR_BASE:   strcpy ( library, "improc" );   break;
      case HOR_PROCESS_ERROR_BASE:  strcpy ( library, "process" );  break;
      case HOR_TOOL_ERROR_BASE:     strcpy ( library, "tool" );     break;
      default:                      strcpy ( library, "????" );     break;
   }

   hor_print ( "%s %s (%s)\n", prefix, message, library );
}
