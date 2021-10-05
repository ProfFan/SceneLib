/*SccsID (c) R.M.Offer @(#)FileSelector.h	1.0  23/Apr/94

  FileSelector.h

  (c) R.M.Offer 1994


As this widget is based on the Xaw Dialog widget, it shares the Dialogs 
copyright.

Copyright 1987, 1988 by Digital Equipment Corporation, Maynard, Massachusetts,
and the Massachusetts Institute of Technology, Cambridge, Massachusetts.

                        All Rights Reserved

Permission to use, copy, modify, and distribute this software and its 
documentation for any purpose and without fee is hereby granted, 
provided that the above copyright notice appear in all copies and that
both that copyright notice and this permission notice appear in 
supporting documentation, and that the names of Digital or MIT not be
used in advertising or publicity pertaining to distribution of the
software without specific, written prior permission.  

DIGITAL DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE, INCLUDING
ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO EVENT SHALL
DIGITAL BE LIABLE FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR
ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
SOFTWARE.

*/

#ifndef _XOW_FILE_SELECTOR_H
#define _XOW_FILE_SELECTOR_H

#ifdef FUNCPROTO
#define XOW_USE_PROTO
#define ARGS(a) a
#else
#define ARGS(a) ()
#endif

/* Additional Parameters (above those defined in the form class ):

 Name		     Class		RepType		Default Value
 ----		     -----		-------		-------------
 XtNokCallback	     Callback		Function	NULL
 XtNcancelCallback   Callback		Function	NULL
 XtNcurrentDir	     CurrentDir		string		"."

*/


typedef struct _XowFileSelectorClassRec	*XowFileSelectorWidgetClass;
typedef struct _XowFileSelectorRec	*XowFileSelectorWidget;


#define XtNokCallback "okCallback"
#define XtNcancelCallback "cancelCallback"
#define XtNcurrentDir "currentDir"
#define XtCCurrentDir "CurrentDir"

/* reason codes for callback */

#define XOW_FILESELECTOR_OK 1000
#define XOW_FILESELECTOR_CANCEL 1001

typedef struct _XowFileSelectorCallbackStruct { 
      Widget	fileSD;
      int	reason;
      char	*filename;
      char	*current_dir;
      char	*basename;
} XowFileSelectorCallbackStruct;


extern WidgetClass XowfileSelectorWidgetClass;


/* provide a Motif-like convenience function */

extern Widget XowCreateFileSelector ARGS((Widget parent,
					  char *name,
					  ArgList args,
					  int nargs));


#endif /* don't add anything after this endif */




