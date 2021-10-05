/* 
  FileSelectorDialogP.h

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

#ifndef _XOW_FILE_SELECTOR_P_H
#define _XOW_FILE_SELECTOR_P_H

#include <X11/Xow/FileSelector.h>

#ifdef USE_XAW3D
#include <X11/Xaw3d/FormP.h>
#else
#include <X11/Xaw/FormP.h>
#endif

typedef struct {
      int empty;
} XowFileSelectorClassPart;


typedef struct _XowFileSelectorClassRec {
      CoreClassPart		core_class;
      CompositeClassPart	composite_class;
      ConstraintClassPart	constraint_class;
      FormClassPart		form_class;
      XowFileSelectorClassPart	filesd_class;
} XowFileSelectorClassRec;

extern XowFileSelectorClassRec XowfileSelectorClassRec;

typedef struct _XowFileSelectorPart {

      char		*current_dir;

      XtCallbackList	ok_callback;
      XtCallbackList	cancel_callback;

      Widget		label;		/* current dir label widget */
      Widget		list;		/* list widget */
      Widget		text;		/* editable text widget */
      Widget		scroller;
      Widget		button_ok;
      Widget		button_cancel;
      Widget		box;
      
} XowFileSelectorPart;

typedef struct _XowFileSelectorRec {
    CorePart		core;
    CompositePart	composite;
    ConstraintPart	constraint;
    FormPart		form;
    XowFileSelectorPart	fileSD;
} XowFileSelectorRec;

typedef struct {
      int empty;
} XowFileSelectorConstraintsPart;

typedef struct _XowFileSelectorConstraintsRec {
    FormConstraintsPart	  form;
    XowFileSelectorConstraintsPart fileSD;
} XowFileSelectorConstraintsRec, *XowFileSelectorConstraints;

#endif /* don't add anything after this endif */
