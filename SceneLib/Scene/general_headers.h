/*************Common file to include all the stuff I usually use**************/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <sys/time.h>

#ifdef __cplusplus
#include <iostream>
#include <fstream>
#include <assert.h>

// STL includes
#include <string>
#include <list>
#include <vector>
#endif

#include <X11/Intrinsic.h>
#include <X11/cursorfont.h>
#include <X11/StringDefs.h>
#include <X11/Shell.h>
#include <X11/Xaw/Cardinals.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/Label.h>
#include <X11/Xaw/AsciiText.h>
#include <X11/Xaw/Toggle.h>
#include <X11/Xaw/Command.h>
// #include <Xow/Canvas.h>

/* Important: swapped round list.h and math.h because some of the kalman
   declarations in math.h depend on list declarations. */

#ifdef __cplusplus
extern "C" {
#include "horatio/global.h"
#include "horatio/list.h"      
#include "horatio/math.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"
#include "horatio/process.h"
#include "horatio/tool.h"
}

using namespace std;
#include "bits.h"

#else
#include "horatio/global.h"
#include "horatio/list.h"      
#include "horatio/math.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"
#include "horatio/process.h"
#include "horatio/tool.h"
#endif


