#include "forms.h"
#include "formcom.h"

int main(int argc, char *argv[])
{
   FD_formcom *fd_formcom;

   fl_initialize(&argc, argv, 0, 0, 0);
   fd_formcom = create_form_formcom();

   /* fill-in form initialization code */

   /* show the first form */
   fl_show_form(fd_formcom->formcom,FL_PLACE_CENTERFREE,FL_FULLBORDER,"formcom");
   fl_do_forms();
   return 0;
}
