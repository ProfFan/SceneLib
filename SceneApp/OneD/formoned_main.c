#include "forms.h"
#include "formoned.h"

int main(int argc, char *argv[])
{
   FD_formoned *fd_formoned;

   fl_initialize(&argc, argv, 0, 0, 0);
   fd_formoned = create_form_formoned();

   /* fill-in form initialization code */

   /* show the first form */
   fl_show_form(fd_formoned->formoned,FL_PLACE_CENTERFREE,FL_FULLBORDER,"formoned");
   fl_do_forms();
   return 0;
}
