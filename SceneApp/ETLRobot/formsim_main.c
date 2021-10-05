#include "forms.h"
#include "formsim.h"

int main(int argc, char *argv[])
{
   FD_formsim *fd_formsim;

   fl_initialize(&argc, argv, 0, 0, 0);
   fd_formsim = create_form_formsim();

   /* fill-in form initialization code */

   /* show the first form */
   fl_show_form(fd_formsim->formsim,FL_PLACE_CENTERFREE,FL_FULLBORDER,"formsim");
   fl_do_forms();
   return 0;
}
