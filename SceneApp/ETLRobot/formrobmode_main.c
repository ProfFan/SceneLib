#include "forms.h"
#include "formrobmode.h"

/* callbacks and freeobj handles for form formrobmode */


int main(int argc, char *argv[])
{
   FD_formrobmode *fd_formrobmode;

   fl_initialize(&argc, argv, 0, 0, 0);
   fd_formrobmode = create_form_formrobmode();

   /* fill-in form initialization code */

   /* show the first form */
   fl_show_form(fd_formrobmode->formrobmode,FL_PLACE_CENTERFREE,FL_FULLBORDER,"formrobmode");
   fl_do_forms();
   return 0;
}
