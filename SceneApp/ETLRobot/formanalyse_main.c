#include "forms.h"
#include "formanalyse.h"

int main(int argc, char *argv[])
{
   FD_formanalyse *fd_formanalyse;

   fl_initialize(&argc, argv, 0, 0, 0);
   fd_formanalyse = create_form_formanalyse();

   /* fill-in form initialization code */

   /* show the first form */
   fl_show_form(fd_formanalyse->formanalyse,FL_PLACE_CENTERFREE,FL_FULLBORDER,"formanalyse");
   fl_do_forms();
   return 0;
}
