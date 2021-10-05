#include "forms.h"
#include "formbrowser.h"

int main(int argc, char *argv[])
{
   FD_formbrowser *fd_formbrowser;

   fl_initialize(&argc, argv, 0, 0, 0);
   fd_formbrowser = create_form_formbrowser();

   /* fill-in form initialization code */

   /* show the first form */
   fl_show_form(fd_formbrowser->formbrowser,FL_PLACE_CENTERFREE,FL_FULLBORDER,"formbrowser");
   fl_do_forms();
   return 0;
}
