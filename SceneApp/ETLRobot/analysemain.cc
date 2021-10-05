#include "forms.h"
#include "formanalyse.h"
#include "analyse.h"

int gargc;
char **gargv;

int main(int argc, char *argv[])
{
  gargc = argc;
  gargv = argv;

   FD_formanalyse *fd_formanalyse;

   fl_initialize(&argc, argv, 0, 0, 0);
   fd_formanalyse = create_form_formanalyse();
   pass_fd_pointer_analyse(fd_formanalyse);

   /* fill-in form initialization code */

   /* show the first form */
   fl_show_form(fd_formanalyse->formanalyse,FL_PLACE_CENTERFREE,FL_FULLBORDER,"formanalyse");

   // Initialise analyse.cc
   set_up_analyse(argc, argv);

   // Enter main control loop in analyse.cc
   main_analyse_loop();   


   return 0;
}
