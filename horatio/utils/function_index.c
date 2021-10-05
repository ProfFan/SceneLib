#include <stdlib.h>
#include <stdio.h>

#define STRING_LENGTH 300
#define MAX_REPETITIONS 10

static replace_underlines ( char *string )
{
   for ( ; *string != '\0'; string++ )
      if ( *string == '_' ) *string = '|';
}

static restore_underlines ( char *string )
{
   for ( ; *string != '\0'; string++ )
      if ( *string == '|' ) *string = '_';
}

static void expand_underlines ( char *name )
{
   char c, new_name[STRING_LENGTH], *ptr1 = name, *ptr2 = new_name;
   
   for(;;)
   {
      if ( (c = *ptr1++) == '\0' ) break;
      if ( c != '_' ) *ptr2++ = c;
      else
      {
	 *ptr2++ = '\\';
	 *ptr2++ = '_';
      }
   }

   *ptr2 = '\0';
   strcpy ( name, new_name );
}

static void read_next_page_entry ( FILE *fd, char *func_name, int *page_no )
{
   char *ptr = func_name, c;

   if ( fscanf ( fd, "\\indexentry{" ) < 0 )
   {
      sprintf ( func_name, "@" );
      return;
   }

   for(;;)
      if ( (c = getc ( fd )) == '}' ) { *ptr = '\0'; break; }
      else *ptr++ = c;

   fscanf ( fd, "{%d}\n", page_no );
}

static void get_next_page_entry ( FILE *fd, char *func_name,
				  int *no_pages, int *page_no )
{
   static char latest_func_name[STRING_LENGTH] = "";
   static int  page_number;

   if ( latest_func_name[0] == '\0' ) /* first call */
      read_next_page_entry ( fd, latest_func_name, &page_number );
   else if ( latest_func_name[0] == '@' )
   {
      sprintf ( func_name, "@" );
      return;
   }

   strcpy ( func_name, latest_func_name );
   page_no[0] = page_number;
   *no_pages = 1;
   for(;;)
   {
      read_next_page_entry ( fd, latest_func_name, &page_number );
      if ( latest_func_name[0] == '@' ||
	   strcasecmp ( func_name, latest_func_name ) != 0 )
	 break;

      page_no[*no_pages] = page_number;
      (*no_pages)++;
   }
}

static void get_next_source_entry (FILE *fd, char *func_name, int *line_number,
				   char *file_name)
{
   int len;

   if ( fscanf ( fd, "%s", func_name ) < 0 )
   {
      sprintf ( func_name, "@" );
      return;
   }

   len = strlen(func_name);
   if ( len < 4 || (*line_number = atoi ( &func_name[len-4] )) < 1000 )
      fscanf ( fd, "%d", line_number );
   else /*assumes that no function names end in 4-digit or longer numbers*/
      func_name[len-4] = '\0';

   fscanf ( fd, "%s", file_name );
   while ( getc ( fd ) != '\n' );
}

void main ( int argc, char *argv[] )
{
   char pfunc_name[STRING_LENGTH] = "", sfunc_name[STRING_LENGTH] = "";
   char file_name[STRING_LENGTH];
   int  page_number[MAX_REPETITIONS], no_pages, line_number, comp;
   FILE *fdp, *fds;

   if ( argc != 3 )
   {
      fprintf ( stderr, "function_index <page file> <source file>\n" );
      exit(-1);
   }

   fdp = fopen ( argv[1], "r" );
   if ( fdp == NULL )
   {
      fprintf ( stderr, "non-existent file %s\n", argv[1] );
      exit(-1);
   }

   fds = fopen ( argv[2], "r" );
   if ( fds == NULL )
   {
      fprintf ( stderr, "non-existent file %s\n", argv[2] );
      exit(-1);
   }

   printf ( "\\begin{theindex}\n" );
   printf ( "Listed below are all type definitions, macros and function\n" );
   printf ( "names together with\n" );
   printf ( "the page number of their description in this document and\n" );
   printf ( "the file name/line number where they appear in the\n" );
   printf ( "source code. The page number entries do not include static functions,\n" );
   printf ( "and because the {\\tt ctags} utility ignores repetitions of\n" );
   printf ( "the same function name, the references to static functions\n" );
   printf ( "in the source code entries are not quite complete.\n" );
   printf ( "The alphabetical ordering is case-{\\em in}sensitive,\n" );
   printf ( "so certain entries appear with the wrong case when two\n" );
   printf ( "cases of the same word appear.\n" );
   printf ( "\\vspace{0.2in}\n" );
   printf ( "{\\small\n" );

   get_next_page_entry   ( fdp, pfunc_name, &no_pages, page_number );
   get_next_source_entry ( fds, sfunc_name, &line_number, file_name );
   for(;;)
   {
      int i;

      replace_underlines ( pfunc_name );
      replace_underlines ( sfunc_name );
      comp = strcasecmp ( pfunc_name, sfunc_name );
      restore_underlines ( pfunc_name );
      restore_underlines ( sfunc_name );
      if ( comp == 0 )
	 if ( pfunc_name[0] == '@' ) break;
         else
	 {
	    expand_underlines ( pfunc_name );
	    expand_underlines ( file_name );
	    printf ( "\\item {\\tt %s} %d", pfunc_name, page_number[0] );
	    for ( i = 1; i < no_pages; i++ )
	       printf ( ", %d", page_number[i] );

	    printf ( "; {\\tt %s}~line~%d.\n", file_name+3, line_number );
	    get_next_page_entry   ( fdp, pfunc_name, &no_pages, page_number );
	    get_next_source_entry ( fds, sfunc_name, &line_number, file_name );
	 }
      else if ( sfunc_name[0] == '@' || (pfunc_name[0] != '@' && comp < 0) )
      {
	 expand_underlines ( pfunc_name );
	 printf ( "\\item {\\tt %s} %d", pfunc_name, page_number[0] );
	 for ( i = 1; i < no_pages; i++ )
	    printf ( ", %d", page_number[i] );

	 printf ( ".\n" );
	 get_next_page_entry ( fdp, pfunc_name, &no_pages, page_number );
      }
      else 
      {
	 expand_underlines ( sfunc_name );
	 expand_underlines ( file_name );
	 printf ( "\\item {\\tt %s} {\\tt %s}~line~%d.\n",
		  sfunc_name, file_name+3, line_number );
	 get_next_source_entry ( fds, sfunc_name, &line_number, file_name );
      }
   }

   printf ( "}\n\\end{theindex}\n" );
}
