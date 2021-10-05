#include <stdio.h>
#include <stdlib.h>

#define MAX_NAMES       50
#define MAX_NAME_LENGTH 100

static char read_func_name ( FILE *fd, char *string )
{
   int c;

   for(;;)
   {
      c = getc ( fd );
      if ( !isalnum ( c ) && c != '_' )
      {
	 *string = '\0';
	 break;
      }

      *string++ = (char) c;
   }

   return c;
}

static void write_headers ( FILE *fd )
{
   char c1 = ' ', c2 = ' ', c3 = ' ', c4 = ' ', c5 = ' ', c6 = ' ', c7, c;
   int  in_func_def = 0, first = 1;
   char func_name[MAX_NAMES][MAX_NAME_LENGTH];
   int  no_names;

   while ( (c7 = getc ( fd )) != EOF )
   {
      if ( in_func_def )
	 if ( c1 == '*' && c2 == '*' && c3 == '*' && c4 == '*' && c5 == '*' &&
	      c6 == '*' && c7 == '/' )
	 {
	    printf ( "\n" );
	    in_func_def = 0;
	 }
	 else if ( c7 == '@' )
	 {
	    char c = read_func_name ( fd, func_name[0] );
	    int  name_no;

	    printf ( "%s%c", func_name[0], c );
	    no_names = 1;
	    for(;;)
	    {
	       c = getc ( fd );
	       if ( c == '@' )
	       {
		  c = read_func_name ( fd, func_name[no_names] );
		  printf ( "%s%c", func_name[no_names], c );
		  no_names++;
	       }
	       else if ( c == '\n' )
		  break;
	       else printf ( "%c", c );
	    }

	    printf ( "\n\\end{verbatim}\\vspace{-1.3mm}" );
	    for ( name_no = 0; name_no < no_names; name_no++ )
	       printf ( "\\index{%s}", func_name[name_no] );

	    printf ( "\n\\begin{verbatim}\n" );
	 }
         else
	    putchar ( c7 );
      else
	 if ( c1 == '/' && c2 == '*' && c3 == '*' && c4 == '*' && c5 == '*' &&
	      c6 == '*' && c7 == '*' )
	 {
	    if ( first )
	       printf ( "\n*******" );
	    else
	       do c = getc ( fd ); while ( c != '\n' );

	    in_func_def = 1;
	    first = 0;
	 }
	    
      c1 = c2; c2 = c3; c3 = c4; c4 = c5; c5 = c6; c6 = c7;
   }
}

static void remove_code ( char *directory_name, char *base_name )
{
   FILE *fdh, *fdc;
   char  path_name[300], file_name[100];

   sprintf ( path_name, "%s/%s.h", directory_name, base_name );
   fdh = fopen ( path_name, "r" );
   sprintf ( path_name, "%s/%s.c", directory_name, base_name );
   fdc = fopen ( path_name, "r" );
   if ( fdh == NULL && fdc == NULL )
   {
      fprintf ( stderr, "non-existent file %s.[ch]\n", base_name );
      exit(-1);
   }

   if ( fdh != NULL && fdc != NULL )
      sprintf ( file_name, "%s.[ch]", base_name );
   else if ( fdc != NULL ) sprintf ( file_name, "%s.c", base_name );
   else                    sprintf ( file_name, "%s.h", base_name );

   printf("##############################################################################\n");
   printf("#                                                                            #\n");
   printf("#               Definitions in %30s:               #\n", file_name );
   printf("#                                                                            #\n");
   printf("##############################################################################\n");

   if ( fdh != NULL )
   {
      write_headers ( fdh );
      fclose ( fdh );
   }

   if ( fdc != NULL )
   {
      write_headers ( fdc );
      fclose ( fdc );
   }
}

void main ( int argc, char *argv[] )
{
   int  i;

   if ( argc < 3 )
   {
      fprintf ( stderr, "extract_func_headers <directory> <C source files>\n");
      exit(0);
   }

   printf ( "{\\setlength{\\topsep}{0mm}\n\\begin{verbatim}\n" );
   remove_code ( argv[1], argv[2] );
   for ( i = 3; i < argc; i++ )
   {
      printf ( "\n" );
      remove_code ( argv[1], argv[i] );
   }

   printf ( "\\end{verbatim}}\n" );
   exit(0);
}
