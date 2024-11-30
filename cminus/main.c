/****************************************************/
/* File: main.c                                     */
/* Main program for TINY compiler                   */
/* Compiler Construction: Principles and Practice   */
/* Kenneth C. Louden                                */
/****************************************************/

#include "globals.h"

/* set NO_PARSE to TRUE to get a scanner-only compiler */
#define NO_PARSE FALSE
/* set NO_ANALYZE to TRUE to get a parser-only compiler */
#define NO_ANALYZE FALSE

/* set NO_CODE to TRUE to get a compiler that does not
 * generate code
 */
#define NO_CODE FALSE

#include "util.h"
#if NO_PARSE
#include "scan.h"
#else
#include "parse.h"
#if !NO_ANALYZE
#include "analyze.h"
#if !NO_CODE
#include "cgen.h"
#endif
#endif
#endif
int absoluteDebugEnabled2 = 0;
int noSilenter = 0;
/* allocate global variables */
int lineno = 0;
FILE * source;
FILE * listing;
FILE * code;

/* allocate and set tracing flags */
int EchoSource = FALSE;
int TraceScan = FALSE;
int TraceParse = TRUE;
int TraceAnalyze = TRUE;
int TraceCode = FALSE;

int Error = FALSE;

int BIOS = FALSE;
int SO = FALSE;
int posSector;
int locationMD;
FAT fat;

int main( int argc, char * argv[] )
{ TreeNode * syntaxTree;
  char pgm[120]; /* source code file name */
  if (argc != 2)
    { fprintf(stderr,"usage: %s <filename>\n",argv[0]);
      exit(1);
    }
  strcpy(pgm,argv[1]) ;
  if (strchr (pgm, '.') == NULL)
     strcat(pgm,".tny");
  source = fopen(pgm,"r");
  if (noSilenter) printf("[main.c] new checkpoint start..\n");	 
  if (noSilenter) printf("[main.c] new checkpoint start..\n");	 
  if (source==NULL)
  { fprintf(stderr,"File %s not found\n",pgm);
    exit(1);
  }
  listing = stdout; /* send listing to screen */
  fprintf(listing,"\nC-MINUS COMPILATION: %s\n",pgm);
#if NO_PARSE
  while (getToken()!=ENDFILE);
#else
  syntaxTree = parse();
  if (TraceParse) {
    fprintf(listing,"\nSyntax tree:\n");
    printTree(syntaxTree);
  }
#if !NO_ANALYZE
  if (! Error)
  { if (TraceAnalyze) fprintf(listing,"\nBuilding Symbol Table...\n");
    buildSymtab(syntaxTree);
    if (TraceAnalyze) fprintf(listing,"\nChecking Types...\n");
    typeCheck(syntaxTree);
    if (TraceAnalyze) fprintf(listing,"\nType Checking Finished!\n");
  }
#if !NO_CODE
  if (! Error)
  { char * codefile;
    if(absoluteDebugEnabled2)
        printf("[main.c] new checkpoint start..\n");	 
    if(absoluteDebugEnabled2)
        printf("[main.c] c p 1 pass\n");	 
    int fnlen = strcspn(pgm,".");
    if(absoluteDebugEnabled2)
        printf("[main.c] c p 2 pass\n");	 
    codefile = (char *) calloc(fnlen+4, sizeof(char));
    if(absoluteDebugEnabled2)
        printf("[main.c] c p 3 pass\n");	 
    strncpy(codefile,pgm,fnlen);
    if(absoluteDebugEnabled2)
        printf("[main.c] c p 4 pass\n");	 
    strcat(codefile,".cm");
    if(absoluteDebugEnabled2)
        printf("[main.c] c p 5 pass\n");	 
    code = fopen(codefile,"w");
    if(absoluteDebugEnabled2)
        printf("[main.c] c p 6 pass\n");	 
    if (code == NULL)
    { printf("Unable to open codefile\n");
      exit(1);
    }
    if(absoluteDebugEnabled2)
        printf("[main.c] CALL codeGen(syntaxTree, code) !!\n");	 
    codeGen(syntaxTree,code);
    if(absoluteDebugEnabled2)
        printf("[main.c] c p 7 pass\n");	 
    fclose(code);
    if(absoluteDebugEnabled2)
        printf("[main.c] c p 8 pass\n");	 
    if(absoluteDebugEnabled2)
        printf("[main.c] ALL PASS SUCCESS.(end of checkpoint)..\n");	 
  }
#endif
#endif
#endif
  fclose(source);
  return 0;
}

