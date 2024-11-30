/****************************************************/
/* File: globals.h                                  */
/* Global types and vars                            */
/* must come before other include files             */
/* Compiler Construction: Principles and Practice   */
/* Kenneth C. Louden                                */
/****************************************************/

#ifndef _GLOBALS_H_
#define _GLOBALS_H_

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <stdbool.h>

#ifndef YYPARSER

/* the name of the following file may change */
#include "y.tab.h"

/* ENDFILE is implicitly defined by Yacc/Bison,
 * and not included in the tab.h file
 */
#define ENDFILE 0

#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

/* MAXRESERVED = the number of reserved words */
#define MAXRESERVED 6

typedef int TokenType;


extern FILE* source; /* source code text file */
extern FILE* listing; /* listing output text file */
extern FILE* code; /* code text file for TM simulator */

extern int lineno; /* source line number for listing */

/**************************************************/
/***********   Syntax tree for parsing ************/
/**************************************************/
typedef struct arrayAttr {
  TokenType type;
  char * name;
  int size;
} ArrayAttr;
typedef enum {StmtK, ExpK, DeclK, ParamK, TypeK} NodeKind;
typedef enum {IfK, IterK, ReturnK, CompK} StmtKind;
typedef enum {AssignK, OpK, ConstK, IdK, ArrIdK, CallK} ExpKind;
typedef enum {FunK, VarK, ArrVarK} DeclKind;
typedef enum {ArrParamK, NonArrParamK} ParamKind; 
typedef enum {TypeNameK} TypeKind;
typedef enum {Void, Integer, Boolean, IntegerArray} ExpType;

#define MAXCHILDREN 3

struct ScopeRec;

typedef struct treeNode
   { struct treeNode * child[MAXCHILDREN];
     struct treeNode * sibling;
     int lineno;
     NodeKind nodekind;
     union { 
      StmtKind stmt;
      ExpKind exp; 
      DeclKind decl; 
      ParamKind param;
      TypeKind type; } kind;
     struct attr { 
      TokenType op; 
      TokenType type;
      int val; 
      char * name;
      ArrayAttr arr;
      struct ScopeRec * scope; } attr;
      ExpType type;
   } TreeNode;


typedef struct program
  { int usage;
    int name;
    int inicio;
    int tamMI;
    int tamMD;
    struct program * next;
  } *Program;

typedef struct FAT
  { int qtd_prog;
    int tam_prog;
    int prox_pos;
    struct program * p;
  } FAT;


/**************************************************/
/***********   Flags for tracing       ************/
/**************************************************/

/* EchoSource = TRUE causes the source program to
 * be echoed to the listing file with line numbers
 * during parsing
 */
extern int EchoSource;

/* TraceScan = TRUE causes token information to be
 * printed to the listing file as each token is
 * recognized by the scanner
 */
extern int TraceScan;

/* TraceParse = TRUE causes the syntax tree to be
 * printed to the listing file in linearized form
 * (using indents for children)
 */
extern int TraceParse;

/* TraceAnalyze = TRUE causes symbol table inserts
 * and lookups to be reported to the listing file
 */
extern int TraceAnalyze;

/* TraceCode = TRUE causes comments to be written
 * to the TM code file as code is generated
 */
extern int TraceCode;

/* Error = TRUE prevents further passes if an error occurs */
extern int Error; 

extern int BIOS;
extern int SO;
extern int posSector;
extern int locationMD;
extern FAT fat;




#endif
