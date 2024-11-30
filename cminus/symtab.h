/****************************************************/
/* File: symtab.h                                   */
/* Symbol table interface for the TINY compiler     */
/* (allows only one symbol table)                   */
/* Compiler Construction: Principles and Practice   */
/* Kenneth C. Louden                                */
/****************************************************/

#ifndef _SYMTAB_H_
#define _SYMTAB_H_

#include "globals.h"

/* SIZE is the size of the hash table */
#define SIZE 211

/* the list of line numbers of the source
 * code in which a variable is referenced
 */
typedef struct LineListRec
   { int lineno;
     struct LineListRec * next;
   } * LineList;

/* The record in the bucket lists for
 * each variable, including name,
 * assigned memory location, and
 * the list of line numbers in which
 * it appears in the source code
 */
typedef struct BucketListRec
   { char * name;
     LineList lines;
     TreeNode *treeNode;
     int memloc ; /* memory location for variable */
     struct BucketListRec * next;
   } * BucketList;


typedef struct ScopeRec
   { char * funcName;
     int nestedLevel;
     struct ScopeRec * parent;
     BucketList hashTable[SIZE]; /* the hash table */
   } * Scope;


extern Scope globalScope;


void st_insert( char * , int , int , TreeNode *  );


int st_lookup ( char *  );
int st_add_lineno(char * , int );
BucketList st_bucket( const char *  );
int st_lookup_top (char * );

Scope sc_create(char *);
Scope sc_top( void );
Scope sc_top_m1( void );
void sc_pop( void );
void sc_push( Scope  );
void sc_push_inout_special( Scope  );
int addLocation( void );



void subcall(TreeNode *, FILE *, int);
void printSymTabRows(BucketList *, FILE *);
void printHierarchyFunDetail(BucketList *, FILE *);

void printScopeEachNamesByTabSpace(FILE *);
void printSymTab(FILE * );

#endif
