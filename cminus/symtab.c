/****************************************************/
/* File: symtab.c                                   */
/* Symbol table implementation for the TINY compiler*/
/* (allows only one symbol table)                   */
/* Symbol table is implemented as a chained         */
/* hash table                                       */
/* Compiler Construction: Principles and Practice   */
/* Kenneth C. Louden                                */
/****************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "globals.h"
#include "symtab.h"

/* SHIFT is the power of two used as multiplier
   in hash function  */
#define SHIFT 4

#define MAX_SCOPE 1000
Scope globalScope;

/* the hash function */
static int hash(const char *key)
{
  int temp = 0;
  int i = 0;
  while (key[i] != '\0')
  {
    temp = ((temp << SHIFT) + key[i]) % SIZE;
    ++i;
  }
  return temp;
}

static Scope scopes[MAX_SCOPE];
static int nScope = 0;
static Scope scopeStack[MAX_SCOPE];
static int nScopeStack = 0;
static int location[MAX_SCOPE];
static int noPRINT = 1;

Scope sc_top(void)
{
  if (noPRINT == 0 && nScopeStack != 0)
    printf("sc_top -> decimal(n-1) == [%d]\t|\t[%s]\n", nScopeStack - 1, (scopeStack[nScopeStack - 1]->funcName == NULL) ? "NULL" : scopeStack[nScopeStack - 1]->funcName);
  return scopeStack[nScopeStack - 1];
}

Scope sc_top_m1(void)
{
  return scopeStack[nScopeStack - 2];
}

void sc_pop(void)
{
  if (noPRINT == 0)
    printf("pop %s\n", sc_top()->funcName);
  --nScopeStack;
}

int addLocation(void) // addLocation: fix - readonly version
{
  return location[nScopeStack - 1]++;
}

void sc_push(Scope scope) // sc_push: fix - readonly version
{
  scopeStack[nScopeStack] = scope;
  location[nScopeStack++] = 0;
  if (noPRINT == 0)
    printf("push %s\n", scope->funcName == NULL ? "NULL,possibleGLOBALscope, " : scope->funcName);
}

void sc_push_inout_special(Scope scope) // sc_push_inout_special: changable - under test
{
  scopeStack[nScopeStack] = scope;
  location[nScopeStack] = 0;
  if (noPRINT == 0)
    printf("push %s\n", scope->funcName == NULL ? "NULL,possibleGLOBALscope, " : scope->funcName);
}

Scope sc_create(char *funcName)
{
  Scope newScope;
  if (funcName == NULL)
  {
    newScope = (Scope)malloc(sizeof(struct ScopeRec));
    newScope->funcName = NULL;
    newScope->nestedLevel = nScopeStack;
    noPRINT = 1; // global sc decimal val debug print purpose
    newScope->parent = sc_top();
    scopes[nScope++] = newScope;
    return newScope;
  }
  else
  {
    // printf("\nsc_create() called!111  this should be shown, extra info:  char *funcName = %s\n", funcName);
    // fprintf(listing, "[SC_CREATE] char * fname = %s, \n", funcName==NULL ? "null" : funcName);

    newScope = (Scope)malloc(sizeof(struct ScopeRec));
    newScope->funcName = funcName;

    // printf("\nsc_create() called!222  this should be shown, extra info:  char *funcName = %s\n", funcName);
    if (strcmp(funcName, "input") == 0 || strcmp(funcName, "output") == 0)
    {

      newScope->nestedLevel = 1;
      noPRINT = 1; // decimal val debug print purpose
      newScope->parent = sc_top();
      scopes[nScope++] = newScope;
    }
    else
    {

      newScope->nestedLevel = nScopeStack;
      noPRINT = 1; // decimal val debug print purpose
      newScope->parent = sc_top();
      scopes[nScope++] = newScope;
    }

    return newScope;
  }
}

BucketList st_bucket(const char *name)
{
  int h = hash(name);
  Scope sc = sc_top();
  while (sc)
  {
    BucketList l = sc->hashTable[h];
    while ((l != NULL) && (strcmp(name, l->name) != 0))
      l = l->next;
    if (l != NULL)
      return l;
    sc = sc->parent;
  }
  return NULL;
}

/* Procedure st_insert inserts line numbers and
 * memory locations into the symbol table
 * loc = memory location is inserted only the
 * first time, otherwise ignored
 */
void st_insert(char *name, int lineno, int loc, TreeNode *treeNode)
{
  int h = hash(name);
  int notdebug = 1;
  if (notdebug == 0)
    fprintf(listing, "[st_insert] char * name = %s, \tAddress of treeNode = [%p]\n", name == NULL ? "null" : name, treeNode);
  Scope top = sc_top();
  BucketList l = top->hashTable[h];
  while ((l != NULL) && (strcmp(name, l->name) != 0))
    l = l->next;
  if (l == NULL) /* variable not yet in table */
  {
    l = (BucketList)malloc(sizeof(struct BucketListRec));
    l->name = name;
    l->treeNode = treeNode;
    l->lines = (LineList)malloc(sizeof(struct LineListRec));
    l->lines->lineno = lineno;
    l->memloc = loc;
    l->lines->next = NULL;
    l->next = top->hashTable[h];
    top->hashTable[h] = l;
  }
  else /* found in table, so just add line number */
  {    // ERROR!
  }
} /* st_insert */

/* Function st_lookup returns the memory
 * location of a variable or -1 if not found
 */
int st_lookup(char *name)
{
  BucketList l = st_bucket(name);
  if (l != NULL)
    return l->memloc;
  return -1;
}

int st_lookup_top(char *name)
{
  int h = hash(name);
  Scope sc = sc_top();
  while (sc)
  {
    BucketList l = sc->hashTable[h];
    while ((l != NULL) && (strcmp(name, l->name) != 0))
      l = l->next;
    if (l != NULL)
      return l->memloc;
    break;
  }
  return -1;
}

int st_add_lineno(char *name, int lineno)
{
  BucketList l = st_bucket(name);
  LineList ll = l->lines;
  while (ll->next != NULL)
    ll = ll->next;
  ll->next = (LineList)malloc(sizeof(struct LineListRec));
  ll->next->lineno = lineno;
  ll->next->next = NULL;
}
/* Variable indentno is used by printTree to
 * store current number of spaces to indent
 */
static int indentno_ = 0;

/* macros to increase/decrease indentation */
#define MAXCHILDREN_ 3

void subcall(TreeNode *tree, FILE *listing, int bools)
{

  if (bools)
  {
    fprintf(listing, "%s\t void\t", "   ");
    return;
  }

  int i;

  while (tree != NULL)
  {

    switch (tree->nodekind)
    {
    case ParamK:
      switch (tree->kind.param)
      {
      case ArrParamK:
        fprintf(listing, "\n");
        fprintf(listing, "\t\t%s\t %s[]\t", tree->attr.name, (tree->attr.type == 260 || tree->attr.type == INT) ? "int" : "void");
        break;
      case NonArrParamK:
        fprintf(listing, "\n");
        fprintf(listing, "\t\t%s\t %s\t", tree->attr.name, (tree->attr.type == 260 || tree->attr.type == INT) ? "int" : "void");
        break;
      default:
        break;
      }
      break;
    default:
      break;
    }

    tree = tree->sibling;
  }
}
void printSymTabRows(BucketList *hashTable, FILE *listing)
{
  int j;

  for (j = 0; j < SIZE; ++j)
  {
    if (hashTable[j] != NULL)
    {
      BucketList l = hashTable[j];
      TreeNode *node = l->treeNode;

      while (l != NULL)
      {
        LineList t = l->lines;

        fprintf(listing, "%-14s ", l->name);

        switch (node->nodekind)
        {
        case DeclK:
          switch (node->kind.decl)
          {
          case FunK:
            fprintf(listing, "Function  ");
            break;
          case VarK:
            fprintf(listing, "Variable  ");
            break;
          case ArrVarK:
            fprintf(listing, "Array Var \t\t");
            break;
          default:
            break;
          }
          break;
        case ParamK:
          switch (node->kind.param)
          {
          case NonArrParamK:
            fprintf(listing, "Variable  ");
            break;
          case ArrParamK:
            fprintf(listing, "Array Var \t\t");
            break;
          default:
            break;
          }
          break;
        default:
          break;
        }

        switch (node->type)
        {
        case Void:
          fprintf(listing, "Void         ");
          break;
        case Integer:
          fprintf(listing, "Integer      ");
          break;
        case Boolean:
          fprintf(listing, "Boolean      ");
          break;
        default:
          break;
        }

        if (t != NULL && l != NULL)
        {
          fprintf(listing, "%5d\t", l->memloc);
        }

        while (t != NULL)
        {
          fprintf(listing, "%4d ", t->lineno);
          t = t->next;
        }

        fprintf(listing, "\n");
        l = l->next;
      }
    }
  }
}

// under test version. i'm cleaning..
void printHierarchyFunDetail(BucketList *hashTable, FILE *listing)
{

  int cMarkerValueForFunction = 1;

  // Print the table header
  fprintf(listing, "\nFName RType PName PType\n");
  fprintf(listing, "----- ----- ----- -----\n");

  // Iterate over all the hash table entries
  for (int j = 0; j < SIZE; ++j)
  {

    if (hashTable[j] != NULL)
    {

      BucketList l = hashTable[j];
      TreeNode *node = l->treeNode;

      while (l != NULL)
      {
        LineList t = l->lines; // List of line numbers associated with this symbol

        switch (node->nodekind)
        {
        case DeclK:
          switch (node->kind.decl)
          {
          case FunK:
            cMarkerValueForFunction = 1;
            break;
          default:
            cMarkerValueForFunction = 0;
            break;
          }
          break;
        default:
          cMarkerValueForFunction = 0;
          break;
        }

        // Print rows by calling sub function
        switch (cMarkerValueForFunction)
        {
        case 1:
          fprintf(listing, "\n");
          fprintf(listing, "%s\t %s\t", node->attr.name, (node->attr.type == 260 || node->attr.type == INT) ? "int" : "void");
          if (node->child[0] != NULL)
            subcall(node->child[0], listing, node->child[0]->attr.type == VOID);
          fprintf(listing, "\n");
          break;
        default:
          break;
        }

        l = l->next;
      }
    }
  } // end of for loop
}

// template form
/*void printSomeDetailExample(BucketList *hashTable, File *listing) {
  fprintf(listing, "\n\n");

  // start from here..

  fprintf(listing, "\n\n");
}*/

// Where is output built in funciton's scope name?
void printScopeEachNamesByTabSpace(FILE *listing)
{

  Scope scope;
  fprintf(listing, "\n\t[All Each Scopes Name:\t | ");
  for (int i = 0; i < nScope; ++i)
  {
    scope = scopes[i];
    fprintf(listing, " %s | ", scope->funcName);
  }
  fprintf(listing, "| ]\n");
}
void printSymTab(FILE *listing)
{
  int verbose = FALSE;   // if want to see the whole print results, set verbose = 1  (0 is quiet mode.)
  // TODO:   production version code  (submission phase 에서 verbose 코드 삭제해야.)
  
  
  
  
  
  
  
  
  
  
                                                                                                                                        if(verbose == 0) return;
  int i;
  printScopeEachNamesByTabSpace(listing); // for debug -> objective is to figure out output() built-in function's scope current status

  for (i = 0; i < nScope; ++i)
  {
    Scope scope = scopes[i];
    BucketList *hashTable = scope->hashTable;

    if (i == 0)
    { // global scope
      fprintf(listing, "<global scope> ");
    }
    else
    {
      fprintf(listing, "function name: %s ", scope->funcName);
    }

    fprintf(
        listing,
        "(nested level: %d)\n",
        scope->nestedLevel);

    fprintf(listing, "Symbol Name    Sym.Type  Data Type  MemLocation  Line Numbers\n");
    fprintf(listing, "-------------  --------  ---------- ------------ ------------\n");

    printSymTabRows(hashTable, listing);

    printHierarchyFunDetail(hashTable, listing); // added

    fputc('\n', listing);
  }
} /* printSymTab */

// dirty version, unorganized
void printHierarchyFunDetail_(BucketList *hashTable, FILE *listing)
{
  // fprintf(listing, "\n\n");

  // // start from here..
  // fprintf(listing,"Function Name  Return Type  Parameter Name  Parameter Type\n");
  // fprintf(listing,"-------------  -----------  --------------  --------------\n");

  // int j;

  // for (j = 0; j < SIZE; ++j)
  // { if (hashTable[j] != NULL)
  //   { BucketList l = hashTable[j];
  //     TreeNode *node = l->treeNode;

  //     while (l != NULL)
  //     { LineList t = l->lines;

  //       //fprintf(listing,"%-14s ",l->name);

  //       switch (node->nodekind) {
  //       case DeclK:
  //         switch (node->kind.decl) {
  //         case FunK:
  //           fprintf(listing, "Function  ");
  //           break;
  // switch(cMarkerValueForFunction) {
  //   case 1:
  //     // do print some  print row // while
  //     break;
  //   default:
  //     return;
  // }

  // fprintf(listing, "\n\n");
}
