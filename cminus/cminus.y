/****************************************************/
/* File: tiny.y                                     */
/* The TINY Yacc/Bison specification file           */
/* Compiler Construction: Principles and Practice   */
/* Kenneth C. Louden                                */
/****************************************************/
%{
#define YYPARSER /* distinguishes Yacc output from other code files */

#include "globals.h"
#include "util.h"
#include "scan.h"
#include "parse.h"

#define YYSTYPE TreeNode *
static char * savedName; /* for use in assignments */
static int savedNumber;
static int savedLineNo;  /* ditto */
static TreeNode * savedTree; /* stores syntax tree for later return */
static int yylex(void);
void mydebug(struct treeNode*);

%}

/* reserved words */
%token ELSE IF INT RETURN VOID WHILE
/* multicharacter tokens */
%token ID NUM
/* special symbols */
%token PLUS MINUS TIMES OVER LT LE GT GE EQ NE ASSIGN SEMI COMMA
%token LPAREN RPAREN LCURLY RCURLY LBRACE RBRACE
/* book-keeping tokens */
%token ERROR

%nonassoc XIF
%nonassoc ELSE

%locations
%debug

%% /* Grammar for TINY */
program     : decl_list
                 { savedTree = $1;}
            ;
decl_list   : decl_list decl
                 { YYSTYPE t = $1;
                   if (t != NULL)
                   { while (t->sibling != NULL)
                        t = t->sibling;
                     t->sibling = $2;
                     $$ = $1; }
                     else $$ = $2;
                 }
            | decl  { $$ = $1; }
            ;
decl        : var_decl  { $$ = $1; }
            | fun_decl  { $$ = $1; }
            ;
saveName    : ID
                 { savedName = copyString(tokenString);
                   savedLineNo = lineno;
                 }
            ;
saveNumber  : NUM
                 { savedNumber = atoi(tokenString);
                   savedLineNo = lineno;
                 }
            ;
var_decl    : type_spec saveName SEMI
                 { $$ = newDeclNode(VarK);
                   mydebug($1);
                   $$->attr.type = $1->attr.type;
                   $$->lineno = lineno;
                   $$->attr.name = savedName;
                 }
            | type_spec saveName LBRACE saveNumber RBRACE SEMI
                 { $$ = newDeclNode(ArrVarK);
                   mydebug($1);
                   $$->attr.type = $1->attr.type;
                   $$->lineno = lineno;
                   $$->attr.arr.name = savedName;
                   $$->attr.arr.size = savedNumber;
                 }
            ;
type_spec   : INT
                 { $$ = newTypeNode(TypeNameK, INT);
                 }
            | VOID
                 { $$ = newTypeNode(TypeNameK, VOID);
                 }
            ;
fun_decl    : type_spec saveName {
                   $$ = newDeclNode(FunK);
                   $$->lineno = lineno;
                   $$->attr.name = savedName;
                 }
              LPAREN params RPAREN comp_stmt
                 {
                   $$ = $3;
                   mydebug($1); //added
                   $$->attr.type = $1->attr.type; //added
                   $$->child[0] = $5;    /* parameters */
                   $$->child[1] = $7;    /* body */
                 }
            ;
params      : param_list  { $$ = $1; }
            | VOID
                 { $$ = newParamNode(NonArrParamK);
                   $$->attr.type = VOID;
		   $$->lineno = 0;
		   $$->attr.name = "";
                   //$$ = newTypeNode(TypeNameK,VOID);
                 }
param_list  : param_list COMMA param
                 { YYSTYPE t = $1;
                   if (t != NULL)
                   { while (t->sibling != NULL)
                        t = t->sibling;
                     t->sibling = $3;
                     $$ = $1; }
                     else $$ = $3; 
                 }
            | param { $$ = $1; };
param       : type_spec saveName
                 { $$ = newParamNode(NonArrParamK);
                   mydebug($1); //added
                   $$->attr.type = $1->attr.type; //added
                   $$->attr.name = savedName;
                 }
            | type_spec saveName
              LBRACE RBRACE
                 { $$ = newParamNode(ArrParamK);
                   mydebug($1); //added
                   $$->attr.type = $1->attr.type; //added
                   $$->attr.name = savedName;
                 }
            ;
comp_stmt   : LCURLY local_decls stmt_list RCURLY
                 { $$ = newStmtNode(CompK);
                   $$->child[0] = $2; /* local variable declarations */
                   $$->child[1] = $3; /* statements */
                 }
            ;
local_decls : local_decls var_decl
                 { YYSTYPE t = $1;
                   if (t != NULL)
                   { while (t->sibling != NULL)
                        t = t->sibling;
                     t->sibling = $2;
                     $$ = $1; }
                     else $$ = $2;
                 }
            | /* empty */ { $$ = NULL; }
            ;
stmt_list   : stmt_list stmt
                 { YYSTYPE t = $1;
                   if (t != NULL)
                   { while (t->sibling != NULL)
                        t = t->sibling;
                     t->sibling = $2;
                     $$ = $1; }
                     else $$ = $2;
                 }
            | /* empty */ { $$ = NULL; }
            ;
stmt        : exp_stmt { $$ = $1; }
            | comp_stmt { $$ = $1; }
            | sel_stmt { $$ = $1; }
            | iter_stmt { $$ = $1; }
            | ret_stmt { $$ = $1; }
            ;
exp_stmt    : exp SEMI { $$ = $1; }
            | SEMI { $$ = NULL; }
            ;
sel_stmt    : IF LPAREN exp RPAREN stmt %prec XIF
                 { $$ = newStmtNode(IfK);
                   $$->child[0] = $3;
                   $$->child[1] = $5;
                   $$->child[2] = NULL;
                 }
            | IF LPAREN exp RPAREN stmt ELSE stmt
                 { $$ = newStmtNode(IfK);
                   $$->child[0] = $3;
                   $$->child[1] = $5;
                   $$->child[2] = $7;
                 }
            ;
iter_stmt   : WHILE LPAREN exp RPAREN stmt
                 { $$ = newStmtNode(IterK);
                   $$->child[0] = $3;
                   $$->child[1] = $5;
                 }
            ;
ret_stmt    : RETURN SEMI
                 { $$ = newStmtNode(ReturnK);
                   $$->child[0] = NULL;
                 }
            | RETURN exp SEMI
                 { $$ = newStmtNode(ReturnK);
                   $$->child[0] = $2;
                 }
            ;
exp         : var ASSIGN exp
                 { $$ = newExpNode(AssignK);
                   $$->child[0] = $1;
                   $$->child[1] = $3;
                 }
            | simple_exp { $$ = $1; }
            ;
var         : saveName
                 { $$ = newExpNode(IdK);
                   $$->attr.name = savedName;
                 }
            | saveName LBRACE exp RBRACE
                 { $$ = newExpNode(ArrIdK);
                   $$->attr.name = savedName;
                   $$->child[0] = $3;
                 }
            ;
simple_exp  : add_exp LE add_exp
                 { $$ = newExpNode(OpK);
                   $$->child[0] = $1;
                   $$->child[1] = $3;
                   $$->attr.op = LE;
                 }
            | add_exp LT add_exp
                 { $$ = newExpNode(OpK);
                   $$->child[0] = $1;
                   $$->child[1] = $3;
                   $$->attr.op = LT;
                 }
            | add_exp GT add_exp
                 { $$ = newExpNode(OpK);
                   $$->child[0] = $1;
                   $$->child[1] = $3;
                   $$->attr.op = GT;
                 }
            | add_exp GE add_exp
                 { $$ = newExpNode(OpK);
                   $$->child[0] = $1;
                   $$->child[1] = $3;
                   $$->attr.op = GE;
                 }
            | add_exp EQ add_exp
                 { $$ = newExpNode(OpK);
                   $$->child[0] = $1;
                   $$->child[1] = $3;
                   $$->attr.op = EQ;
                 }
            | add_exp NE add_exp
                 { $$ = newExpNode(OpK);
                   $$->child[0] = $1;
                   $$->child[1] = $3;
                   $$->attr.op = NE;
                 }
            | add_exp { $$ = $1; }
            ;
add_exp     : add_exp PLUS term
                 { $$ = newExpNode(OpK);
                   $$->child[0] = $1;
                   $$->child[1] = $3;
                   $$->attr.op = PLUS;
                 }
            | add_exp MINUS term
                 { $$ = newExpNode(OpK);
                   $$->child[0] = $1;
                   $$->child[1] = $3;
                   $$->attr.op = MINUS;
                 }
            | term { $$ = $1; }
            ;
term        : term TIMES factor
                 { $$ = newExpNode(OpK);
                   $$->child[0] = $1;
                   $$->child[1] = $3;
                   $$->attr.op = TIMES;
                 }
            | term OVER factor
                 { $$ = newExpNode(OpK);
                   $$->child[0] = $1;
                   $$->child[1] = $3;
                   $$->attr.op = OVER;
                 }
            | factor { $$ = $1; }
            ;
factor      : LPAREN exp RPAREN { $$ = $2; }
            | var { $$ = $1; }
            | call { $$ = $1; }
            | NUM
                 { $$ = newExpNode(ConstK);
                   $$->attr.val = atoi(tokenString);
                 }
            ;
call        : saveName {
                 $$ = newExpNode(CallK);
                 $$->attr.name = savedName;
              }
              LPAREN args RPAREN
                 { $$ = $2;
                   $$->child[0] = $4;
                 }
            ;
args        : arg_list { $$ = $1; }
            | /* empty */ { $$ = NULL; }
            ;
arg_list    : arg_list COMMA exp
                 { YYSTYPE t = $1;
                   if (t != NULL)
                   { while (t->sibling != NULL)
                        t = t->sibling;
                     t->sibling = $3;
                     $$ = $1; }
                     else $$ = $3;
                 }
            | exp { $$ = $1; }
            ;

%%

int yyerror(const char * message){
  Error = TRUE;
  printf("Error: %s\n", message);
  return 0;
}

void mydebug(struct treeNode * target) {
  int flagOn = 1;
  char* dummy = target->attr.type==INT? "INT":"VOID";
  if(flagOn == 1) return;
  dummy = NULL;
  printf("[mydebug] "/* target address is [%p]\n", target*/);
  printf("[mydebug] "/* target($->attr.type) is [%s], actual value of target->attr.type = [%d]\n", target->attr.type==INT? "INT":"VOID", target->attr.type*/);
}

static int yylex(void)
{ return getToken(); }

TreeNode * parse(void)
{ yyparse();
  return savedTree;
}
