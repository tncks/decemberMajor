%{
#define YYPARSER

#include "globals.h"
#include "util.h"
#include "scan.h"
#include "parse.h"


#define YYSTYPE TreeNode *
static TreeNode * savedTree;
static char * savedName;
static int savedNumber;
static int savedType;
static TreeNode * tempode;
static int yylex(void);
int yyerror(const char * message);
%}

%token IF ELSE RETURN WHILE
%token INT VOID NUM ID 
%token PLUS MINUS OVER TIMES LT LE GT GE EQ NE ASSIGN SEMI COMMA LPAREN RPAREN LBRACE RBRACE LCURLY RCURLY
%token ERROR
//%expect 1

%nonassoc XIF
%nonassoc ELSE

%locations
%debug


%% /* Grammar */

program     : declaration_list             ////////////////
                 { savedTree = $1; savedType = Integer; } ////////////////////////////////////////////////////////////////////////// 
            ;
declaration_list    : declaration_list declaration
                 { YYSTYPE t = $1;
                   
                   if (t != NULL)
                   { while (t->sibling != NULL)
                        t = t->sibling;
                     t->sibling = $2;
                     $$ = $1; }
                     else $$ = $2;
                 }
            | declaration  {
               $$ = $1;
              }
            ;
declaration        : var_declaration { 
                      $$ = $1;
                       }
                    | fun_declaration { 
                      $$ = $1;
                    }
            ;
var_declaration : type_specifier identifier SEMI {

			$$ = newDeclNode(VarK);
			$$->attr.name = savedName;
			$$->attr.type = 0;
			//$$->attr.type = savedType;
                }
	      | type_specifier identifier LBRACE number RBRACE SEMI  {
                $$ = newDeclNode(ArrVarK);  
                $$->attr.type = 0;  
                $$->attr.name = savedName;
                $$->attr.arr.name = savedName;
		$$->child[0] = (TreeNode *) newExpNode(ConstK);
                $$->child[0]->attr.val = savedNumber;
		}
	    	;
type_specifier   : INT { 
                        $$ = newTypeNode(TypeNameK);
                        $$->attr.type = Integer;   //MARK ## MARK
                        //savedType = $$->attr.type;
                       }
            | VOID { 
                    $$ = newTypeNode(TypeNameK);
                    $$->attr.type = Void;         //MARK ## MARK
                    //savedType = $$->attr.type;
                   }
            ;
fun_declaration    : type_specifier identifier {
                   $$ = newDeclNode(FunK);
                   $$->attr.name = savedName;
                 }
              LPAREN params RPAREN compound_stmt
                 {
                   $$ = $3;                                        //MARK ## MARK
                   $$->child[0] = $5;  // under test <- marked.    //MARK ## MARK
                   $$->child[1] = $7;  // under test <- marked.    //MARK ## MARK
                 }
            ;
params      : param_list { 
                $$ = $1;
                 }       /*7*/
            | VOID { 
                $$ = newTypeNode(TypeNameK);  //$$ = newDeclNode(ParamK); changed
                $$->attr.type = Void;  
                $$->attr.name = "";   // "" vs NULL  (under test, for symtab printing showing)
               }
            | { 
                $$ = newTypeNode(TypeNameK);  //$$ = newDeclNode(ParamK); changed
                $$->attr.type = Void;  
                $$->attr.name = "";   // "" vs NULL  (under test, for symtab printing showing)
               }
            ;
param_list  : param_list COMMA param     /*8*/
				      { YYSTYPE t = $1;
                if (t != NULL){
                  while (t->sibling != NULL)
                    t = t->sibling;
                  t->sibling = $3;
                  $$ = $1;
                }
                else $$ = $3;
				      }
	    	    | param { $$ = $1; }
	    	    ;
param	      : type_specifier identifier      /*9*/
				      { $$ = newParamNode(NonArrParamK); // changed to newParamNode(NonArrParamK) 
                $$->attr.name = savedName;

                //$$->type = $1->kind.type;      // under test   // under test <- marked.. (currently unchanged)
                //$$->attr.type = $1->attr.type; // also under test (above one line and this line comparison needed)
                //$$->attr.name = $2->attr.name;  // keep it as it is 
                //$$->lineno = $2->lineno;        // keep it as it is
                //$$->child[0] = $1;              // newly added, but for test, still commented
				      }
	    	    | type_specifier identifier LBRACE RBRACE
				      { $$ = newParamNode(ArrParamK); // changed to newParamNode(ArrParamK)
                $$->attr.type = Integer;
                $$->attr.name = savedName;

				        //$$->type = $1->kind.type;   // under test   // under test <- marked.. (currently unchanged)
                //$$->attr.type = $1->attr.type; // also under test (above one line and this line comparison needed)
                //$$->attr.name = $2->attr.name;  // keep it as it is 
                //$$->lineno = $2->lineno;        // keep it as it is
                //$$->child[0] = $1;   // newly added, but for test, still commented
				      }
	    	    ;
compound_stmt   : LCURLY local_declarations statement_list RCURLY/*10*/
              { 
                $$ = newStmtNode(CompK);

                if ($2 != NULL){  
                  $$->child[0] = $2;
                  $$->child[1] = $3;
                }
                else {
                  $$->child[0] = NULL;
                  $$->child[1] = $3;
                  }
                }
	    	    | LCURLY statement_list RCURLY 
              {
                $$ = newStmtNode(CompK);
                  
                if ($2 != NULL){
                  $$->child[0] = NULL;
                  $$->child[1] = $2;
                } else {
                  $$->child[0] = NULL;
                  $$->child[1] = NULL;
                }
            
              }
	    	    | LCURLY local_declarations RCURLY
              {

                $$ = newStmtNode(CompK);
                  
                if ($2 != NULL) {
                  $$->child[0] = $2;
                  $$->child[1] = NULL;
                } else {
                  $$->child[0] = NULL;
                  $$->child[1] = NULL;
                }
              }
	    	    | LCURLY RCURLY
              { 
                $$ = newStmtNode(CompK);
                $$->child[0] = NULL;
                $$->child[1] = NULL;
              }
              
	    	    ;
local_declarations  : local_declarations var_declaration  /*11*/
              { YYSTYPE t = $1;
              
                if(t != NULL) {
                  while(t->sibling != NULL)
                    t = t->sibling;
                  t->sibling = $2;
                  $$ = $1;
                }
                else $$ = $2; }
            | var_declaration { 
              $$ = $1;
              
             }
            ;
statement_list   : statement_list statement /*12*/
              { YYSTYPE t = $1;
                if(t != NULL){
                  while(t->sibling != NULL)
                    t = t->sibling;
                  t->sibling = $2;
                  $$ = $1;
                }
                else $$ = $2; }
            | statement { $$ = $1; }
            ;
statement   : expression_stmt { $$ = $1; }/*13*/
            | compound_stmt { $$ = $1; }
            | selection_stmt { $$ = $1; }
            | iteration_stmt { $$ = $1; }
            | return_stmt { $$ = $1; }
            ;
expression_stmt   : expression SEMI { $$ = $1; }/*14*/
            | SEMI { $$ = NULL; }
            ;
selection_stmt    : IF LPAREN expression RPAREN statement %prec XIF/*15; selection_stmt*/
              { $$ = newStmtNode(IfK);
                $$->child[0] = $3;
                $$->child[1] = $5; 
                $$->child[2] = NULL; } 
            | IF LPAREN expression RPAREN statement ELSE statement
              { $$ = newStmtNode(IfK);
                $$->child[0] = $3;
                $$->child[1] = $5;
                $$->child[2] = $7; }
            ;
iteration_stmt     : WHILE LPAREN expression RPAREN statement/*16; iteration_stmt*/
              { $$ = newStmtNode(IterK);
                $$->child[0] = $3;
                $$->child[1] = $5; }
            ;
return_stmt    : RETURN SEMI { $$ = newStmtNode(ReturnK);  }/*17*/
            | RETURN expression SEMI
              { $$ = newStmtNode(ReturnK); 
                $$->child[0] = $2; }
            ;
expression        : var ASSIGN expression  /*18; expression*/
              { $$ = newExpNode(AssignK);
                $$->child[0] = $1;
                $$->child[1] = $3; }
            | simple_expression { $$ = $1; }
            ;
var         : identifier 
              { 
                $$ = newExpNode(IdK);
                $$->attr.name = savedName; 
                //$$->attr.name = $1->attr.name;
              } /*19*/
            | identifier
                  { $$ = newExpNode(ArrIdK);
                    $$->attr.name = savedName;  //$$->attr.name = $1->attr.name;
                    $$->attr.type = Integer;
                    tempode = (TreeNode *)$$;
                  } 
              LBRACE expression RBRACE
                  { if(tempode != NULL) tempode->child[0] = $3;
                    //$$->child[0] = $3;                
                  }
            ;
simple_expression   : additive_expression relop additive_expression/*20; simple_expression*/
              { $$ = $2;
                $$->child[0] = $1;
                $$->child[1] = $3; }
            | additive_expression { $$ = $1; }
            ;
relop       : LT { $$ = newExpNode(OpK); $$->attr.op = LT; }/*21; relop*/
            | LE { $$ = newExpNode(OpK); $$->attr.op = LE; }
            | GT { $$ = newExpNode(OpK); $$->attr.op = GT; }
            | GE { $$ = newExpNode(OpK); $$->attr.op = GE; }
            | EQ { $$ = newExpNode(OpK); $$->attr.op = EQ; }
            | NE { $$ = newExpNode(OpK); $$->attr.op = NE; }
            ;
additive_expression     : additive_expression addop term/*22; additive_expression*/
              { $$ = $2;
                $$->child[0] = $1;
                $$->child[1] = $3; }
            | term { $$ = $1; }
            ;
addop         : PLUS { $$ = newExpNode(OpK); $$->attr.op = PLUS; }/*23; addop*/

            | MINUS { $$ = newExpNode(OpK); $$->attr.op = MINUS; }
            ;
term        : term mulop factor/*24*/
              { $$ = $2;
                $$->child[0] = $1;
                $$->child[1] = $3; }
            | factor { $$ = $1; }
            ;
mulop        : TIMES { $$ = newExpNode(OpK); $$->attr.op = TIMES; }/*25; mulop*/
            | OVER { $$ = newExpNode(OpK); $$->attr.op = OVER; }
            ;
factor      : LPAREN expression RPAREN { $$ = $2; }
            | var { $$ = $1; }
            | call { $$ = $1; }
            | NUM/*number*/ { /*$$ = $1;*/ $$ = newExpNode(ConstK); $$->attr.val = atoi(tokenString); }  // changed - number -> to NUM
            ;
call  : identifier {
                $$ = newExpNode(CallK);
                $$->attr.name = savedName; 
                
                //$$->lineno = $1->lineno; 
         }
         LPAREN RPAREN
            { 
            }
        | identifier {
                $$ = newExpNode(CallK);
                $$->attr.name = savedName; 
                
                //$$->lineno = $1->lineno; 
         } 
         LPAREN arg_list RPAREN
            { 
                $$ = $2;
                $$->child[0] = $4;
            }
              
            ;
arg_list    : arg_list COMMA expression/**/
              { YYSTYPE t = $1;
                if (t != NULL){
                  while (t->sibling != NULL)
                    t = t->sibling;
                  t->sibling = $3;
                  $$ = $1;
                }
                else $$ = $3; }
            | expression { $$ = $1; }
            ;
number      : NUM
              { //$$ = newExpNode(ConstK);
                //$$->attr.val = atoi(tokenString);
                //$$->attr.type = Integer;
                savedNumber = atoi(tokenString);
                
                 }
            ;
identifier  : ID  
              { savedName = copyString(tokenString);  }
            ;
%%

int yyerror(const char * message){
  Error = TRUE;
  printf("Error: %s\n", message);
  return 0;
}

/* yylex calls getToken to make Yacc/Bison output
 * compatible with ealier versions of the TINY scanner
 */
static int yylex(void)
{ return getToken(); }

TreeNode * parse(void)
{ yyparse();
  return savedTree;
}

