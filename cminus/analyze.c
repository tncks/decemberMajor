/****************************************************/
/* File: analyze.c                                  */
/* Semantic analyzer implementation                 */
/* for the TINY compiler                            */
/* Compiler Construction: Principles and Practice   */
/* Kenneth C. Louden                                */
/****************************************************/

#include "globals.h"
#include "symtab.h"
#include "analyze.h"
#include "util.h"


static char * funcName;
static int preserveLastScope = FALSE;

/* counter for variable memory locations */

/* Procedure traverse is a generic recursive
 * syntax tree traversal routine:
 * it applies preProc in preorder and postProc
 * in postorder to tree pointed to by t
 */
static void traverse( TreeNode * t,
               void (* preProc) (TreeNode *),
               void (* postProc) (TreeNode *) )
{ if (t != NULL)
  { preProc(t);
    { int i;
      for (i=0; i < MAXCHILDREN; i++)
        traverse(t->child[i],preProc,postProc);
    }
    postProc(t);
    traverse(t->sibling,preProc,postProc);
  }
}

static void insertIOFunc(void)   
{ TreeNode *func;
  TreeNode *typeSpec;
  TreeNode *param;
  TreeNode *compStmt;




  func = newDeclNode((DeclKind) FunK);

  typeSpec = newTypeNode((TypeKind) FunK, INT);
  func->type = Integer;

  param = newParamNode(NonArrParamK);
  param->attr.name = "";
  param->attr.type = VOID;

  compStmt = newStmtNode(CompK);
  compStmt->child[0] = NULL;      // no local var
  compStmt->child[1] = NULL;      // no stmt

  func->lineno = 0;
  func->attr.name = "input";
  func->attr.type = typeSpec->attr.type; 
  func->child[0] = param;          
  func->child[1] = compStmt;

  funcName = func->attr.name;

  st_insert(funcName, -1, addLocation(), func);
  sc_push(sc_create(funcName));
  param->attr.scope = sc_top();
  compStmt->attr.scope = sc_top();
  sc_pop();
  






  func = newDeclNode((DeclKind) FunK);

  typeSpec = newTypeNode((TypeKind) FunK, VOID);
  func->type = Void;

  param = newParamNode(NonArrParamK);
  param->attr.name = "value";
  param->attr.type = INT;
  

  compStmt = newStmtNode(CompK);
  compStmt->child[0] = NULL;      // no local var
  compStmt->child[1] = NULL;      // no stmt

  func->lineno = 0;
  func->attr.name = "output";
  func->attr.type = typeSpec->attr.type;
  func->child[0] = param;
  func->child[1] = compStmt;

  funcName = func->attr.name;

  st_insert(funcName, -1, addLocation(), func);
  sc_push(sc_create(funcName));
  param->attr.scope = sc_top();
  compStmt->attr.scope = sc_top();
  
  st_insert(param->attr.name, -1, addLocation(), param);
  //if (param->kind.param == NonArrParamK) param->type = INT?Integer?pickone;
  //afterInsertNode( TreeNode * t )   // is it necessary?
  sc_pop();    
}




/* nullProc is a do-nothing procedure to
 * generate preorder-only or postorder-only
 * traversals from traverse
 */
static void nullProc(TreeNode * t)
{ if (t==NULL) return;
  else return;
}

static void symbolError(TreeNode * t, char * message)
{
  t==NULL? fprintf(listing, "EMessage: %s\n", message) : fprintf(listing, "Symbol error at line %d: %s\n",t->lineno,message);
  Error = TRUE;
}

/* Procedure insertNode inserts
 * identifiers stored in t into
 * the symbol table
 */
static void insertNode( TreeNode * t)
{ switch (t->nodekind)
  { case StmtK:
      switch (t->kind.stmt)
      { case CompK:
          if (preserveLastScope) {
            preserveLastScope = FALSE;
          } else {
            Scope scope = sc_create(funcName);
            sc_push(scope);
          }
          t->attr.scope = sc_top();
          break;
        default:
          break;
      }
      break;
    case ExpK:
      switch (t->kind.exp)
      { case IdK:
        case ArrIdK:
        case CallK:
          if (st_lookup(t->attr.name) == -1) {
          /* not yet in table, error */
          fprintf(listing, "debug)notYetInTable(IdKOrArrIdKOrCallK)YetDeclared.>>\t");
          if(t->kind.exp == CallK) fprintf(listing, "Error: undeclared function \"%s\" is called at line %d\n", t->attr.name, t->lineno);
          else {
            fprintf(listing, "Error: undeclared variable \"%s\" is used at line %d\n", t->kind.exp == IdK ? t->attr.name : t->attr.arr.name, t->lineno);
          }
          fprintf(listing, "checkpoint->case ExpK -> IdK/ArrIdK/CallK -> symerror\t");
          Error = TRUE;
          symbolError(t, "undelcared symbol"); // TODO: remove or comment the symbolError line
            // TODO : add the undetermined symbol to the our symbol table.
          } //TODO: code clean up and organize IdK/ArrIdK/CallK ->  separately
          else
          /* already in table, so ignore location,
             add line number of use only */
            st_add_lineno(t->attr.name,t->lineno);
          break;
        default:
          break;
      }
      break;
    case DeclK:
      switch (t->kind.decl)
      { case FunK:
          funcName = t->attr.name;
          if (st_lookup_top(funcName) >= 0) {
          /* already in table, so it's an error */
            fprintf(listing, "debug)functionAlreadyDeclared.>>\t");
            //TODO: make proper code about printing ? ?  part.
            fprintf(listing, "Error: Symbol \"%s\" is redefined at line %d (already defined at line ? ?)\n", t->attr.name, t->lineno);
            Error = TRUE;
            //symbolError(t,"function already declared");
            break;
          }
          st_insert(funcName,t->lineno,addLocation(),t);
          sc_push(sc_create(funcName));
          preserveLastScope = TRUE;
          switch (t->attr.type)
          { case INT:
              t->type = Integer;
              break;
            case VOID:
            default:
              t->type = Void;
              break;
          }
          break;
        case VarK:
        case ArrVarK:
          { char *name;

            if (t->attr.type == VOID) {
              fprintf(listing, "Error: The void-type variable is declared at line %d (name : \"%s\")\n", t->lineno, t->attr.name);
              Error = TRUE;
              //symbolError(t,"variable should have non-void type");
              break;
            }

            if (t->kind.decl == VarK) {
              name = t->attr.name;
              t->type = Integer;
            } else {
              name = t->attr.arr.name;
              t->type = IntegerArray;
            }

            if (st_lookup_top(name) < 0)
              st_insert(name,t->lineno,addLocation(),t);
            else
            {
              fprintf(listing, "debug)SomeVarInThisScopeAlreadyDeclared.>>\t");
              fprintf(listing, "Error: Symbol \"%s\" is redefined at line %d (already defined at line ? ?)\n", t->kind.decl == VarK ? t->attr.name : t->attr.arr.name, t->lineno);
              // TODO: print more info about at line ? by traversing the symbol table, where ? ? related info.
              Error = TRUE;
              //symbolError(t,"symbol already declared for current scope");
            }
              
          }
          break;
        default:
          break;
      }
      break;
    case ParamK:
      int debugOff = 0;
      if(debugOff==0 && t != NULL && strcmp(t->attr.name, "")==0 ) fprintf(listing, "\n\t\tdebugmode-> found: attr.name has "" node (at case ParamK:  in analyze.c)\n");
      if (t->attr.type == VOID)
        symbolError(t,"void type parameter is not allowed"); // <- change from: symbolError(t->child[0],"void type parameter is not allowed");
      if (st_lookup(t->attr.name) == -1) {
        st_insert(t->attr.name,t->lineno,addLocation(),t);
        if (t->kind.param == NonArrParamK)
          t->type = Integer;
        else {
          fprintf(listing, "debug)SomeParamKInThisScopeAlreadyDeclared.>>\t");
          fprintf(listing, "Error: Symbol \"%s\" is redefined at line %d (already defined at line ? ?)\n", t->attr.name==NULL ? "attr.nameNULL" : t->attr.name, t->lineno);
          Error = TRUE;
          //symbolError(t,"symbol already declared for current scope");
        }
        
      }
      break;
    default:
      break;
  }
}

static void afterInsertNode( TreeNode * t )
{ switch (t->nodekind)
  { case StmtK:
      switch (t->kind.stmt)
      { case CompK:
          sc_pop();
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

/* Function buildSymtab constructs the symbol
 * table by preorder traversal of the syntax tree
 */
void buildSymtab(TreeNode * syntaxTree)
{ globalScope = sc_create(NULL);
  sc_push(globalScope);
  insertIOFunc();
  traverse(syntaxTree,insertNode,afterInsertNode);
  sc_pop();
  if (TraceAnalyze)
  { fprintf(listing,"\nSymbol table:\n\n");
    printSymTab(listing);
  }
}

static void typeError(TreeNode * t, char * message)
{ t==NULL? fprintf(listing, "EMessage: %s\n", message) : fprintf(listing,"Type error at line %d: %s\n",t->lineno,message);
  Error = TRUE;
}

static void beforeCheckNode(TreeNode * t)
{ switch (t->nodekind)
  { case DeclK:
      switch (t->kind.decl)
      { case FunK:
          funcName = t->attr.name;
          break;
        default:
          break;
      }
      break;
    case StmtK:
      switch (t->kind.stmt)
      { case CompK:
          sc_push(t->attr.scope);
          break;
        default:
          break;
      }
    default:
      break;
  }
}

/* Procedure checkNode performs
 * type checking at a single tree node
 */
static void checkNode(TreeNode * t)
{ switch (t->nodekind)
  { case StmtK:
      switch (t->kind.stmt)
      { case CompK:
          sc_pop();
          break;
        case IterK:
          if (t->child[0]->type == Void)
          /* while test should be void function call */
            typeError(t->child[0],"while test has void value");
          break;
        case ReturnK:
          { const TreeNode * funcDecl =
                st_bucket(funcName)->treeNode;
            const ExpType funcType = funcDecl->type;
            const TreeNode * expr = t->child[0];

            if (funcType == Void &&
                (expr != NULL && expr->type != Void)) {
              fprintf(listing, "Error: Invalid return at line %d\n", t->lineno);
              Error = TRUE;
              typeError(t,"expected no return value");
              // TODO:  remove or comment typeError line
              //ValueReturned = TRUE;
            } else if (funcType == Integer &&
                (expr == NULL || expr->type != Integer)) {
              fprintf(listing, "Error: Invalid return at line %d\n", t->lineno);
              Error = TRUE;
              typeError(t,"expected return value");
              // TODO:  remove or comment typeError line
            }
          }
          break;
        default:
          break;
      }
      break;
    case ExpK:
      switch (t->kind.exp)
      { case AssignK:
          if (t->child[0]->type == IntegerArray) {
            /* no value can be assigned to array variable */
            fprintf(listing, "Error: invalid assignment at line %d\n", t->child[0]->lineno);
            Error = TRUE;
            typeError(t->child[0],"assignment to array variable");
            // TODO: remove or comment typeError line
          }            
          else if (t->child[1]->type == Void) {
            /* r-value cannot have void type */
            fprintf(listing, "Error: invalid assignment at line %d\n", t->child[0]->lineno);
            Error = TRUE;
            typeError(t->child[0],"assignment of void value");
            // TODO: remove or comment typeError line 
          }            
          else {
            t->type = t->child[0]->type;
          }            
          break;
        case OpK:
          { ExpType leftType, rightType;
            TokenType op;

            leftType = t->child[0]->type;
            rightType = t->child[1]->type;
            op = t->attr.op;

            if (leftType == Void || rightType == Void)
            {
              fprintf(listing, "Error: invalid operation at line %d\n", t->lineno);
              Error = TRUE;
              typeError(t,"two operands should have non-void type");
              // TODO: remove or comment the typeError line
            }              
            else if (leftType == IntegerArray && rightType == IntegerArray)
            {
              fprintf(listing, "Error: invalid operation at line %d\n", t->lineno);
              Error = TRUE;
              typeError(t,"not both of operands can be array");
              // TODO: remove or comment the typeError line            
            }              
            else if (op == MINUS &&
                leftType == Integer &&
                rightType == IntegerArray)
              typeError(t,"invalid operands to binary expression");   //TODO  (check and test and make proper fprintf code to replace typeError line)
            else if ((op == TIMES || op == OVER) &&
                (leftType == IntegerArray ||
                 rightType == IntegerArray))
              typeError(t,"invalid operands to binary expression");  //TODO  (check and test and make proper fprintf code to replace typeError line)
            else {
              t->type = Integer;
            }
          }
          break;
        case ConstK:
          t->type = Integer;
          break;
        case IdK:
        case ArrIdK:
          { const char *symbolName = t->attr.name;
            const BucketList bucket =
                st_bucket(symbolName);
            TreeNode *symbolDecl = NULL;

            if (bucket == NULL)
              break;
            symbolDecl = bucket->treeNode;

            if (t->kind.exp == ArrIdK) {
              if (symbolDecl->kind.decl != ArrVarK &&
                  symbolDecl->kind.param != ArrParamK)
                typeError(t,"expected array symbol");
                // TODO: if statement 블록으로 감싸주기, typeError 주석처리하고, fprintf  에러 양식으로 바꿔주기 (make proper code)
              else if (t->child[0]->type != Integer)
              {                
                fprintf(listing, "Error: Invalid array indexing at line %d (name : \"%s\"). indicies should be integer\n", t->lineno, t->attr.name);
                Error = TRUE;
                //typeError(t,"index expression should have integer type");
              }                
              else
                t->type = Integer;
            } else {
              t->type = symbolDecl->type;
            }
          }
          break;
        case CallK:
          { const char *callingFuncName = t->attr.name;
            const TreeNode * funcDecl = st_bucket(callingFuncName)->treeNode;
            TreeNode *arg;
            TreeNode *param;

            if (funcDecl == NULL)
              break;

            arg = t->child[0];
            param = funcDecl->child[1];

            if (funcDecl->kind.decl != FunK)
            { typeError(t,"expected function symbol");
              break;
            }

            while (arg != NULL)
            { if (param == NULL) {
              /* the number of arguments does not match to
                 that of parameters */
                fprintf(listing, "debug)Error:InvalidFunctionCallBlabla..>>\t");
                typeError(arg,"the number of parameters is wrong");
                
                //fprintf(listing, "Error: Invalid function call at line %d (name : \"%s\")\n", arg->lineno, arg->attr.name);
                //Error = TRUE;
              }

              /*else if (arg->type == IntegerArray &&
                  param->type != IntegerArray)
                typeError(arg,"expected non-array value");
              else if (arg->type == Integer &&
                  param->type == IntegerArray)
                typeError(arg,"expected array value");*/
              else if (arg->type == Void) {
                typeError(arg,"void value cannot be passed as an argument");
              }                
              else {  // no problem!
                arg = arg->sibling;
                param = param->sibling;
                continue;
              }
              /* any problem */
              break;
            }

            if (arg == NULL && param != NULL)
            /* the number of arguments does not match to
               that of parameters */
              typeError(t->child[0],"the number of parameters is wrong");

            t->type = funcDecl->type;
          }
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

/* Procedure typeCheck performs type checking
 * by a postorder syntax tree traversal
 */
void typeCheck(TreeNode * syntaxTree)
{ sc_push(globalScope);
  traverse(syntaxTree,beforeCheckNode,checkNode);
  sc_pop();
}
