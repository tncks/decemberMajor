# Makefile for C-Minus
#
# ./lex/tiny.l        --> ./cminus.l (from Project 1)
# ./yacc/tiny.y       --> ./cminus.y
# ./yacc/globals.h    --> ./globals.h

CC = gcc

CFLAGS = -g -W -Wall

OBJS = main.o util.o lex.yy.o y.tab.o symtab.o analyze.o code.o cgen.o

.PHONY: all clean run
all: cminus_semantic

clean:
	rm -vf cminus_semantic *.o log* lex.yy.c y.tab.c y.output

cminus_semantic: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $@ -lfl

main.o: main.c globals.h util.h scan.h parse.h analyze.h cgen.h code.h
	$(CC) $(CFLAGS) -c main.c

util.o: util.c util.h globals.h
	$(CC) $(CFLAGS) -c util.c

scan.o: scan.c scan.h util.h globals.h
	$(CC) $(CFLAGS) -c scan.c

lex.yy.o: lex.yy.c scan.h util.h globals.h
	$(CC) $(CFLAGS) -c lex.yy.c


y.tab.o: y.tab.c y.tab.h parse.h util.h globals.h
	$(CC) $(CFLAGS) -c y.tab.c

lex.yy.c: cminus.l
	flex cminus.l
y.tab.c y.tab.h: cminus.y
	bison -d -v -o y.tab.c cminus.y

cgen.o: cgen.c cgen.h code.h
	$(CC) $(CFLAGS) -c cgen.c

code.o: code.c code.h
	$(CC) $(CFLAGS) -c code.c

symtab.o: symtab.c symtab.h
	$(CC) $(CFLAGS) -c symtab.c

analyze.o: analyze.c analyze.h globals.h symtab.h util.h
	$(CC) $(CFLAGS) -c analyze.c

run:
	./cminus_semantic ../../basic/test_cases/p8.txt > log_p8   2>/dev/null
	./cminus_semantic ../../basic/test_cases/p8_2.txt > log_p8_2   2>/dev/null
	./cminus_semantic ../../basic/test_cases/p9.txt > log_p9   2>/dev/null
	./cminus_semantic ../../basic/test_cases/p9_2.txt > log_p9_2   2>/dev/null
	./cminus_semantic ../../basic/test_cases/p10.txt > log_p10   2>/dev/null
	./cminus_semantic ../../basic/test_cases/p11.txt > log_p11   2>/dev/null
	./cminus_semantic ../../basic/test_cases/p12.txt > log_p12   2>/dev/null
	./cminus_semantic ../../basic/test_cases/p13.txt > log_p13   2>/dev/null
#	./cminus_semantic ../../basic/test_cases/p14.txt > log_p14   2>/dev/null
	./cminus_semantic ../../basic/test_cases/p15.txt > log_p15   2>/dev/null
	./cminus_semantic ../../basic/test_cases/p16.txt > log_p16   2>/dev/null
	./cminus_semantic ../../basic/test_cases/p17.txt > log_p17   2>/dev/null
	./cminus_semantic ../../basic/test_cases/p18.txt > log_p18   2>/dev/null
	./cminus_semantic ../../basic/test_cases/p19.txt > log_p19   2>/dev/null
	echo "Half done. Wait.. now perform progress until end..."
	./cminus_semantic ../../basic/test_cases/test_1.txt > log_t1 2>/dev/null
	./cminus_semantic ../../basic/test_cases/test_2.txt > log_t2 2>/dev/null
	./cminus_semantic ../../basic/test_cases/test_3.txt > log_t3 2>/dev/null
	./cminus_semantic ../../basic/test_cases/test_4.txt > log_t4 2>/dev/null
	cat log* > log_all
	echo "All done! (2>/dev/null option is ON status.)"