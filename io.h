#ifndef IO_H		//Do only once the first time this file is used
#define	IO_H
int nGDB = 0;
int nRec = 0;
void ReplaceChar(char *szCommand, char cOld, char cNew);
void upCase(char *p);
int  DecipherCommand(char *p)__attribute__((__optimize__("O1")));
//void ReplaceChar();

#endif

