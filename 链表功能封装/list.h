#ifndef __LIST_H__
#define __LIST_H__

#include <stdlib.h>

struct Data
{
    int x;
    int y;
};

struct Node
{
    Data data;
    struct Node *pnext;
};

struct List
{
    Node *pfront;
    Node *prear;
    int count;
};

int ListInit(List **pplist);
int IsEmpty (List *plist);
void InserList(List *plist, Node *pnode);
void TraverList(List *plist, void(*Traver)(Node* pnode));

#endif


