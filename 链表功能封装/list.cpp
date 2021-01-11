#include "list.h"

int ListInit(List **pplist) //³õÊ¼»¯
{
    *pplist = (List*)malloc(sizeof(List));
    if (NULL == *pplist)
        return 0;
    else {
        (*pplist) -> pfront = NULL;
        (*pplist) -> prear = NULL;
        (*pplist) -> count = 0;
    }
    return 1;
}

int IsEmpty (List *plist)
{
    if (plist -> count == 0)
        return 1;
    else
        return 0;
}

void InserList(List *plist, Node *pnode) //Î²²å
{
    if (IsEmpty(plist))
        plist -> pfront = pnode;
    else
        plist -> prear -> pnext = pnode;

    plist -> prear = pnode;
    plist -> count++;
}

void TraverList(List *plist, void(*Traver)(Node* pnode))//±éÀú
{
    Node *ptemp = plist -> pfront;
    int listsize = plist -> count;

    while (listsize)
    {
        Traver(ptemp);
        ptemp = ptemp ->pnext;
        listsize--;
    }
}
