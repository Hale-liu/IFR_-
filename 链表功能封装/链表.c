#include <stdio.h> 
#include <stdlib.h>

struct Data
{
    int x;
    int y;
};

struct Node
{
    struct Data data;
    struct Node *pnext;
};

struct List
{
    struct Node *pfront;
    struct Node *prear;
    int count;
};

int ListInit(struct List **pplist);
int IsEmpty (struct List *plist);
void InserList(struct List *plist,struct Node *pnode);
void TraverList(struct List *plist, void(*Traver)(struct Node* pnode));

void ShowData (struct Node *pnode)
{
	printf("x=%d\ty=%d\n",pnode->data.x,pnode->data.y);
}

int main()
{
    int i;
    struct List *pList;
    ListInit(&pList);
    for(i=0;i<10;i++)
	{
        struct Node *pnode = (struct Node*)malloc(sizeof(struct Node));
        pnode -> data.x =1;
        pnode -> data.y =0;
        pnode -> pnext = NULL;
        InserList(pList, pnode);
    }
    TraverList(pList, ShowData);
    return 0;
}

int ListInit(struct List **pplist) //³õÊ¼»¯
{
    *pplist = (struct List*)malloc(sizeof(struct List));
    if (NULL == *pplist)
        return 0;
    else {
        (*pplist) -> pfront = NULL;
        (*pplist) -> prear = NULL;
        (*pplist) -> count = 0;
    }
    return 1;
}

int IsEmpty (struct List *plist)
{
    if (plist -> count == 0)
        return 1;
    else
        return 0;
}

void InserList(struct List *plist, struct Node *pnode) //Î²²å
{
    if (IsEmpty(plist))
        plist -> pfront = pnode;
    else
        plist -> prear -> pnext = pnode;

    plist -> prear = pnode;
    plist -> count++;
}

void TraverList(struct List *plist, void(*Traver)(struct Node* pnode))//±éÀú
{
    struct Node *ptemp = plist -> pfront;
    int listsize = plist -> count;

    while (listsize)
    {
        Traver(ptemp);
        ptemp = ptemp ->pnext;
        listsize--;
    }
}
