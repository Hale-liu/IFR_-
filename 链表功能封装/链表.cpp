#include <stdio.h> 
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

void ShowData (Node *pnode)
{
	printf("x=%d\ty=%d\n",pnode->data.x,pnode->data.y);
}

int main()
{
    List *pList;
    ListInit(&pList);
    for(int i=0;i<10;i++)
	{
        Node *pnode = (Node*)malloc(sizeof(Node));
        pnode -> data.x =0;
        pnode -> data.y =1;
        pnode -> pnext = NULL;
        InserList(pList, pnode);
    }
    TraverList(pList, ShowData);
    return 0;
}

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
