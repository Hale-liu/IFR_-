#include <stdio.h> 
#include <stdlib.h>

struct Data
{
    int x;
};

struct Node
{
    Data data;
    struct Node *pnext;
    struct Node *pprev;
};

struct List
{
    Node *pfront;
    Node *prear;
    int count;
};

int p=0;

int ListInit(List **pplist);
int IsEmpty (List *plist);
void InserList1(List *plist, Node *pnode);
void InserList2(List *plist, Node *pnode);
void InserList3(List *plist, Node *pnode,int goal);
void InserList4(List *plist, Node *pnode);
void TraverList(List *plist, void(*Traver)(Node* pnode));
Node *SeclectNode(List *plist,int goal);
void ShowData (Node *pnode); 
void ChangeDate(Node *pnode);
void Deletedate(List *plist, int goal);
void Merge_Node(List *plist_1,List *plist_2,List *plist_3);
void Ensure_order(List *plist);
void BubbleSort(List *plist);

int main()
{
    List *pList_1;
    List *pList_2;
    List *pList_3;
    ListInit(&pList_1);
    ListInit(&pList_2);
    ListInit(&pList_3);
    for(int i=0;i<10;i++)
	{
        Node *pnode = (Node*)malloc(sizeof(Node));
        scanf("%d",&pnode -> data.x); 
        pnode -> pnext = NULL;
        InserList4(pList_1,pnode);
    }
    for(int i=0;i<10;i++)
	{
        Node *pnode = (Node*)malloc(sizeof(Node));
        scanf("%d",&pnode -> data.x); 
        pnode -> pnext = NULL;
		InserList4(pList_2,pnode);
    }
    TraverList(pList_1, ShowData);
    printf("\n\n\n");
    TraverList(pList_2, ShowData);
    printf("\n\n\n");
    Merge_Node(pList_1,pList_2,pList_3);
    TraverList(pList_3, ShowData);
    return 0;
}

void InserList4(List *plist, Node *pnode)
{
	if (IsEmpty(plist))
    {
    	plist -> pfront = pnode;
    	plist -> prear = pnode;
	} 
    else
    {
    	plist -> prear -> pnext = pnode;
    	pnode -> pprev = plist -> prear;
	}
    plist -> prear = pnode;
    plist -> count++;
    BubbleSort(plist);
}

void BubbleSort(List *plist)
{	
	int i,j;
 	for(i = plist -> count - 1; i>0; i--)
 	{
 		Node *ptemp = plist -> pfront;
 		ptemp->pprev=NULL; 
    	for(j=0;j<i;j++)
     	{
        	if(ptemp -> data.x > ptemp -> pnext -> data.x)
        	{
        		if(ptemp == plist -> pfront) plist -> pfront = ptemp -> pnext;
				if(ptemp == plist -> prear -> pprev) plist->prear = ptemp;
				if(ptemp -> pnext -> pnext == NULL)
				{
					ptemp -> pnext -> pprev = NULL;
					ptemp -> pprev = ptemp -> pnext;
					ptemp -> pprev -> pnext = ptemp; 
				}
				else
				{
					ptemp -> pnext -> pnext -> pprev = ptemp;
					ptemp -> pnext -> pprev = NULL;
					ptemp -> pprev = ptemp -> pnext;
					ptemp -> pnext = ptemp -> pnext->pnext;
					ptemp -> pprev -> pnext = ptemp; 
				}
        	}
        	else ptemp = ptemp->pnext;
     	}	
	}
}

void ShowData (Node *pnode)//展示链表 
{
	printf("x = %d\n",pnode -> data.x);
}

void ChangeDate(Node *pnode)
{ 
	p++;
	pnode -> data.x =p;
}

void Ensure_order(List *plist)//保证链表有序 
{
	TraverList(plist, ChangeDate);
	p=0;
}

void Merge_Node(List *plist_1,List *plist_2,List *plist_3)//合并两个链表 
{
	plist_1 -> prear -> pnext = plist_2 -> pfront;
	plist_2 -> pfront -> pprev = plist_1 -> prear;
	plist_3 -> pfront = plist_1 -> pfront;
	plist_3 -> prear = plist_2 -> prear;
	plist_3 -> count = plist_1 -> count + plist_2 -> count;
	BubbleSort(plist_3);
}

int ListInit(List **pplist) //链表创建
{
    *pplist = (List*)malloc(sizeof(List));
    if (NULL == *pplist)
        return 0;
    else 
	{
        (*pplist) -> pfront = NULL;
        (*pplist) -> prear = NULL;
        (*pplist) -> count = 0;
    }
    return 1;
}

void Deletedate(List *plist, int goal)//删除任意节点 
{
	if(NULL == plist) 
	{
		printf("此链表无节点，无法删除！！！\n");
		return;
	}
	Node *ptemp = SeclectNode(plist,goal); 
	if(NULL == ptemp) 
	{
		printf("没有此节点，无法删除！！！\n"); 
		return; 
	} 
	if(plist -> count == 1)
	{
		plist -> pfront = NULL;
		plist -> prear = NULL;
		free(ptemp); 
	}
	else if(plist -> count == 2)
	{	
		if(goal == 1) plist -> pfront = plist -> prear;
		else if(goal == 2) plist -> prear = plist -> pfront;
		free(ptemp); 
	}
	else 
	{
		ptemp->pprev->pnext=ptemp->pnext;
		ptemp->pnext->pprev=ptemp->pprev;
		free(ptemp); 
	}
	plist -> count--;
	//Ensure_order(plist);
} 

void InserList1(List *plist, Node *pnode) //尾插
{
    if (IsEmpty(plist))
    {
    	plist -> pfront = pnode;
    	plist -> prear = pnode;
	} 
    else
    {
    	plist -> prear -> pnext = pnode;
    	pnode -> pprev = plist -> prear;
	}
    plist -> prear = pnode;
    plist -> count++;
    pnode -> data.x = plist -> count;
}

void InserList2(List *plist, Node *pnode) //头插
{
	if (IsEmpty(plist))
    {
    	plist -> pfront = pnode;
    	plist -> prear = pnode;
	} 
    else
    {
    	plist -> pfront -> pprev = pnode;
		pnode -> pnext = plist -> pfront;
	}
    plist -> pfront = pnode;
    plist -> count++;
    Ensure_order(plist);
}

void InserList3(List *plist, Node *pnode,int goal) //任意插 
{
	if(goal == 0) 
	{
		InserList2(plist,pnode);
		return;
	}
	Node *ptemp = SeclectNode(plist,goal); 
	if(NULL == ptemp) 
	{
		printf("没有此节点，无法在其后边添加新节点！！！\n"); 
		return; 
	} 
	if(ptemp->pnext==NULL)
	{
		InserList1(plist, pnode);
	}
	else
	{
		pnode -> pnext = ptemp -> pnext;
		pnode -> pprev = ptemp;
		ptemp -> pnext -> pprev = pnode;
		ptemp -> pnext = pnode;
		plist -> count++;
		Ensure_order(plist);
	}
} 

void TraverList(List *plist, void(*Traver)(Node* pnode))//遍历
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

Node *SeclectNode(List *plist,int goal)//查询 
{
    Node *ptemp = plist -> pfront;
    int listsize = plist -> count;

    while (listsize)
    {
        if(goal==plist -> count - listsize + 1) return ptemp;
        ptemp = ptemp ->pnext;
        listsize--;
    }
    return NULL;
}

int IsEmpty (List *plist)
{
    if (plist -> count == 0)
        return 1;
    else
        return 0;
}
