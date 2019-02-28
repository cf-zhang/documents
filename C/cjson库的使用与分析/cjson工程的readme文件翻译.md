欢迎学习CJSON


cJSON 致力于成为轻量级的解析器，只有一个C文件和一个头文件，JSON 格式在http://www.json.org/有更详细的描述。


就像XML，但是更简洁，可以用来传输数据，存储内容或者只是用来表达程序的状态


首先，我应该如何使用呢?


    添加cJSON.c到你的工程中，然后将cJSON.h加入到头文件搜索目录中。
    
    
    例如，可以这样建立一个测试的应用程序:
    
    
    gcc cJSON.c test.c -o test -lm
    
    
    ./test
    
    
作为一个库，cJSON的存在是为了承担尽可能多的任务，但是不会成为你编码的阻碍。


作为一个实用主义者，我可以说你可以两种模式之一使用它:自动或者手动。让我们来一次快速贯通。


我从http://www.json.org/fatfree.html拿到了一些JSON数据，这个网页激发了我写一个解析器cJSON用来分享JSON的初衷，简单，轻量，实用。


一些JSON:

```
{
    "name": "Jack (\"Bee\") Nimble", 
    "format": {
        "type":       "rect", 
        "width":      1920, 
        "height":     1080, 
        "interlace":  false, 
        "frame rate": 24
    }
}
```

假设你从文件或者web服务器或者什么中拿到了这个数据，然而你只有一个char *指向它。一切都是cJSON结构体。


解析它:


	cJSON *root = cJSON_Parse(my_json_string);
	
	
这是一个对象，我们使用的C，没有对象。但是我们有结构体。


framerate的值是多少?


	cJSON *format = cJSON_GetObjectItem(root,"format");
	int framerate = cJSON_GetObjectItem(format,"frame rate")->valueint;
	
	
想改变framerate的值?


	cJSON_GetObjectItem(format,"frame rate")->valueint=25;
	
	
输出到磁盘?


	char *rendered=cJSON_Print(root);
	
	
完成了? 删除根节点(要留心).


	cJSON_Delete(root);
	
	
这是自动模式，如果你使用自动模式，你一定要在解引用之前检查指针。如果你想看一下如何用代码构造一个数据?


	cJSON *root,*fmt;
	root=cJSON_CreateObject();	
	cJSON_AddItemToObject(root, "name", cJSON_CreateString("Jack (\"Bee\") Nimble"));
	cJSON_AddItemToObject(root, "format", fmt=cJSON_CreateObject());
	cJSON_AddStringToObject(fmt,"type",		"rect");
	cJSON_AddNumberToObject(fmt,"width",		1920);
	cJSON_AddNumberToObject(fmt,"height",		1080);
	cJSON_AddFalseToObject (fmt,"interlace");
	cJSON_AddNumberToObject(fmt,"frame rate",	24);


所幸，我们认同这不是太多的代码。没有超载，没有不必要的设置，


看一下test.c这个完美的样例程序，大多数都是从json.org站点上获取的，还有其他地方。


手动模式是什么样的呢?首先你需要一些细节，让我们科普一下cJSON对象是如何表达JSON数据的吧。


cJSON在处理对象中数组的时候并进行区别对待,只有类型，每一个cJSON可能有一个孩子，兄弟，值，名字



根节点对象拥有:对象类型以及一个孩子


孩子有一个名字"name"，值是:"Jack ("Bee") Nimble"，一个兄弟:


兄弟有类型是object，名字是"format"，一个孩子。


这个孩子的类型是string，名字是"type"，值是"rect"，以及有一个兄弟:


这个兄弟的类型是Number，名字是"width"，值是1920，以及有一个兄弟:


这个兄弟的类型是Number,名字是"height"，值是1080，以及有一个兄弟:


这个兄弟的类型是False，名字是"interlace"，以及有一个兄弟:


这个兄弟的类型是Number,名字是"frame rate"，值是24。



这里是cJSON类型的结构体:

```
typedef struct cJSON {
	struct cJSON *next,*prev;
	struct cJSON *child;
	int type;
	char *valuestring;
	int valueint;
	double valuedouble;
	char *string;
} cJSON;
```

默认所有的值是0除非是故意设置成非0。

next/prev是一个连接兄弟节点的双向链表，next带你走向你的兄弟，prev带你回到你前一个兄弟。

只有对象和数组有孩子节点，并且节点中会有一个双向链表，一个孩子节点的prev为0，

如果存在兄弟节点会用next进行指向，最后一个兄弟的next为0.

类型的标识方法为:Null/True/False/Number/String/Array/Object,都是以宏的方式定义在cJSON.h文件中


Number包括valueint 和 valuedouble，如果你需要一个int，那么就读valueint，不然就读valuedouble。

每一个链表中的节点每一个子节点都会有一个"string"是"name"的存储位置当我说上面例子中的"name"的时候

那我说的就是"string"。如果你乐意，"string"是JSON的名字而且可以是个变量。


现在你可以递归的遍历整个链表，想如何解析就如何进行解析

你可以调用cJSON_Parse来使用cJSON为你解析，然后你就可以拿到对象的入口根部，然后遍历这个数据结构(一般是一棵N叉树)

然后随心所欲的进行分析。如果你想建立一个回调类型解析器，下面的例子说明了应该如何做(就是一个例子，因为这个例子非常特殊)

```
void parse_and_callback(cJSON *item,const char *prefix)
{
	while (item)
	{
		char *newprefix=malloc(strlen(prefix)+strlen(item->name)+2);
		sprintf(newprefix,"%s/%s",prefix,item->name);
		int dorecurse=callback(newprefix, item->type, item);
		if (item->child && dorecurse) parse_and_callback(item->child,newprefix);
		item=item->next;
		free(newprefix);
	}
}
```

这个前缀程序将会给你建立一个分割表。来简化你的回调处理。

'dorecurse'标记将会决定callback是否处理子数组，或者由你调用每一个节点，对于上面的情况来说，

你的回调可能看起来像这样的:

```
int callback(const char *name,int type,cJSON *item)
{
	if (!strcmp(name,"name"))	{ /* populate name */ }
	else if (!strcmp(name,"format/type")	{ /* handle "rect" */ }
	else if (!strcmp(name,"format/width")	{ /* 800 */ }
	else if (!strcmp(name,"format/height")	{ /* 600 */ }
	else if (!strcmp(name,"format/interlace")	{ /* false */ }
	else if (!strcmp(name,"format/frame rate")	{ /* 24 */ }
	return 1;
}
```

作为一种选择，你可能会倾向于迭代的去解析。

你可以使用:

```
void parse_object(cJSON *item)
{
	int i; 
	for (i=0;i<cJSON_GetArraySize(item);i++)
	{
		cJSON *subitem=cJSON_GetArrayItem(item,i);
		// handle subitem.	
	}
}
```

或者，用恰当的手动模式:

```
void parse_object(cJSON *item)
{
	cJSON *subitem=item->child;
	while (subitem)
	{
		// handle subitem
		if (subitem->child) parse_object(subitem->child);
		
		subitem=subitem->next;
	}
}
```

当然，真个可能看起来很熟悉，因为这就是一个裸版的回调解析。

这里包括了大部分的你可能会需要的解析方法，接下来的可能是一些猜想或者是怀疑，读一下源码，毕竟只有几百行而已。


在构建JSON数据方面，上面的样例代码就是正确的使用方式。

当然了你也可以处理使用其他的方法来填充子对象，如果你发现了一个这样的用法，可以手动的建立对象，

比如，假设你想建立一个对象数组

```
cJSON *objects[24];
cJSON *Create_array_of_anything(cJSON **items,int num)
{
	int i;
	cJSON *prev, *root=cJSON_CreateArray();
	for (i=0;i<24;i++)
	{
		if (!i)	
			root->child=objects[i];
		else
			prev->next=objects[i], objects[i]->prev=prev;
		prev=objects[i];
	}
	return root;
}
```

用法如下:

```
Create_array_of_anything(objects,24);
```

cJSON并不对你创建内容的顺序做任何假设，你像上面那样可以先指向到已经创建的这个对象上，

然后后续添加孩子到任意的每一个对象中。

当你调用cJSON_Print的时候，将会把结构体打印成文本。

代码test.c演示了一个如何处理一些特定的用例，如果你运行这个代码，它将会加载，解析然后打印这些数据成文本文件，这是一个比我尝试插入到一个常量数组里面更复杂的事情。

尽情享受cJSON吧

- Dave Gamble, Aug 2009