本着学习的目的下载到了tinnyhttpd的源码，首先阅读了一下readme文档，以下是tinnyhttpd的readme的大概意思:

```
  这个软件是 J. David Blackstone在1999年写的，根据从http://www.gnu.org/获得的
GNU通用公共许可证被允许修改和分发。
  如果你使用这个软件或者测试这个代码，我将会非常感激，并想知道关于这件事情，
可以联系我jdavidb@sourceforge.net
  这个软件并不能保证生产质量。没有提供任何形式的保证，甚至是某种目的的
默认的保证。如果你在你的计算机系统中使用这个软件造成了损坏，我不会负责的。
  我写这个web服务器是在1999年的时候因为我的网络课程的一次作业。
我们被告知在一个最小型的服务器提供网页服务，以及告诉我们如果我们做了额
外的任务将会得到额外的学分。Perl向我展示了一些UNIX功能(学习了sockers以及fork)，
同时O'Reilly的关于UNIX系统与CGI和Perl写一个web客户端的书籍让我意识到我可以轻易
的制作一个web服务器来提供CGI功能。
  现在如果你是一个apache核心团体的成员，你或许一点儿印象都没有，但是我的教授被搞定了。
测试color.cgi脚本例程然后输入"chartreuse."，让我看起来比较聪明一些。
  Apache不会，但是我希望这个程序对于那些对http/socket编程感兴趣的人是一个好的教学工具，
就像UNIX系统调用一样。(这里有关于使用pipe，环境变量，forks等等)
  一件最后的事情:如果你看到了我的web服务器或者(你是心不在焉的吗?)使用它，我都会非常高兴的知道这件事。
请给我发email。我或许不会真正的去更新程序，但是我想知道知道我帮助你学习了一些东西；


  快乐的黑客
                                                   J. David Blackstone
```

其源代码网上资源很多，可以搜一下就可以找到下载链接，在拿到代码执行Makefile的时候可能会遇到-lsocket找不到，可以在Makefile里面讲这个库的引用直接删去即可。编译通过之后，将htdoc下面的color.cgi添加一个可执行权限，然后执行httpd可以拿到一个端口号，然后可以使用浏览器通过输入ip进行访问，例如我的访问方式如下：10.95.4.211:12345   其中12345代表端口号。



一共500多行代码，其实在技巧上并没有多少让人眼前一亮的感觉，不如阅读cJSON源码的时候那种时刻都会感觉到闪光点存在。但是从整体上来看在结构上有许多可以去学习和借鉴的地方：

服务端socket建立之后，便阻塞等待客户端来连接，每来一个连接就创建一个线程去处理这个连接。处理流程上，根据收到的请求方式来决定给客户端反馈的方式。有直接返回请求的文件，有执行cgi脚本产生的内容。其中解析数据的时候需要一些http协议相关的协议知识，另外就是在执行cgi的时候通过管道的重映射方式，将程序的输入和输出与cgi脚本的输入输出联系起来。对于整个网页的访问流程会有一些学习，比如说，发起一个网页访问的时候，其实就是向web服务器发起一个资源请求，web服务器接收到这个请求之后会根据携带的目录或者资源标识去找到请求的资源并发送到客户端。执行cgi脚本，其实也是将cgi脚本处理后所输出的内容发送给了客户端。

对于cgi脚本没有接触过，html文本也就有一点点儿概念，所以就没有自己再做一下改编版的例程了。读一下理解了原理也是很好的。

在读源码的时候有一些注释就顺道加上了，还有一些根据英文内容做了一下自己的理解。源码如下：

tinnyhttpd.c

```
/* 程序是在 Sparc Solaris 2.6.上编译的
 * 如果要在Linux上编译:
 *  1) 注释 #include <pthread.h> line.
 *  2) 注释掉定义的newthread变量.
 *  3) 注释掉pthread_create().
 *  4) 取消注释accept_request().
 *  5) 从Makefile移除 -lsocket
 */
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <ctype.h>
#include <strings.h>
#include <string.h>
#include <sys/stat.h>
#include <pthread.h>
#include <sys/wait.h>
#include <stdlib.h>


#define ISspace(x) isspace((int)(x))


#define SERVER_STRING "Server: jdbhttpd/0.1.0\r\n"


void accept_request(int);
void bad_request(int);
void cat(int, FILE *);
void cannot_execute(int);
void error_die(const char *);
void execute_cgi(int, const char *, const char *, const char *);
int get_line(int, char *, int);
void headers(int, const char *);
void not_found(int);
void serve_file(int, const char *);
int startup(u_short *);
void unimplemented(int);


/**********************************************************************/
/* 一个请求是的accept调用返回。合适的处理这个请求
 * 参数: 连接到服务器的socket
/**********************************************************************/
void accept_request(int client)
{
 char buf[1024];
 int numchars;
 char method[255];
 char url[255];
 char path[512];
 size_t i, j;
 struct stat st;
 int cgi = 0;      /* 为真时，服务器确定是一个cgi程序*/
 char *query_string = NULL;
//获取一行
 numchars = get_line(client, buf, sizeof(buf));
 i = 0; j = 0;//将读到的一行内容的行为备份到method中
 while (!ISspace(buf[j]) && (i < sizeof(method) - 1))
 {//读取直到空行
  method[i] = buf[j];
  i++; j++;
 }
 method[i] = '\0';//结束字符
//如果不"GET"或者"POST"请求，那么就不做处理
 if (strcasecmp(method, "GET") && strcasecmp(method, "POST"))
 {
  unimplemented(client);
  return;
 }
//如果是post请求，那么置一个标志位
 if (strcasecmp(method, "POST") == 0)
  cgi = 1;
 //在这里获取url
 i = 0;
 while (ISspace(buf[j]) && (j < sizeof(buf)))
  j++;
 while (!ISspace(buf[j]) && (i < sizeof(url) - 1) && (j < sizeof(buf)))
 {
  url[i] = buf[j];
  i++; j++;
 }
 url[i] = '\0';
 
 if (strcasecmp(method, "GET") == 0)
 {
  query_string = url;
  while ((*query_string != '?') && (*query_string != '\0'))
   query_string++;
  if (*query_string == '?')
  {
   cgi = 1;
   *query_string = '\0';
   query_string++;
  }
 }
 //拼接资源目录
 sprintf(path, "htdocs%s", url);
 if (path[strlen(path) - 1] == '/')
  strcat(path, "index.html");//如果没有url目录那么就是访问主页
 if (stat(path, &st) == -1) 
 {
  while ((numchars > 0) && strcmp("\n", buf))  /* read & discard headers */
   numchars = get_line(client, buf, sizeof(buf));
  not_found(client);
 }
 else
 {
  if ((st.st_mode & S_IFMT) == S_IFDIR)
   strcat(path, "/index.html");//拼接目录
  if ((st.st_mode & S_IXUSR) ||
      (st.st_mode & S_IXGRP) ||
      (st.st_mode & S_IXOTH)    )
   cgi = 1;
  if (!cgi)//根据cgi的标志位选择如何进行返回
   serve_file(client, path);
  else
   execute_cgi(client, path, method, query_string);
 }


 close(client);
}




/**********************************************************************/
/* 通知客户端请求造成了一个错误
/**********************************************************************/
void bad_request(int client)
{
 char buf[1024];
//拼接字符串发送出去
 sprintf(buf, "HTTP/1.0 400 BAD REQUEST\r\n");
 send(client, buf, sizeof(buf), 0);
 sprintf(buf, "Content-type: text/html\r\n");
 send(client, buf, sizeof(buf), 0);
 sprintf(buf, "\r\n");
 send(client, buf, sizeof(buf), 0);
 sprintf(buf, "<P>Your browser sent a bad request, ");
 send(client, buf, sizeof(buf), 0);
 sprintf(buf, "such as a POST without a Content-Length.\r\n");
 send(client, buf, sizeof(buf), 0);
}


/**********************************************************************/
/* Put the entire contents of a file out on a socket.  This function
 * is named after the UNIX "cat" command, because it might have been
 * easier just to do something like pipe, fork, and exec("cat").
 * Parameters: the client socket descriptor
 *             FILE pointer for the file to cat */
/**********************************************************************/
/**********************************************************************/
/* 通过sockt将整个文件发送出去.  这个命令一个nuix的cat命令命名，因为它
 * 或许比较容易的去做管道所做的事情，比如fork 然后执行cat
/**********************************************************************/
void cat(int client, FILE *resource)
{
 char buf[1024];


 fgets(buf, sizeof(buf), resource);
 while (!feof(resource))
 {//将源文件发送出去
  send(client, buf, strlen(buf), 0);
  fgets(buf, sizeof(buf), resource);
 }
}


/**********************************************************************/
/* 通知客户端cgi文件无法被执行
/**********************************************************************/
void cannot_execute(int client)
{
 char buf[1024];
 //组装字符串发出去
 sprintf(buf, "HTTP/1.0 500 Internal Server Error\r\n");
 send(client, buf, strlen(buf), 0);
 sprintf(buf, "Content-type: text/html\r\n");
 send(client, buf, strlen(buf), 0);
 sprintf(buf, "\r\n");
 send(client, buf, strlen(buf), 0);
 sprintf(buf, "<P>Error prohibited CGI execution.\r\n");
 send(client, buf, strlen(buf), 0);
}


/**********************************************************************/
/* 使用perror()打印错误信息 (适用于系统错误; 依赖于代表系统调用错误的errorno)
 * 使程序退出执行 */
/**********************************************************************/
void error_die(const char *sc)
{
 perror(sc);
 exit(1);
}




/**********************************************************************/
/* 执行一个cgi脚本，需要恰当的设置环境变量
/**********************************************************************/
void execute_cgi(int client, const char *path,
                 const char *method, const char *query_string)
{
 char buf[1024];
 int cgi_output[2];
 int cgi_input[2];
 pid_t pid;
 int status;
 int i;
 char c;
 int numchars = 1;
 int content_length = -1;


 buf[0] = 'A'; buf[1] = '\0';
 if (strcasecmp(method, "GET") == 0)
  while ((numchars > 0) && strcmp("\n", buf))  /* read & discard headers */
   numchars = get_line(client, buf, sizeof(buf));
 else    /* POST */
 {//获取执行cgi相关的信息
  numchars = get_line(client, buf, sizeof(buf));
  while ((numchars > 0) && strcmp("\n", buf))
  {
   buf[15] = '\0';
   if (strcasecmp(buf, "Content-Length:") == 0)
    content_length = atoi(&(buf[16]));
   numchars = get_line(client, buf, sizeof(buf));
  }
  if (content_length == -1) {
   bad_request(client);
   return;
  }
 }


 sprintf(buf, "HTTP/1.0 200 OK\r\n");
 send(client, buf, strlen(buf), 0);


 if (pipe(cgi_output) < 0) {
  cannot_execute(client);
  return;
 }
 if (pipe(cgi_input) < 0) {
  cannot_execute(client);
  return;
 }


 if ( (pid = fork()) < 0 ) {
  cannot_execute(client);
  return;
 }
 if (pid == 0)  /* child: CGI script */
 {
  char meth_env[255];
  char query_env[255];
  char length_env[255];
 //子进程中设置环境变量，重映射标准输入输出为打开的管道
  dup2(cgi_output[1], 1);
  dup2(cgi_input[0], 0);
  close(cgi_output[0]);
  close(cgi_input[1]);
  sprintf(meth_env, "REQUEST_METHOD=%s", method);
  putenv(meth_env);
  if (strcasecmp(method, "GET") == 0) {
   sprintf(query_env, "QUERY_STRING=%s", query_string);
   putenv(query_env);
  }
  else {   /* POST */
   sprintf(length_env, "CONTENT_LENGTH=%d", content_length);
   putenv(length_env);
  }
  execl(path, path, NULL);//设置好了环境变量之后就执行cgi脚本
  exit(0);
 } 
 else 
 {    /* parent */
  close(cgi_output[1]);
  close(cgi_input[0]);
  if (strcasecmp(method, "POST") == 0)
   for (i = 0; i < content_length; i++) 
   {//将接受到的参数写入管道其实就是为了给cgi脚本传参数
    recv(client, &c, 1, 0);
    write(cgi_input[1], &c, 1);
   }
  while (read(cgi_output[0], &c, 1) > 0)
   send(client, &c, 1, 0);//接受cgi脚本的输出


  close(cgi_output[0]);
  close(cgi_input[1]);
  waitpid(pid, &status, 0);
 }
}




/**********************************************************************/
/* 从socket中获取一行, 不论这一行是以一个换行符\n还是\r或者是\r\n。遇到'\0'
 * 时结束读取内容。如果在完成读取之前没有换行提示，那么这个字符串就以'\0'结尾
 * 如果上述的三个行结束标记被读取到，那么这个字符串的最后一个字符是回车换行符
 * 字符串会以'\0'结尾
 * 参数: socket 套接字
 *       buf	保存信息的缓存区
 *       size   缓冲区的大小
 * 返回: 保存的字节数，不含'\0'
/**********************************************************************/
int get_line(int sock, char *buf, int size)
{
 int i = 0;
 char c = '\0';
 int n;
//寻找'\n'直到超出缓冲区大小
 while ((i < size - 1) && (c != '\n'))
 {//接收一个字符
  n = recv(sock, &c, 1, 0);
//  printf("%02X\n", c);
  if (n > 0)
  {
   if (c == '\r')
   {//处理\r\n的情况，再取一个直接进行判断一次
    n = recv(sock, &c, 1, MSG_PEEK);
    /* DEBUG printf("%02X\n", c); */
    if ((n > 0) && (c == '\n'))
     recv(sock, &c, 1, 0);//如果是那么就把这个字符读出来
    else
     c = '\n';//否则就将\r变成\n
   }
   buf[i] = c;//放到缓存中
   i++;
  }
  else
   c = '\n';//没有接收到数据，那么就认为接收到'\n'触发退出条件
 }
 buf[i] = '\0';//将'\0'填充到末尾
 
 return(i);//返回字节数
}


/**********************************************************************/
/* Return the informational HTTP headers about a file. */
/* Parameters: the socket to print the headers on
 *             the name of the file */
/**********************************************************************/
/**********************************************************************/
/* 返回一个关于文件信息的HTTP头. */
/**********************************************************************/
void headers(int client, const char *filename)
{
 char buf[1024];
 (void)filename;  /* could use filename to determine file type */
//组装字符串发送出去
 strcpy(buf, "HTTP/1.0 200 OK\r\n");
 send(client, buf, strlen(buf), 0);
 strcpy(buf, SERVER_STRING);
 send(client, buf, strlen(buf), 0);
 sprintf(buf, "Content-Type: text/html\r\n");
 send(client, buf, strlen(buf), 0);
 strcpy(buf, "\r\n");
 send(client, buf, strlen(buf), 0);
}


/**********************************************************************/
/* 给出一个404错误 */
/**********************************************************************/
void not_found(int client)
{
 char buf[1024];
//组装发送
 sprintf(buf, "HTTP/1.0 404 NOT FOUND\r\n");
 send(client, buf, strlen(buf), 0);
 sprintf(buf, SERVER_STRING);
 send(client, buf, strlen(buf), 0);
 sprintf(buf, "Content-Type: text/html\r\n");
 send(client, buf, strlen(buf), 0);
 sprintf(buf, "\r\n");
 send(client, buf, strlen(buf), 0);
 sprintf(buf, "<HTML><TITLE>Not Found</TITLE>\r\n");
 send(client, buf, strlen(buf), 0);
 sprintf(buf, "<BODY><P>The server could not fulfill\r\n");
 send(client, buf, strlen(buf), 0);
 sprintf(buf, "your request because the resource specified\r\n");
 send(client, buf, strlen(buf), 0);
 sprintf(buf, "is unavailable or nonexistent.\r\n");
 send(client, buf, strlen(buf), 0);
 sprintf(buf, "</BODY></HTML>\r\n");
 send(client, buf, strlen(buf), 0);
}




/**********************************************************************/
/* 发送一个规则文件到客户端. 使用headers, 上报错误如果发生错误
/**********************************************************************/
void serve_file(int client, const char *filename)
{
 FILE *resource = NULL;
 int numchars = 1;
 char buf[1024];


 buf[0] = 'A'; buf[1] = '\0';
 while ((numchars > 0) && strcmp("\n", buf))  /* read & discard headers */
  numchars = get_line(client, buf, sizeof(buf));
 //打开文件
 resource = fopen(filename, "r");
 if (resource == NULL)
  not_found(client);
 else
 {//组织头部
  headers(client, filename);
  cat(client, resource);//传输文件到客户端
 }
 fclose(resource);
}




/**********************************************************************
 * 这个函数在一个指定的端口上启动一个监听web连接的处理。如果*port==0，那么
 * 就动态的申请一个端口号然后修改*port的值
 * 参数: 指向待连接端口号值的指针
 * 返回: socket 
 **********************************************************************/
int startup(u_short *port)
{
 int httpd = 0;
 struct sockaddr_in name;
 //建立socket
 httpd = socket(PF_INET, SOCK_STREAM, 0);
 if (httpd == -1)
  error_die("socket");
 memset(&name, 0, sizeof(name));
 //为绑定做准备后进行绑定
 name.sin_family = AF_INET;
 name.sin_port = htons(*port);
 name.sin_addr.s_addr = htonl(INADDR_ANY);
 if (bind(httpd, (struct sockaddr *)&name, sizeof(name)) < 0)
  error_die("bind");
 if (*port == 0)  /* if dynamically allocating a port */
 {//如果port传入的原值为0 ，那么动态的端口号进行回填
  int namelen = sizeof(name);
  //用于获取一个套接字的名字。它用于一个已捆绑或已连接套接字s，本地地址将被返回
  if (getsockname(httpd, (struct sockaddr *)&name, &namelen) == -1)
   error_die("getsockname");
  *port = ntohs(name.sin_port);//回填
 }
 if (listen(httpd, 5) < 0)
  error_die("listen");
 return(httpd);
}




/**********************************************************************/
/* 通知客户端，请求方法还不支持
 * 参数: 客户端socket */
/**********************************************************************/
void unimplemented(int client)
{
 char buf[1024];
//组装一个xml说明文件进行返回到client
 sprintf(buf, "HTTP/1.0 501 Method Not Implemented\r\n");
 send(client, buf, strlen(buf), 0);
 sprintf(buf, SERVER_STRING);
 send(client, buf, strlen(buf), 0);
 sprintf(buf, "Content-Type: text/html\r\n");
 send(client, buf, strlen(buf), 0);
 sprintf(buf, "\r\n");
 send(client, buf, strlen(buf), 0);
 sprintf(buf, "<HTML><HEAD><TITLE>Method Not Implemented\r\n");
 send(client, buf, strlen(buf), 0);
 sprintf(buf, "</TITLE></HEAD>\r\n");
 send(client, buf, strlen(buf), 0);
 sprintf(buf, "<BODY><P>HTTP request method not supported.\r\n");
 send(client, buf, strlen(buf), 0);
 sprintf(buf, "</BODY></HTML>\r\n");
 send(client, buf, strlen(buf), 0);
}


/**********************************************************************/


int main(void)
{
 int server_sock = -1;
 u_short port = 0;
 int client_sock = -1;
 struct sockaddr_in client_name;
 int client_name_len = sizeof(client_name);
 pthread_t newthread;
 //打开服务器socket等待连接
 server_sock = startup(&port);
 printf("httpd running on port %d\n", port);
 while (1)
 {
    //等待连接
	client_sock = accept(server_sock, (struct sockaddr *)&client_name, &client_name_len);
	if (client_sock == -1)
		error_die("accept");//如果连接失败那么就打印下错误然后退出
	//连接成功则为这次连接创建一个线程进行处理
	if (pthread_create(&newthread , NULL, accept_request, client_sock) != 0)
		perror("pthread_create");
 }
 close(server_sock);


 return(0);
}
```