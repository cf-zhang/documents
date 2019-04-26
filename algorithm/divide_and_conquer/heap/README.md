### Array Representation of Binary Tree
* if a node is at index --- i
* its left child is at  --- 2*i
* its right child is at --- 2*i+1
* its parent is at      --- ⌊i/2⌋ 
* 当缺少某些节点时，需要留白处理机， i从1开始计算
### Complete Binary Tree
* Full Binary: 树的高度容许范围内不可再放下任何一个节点（最多2^(n+1)-1个节点），如果增加一个节点，则会导致树长高一层
* Complete Binary： 在一维数组中表述时，节点与节点之间不可再有留白或者gaps； 高度为log n
* Full --> Complete; Complete may not Full;
#### Heap
* is Complete Binary Tree
* MaxHeap: parent > descendants
* MinHeap: parent < descendants
#### Insert & Delete
Insert
* insert to end free space
* adjust child element with parent elements by value
* O(1) --> O(log n)

Delete
* Heap只能删除根节点
* 删除Root
* last element -> root element
* then adjust from top to down; compare 2 child and select the bigger
* O(log n)
Note: 删除一个节点放到最后留白的位置，迭代删完所有的节点后，会得到一个有序序列，也就是Heap-Sort
#### Heap Sort
* 创建一个Heap 通过一个接一个的插入来实现 O(log n)
* delete 一个接一个的来得到有序序列 O(log n)
* 2*log n---->O(log n)

#### Heapify
* 根据堆的性质, 只要保证部分有序即可, 即根节点大于左右节点的值. 将数组抽象为一个完全二叉树, 所以只要从最后一个非叶子节点向前遍历每一个节点即可. 如果当前节点比左右子树节点都大, 则已经是一个最大堆, 否则将当前节点与左右节点较大的一个交换, 并且交换过之后依然要递归的查看子节点是否满足堆的性质, 不满足再往下调整. 如此即可完成数组的堆化.


#### Priority Queue
* 如果是数字越小优先级越高，则建立一个MinHeap
* 如果数字越大优先级越高，则建立一个MaxHeap
