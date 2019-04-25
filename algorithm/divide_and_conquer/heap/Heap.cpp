//
// Created by MS on 2019/4/24.
//
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>

using namespace std;

void swap(int &a, int &b) {
    int tmp = a;
    a = b;
    b = tmp;
}

class Heap {
public:
    Heap(const vector<int> &array) {
        if (array.size() > 0) {
            data_.insert(data_.begin(), -1);
            data_.insert(data_.end(), array.begin(), array.end());
            heapify();
        }
    }

    //根据堆的性质, 只要保证部分有序即可, 即根节点大于左右节点的值.
    // 将数组抽象为一个完全二叉树, 所以只要从最后一个非叶子节点向前遍历每一个节点即可.
    // 如果当前节点比左右子树节点都大, 则已经是一个最大堆
    // , 否则将当前节点与左右节点较大的一个交换,
    // 并且交换过之后依然要递归的查看子节点是否满足堆的性质, 不满足再往下调整. 如此即可完成数组的堆化.
    void heapify() {
        int node_to_be_disposal = floor((data_.size() - 1) / 2);
        while (node_to_be_disposal > 0) {
            adjustHeap(node_to_be_disposal);
            node_to_be_disposal--;
        }
    }

    void adjustHeap(int node_index) {
        if (node_index > (data_.size() - 1) / 2) {
            return;
        }
        int next_disposal = adjust(node_index);
        if(next_disposal != -1){
            adjustHeap(next_disposal);
        }
    }

    int adjust(int node_index){
        int max_index = -1;
        int left_child_index = node_index * 2;
        int right_child_index = node_index * 2 + 1;
        max_index = data_[left_child_index] > data_[right_child_index] ? left_child_index : right_child_index;
        if(data_[max_index] > data_[node_index]){
            swap(data_[max_index],data_[node_index]);
        }
        return max_index;
    }

    friend ostream &operator<<(ostream &os, const Heap &heap) {
        stringstream ss;
        ss << "data_: [";
        for (int x = 0; x < heap.data_.size(); x++) {
            ss << heap.data_[x] << ", ";
        }
        ss << "]" << endl;
        os << ss.str();
        return os;
    }

private:
    vector<int> data_;
};

int main(int argc, char *argv[]) {
    vector<int> array{3, 6, 2, 4, 1, 7, 8};
    Heap heap(array);
    cout << heap;
}