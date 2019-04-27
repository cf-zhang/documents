//
// Created by MS on 2019/4/24.
//
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <queue>

using namespace std;

void swap(int &a, int &b) {
    int tmp = a;
    a = b;
    b = tmp;
}
template <typename T>
class Heap {
public:
    Heap(const vector<T> &array, bool (*compare)(T, T)) : compare_(compare) {
        if (array.size() > 0) {
            data_.insert(data_.begin(), -1);
            data_.insert(data_.end(), array.begin(), array.end());
            heapify();
        }
    }

    /**
     *     根据堆的性质, 只要保证部分有序即可, 即根节点大于左右节点的值.
     * 将数组抽象为一个完全二叉树, 所以只要从最后一个非叶子节点向前遍历每一个节点即可.
     * 如果当前节点比左右子树节点都大, 则已经是一个最大堆 否则将当前节点与左右节点较大的一个交换,
     * 并且交换过之后依然要递归的查看子节点是否满足堆的性质, 不满足再往下调整. 如此即可完成数组的堆化.
     */
    void heapify() {
        int node_to_be_disposal = floor((data_.size() - 1) / 2);
        while (node_to_be_disposal > 0) {
            adjustHeap(node_to_be_disposal);
            node_to_be_disposal--;
        }
    }

    void heapSort(vector<T> &array){
        int value;
        while (data_.size()>0){
            pop(value);
            array.push_back(value);
        }
    }
    /**
     * insert to end free space
     * adjust child element with parent elements by value
     * @param node  the element to insert
     */
    void push(T node) {
        data_.insert(data_.end(), node);
        int next_disposal = adjustUpward(data_.size() - 1);
        while (-1 != next_disposal) {
            next_disposal = adjustUpward(next_disposal);
        }
    }
    /**
     * get the optimal value
     * @return optimal value
     */
    int top() const {
        return data_[1];
    }
    /**
     *Heap只能删除根节点
     *删除Root
     *last element -> root element
     *then adjust from top to down; compare 2 child and select the bigger
     * @param value [out] the value poped
     * @return the poped value
     */
    T pop(T &value) {
        value = top();
        swap(data_[1], data_.back());
        data_.erase(data_.end()-1);

        adjustHeap(1);

        return value;
    }

    bool isEmpty()const{
        return data_.size() == 1;
    }
    void adjustHeap(int node_index) {
        if (node_index > (data_.size() - 1) / 2) {
            return;
        }
        int next_disposal = adjustDownward(node_index);
        if (-1 != next_disposal) {
            adjustHeap(next_disposal);
        }
    }

    int adjustUpward(int node_index) {
        int _index = -1;
        if (node_index > 1) {
            int parent_index = floor(node_index / 2);
            _index = compare_(data_[node_index], data_[parent_index]) ? parent_index : -1;
            if (compare_(data_[node_index], data_[_index])) {
                swap(data_[_index], data_[node_index]);
            } else {
                return -1;
            }
        }
        return _index;
    }

    int adjustDownward(int node_index) {
        int _index = -1;
        int left_child_index = node_index * 2;
        int right_child_index = node_index * 2 + 1;
        if(right_child_index < data_.size()){
            _index = compare_(data_[left_child_index], data_[right_child_index]) ? left_child_index : right_child_index;
        }else if(left_child_index < data_.size()){
            _index = left_child_index;
        } else{
            return -1;
        }
        if (compare_(data_[_index], data_[node_index])) {
            swap(data_[_index], data_[node_index]);
        }
        return _index;
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
    vector<T> data_;

    bool (*compare_)(T, T);
};

bool compare_greater(int a, int b) {
    return a > b;
}

bool compare_less(int a, int b) {
    return a < b;
}

int main(int argc, char *argv[]) {
    vector<int> array{3, 6, 2, 4, 1, 7, 8};

    Heap<int> heap(array, compare_greater);
    heap.push(9);
    int value;
    heap.pop(value);
//    vector<int> a;
//    heap.heapSort(a);
    cout << heap;
}