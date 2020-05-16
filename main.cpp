#include <bits/stdc++.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <mutex>
#include <cstddef>
#include <algorithm>
#include <cmath>
#include "robin_map.h"
using tsl::robin_map;

using namespace std;
#define THREAD_COUNT 4
#define THREAD_COUNT_READ 4
#define PATCH_SIZE 1000
#define MAX_NODE 2000000
#define MAX_EDGE 2000000
#define MEM_INT_MAX 805306368
#define P3  30000
#define P   400
const unsigned int sizeTable[12] = {9, 99, 999, 9999, 99999, 999999, 9999999, 99999999, 999999999, 4294967295};
const char digits[256] =   //用于整数转字符串的查表
        "0001020304050607080910111213141516171819"
        "2021222324252627282930313233343536373839"
        "4041424344454647484950515253545556575859"
        "6061626364656667686970717273747576777879"
        "8081828384858687888990919293949596979899";
//TODO ***********************
struct Array {   //结果集合
    char l3[33 * 20000000 / THREAD_COUNT];
    char l4[44 * 20000000 / THREAD_COUNT];
    char l5[55 * 20000000 / THREAD_COUNT];
    char l6[66 * 20000000 / THREAD_COUNT];
    char l7[77 * 20000000 / THREAD_COUNT];
    char l8[88 * 20000000 / THREAD_COUNT];
} resultArray[THREAD_COUNT];
//TODO ***********************
struct Edge {    //边集合
    unsigned int to;    //unsigned int to:24
    unsigned int emtpy;
    long weight;    //long weight:40
};
struct Node {      //节点邻接表的区间
    unsigned int l;
    unsigned int r;
};
struct patchInfo {   //每一个均衡任务片的结果指向和长度
    char *address[6];
    unsigned int len[8];
} patchtoThread[MAX_NODE];
struct PathP3 {      //反向三层路径的两个中间点及起始边权值
    unsigned int midPoint1;
    unsigned int midPoint2;
    long weightStart;
};
struct MidPoint3 {
    PathP3 path[P];
};
unsigned int memPoolInt[MEM_INT_MAX];    //简单内存池，手动取址
long *pMap[THREAD_COUNT][5];       //用于P3查表的标志位
char Comma[MAX_NODE][12];        //逗号复用
unsigned int circle[(THREAD_COUNT / 4 + 1) * 4];   //结果
Node *G;   //正向图
Node *Ginv;  //反向图
Edge *neighborsTable;  //正向邻接表
Edge *neighborsTableInv;  //反向邻接表
MidPoint3 *midPoint3[THREAD_COUNT];  //反向三层搜索结果
unsigned int *inDegree[THREAD_COUNT_READ];    //入度
unsigned int *outDegree[THREAD_COUNT_READ];   //出度
unsigned int nodeIdxMulti[THREAD_COUNT_READ][3];  //读入数据时的多线程变量
unsigned int nodeCnt, circleCnt;  //节点数，环数
unsigned int patchCount, processedId;  //任务片数，任务执行进度数
unsigned int memBegin, memEnd = MEM_INT_MAX;    //取址时的位置
mutex mtx;
/**
 * 整数转字符串
 * @param value
 * @param dst
 */
void intToString(unsigned int value, char *dst) {
    unsigned int length = 0;
    for (;; ++length) {
        if (value <= sizeTable[length]) {
            ++length;
            break;
        }
    }
    unsigned int next = length - 1;
    while (value >= 100) {
        unsigned int i = (value % 100) * 2;
        value /= 100;
        dst[next] = digits[i + 1];
        dst[next - 1] = digits[i];
        next -= 2;
    }
    if (value < 10) {
        dst[next] = '0' + value;
    } else {
        unsigned int i = value * 2;
        dst[next] = digits[i + 1];
        dst[next - 1] = digits[i];
    }
    dst[11] = length + 1;
    dst[length] = ',';
}
/**
 * 加边
 * @param u
 * @param v
 * @param m
 */
void addEdge(unsigned int u, unsigned int v, long m) {
    if (!v) {   //过滤含有出度为0的点的边
        outDegree[0][u]--;
        return;
    }
    neighborsTable[G[u].r].to = v;
    neighborsTable[G[u].r].weight = m;
    ++G[u].r;
}
/**
 * 多线程映射
 * @param threadId
 * @param nodeData
 * @param id_Index
 */
void idToIndex(unsigned int threadId, unsigned int *nodeData, robin_map<unsigned int, unsigned int> id_Index) {
    //id_to_index
    for (unsigned int idx = 0; idx < nodeIdxMulti[threadId][0]; idx += 2) {
        nodeData[idx] = id_Index[nodeData[idx]];
        nodeData[idx + 1] = id_Index[nodeData[idx + 1]];
        ++inDegree[threadId][nodeData[idx + 1]];
        ++outDegree[threadId][nodeData[idx]];
    }
}
/**
 * 归并排序后的结果
 * @param A
 * @param m
 * @param B
 * @param n
 */
void merge(unsigned int *A, unsigned int m, const unsigned int *B, unsigned int n) {
    int pa = m - 1, pb = n - 1;
    unsigned int tail = m + n - 1;
    while (pa >= 0 && pb >= 0) {
        if (A[pa] > B[pb])
            A[tail--] = A[pa--];
        else
            A[tail--] = B[pb--];
    }
    while (pb >= 0) {
        A[tail--] = B[pb--];
    }
}
/**
 * 读入数据并排序
 * @param buf
 * @param end
 * @param nodeData
 * @param moneyData
 * @param index_Id
 * @param threadId
 */
void sortThread(char *buf, const char *end, unsigned int *nodeData, long *moneyData,
                unsigned int *index_Id, unsigned int threadId) {
    unsigned int nodeIdx = 0, nodeIdx2 = 0, nodeIdx3 = 0;
    while (buf < end) {
        unsigned int from = 0, to = 0;
        long money = 0;
        while ((*buf) & 0x10) {
            from *= 10;
            from += (*buf) & 0x0f;
            buf++;
        }
        ++buf;
        while ((*buf) & 0x10) {
            to *= 10;
            to += (*buf) & 0x0f;
            buf++;
        }
        ++buf;
        unsigned int fix = 2;
        while ((*buf) & 0x10) {
            money *= 10;
            money += (*buf) & 0x0f;
            buf++;
        }
        if((*buf) == '.'){
            buf++;
            while ((*buf) & 0x10){
                money *= 10;
                money += (*buf) & 0x0f;
                buf++;fix--;
            }
        }
        while(fix--!=0){
            money*=10;
        }
        if (*buf == '\r') {
            ++buf;
        }
        ++buf;
        if (money == 0) continue;
        nodeData[nodeIdx++] = from;
        nodeData[nodeIdx++] = to;
        moneyData[nodeIdx2++] = money;
        index_Id[nodeIdx3++] = from;
    }
    sort(index_Id, index_Id + nodeIdx3);
    nodeIdxMulti[threadId][0] = nodeIdx;
    nodeIdxMulti[threadId][1] = nodeIdx2;
    nodeIdxMulti[threadId][2] = nodeIdx3;
}
/**
 * 读入数据
 * @param inputFilePath
 */
void readData(char *inputFilePath) {
    //分四份
    unsigned int *nodeData[THREAD_COUNT_READ], *index_Id[THREAD_COUNT_READ];
    long  *moneyData[THREAD_COUNT_READ];
    for (unsigned int i = 0; i < THREAD_COUNT_READ; i++) {
        memEnd -= 2 * MAX_EDGE;
        nodeData[i] = &memPoolInt[memEnd];  //size:2*MAX_EDGE
    }
    for (unsigned int i = 0; i < THREAD_COUNT_READ; i++) {
        memEnd -= MAX_EDGE;
        moneyData[i] = (long *) &memPoolInt[memEnd];  //size:MAX_EDGE
    }
    /***********************************************************************************************************/
    for (unsigned int i = 0; i < THREAD_COUNT_READ; i++) {
        memEnd -= 2 * MAX_EDGE;
        index_Id[i] = &memPoolInt[memEnd];  //size:2*MAX_EDGE
    }
    robin_map<unsigned int, unsigned int> id_Index;
    //多线程mmap+排序
    int fd = open(inputFilePath, O_RDONLY);
    int fileLen = lseek(fd, 0, SEEK_END);
    char *buf = (char *) mmap(NULL, fileLen, PROT_READ, MAP_PRIVATE, fd, 0);
    close(fd);
    char *start[] = {buf, buf + fileLen / THREAD_COUNT_READ, buf + fileLen / THREAD_COUNT_READ * 2,
                     buf + fileLen / THREAD_COUNT_READ * 3, buf + fileLen};
    for (unsigned int i = 1; i < THREAD_COUNT_READ; i++) {
        while (*(start[i]++) != '\n');
    }
    thread t[THREAD_COUNT_READ];
    for (unsigned int i = 0; i < THREAD_COUNT_READ; i++) {
        //TODO 传参太多了，不知道nodeData这些改全局会不会更好。
        t[i] = thread(sortThread, start[i], start[i + 1], nodeData[i], moneyData[i], index_Id[i], i);
    }
    for (unsigned int i = 0; i < THREAD_COUNT_READ; i++)
        t[i].join();
    //归并排序结果
    merge(index_Id[0], nodeIdxMulti[0][2], index_Id[1], nodeIdxMulti[1][2]);
    merge(index_Id[2], nodeIdxMulti[2][2], index_Id[3], nodeIdxMulti[3][2]);
    merge(index_Id[0], nodeIdxMulti[0][2] + nodeIdxMulti[1][2], index_Id[2], nodeIdxMulti[2][2] + nodeIdxMulti[3][2]);
    unsigned int nodeIdx3 = 0;
    for (unsigned int i = 0; i < THREAD_COUNT_READ; i++)
        nodeIdx3 += nodeIdxMulti[i][2];
    /************************************映射****************************************/
    nodeCnt = unique(index_Id[0], index_Id[0] + nodeIdx3) - index_Id[0];
    id_Index.rehash(1024 * 1024);
    id_Index.reserve(nodeCnt);
    for (unsigned int idx = 0; idx < nodeCnt; idx++) {
        id_Index[index_Id[0][idx]] = idx + 1;
        intToString(index_Id[0][idx], Comma[idx + 1]);
    }
    for (unsigned int i = 0; i < THREAD_COUNT_READ; ++i) {
        memEnd -= (nodeCnt + 1);
        inDegree[i] = &memPoolInt[memEnd];
    }
    for (unsigned int i = 0; i < THREAD_COUNT_READ; ++i) {
        memEnd -= (nodeCnt + 1);
        outDegree[i] = &memPoolInt[memEnd];
    }
    for (unsigned int i = 0; i < THREAD_COUNT_READ; i++)
        t[i] = thread(idToIndex, i, nodeData[i], id_Index);
    for (unsigned int i = 0; i < THREAD_COUNT_READ; i++)
        t[i].join();
    //综合出入度
    for (unsigned int idx = 1; idx < nodeCnt; idx++) {
        outDegree[0][idx] += outDegree[1][idx] + outDegree[2][idx] + outDegree[3][idx];
        inDegree[0][idx] += inDegree[1][idx] + inDegree[2][idx] + inDegree[3][idx];
    }
    /************************************开辟空间****************************************/
    neighborsTable = (Edge *) &memPoolInt[memBegin];
    memBegin += sizeof(Edge) * (nodeIdx3 + 1) / sizeof(unsigned int);
    neighborsTableInv = (Edge *) &memPoolInt[memBegin];
    memBegin += sizeof(Edge) * (nodeIdx3 + 1) / sizeof(unsigned int);
    G = (Node *) &memPoolInt[memBegin];
    memBegin += sizeof(Node) * (nodeCnt + 1) / sizeof(unsigned int);
    Ginv = (Node *) &memPoolInt[memBegin];
    memBegin += sizeof(Node) * (nodeCnt + 1) / sizeof(unsigned int);
    G[1].l = 0;
    Ginv[1].l = 0;
    G[1].r = 0;
    Ginv[1].r = 0;
    for (unsigned int idx = 2; idx <= nodeCnt; idx++) {
        G[idx].l = G[idx - 1].l + outDegree[0][idx - 1];
        G[idx].r = G[idx].l;
        Ginv[idx].l = Ginv[idx - 1].l + inDegree[0][idx - 1];
        Ginv[idx].r = Ginv[idx].l;
    }
    /************************************构图***************************************/
    for (unsigned int i = 0; i < THREAD_COUNT_READ; i++) {
        for (unsigned int idx = 0, idx2 = 0; idx < nodeIdxMulti[i][0]; idx += 2) {
            addEdge(nodeData[i][idx], nodeData[i][idx + 1], moneyData[i][idx2++]);
        }
    }
    patchCount = nodeCnt / PATCH_SIZE;
}

bool cmp(Edge a, Edge b) {
    return a.to < b.to;
}

bool cmp2(PathP3 a, PathP3 b) {
    if (a.midPoint1 != b.midPoint1)
        return a.midPoint1 < b.midPoint1;
    else
        return a.midPoint2 < b.midPoint2;
}
/**
 * 排序邻接表
 * @param start
 * @param len
 */
void sortGraphThread(unsigned int start, unsigned int len) {
    unsigned int idx;
    for (idx = start; idx < start + len; ++idx) {
        sort(&neighborsTable[G[idx].l], &neighborsTable[G[idx].r], cmp);
    }
}
/**
 * 排序邻接表，并根据排序后的正向图遍历建立反向图，比在读数据时建立反向图快
 */
void sortGraph() {
    thread t[4];
    unsigned int patch = nodeCnt / 4;
    for (unsigned int i = 0; i < 3; i++)
        t[i] = thread(sortGraphThread, patch * i + 1, patch);
    t[3] = thread(sortGraphThread, patch * (3) + 1, nodeCnt - patch * 3);
    for (unsigned int i = 0; i < 4; i++)
        t[i].join();
    for (unsigned int idx = 1; idx <= nodeCnt; idx++) {
        for (unsigned int idx2 = G[idx].l; idx2 < G[idx].r; ++idx2) {
            unsigned int &node = neighborsTable[idx2].to;
            neighborsTableInv[Ginv[node].r].to = idx;
            neighborsTableInv[Ginv[node].r].weight = neighborsTable[idx2].weight;
            ++Ginv[node].r;
        }
    }
}
/**
 * 排序邻接表，变量取址
 */
void preProcessing() {
    sortGraph();
    for (unsigned int i = 0; i < THREAD_COUNT; ++i) {
        pMap[i][0] = (long *)&memPoolInt[memBegin];
        memBegin += (nodeCnt + 1)*2;
        pMap[i][1] = (long *)&memPoolInt[memBegin];
        memBegin += (nodeCnt + 1)*2;
        pMap[i][2] = (long *)&memPoolInt[memBegin];
        memBegin += (nodeCnt + 1)*2;
        pMap[i][3] = (long *)&memPoolInt[memBegin];
        memBegin += (nodeCnt + 1)*2;
        pMap[i][4] = (long *)&memPoolInt[memBegin];
        memBegin += (nodeCnt + 1)*2;
        midPoint3[i] = (MidPoint3 *) &memPoolInt[memBegin];
        memBegin += P3 * sizeof(MidPoint3);
    }
}
/**
 * 反向三层的P3路径搜索，复赛增加了第四层打标记，即5+3记录三层路径，第四层打标记
 * @param begin 
 * @param threadId 
 * @return 
 */
unsigned int smallMapBuild(unsigned int begin, unsigned int threadId) {
    unsigned int idx1, idx2, idx3,idx4;
    unsigned int p1, p2, p3,p4;
    unsigned int add, addressP3 = 0;
    unsigned int idP3[P3];
    long X, Y, Z,K;
    unsigned int p, num;
    //P2
    for (idx1 = Ginv[begin].l; idx1 < Ginv[begin].r; ++idx1) {
        p1 = neighborsTableInv[idx1].to;
        Y = neighborsTableInv[idx1].weight;
        pMap[threadId][3][p1] = Y;
        if (p1 < begin) continue;
        for (idx2 = Ginv[p1].l; idx2 < Ginv[p1].r; ++idx2) {
            p2 = neighborsTableInv[idx2].to;
            X = neighborsTableInv[idx2].weight;
            if (p2 <= begin || X >  5 * Y || Y >  3 * X) continue;
            for (idx3 = Ginv[p2].l; idx3 < Ginv[p2].r; idx3++) {
                p3 = neighborsTableInv[idx3].to;
                if (p3 < begin) continue;
                if (p3 == begin) {
                    Z = neighborsTableInv[idx3].weight;
                    if (Y >  5 * Z || Z >  3 * Y) continue;
                } else {
                    if (p1 == p3) continue;
                    Z = neighborsTableInv[idx3].weight;
                }
                if (Z >  5 * X || X >  3 * Z) continue;
                if (pMap[threadId][0][p3] != begin) {
                    idP3[addressP3] = p3;
                    pMap[threadId][0][p3] = begin;
                    pMap[threadId][1][p3] = addressP3;
                    pMap[threadId][2][p3] = 0;
                    ++addressP3;
                }
                add = pMap[threadId][1][p3];
                midPoint3[threadId][add].path[pMap[threadId][2][p3]].midPoint1 = p2;
                midPoint3[threadId][add].path[pMap[threadId][2][p3]].midPoint2 = p1;
                midPoint3[threadId][add].path[pMap[threadId][2][p3]].weightStart = Z;
                // midPoint3[threadId][add].path[pMap[threadId][2][p3]].weightEnd = Y;
                ++pMap[threadId][2][p3];
                for(idx4=Ginv[p3].l;idx4<Ginv[p3].r;idx4++){
                    p4 = neighborsTableInv[idx4].to;
                    if (p3 <= begin) continue;
                    K = neighborsTableInv[idx4].weight;
                    if (K >  5 * Z || Z >  3 * K) continue;
                    pMap[threadId][4][p4] = begin;
                }
            }
        }
    }
    if (addressP3 == 0)
        return 0;
    for (idx1 = 0; idx1 < addressP3; idx1++) {
        p = idP3[idx1];
        add = pMap[threadId][1][p];
        num = pMap[threadId][2][p];
        if (num > 1)
            sort(midPoint3[threadId][add].path, midPoint3[threadId][add].path + num, cmp2);
    }
    return addressP3;
}
/**
 * dfs循环展开搜索结果
 * @param threadId 
 */
void searchResult(unsigned int threadId) {
    char *resultPosition[6]{};
    char *resultPositionLast[6]{resultArray[threadId].l3, resultArray[threadId].l4, resultArray[threadId].l5,
                                resultArray[threadId].l6, resultArray[threadId].l7, resultArray[threadId].l8};
    unsigned int p1;
    unsigned int circntArray = 0;
    unsigned int address, numPath, idx;
    unsigned int idx2, idx3, idx4, idx5, idx6;
    while (true) {
        //负载均衡
        mtx.lock();
        unsigned int patchId = processedId++;
        mtx.unlock();
        if (patchId >= patchCount) {
            circle[threadId] = circntArray;
            return;
        }
        unsigned int i, startId = patchId * PATCH_SIZE + 1, endId;
        if (patchId == patchCount - 1)
            endId = nodeCnt + 1;
        else
            endId = (patchId + 1) * PATCH_SIZE + 1;
        for (i = 0; i < 6; i++)
            resultPosition[i] = resultPositionLast[i];
        //***********搜索**********************
        //depth  1***************
        for (p1 = startId; p1 < endId; ++p1) {
            if (!inDegree[0][p1] || !outDegree[0][p1]) continue;
            if (!smallMapBuild(p1, threadId)) continue;
            //length 3
            if (pMap[threadId][0][p1] == p1) {
                address = pMap[threadId][1][p1];
                numPath = pMap[threadId][2][p1];
                for (idx = 0; idx < numPath; ++idx) {
                    PathP3 &path = midPoint3[threadId][address].path[idx];
                    memcpy(resultPosition[0], Comma[p1], 16);
                    resultPosition[0] += Comma[p1][11];
                    memcpy(resultPosition[0], Comma[path.midPoint1], 16);
                    resultPosition[0] += Comma[path.midPoint1][11];
                    memcpy(resultPosition[0], Comma[path.midPoint2], 16);
                    resultPosition[0] += Comma[path.midPoint2][11];
                    *(resultPosition[0] - 1) = '\n';
                    ++circntArray;
                }
            }
            // depth2*******************************
            for (idx2 = G[p1].l; idx2 < G[p1].r; idx2++) {
                long &m12 = neighborsTable[idx2].weight;
                unsigned int &p2 = neighborsTable[idx2].to;
                if (p2 <= p1) continue;
                //  length 4
                if (pMap[threadId][0][p2] == p1) {
                    address = pMap[threadId][1][p2];
                    numPath = pMap[threadId][2][p2];
                    for (idx = 0; idx < numPath; ++idx) {
                        PathP3 &path = midPoint3[threadId][address].path[idx];
                        long &wBegin = path.weightStart;
                        long &wEnd = pMap[threadId][3][path.midPoint2];
                        if (m12 >  5 * wBegin || wBegin >  3 * m12 || wEnd >  5 * m12 ||
                            m12 >  3 * wEnd)
                            continue;
                        memcpy(resultPosition[1], Comma[p1], 16);
                        resultPosition[1] += Comma[p1][11];
                        memcpy(resultPosition[1], Comma[p2], 16);
                        resultPosition[1] += Comma[p2][11];
                        memcpy(resultPosition[1], Comma[path.midPoint1], 16);
                        resultPosition[1] += Comma[path.midPoint1][11];
                        memcpy(resultPosition[1], Comma[path.midPoint2], 16);
                        resultPosition[1] += Comma[path.midPoint2][11];
                        *(resultPosition[1] - 1) = '\n';
                        ++circntArray;
                    }
                }
                //   depth 3******************************
                for (idx3 = G[p2].l; idx3 < G[p2].r; idx3++) {
                    unsigned int &p3 = neighborsTable[idx3].to;
                    long &m23 = neighborsTable[idx3].weight;
                    if (p3 <= p1) continue;
                    if (m12 >  5 * m23 || m23 >  3 * m12) continue;
                    //length 5
                    if (pMap[threadId][0][p3] == p1) {
                        address = pMap[threadId][1][p3];
                        numPath = pMap[threadId][2][p3];
                        for (idx = 0; idx < numPath; ++idx) {
                            PathP3 &path = midPoint3[threadId][address].path[idx];
                            long &wBegin = path.weightStart;
                            long &wEnd = pMap[threadId][3][path.midPoint2];
                            if (m23 >  5 * wBegin || wBegin >  3 * m23 || wEnd >  5 * m12 ||
                                m12 >  3 * wEnd || path.midPoint1 == p2 || path.midPoint2 == p2)
                                continue;
                            memcpy(resultPosition[2], Comma[p1], 16);
                            resultPosition[2] += Comma[p1][11];
                            memcpy(resultPosition[2], Comma[p2], 16);
                            resultPosition[2] += Comma[p2][11];
                            memcpy(resultPosition[2], Comma[p3], 16);
                            resultPosition[2] += Comma[p3][11];
                            memcpy(resultPosition[2], Comma[path.midPoint1], 16);
                            resultPosition[2] += Comma[path.midPoint1][11];
                            memcpy(resultPosition[2], Comma[path.midPoint2], 16);
                            resultPosition[2] += Comma[path.midPoint2][11];
                            *(resultPosition[2] - 1) = '\n';
                            ++circntArray;
                        }
                    }
                    //  depth 4******************************
                    for (idx4 = G[p3].l; idx4 < G[p3].r; idx4++) {
                        unsigned int &p4 = neighborsTable[idx4].to;
                        long &m34 = neighborsTable[idx4].weight;
                        if (p4 <= p1) continue;
                        if (p4 == p2 || m23 >  5 * m34 || m34 >  3 * m23) continue;
                        //length 6
                        if (pMap[threadId][0][p4] == p1) {
                            address = pMap[threadId][1][p4];
                            numPath = pMap[threadId][2][p4];
                            for (idx = 0; idx < numPath; ++idx) {
                                PathP3 &path = midPoint3[threadId][address].path[idx];
                                long &wBegin = path.weightStart;
                                long &wEnd = pMap[threadId][3][path.midPoint2];
                                if (m34 >  5 * wBegin || wBegin >  3 * m34 || wEnd >  5 * m12 ||
                                    m12 >  3 * wEnd || path.midPoint1 == p2 || path.midPoint1 == p3 ||
                                    path.midPoint2 == p2 || path.midPoint2 == p3)
                                    continue;
                                memcpy(resultPosition[3], Comma[p1], 16);
                                resultPosition[3] += Comma[p1][11];
                                memcpy(resultPosition[3], Comma[p2], 16);
                                resultPosition[3] += Comma[p2][11];
                                memcpy(resultPosition[3], Comma[p3], 16);
                                resultPosition[3] += Comma[p3][11];
                                memcpy(resultPosition[3], Comma[p4], 16);
                                resultPosition[3] += Comma[p4][11];
                                memcpy(resultPosition[3], Comma[path.midPoint1], 16);
                                resultPosition[3] += Comma[path.midPoint1][11];
                                memcpy(resultPosition[3], Comma[path.midPoint2], 16);
                                resultPosition[3] += Comma[path.midPoint2][11];
                                *(resultPosition[3] - 1) = '\n';
                                ++circntArray;
                            }
                        }
                        //depth  5*******************************
                        for (idx5 = G[p4].l; idx5 < G[p4].r; idx5++) {
                            unsigned int &p5 = neighborsTable[idx5].to;
                            long &m45 = neighborsTable[idx5].weight;
                            if (p5 <= p1) continue;
                            if (p5 == p2 || p5 == p3 || m34 >  5 * m45 || m45 >  3 * m34) continue;
                            //length 7
                            if (pMap[threadId][0][p5] == p1) {
                                address = pMap[threadId][1][p5];
                                numPath = pMap[threadId][2][p5];
                                for (idx = 0; idx < numPath; ++idx) {
                                    PathP3 &path = midPoint3[threadId][address].path[idx];
                                    long &wBegin = path.weightStart;
                                    long &wEnd = pMap[threadId][3][path.midPoint2];
                                    if (m45 >  5 * wBegin || wBegin >  3 * m45 || wEnd >  5 * m12 ||
                                        m12 >  3 * wEnd || path.midPoint1 == p2 || path.midPoint1 == p3 ||
                                        path.midPoint1 == p4 ||path.midPoint2 == p2 || path.midPoint2 == p3 || path.midPoint2 == p4)
                                        continue;
                                    memcpy(resultPosition[4], Comma[p1], 16);
                                    resultPosition[4] += Comma[p1][11];
                                    memcpy(resultPosition[4], Comma[p2], 16);
                                    resultPosition[4] += Comma[p2][11];
                                    memcpy(resultPosition[4], Comma[p3], 16);
                                    resultPosition[4] += Comma[p3][11];
                                    memcpy(resultPosition[4], Comma[p4], 16);
                                    resultPosition[4] += Comma[p4][11];
                                    memcpy(resultPosition[4], Comma[p5], 16);
                                    resultPosition[4] += Comma[p5][11];
                                    memcpy(resultPosition[4], Comma[path.midPoint1], 16);
                                    resultPosition[4] += Comma[path.midPoint1][11];
                                    memcpy(resultPosition[4], Comma[path.midPoint2], 16);
                                    resultPosition[4] += Comma[path.midPoint2][11];
                                    *(resultPosition[4] - 1) = '\n';
                                    ++circntArray;
                                }
                            }
                            if(pMap[threadId][4][p5] != p1) continue;
                            for (idx6 = G[p5].l; idx6 < G[p5].r; idx6++) {
                                unsigned int &p6 = neighborsTable[idx6].to;
                                long &m56 = neighborsTable[idx6].weight;
                                if (p6 <= p1) continue;
                                if (pMap[threadId][0][p6] != p1) continue;
                                if (p6 == p2 || p6 == p3 || p6 == p4 || m45 >  5 * m56 ||m56 >  3 * m45)
                                    continue;
                                address = pMap[threadId][1][p6];
                                numPath = pMap[threadId][2][p6];
                                for (idx = 0; idx < numPath; ++idx) {
                                    PathP3 &path = midPoint3[threadId][address].path[idx];
                                    long &wBegin = path.weightStart;
                                    long &wEnd = pMap[threadId][3][path.midPoint2];
                                    if (m56 >  5 * wBegin || wBegin >  3 * m56 || wEnd >  5 * m12 ||
                                        m12 >  3 * wEnd || path.midPoint1 == p2 || path.midPoint1 == p3 ||
                                        path.midPoint1 == p4 || path.midPoint1 == p5
                                        || path.midPoint2 == p2 || path.midPoint2 == p3 || path.midPoint2 == p4 ||
                                        path.midPoint2 == p5)
                                        continue;
                                    memcpy(resultPosition[5], Comma[p1], 16);
                                    resultPosition[5] += Comma[p1][11];
                                    memcpy(resultPosition[5], Comma[p2], 16);
                                    resultPosition[5] += Comma[p2][11];
                                    memcpy(resultPosition[5], Comma[p3], 16);
                                    resultPosition[5] += Comma[p3][11];
                                    memcpy(resultPosition[5], Comma[p4], 16);
                                    resultPosition[5] += Comma[p4][11];
                                    memcpy(resultPosition[5], Comma[p5], 16);
                                    resultPosition[5] += Comma[p5][11];
                                    memcpy(resultPosition[5], Comma[p6], 16);
                                    resultPosition[5] += Comma[p6][11];
                                    memcpy(resultPosition[5], Comma[path.midPoint1], 16);
                                    resultPosition[5] += Comma[path.midPoint1][11];
                                    memcpy(resultPosition[5], Comma[path.midPoint2], 16);
                                    resultPosition[5] += Comma[path.midPoint2][11];
                                    *(resultPosition[5] - 1) = '\n';
                                    ++circntArray;
                                }
                            }
                        }
                    }
                }
            }
        }
        for (i = 0; i < 6; i++) {
            patchtoThread[patchId].address[i] = resultPositionLast[i];
            patchtoThread[patchId].len[i] = resultPosition[i] - resultPositionLast[i];
            resultPositionLast[i] += (patchtoThread[patchId].len[i] / 16 + 1) * 16;
        }
    }
}
/**
 * 写结果到文件
 * @param outputFilePath 
 */
static void writeResult(char *outputFilePath) {
    for (unsigned int t = 0; t < THREAD_COUNT; ++t) {
        circleCnt += circle[t];
    }
    char pathCnt[16];
    intToString(circleCnt, pathCnt);
    pathCnt[pathCnt[11] - 1] = '\n';
    int file_fd = open(outputFilePath, O_WRONLY | O_CREAT, 00666);
    write(file_fd, pathCnt, pathCnt[11]);
    for (unsigned int k = 0; k < 6; ++k) {
        for (unsigned int patch = 0; patch < patchCount; ++patch) {
            if (patchtoThread[patch].len[k])
                write(file_fd, patchtoThread[patch].address[k], patchtoThread[patch].len[k]);
        }
    }
}

int main() {
    nice(-20);
//    char *inputFilePath = (char *) "/home/tt.txt";
//    char *outputFilePath = (char *) "/home/myResult.txt";
    char *inputFilePath = (char *) "/data/test_data.txt";
    char *outputFilePath = (char *) "/projects/student/result.txt";
    readData(inputFilePath);
    preProcessing();
    std::thread threads[THREAD_COUNT - 1];
    int t;
    for (t = 0; t < THREAD_COUNT - 1; t++) {
        threads[t] = std::thread(searchResult, t);
    }
    searchResult(t);
    for (t = 0; t < THREAD_COUNT - 1; t++) {
        threads[t].join();
    }
    writeResult(outputFilePath);
    cout << "result:" << circleCnt << endl;
    exit(0);
}