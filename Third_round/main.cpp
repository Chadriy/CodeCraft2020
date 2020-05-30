#include <bits/stdc++.h>
#include <ext/pb_ds/hash_policy.hpp>
#include <ext/pb_ds/priority_queue.hpp>
#include <fcntl.h>
#include <unistd.h>

using namespace std;
#include <sys/mman.h>
typedef uint32_t uint32;
typedef uint16_t uint16;
#define MAXE  2500000
#define MAXN  1000000
#define THREAD_COUNT 12
#define THREAD_COUNT_READ 8
#define PATCH_COUNT 100000

/**
 * ��дvector
 * @tparam T
 * @tparam Alloc
 */
template<class T, class Alloc=std::allocator<T>>
class ReservedVector {
public:
    typedef T value_type;
    typedef value_type *iterator;
    typedef const value_type *const_iterator;
    typedef value_type &reference;
    typedef const T &const_reference;
    typedef size_t size_type;
    typedef ptrdiff_t difference_type; //distance
protected:
    std::allocator<value_type> _alloc;
    iterator _start;
    iterator _end;
    iterator _end_of_storage;
    uint32 tail;
    value_type *data;
public:
    ReservedVector() {
        data = (T *) malloc(100000 * sizeof(T));
        _start = data;
        _end = _start;
        tail = 0;
    }

    ReservedVector &operator=(const ReservedVector &rhs) {
        if (this == &rhs)
            return *this;
        size_type n = rhs.cend() - rhs.cbegin();
        _start = _alloc.allocate(n);
        _end = _end_of_storage = std::uninitialized_copy(rhs.cbegin(), rhs.cend(), _start);
    }

    ~ReservedVector() {
        if (_start) {
            iterator it(_end);
            while (it != _start)
                _alloc.destroy(--it);
        }
        _alloc.deallocate(_start, _end_of_storage - _start);
        _start = _end_of_storage = _end = NULL;
    }

    iterator begin() { return _start; }

    iterator end() { return _end; }

    const_iterator cbegin() const { return _start; }

    const_iterator cend() const { return _end; }

    size_type size() { return size_type(_end - _start); }
    size_type capacity() { return size_type(_end - _start); }

    inline bool empty() const { return _start == _end; }

    void swap(ReservedVector &other) {
        std::swap(_start, other._start);
        std::swap(_end, other._end);
        std::swap(_end_of_storage, other._end_of_storage);
    }

    reference front() { return *_start; }

    const_reference front() const { return *_start; }

    reference back() { return *(end() - 1); }

    reference operator[](size_type n) { return *(begin() + n); }

    const_reference top() {
        if (!empty())
            return data[tail];
        else return (T) NULL;
    }

    inline void push_back(const T &value) {
        data[tail++] = value;
        ++_end;
    }

    inline void pop_back() {
        --tail;
        --_end;
    }

    void insert(iterator pos, const value_type &elem) {
        if (pos > end()) {
            *pos = elem;
            _end = pos + 1;
            return;
        }
        iterator p = _end;
        while (p != _start && p != pos) {
            *p = *(p - 1);
        }
        *pos = elem;
        ++_end;
    }
};
/**
 * �������ݱ�Ȩ������65536��ֱ����short�������룬������������10S
 */
struct Edge {
    uint32 to;
    uint16 w;
    uint16 n;
};
struct  Result{
    double bc;  //�н��м���
    double delta;
};
/**
 * �߳�����
 */
struct ThreadData {
    uint16 sigma[MAXN];  //���·������
    Result result[MAXN];  //���������
    uint32 P[MAXN][150];   //ǰ������
    uint8_t PIndex[MAXN];  //ǰ������ߴ��¼
    uint32 S[MAXN];    //��ջ
    priority_queue<uint16, ReservedVector<uint16>, greater<uint16>> Q_32;
    vector<uint32> noHashIDList[4000];  //������4000��dis������ӳ��
    unordered_map<uint16, vector<uint32>> disMap_32;   //����4000��dis��hashӳ��
    //��ȡӳ���ID vector����
    inline vector<uint32> *getIDList_16(const uint16 &distance) {
        if (distance < 4000) {
            return &noHashIDList[distance];
        } else {
            return &disMap_32[distance];
        }
    }
};

Edge neighborsTable[MAXE];  //һά�ڽӱ�
Edge neighborsTable2[MAXE];  //bfs���ؽ����ڽӱ�
uint16 dis_16[THREAD_COUNT][MAXN];   //16λdis��¼
uint32 G[MAXN];  //��һ�ι�ͼÿ���ڵ���ڽӱ�����
uint32 G2[MAXN];//bfs��ڶ��ι�ͼÿ���ڵ���ڽӱ�����
ThreadData threadDatas[THREAD_COUNT];  //�߳�����
uint32 overlap[MAXN];  //���ڳ���1���0���Ȩ��ֵ��ͼ1��Ч��
uint32 patchCount;  //���ؾ����Ƭ��
uint32 patchSize;  //���ؾ���ÿƬ�ߴ�
atomic_int processingId(0);  //����Ƭ����
uint32 nodeData[THREAD_COUNT_READ][2 * MAXE];  //������ID
uint32 index_Id[THREAD_COUNT_READ][2 * MAXE];   //�±굽ID��ӳ��
uint32 moneyData[THREAD_COUNT_READ][MAXE];   //�����ݽ��
uint32 nodeIdxMulti[THREAD_COUNT_READ][3];   //�����̵߳�ID���������ͳ��
uint16 inDegree[THREAD_COUNT_READ][MAXN];   //��һ�ι�ͼ���
uint16 outDegree[THREAD_COUNT_READ][MAXN];  //��һ�ι�ͼ����
uint16 inDegree2[MAXN];//bfs��ڶ��ι�ͼ���
uint16 outDegree2[MAXN];//bfs��ڶ��ι�ͼ����
uint32 index2Id[MAXN];  //�ڶ��ι�ͼ���±굽ID��ӳ��
uint32 nodeCnt;  //�ڵ���
/**
 * ��ͼ������һ����
 * @param u
 * @param v
 * @param m
 */
void addEdge(uint32 u, uint32 v, uint32 m) {
    neighborsTable[G[u]+outDegree2[u]].to = v;
    neighborsTable[G[u]+outDegree2[u]].w = m;
    ++outDegree2[u];
}
/**
 * �Ѷ������ݵı߽���id���±��ӳ�䣬����ID������ֱ������ӳ��
 * @param threadId
 * @param nodeData
 * @param id_Index
 */
void idToIndex(uint32 threadId, uint32 *nodeData, uint32* id_Index) {
    //id_to_index
    for (uint32 idx = 0; idx < nodeIdxMulti[threadId][0]; idx += 2) {
        nodeData[idx] = id_Index[nodeData[idx]];
        nodeData[idx + 1] = id_Index[nodeData[idx + 1]];
        ++inDegree[threadId][nodeData[idx + 1]];
        ++outDegree[threadId][nodeData[idx]];
    }
}
/**
 * �鲢������
 * @param A
 * @param m
 * @param B
 * @param n
 */
void mergeData(uint32 *A, uint32 m, const uint32 *B, uint32 n) {
    int pa = m - 1, pb = n - 1;
    uint32 tail = m + n - 1;
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
 * ���̶߳������ݣ���ID����
 * @param buf
 * @param end
 * @param nodeData
 * @param moneyData
 * @param index_Id
 * @param threadId
 */
void sortThread(char *buf, const char *end, uint32 *nodeData, uint32 *moneyData,
                uint32 *index_Id, uint32 threadId) {
    uint32 nodeIdx = 0, nodeIdx2 = 0, nodeIdx3 = 0;
    while (buf < end) {
        uint32 from = 0, to = 0;
        uint32 money = 0;
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
        while ((*buf) & 0x10) {
            money *= 10;
            money += (*buf) & 0x0f;
            buf++;
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
        index_Id[nodeIdx3++] = to;
    }
    sort(index_Id, index_Id + nodeIdx3);
    nodeIdxMulti[threadId][0] = nodeIdx;
    nodeIdxMulti[threadId][1] = nodeIdx2;
    nodeIdxMulti[threadId][2] = nodeIdx3;
}
uint32 id_Index[3*MAXE];  //����ӳ��
/**
 * ���ļ��ж�������  8�߳�
 * @param inputFilePath
 */
void readData(const char *inputFilePath) {
    int fd = open(inputFilePath, O_RDONLY);
    int fileLen = lseek(fd, 0, SEEK_END);
    //�����������  ��ͼ+����
    char *buf = (char *) mmap(NULL, fileLen, PROT_READ, MAP_PRIVATE, fd, 0);
    close(fd);
    uint32 sz = fileLen / THREAD_COUNT_READ;
    char *start[] = {buf, buf + sz, buf + sz * 2, buf + sz * 3, buf + sz * 4, buf + sz * 5, buf + sz * 6, buf + sz * 7,
                     buf + fileLen};
    for (uint16 i = 1; i < THREAD_COUNT_READ; i++) {
        while (*(start[i]++) != '\n');
    }
    thread t[THREAD_COUNT_READ];
    for (uint16 i = 0; i < THREAD_COUNT_READ; i++) {
        t[i] = thread(sortThread, start[i], start[i + 1], nodeData[i], moneyData[i], index_Id[i], i);
    }
    for (uint16 i = 0; i < THREAD_COUNT_READ; i++)
        t[i].join();
    for (uint16 i = 0; i < 4; i++) {
        t[i] = thread(mergeData, index_Id[i], nodeIdxMulti[i][2], index_Id[i + 4], nodeIdxMulti[i + 4][2]);
    }
    for (uint16 i = 0; i < 4; i++)
        t[i].join();
    //�鲢������
    mergeData(index_Id[0], nodeIdxMulti[0][2] + nodeIdxMulti[4][2], index_Id[1],
              nodeIdxMulti[1][2] + nodeIdxMulti[5][2]);
    mergeData(index_Id[2], nodeIdxMulti[2][2] + nodeIdxMulti[6][2], index_Id[3],
              nodeIdxMulti[3][2] + nodeIdxMulti[7][2]);
    mergeData(index_Id[0], nodeIdxMulti[0][2] + nodeIdxMulti[1][2] + nodeIdxMulti[4][2] + nodeIdxMulti[5][2],
              index_Id[2], nodeIdxMulti[2][2] + nodeIdxMulti[3][2] + nodeIdxMulti[6][2] + nodeIdxMulti[7][2]);
    uint32 nodeIdx3 = 0;
    for (uint16 i = 0; i < THREAD_COUNT_READ; i++)
        nodeIdx3 += nodeIdxMulti[i][2];
    //ȥ�أ�ӳ��ID
    nodeCnt = unique(index_Id[0], index_Id[0] + nodeIdx3) - index_Id[0];
    for (uint32 idx = 0; idx < nodeCnt; idx++) {
        id_Index[index_Id[0][idx]] = idx;
    }
    for (uint16 i = 0; i < THREAD_COUNT_READ; i++)
        t[i] = thread(idToIndex, i, nodeData[i], id_Index);
    for (uint16 i = 0; i < THREAD_COUNT_READ; i++)
        t[i].join();
    //�����ͳ��
    for (uint32 idx = 0; idx < nodeCnt; idx++) {
        outDegree[0][idx] += outDegree[1][idx] + outDegree[2][idx] + outDegree[3][idx] +
                             outDegree[4][idx] + outDegree[5][idx] + outDegree[6][idx] + outDegree[7][idx];
        inDegree[0][idx] += inDegree[1][idx] + inDegree[2][idx] + inDegree[3][idx] +
                            inDegree[4][idx] + inDegree[5][idx] + inDegree[6][idx] + inDegree[7][idx];
    }
    //�ڽӱ���������
    for (uint32 idx = 1; idx < nodeCnt; idx++) {
        G[idx] = G[idx - 1] + outDegree[0][idx - 1];
    }
    G[nodeCnt]=G[nodeCnt-1]+outDegree[0][nodeCnt-1];
    //��һ�ι�ͼ
    for (uint16 i = 0; i < THREAD_COUNT_READ; i++) {
        for (uint32 idx = 0, idx2 = 0; idx < nodeIdxMulti[i][0]; idx += 2) {
            addEdge(nodeData[i][idx], nodeData[i][idx + 1], moneyData[i][idx2++]);
        }
    }
}
/**
 * ԭ������16/32/64λdis���л����������ݽ�����65535��ֱ�Ӳ���uint16
 * @param patchId
 * @param threadId
 */
void dijkstraThread_16(uint32 patchId, uint32 threadId) {
    ThreadData &threadData = threadDatas[threadId];
    uint32 startIdx = patchId * patchSize;
    uint32 endIdx = (patchId + 1) * patchSize;
    endIdx = nodeCnt < endIdx ? nodeCnt : endIdx;
    if (endIdx <= startIdx) return;
    uint16 *sigma = threadData.sigma;
    uint16 *dis = dis_16[threadId];
    auto P = threadData.P;
    auto PIndex = threadData.PIndex;
    uint32 *S = threadData.S;
    auto &Q = threadData.Q_32;
    Result *result = threadData.result;
    for (uint32 start = startIdx; start < endIdx; ++start) {
        if (overlap[start] == 0) continue;
        uint32 sIdx = 0;
        dis[start] = 0;
        sigma[start] = 1;
        auto &pi = *threadData.getIDList_16(0);
        pi.push_back(start);
        Q.push(0);
        while (!Q.empty()) {  //���ȶ����У�ֻ��dis���ٸ���disӳ��ID��vector�������ڱ���������ȼ�С���ȶ��й�ģ
            uint16 curDis = Q.top();
            auto &idList = *threadData.getIDList_16(curDis);
            Q.pop();
            for (const uint32 &u:idList) {
                if (dis[u] != curDis) continue;  //���������ȶ��е�disһ����֮ǰ�Ĵ��滻vis����
                S[sIdx++] = u;
                ushort cur_sigma = sigma[u];
                for (uint32 st = G2[u],ed=G2[u + 1]; st < ed; ++st) {  //�����ڽӱ�
                    uint32 v = neighborsTable2[st].to;
                    uint16 dis_v =dis[v];
                    uint16 disSum = curDis + neighborsTable2[st].w;
                    if (dis_v > disSum) {
                        dis[v] = disSum;
                        sigma[v] = cur_sigma;
                        PIndex[v] = 0;
                        P[v][PIndex[v]++] = u;
                        auto &piv = *threadData.getIDList_16(dis[v]);
                        if (piv.empty()) {
                            Q.push(dis[v]);
                        }
                        piv.push_back(v);
                    } else if (dis_v == disSum) {
                        sigma[v] +=cur_sigma;
                        P[v][PIndex[v]++]  = u;
                    }
                }
            }
            idList.resize(0);
        }
        for (uint32 si = sIdx-1; si > 0; si--) {
            uint32 v = S[si];
            const double dv = result[v].delta + 1.0 / sigma[v];
            for (uint32 st = 0,ed = PIndex[v]; st < ed; ++st) {
                result[P[v][st]].delta += dv;
            }
            result[v].bc +=result[v].delta * sigma[v] * overlap[start];
            result[v].delta= 0;
            dis[v] = -1;
        }
        result[start].bc += (sIdx-1)*(overlap[start] - 1); //����1���0��Ĳ��㴦��
        dis[start] = -1;
        result[start].delta = 0;
    }
}
/**
 * �߳������������16λ����
 * @param threadId
 */
inline void searchResultThread(int threadId) {
    fill(dis_16[threadId], dis_16[threadId] + nodeCnt, -1);
    while (processingId < patchCount) {
        dijkstraThread_16(atomic_fetch_add_explicit(&processingId, 1, std::memory_order_relaxed), threadId);
    }
}
/**
 * ���߳��������
 */
void searchResult() {
    patchCount = PATCH_COUNT < nodeCnt ? PATCH_COUNT : nodeCnt;
    patchSize = (nodeCnt + patchCount - 1) / patchCount;
    std::thread threads[THREAD_COUNT];
    for (uint32  i = 0; i < THREAD_COUNT; ++i) {
        threads[i] = thread(searchResultThread, i);
    }
    for (auto &thread : threads) {
        thread.join();
    }
}
/**
 * ����������
 */
struct FinalResult{
    uint32 id;
    double bc;
}resultGlobal[MAXN];
bool cmp(FinalResult &a,FinalResult &b){
    if (fabs(a.bc - b.bc) < 1e-6)
        return a.id < b.id;
    return a.bc > b.bc;
}
void writeResult(const string &outputFile) {
    for (uint32 i = 0; i < THREAD_COUNT; ++i) {
        auto &resultThread = threadDatas[i].result;
        for (uint32 idx = 0; idx < nodeCnt; ++idx)
            resultGlobal[idx].bc += resultThread[idx].bc;
    }
    for (uint32 i = 0; i < nodeCnt; i++) {
        resultGlobal[i].id =index2Id[i];
    }
    sort(resultGlobal, resultGlobal + nodeCnt, cmp);
    FILE *fp = fopen(outputFile.c_str(), "wb");
    char buf[64];
    for (uint32 i = 0; i < 100; i++) {
        uint32 idx = sprintf(buf, "%d,%.3lf\n", resultGlobal[i].id, resultGlobal[i].bc);
        fwrite(buf, idx, sizeof(char), fp);
    }
    fclose(fp);
}
/**
 * ����ǰ����һ��bfs���¹�ͼ����������������ʾ����ڴ�������ͼ1����30S->10S
 */
uint32 bfsNode[MAXN];
bool bfsVis[MAXN];
void bfs(uint32 start) {
    if (bfsVis[start]) return;
    bfsVis[start] = true;
    queue<uint32> que;
    que.push(start);
    while (!que.empty()) {
        uint32 now = que.front();
        que.pop();
        bfsNode[++bfsNode[0]] = now;
        for (uint32 st = G[now],ed =G[now + 1]; st <ed ; st++) {
            uint32 next = neighborsTable[st].to;
            if (!bfsVis[next]) {
                bfsVis[next] = true;
                que.push(next);
            }
        }
    }
}
/**
 * BFS���¹�ͼ�����0����1�ĵ���в��㴦��
 */
uint32 idMap[MAXN];
void preProcessing() {
    for (uint32 i = 0; i < nodeCnt; i++)
        bfs(i);
    for (uint32 st = 0,ed =bfsNode[0]; st <ed ; st++) {
        uint32 node = bfsNode[st + 1];
        index2Id[st] = index_Id[0][node];
        idMap[node] = st;
    }
    uint tableInv = 0;
    for (uint32 start = 0,end =bfsNode[0]; start < end; start++) {
        uint32 node = bfsNode[start + 1];
        G2[start] = tableInv;
        for (uint32 st = G[node],ed =G[node + 1] ; st <ed ; st++) {
            neighborsTable2[tableInv].to = idMap[neighborsTable[st].to];
            neighborsTable2[tableInv].w = neighborsTable[st].w;
            tableInv++;
        }
        inDegree2[start] = inDegree[0][node];
        outDegree2[start] = outDegree[0][node];
    }
    G2[nodeCnt]=G2[nodeCnt-1]+outDegree2[nodeCnt-1];
    for (uint i = 0; i < nodeCnt; i++) {
        if (inDegree2[i] == 0 && outDegree2[i] == 1) {
            overlap[neighborsTable2[G2[i]].to]++;
        } else if (outDegree2[i] != 0)
            overlap[i] += 1;
    }
}

int main(int argc, char *argv[]) {
    nice(-20);
    string testFile = "/home/data13.txt";
    string outputFile = "/home/myResult.txt";
//    string testFile= "/data/test_data.txt";
//    string outputFile = "/projects/student/result.txt";
    readData(testFile.c_str());
    preProcessing();
    searchResult();
    writeResult(outputFile);
    _exit(0);
}
