/**
 * IOT_team 成渝赛区
 */
#include <bits/stdc++.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <mutex>
using namespace std;
//*****************************************************定义常量**********************************************//
//const unsigned int THREAD_COUNT = 10;
//const unsigned int PATCH_COUNT = 512;
//const unsigned int MAX_NODE=262144;
//const unsigned int MAX_DEGREE=50;
//const unsigned int MAX_P2=250;
//const unsigned int MAX_P3=3000;
//const unsigned int u32 = 32;
//const unsigned int u1 = 0x1;
/***********************************************全局变量****************************************************/
const char digits[] = "0,001,002,003,004,005,006,007,008,009,0010,011,012,013,014,015,016,017,018,019,020,021,022,023,024,025,026,027,028,029,030,031,032,033,034,035,036,037,038,039,040,041,042,043,044,045,046,047,048,049,050,051,052,053,054,055,056,057,058,059,060,061,062,063,064,065,066,067,068,069,070,071,072,073,074,075,076,077,078,079,080,081,082,083,084,085,086,087,088,089,090,091,092,093,094,095,096,097,098,099,0100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255,256,257,258,259,260,261,262,263,264,265,266,267,268,269,270,271,272,273,274,275,276,277,278,279,280,281,282,283,284,285,286,287,288,289,290,291,292,293,294,295,296,297,298,299,300,301,302,303,304,305,306,307,308,309,310,311,312,313,314,315,316,317,318,319,320,321,322,323,324,325,326,327,328,329,330,331,332,333,334,335,336,337,338,339,340,341,342,343,344,345,346,347,348,349,350,351,352,353,354,355,356,357,358,359,360,361,362,363,364,365,366,367,368,369,370,371,372,373,374,375,376,377,378,379,380,381,382,383,384,385,386,387,388,389,390,391,392,393,394,395,396,397,398,399,400,401,402,403,404,405,406,407,408,409,410,411,412,413,414,415,416,417,418,419,420,421,422,423,424,425,426,427,428,429,430,431,432,433,434,435,436,437,438,439,440,441,442,443,444,445,446,447,448,449,450,451,452,453,454,455,456,457,458,459,460,461,462,463,464,465,466,467,468,469,470,471,472,473,474,475,476,477,478,479,480,481,482,483,484,485,486,487,488,489,490,491,492,493,494,495,496,497,498,499,500,501,502,503,504,505,506,507,508,509,510,511,512,513,514,515,516,517,518,519,520,521,522,523,524,525,526,527,528,529,530,531,532,533,534,535,536,537,538,539,540,541,542,543,544,545,546,547,548,549,550,551,552,553,554,555,556,557,558,559,560,561,562,563,564,565,566,567,568,569,570,571,572,573,574,575,576,577,578,579,580,581,582,583,584,585,586,587,588,589,590,591,592,593,594,595,596,597,598,599,600,601,602,603,604,605,606,607,608,609,610,611,612,613,614,615,616,617,618,619,620,621,622,623,624,625,626,627,628,629,630,631,632,633,634,635,636,637,638,639,640,641,642,643,644,645,646,647,648,649,650,651,652,653,654,655,656,657,658,659,660,661,662,663,664,665,666,667,668,669,670,671,672,673,674,675,676,677,678,679,680,681,682,683,684,685,686,687,688,689,690,691,692,693,694,695,696,697,698,699,700,701,702,703,704,705,706,707,708,709,710,711,712,713,714,715,716,717,718,719,720,721,722,723,724,725,726,727,728,729,730,731,732,733,734,735,736,737,738,739,740,741,742,743,744,745,746,747,748,749,750,751,752,753,754,755,756,757,758,759,760,761,762,763,764,765,766,767,768,769,770,771,772,773,774,775,776,777,778,779,780,781,782,783,784,785,786,787,788,789,790,791,792,793,794,795,796,797,798,799,800,801,802,803,804,805,806,807,808,809,810,811,812,813,814,815,816,817,818,819,820,821,822,823,824,825,826,827,828,829,830,831,832,833,834,835,836,837,838,839,840,841,842,843,844,845,846,847,848,849,850,851,852,853,854,855,856,857,858,859,860,861,862,863,864,865,866,867,868,869,870,871,872,873,874,875,876,877,878,879,880,881,882,883,884,885,886,887,888,889,890,891,892,893,894,895,896,897,898,899,900,901,902,903,904,905,906,907,908,909,910,911,912,913,914,915,916,917,918,919,920,921,922,923,924,925,926,927,928,929,930,931,932,933,934,935,936,937,938,939,940,941,942,943,944,945,946,947,948,949,950,951,952,953,954,955,956,957,958,959,960,961,962,963,964,965,966,967,968,969,970,971,972,973,974,975,976,977,978,979,980,981,982,983,984,985,986,987,988,989,990,991,992,993,994,995,996,997,998,999,";
const char digits1[] = "000,001,002,003,004,005,006,007,008,009,010,011,012,013,014,015,016,017,018,019,020,021,022,023,024,025,026,027,028,029,030,031,032,033,034,035,036,037,038,039,040,041,042,043,044,045,046,047,048,049,050,051,052,053,054,055,056,057,058,059,060,061,062,063,064,065,066,067,068,069,070,071,072,073,074,075,076,077,078,079,080,081,082,083,084,085,086,087,088,089,090,091,092,093,094,095,096,097,098,099,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255,256,257,258,259,260,261,262,263,264,265,266,267,268,269,270,271,272,273,274,275,276,277,278,279,280,281,282,283,284,285,286,287,288,289,290,291,292,293,294,295,296,297,298,299,300,301,302,303,304,305,306,307,308,309,310,311,312,313,314,315,316,317,318,319,320,321,322,323,324,325,326,327,328,329,330,331,332,333,334,335,336,337,338,339,340,341,342,343,344,345,346,347,348,349,350,351,352,353,354,355,356,357,358,359,360,361,362,363,364,365,366,367,368,369,370,371,372,373,374,375,376,377,378,379,380,381,382,383,384,385,386,387,388,389,390,391,392,393,394,395,396,397,398,399,400,401,402,403,404,405,406,407,408,409,410,411,412,413,414,415,416,417,418,419,420,421,422,423,424,425,426,427,428,429,430,431,432,433,434,435,436,437,438,439,440,441,442,443,444,445,446,447,448,449,450,451,452,453,454,455,456,457,458,459,460,461,462,463,464,465,466,467,468,469,470,471,472,473,474,475,476,477,478,479,480,481,482,483,484,485,486,487,488,489,490,491,492,493,494,495,496,497,498,499,500,501,502,503,504,505,506,507,508,509,510,511,512,513,514,515,516,517,518,519,520,521,522,523,524,525,526,527,528,529,530,531,532,533,534,535,536,537,538,539,540,541,542,543,544,545,546,547,548,549,550,551,552,553,554,555,556,557,558,559,560,561,562,563,564,565,566,567,568,569,570,571,572,573,574,575,576,577,578,579,580,581,582,583,584,585,586,587,588,589,590,591,592,593,594,595,596,597,598,599,600,601,602,603,604,605,606,607,608,609,610,611,612,613,614,615,616,617,618,619,620,621,622,623,624,625,626,627,628,629,630,631,632,633,634,635,636,637,638,639,640,641,642,643,644,645,646,647,648,649,650,651,652,653,654,655,656,657,658,659,660,661,662,663,664,665,666,667,668,669,670,671,672,673,674,675,676,677,678,679,680,681,682,683,684,685,686,687,688,689,690,691,692,693,694,695,696,697,698,699,700,701,702,703,704,705,706,707,708,709,710,711,712,713,714,715,716,717,718,719,720,721,722,723,724,725,726,727,728,729,730,731,732,733,734,735,736,737,738,739,740,741,742,743,744,745,746,747,748,749,750,751,752,753,754,755,756,757,758,759,760,761,762,763,764,765,766,767,768,769,770,771,772,773,774,775,776,777,778,779,780,781,782,783,784,785,786,787,788,789,790,791,792,793,794,795,796,797,798,799,800,801,802,803,804,805,806,807,808,809,810,811,812,813,814,815,816,817,818,819,820,821,822,823,824,825,826,827,828,829,830,831,832,833,834,835,836,837,838,839,840,841,842,843,844,845,846,847,848,849,850,851,852,853,854,855,856,857,858,859,860,861,862,863,864,865,866,867,868,869,870,871,872,873,874,875,876,877,878,879,880,881,882,883,884,885,886,887,888,889,890,891,892,893,894,895,896,897,898,899,900,901,902,903,904,905,906,907,908,909,910,911,912,913,914,915,916,917,918,919,920,921,922,923,924,925,926,927,928,929,930,931,932,933,934,935,936,937,938,939,940,941,942,943,944,945,946,947,948,949,950,951,952,953,954,955,956,957,958,959,960,961,962,963,964,965,966,967,968,969,970,971,972,973,974,975,976,977,978,979,980,981,982,983,984,985,986,987,988,989,990,991,992,993,994,995,996,997,998,999,";
unsigned int circntArray[16];
unsigned int nodeCnt = 0, circleCnt = 0;  //节点数、边数、环数
unsigned int patchSize;
unsigned int processedId;
struct patchInfo{
    char* address[5];
    unsigned int len[5];
}patchtoThread[512];
unsigned int Graph[65536][64];
unsigned int GraphInv[65536][64];
unsigned int nodeID[65536];
char Comma[65536][16];  //逗号复用
unsigned int idFlag[65536];
unsigned int inDegree[65536];  //入度
unsigned int outDegree[65536];//出度
unsigned int removeFLag[65536];
unsigned int reachable[10][65536][8];
unsigned int midPointP2[10][256][256];
unsigned long int midPointP3[10][3000][256];
unsigned int que[65536];
struct Array{
    char l3[360776];
    char l4[7201000];
    char l5[14401000];
    char l6[28801000];
    char l7[57600000];
}resultArray[10];
mutex mtx;
/***********************************************函数****************************************************/
void intToString(unsigned int value, char *dst)
{
    if(value < 1000){
        if(value < 100){
            if(value < 10){
                dst[0] = 2;
                mempcpy(dst+1,digits+4*value,16);}
            else{
                dst[0] = 3;
                mempcpy(dst+1,digits+4*value,16);}}
        else{
            dst[0] = 4;
            mempcpy(dst+1,digits+4*value,16);}}
    else if(value > 9999){
        dst[0] = 6;
        mempcpy(dst+1,digits+4*(value/1000),16);
        mempcpy(dst+3,digits1+4*(value%1000),16);}
    else{
        dst[0] = 5;
        mempcpy(dst+1,digits+4*(value/1000),16);
        mempcpy(dst+2,digits1+4*(value%1000),16);}
}
/**
 * 增加地图中的一条边
 * @param u
 * @param v
 * @param w
 */
void addEdge(unsigned int u, unsigned int v) {
    Graph[u][outDegree[u]] = v;
    GraphInv[v][inDegree[v]]=u;
    ++inDegree[v];
    ++outDegree[u];
}
/**
 * 从输入文件中读取数据并构建地图
 */
void readData(char *inputFilePath) {
    //mmap映射文件
    int fd = open(inputFilePath, O_RDONLY);
    unsigned int fileLen = lseek(fd, 0, SEEK_END);
    char *buf = (char *) mmap(NULL, fileLen, PROT_READ, MAP_PRIVATE, fd, 0);
    close(fd);
    unsigned int preNum=0; //记录输入的数字,避免使用push_back以提升性能
    unsigned int curNum = 0;//当前记录的数字
    unsigned int i=0;
    while(fileLen) {
        char c = buf[i++];
        if (c > '/'){
            curNum = curNum * 10 + c - '0';}
        else{
            if(curNum > 50000){
                while(buf[i++] != '\n');
                buf+=i;fileLen-=i;i=0;curNum=0;preNum=0;
                continue;
            }
            buf+=i;fileLen-=i;
            i=0;
            if(preNum){
                idFlag[preNum] = 1;
//                idFlag[curNum + 1] = 1;
                addEdge(preNum, curNum + 1);
                preNum=0;
                while(buf[i++] != '\n');
                buf+=i;fileLen-=i;i=0;
            }else preNum = curNum + 1;
            curNum=0;
        }
    }
    for (i = 1; i <= 50000; ++i) {
        if (idFlag[i]) {
            nodeID[nodeCnt++] = i;
            intToString(i - 1, Comma[i]);
        }
    }
    patchSize = nodeCnt/512;
}
void task1(){
    unsigned int pFront = 0, pEnd = 0, i;
    for (i = 0; i < nodeCnt; i++) {
        unsigned int &node = nodeID[i];
        if (!inDegree[node])
            que[pEnd++] = node;
    }
    while (pFront < pEnd) {
        unsigned int &u = que[pFront++];
        unsigned int &outSize = outDegree[u];
        for (i = 0; i < outSize; ++i) {
            unsigned int &v = Graph[u][i];
            if (1 == inDegree[v]) {
                que[pEnd++] = v;
                inDegree[v]=0;
            }
        }
        outDegree[u]=0; removeFLag[u]=1;
    }
}
void task2(){
    unsigned int pFront = 0, pEnd = 0, i;
    for (i = 0; i < nodeCnt; i++) {
        unsigned int &node = nodeID[i];
        if (!outDegree[node])
            que[pEnd++] = node;
    }
    while (pFront < pEnd) {
        unsigned int &u = que[pFront++];
        unsigned int &inSize = inDegree[u];
        for (i = 0; i < inSize; ++i) {
            unsigned int &v = GraphInv[u][i];
            if (1 == outDegree[v]) {
                que[pEnd++] = v;
                outDegree[v]=0;removeFLag[v]=1;
            }
        }
        inDegree[u]=0; removeFLag[u]=1;
    }
}
void sortGraph(unsigned int threadId){
//    while(true) {
//        mtx.lock();
//        unsigned int patchId = processedId++;
//        mtx.unlock();
//        if (patchId >= 512) return;
//        unsigned int i, startId = patchId * patchSize, endId;
//        if (patchId == 512 - 1)
//            endId = nodeCnt;
//        else
//            endId = (patchId + 1) * patchSize;
//        for (i = startId; i < endId; ++i) {
//            unsigned int &node = nodeID[i];
//            if(removeFLag[node]) continue;
//            sort(Graph[node], Graph[node] + outDegree[node]);
//            sort(GraphInv[node], GraphInv[node] + inDegree[node]);
//        }
//    }
    unsigned int i, startId = (1.0*threadId/4)*nodeCnt, endId=(1.0*(threadId+1)/4)*nodeCnt;
        for (i = startId; i < endId; ++i) {
            unsigned int &node = nodeID[i];
            if(removeFLag[node]) continue;
            sort(Graph[node], Graph[node] + outDegree[node]);
            sort(GraphInv[node], GraphInv[node] + inDegree[node]);
        }
}
unsigned int smallMapBuild(unsigned int begin,unsigned int threadId) {
    if(Graph[begin][outDegree[begin]-1]<begin||GraphInv[begin][inDegree[begin]-1]<begin){
        return -1;
    }
    unsigned int iter1, iter2, iter3, add, addressP2 = 0, addressP3 = 0;
    unsigned int idP2[250], pointP2 = 0;
    unsigned long int s;
    unsigned int flagP3 = 0;
    for (iter1 = 0; iter1 < inDegree[begin]; ++iter1) {
        unsigned int &mid = GraphInv[begin][iter1];
        if (mid <= begin||removeFLag[mid]) continue;
        reachable[threadId][mid][1] = begin;
        for (iter2 = 0; iter2 < inDegree[mid]; ++iter2) {
            unsigned int &next = GraphInv[mid][iter2];
            if (next <= begin||removeFLag[next]) continue;
            if (reachable[threadId][next][2] != begin) {
                idP2[pointP2++] = next;
                reachable[threadId][next][2] = begin;
                reachable[threadId][next][4] = addressP2++;
                reachable[threadId][next][5] = 0;
            }
            add = reachable[threadId][next][4];
            midPointP2[threadId][add][reachable[threadId][next][5]++] = mid;
        }
    }
    if (pointP2 == 0) {
        return -1;
    } else {
        //p2排序
        sort(idP2, idP2 + pointP2);
        //P2求P3
        for (iter1 = 0; iter1 < pointP2; ++iter1) {
            unsigned int &next = idP2[iter1];
            addressP2 = reachable[threadId][next][4];
            for (iter2 = 0; iter2 < reachable[threadId][next][5]; ++iter2) {
                unsigned int &mid = midPointP2[threadId][addressP2][iter2];
                for (iter3 = 0; iter3 < inDegree[next]; iter3++) {
                    unsigned int &end = GraphInv[next][iter3];
                    if (end <= begin || mid == end||removeFLag[end]) continue;
                    if (reachable[threadId][end][3] != begin) {
                        flagP3 = 1;
                        reachable[threadId][end][3] = begin;
                        reachable[threadId][end][6] = addressP3++;
                        reachable[threadId][end][7] = 0;
                    }
                    add = reachable[threadId][end][6];
                    s = next;
                    s = (s << 32) | mid;
                    midPointP3[threadId][add][reachable[threadId][end][7]++] = s;
                }
            }
        }
    }
    return flagP3;
}

void searchResult(unsigned int threadId) {
    unsigned int  last_patch = 0;
    unsigned char vis[65536] = {};
    char *resultPosition[5]{};
    char *resultPositionLast[5]{resultArray[threadId].l3,resultArray[threadId].l4,resultArray[threadId].l5,
                                resultArray[threadId].l6,resultArray[threadId].l7};
    //线程循环
    while(true){
        mtx.lock();
        unsigned int patchId = processedId++;
        mtx.unlock();
        if(patchId >= 512) return;
        unsigned int i, startId = patchId*patchSize, endId;
        if(patchId == 512-1)
            endId = nodeCnt;
        else
            endId = (patchId+1)*patchSize;
        for(i = last_patch; i < startId; ++i){
            vis[nodeID[i]] |= 0x1;
        }
        for(i = 0;i < 5;i++)
            resultPosition[i] = resultPositionLast[i];
        for (i = startId; i < endId; ++i) {
            unsigned int &begin = nodeID[i];
            //这里强制标注一次，可以省略next和begin的判断
            vis[begin] |= 0x1;
            unsigned int flagP3;
            if (removeFLag[begin]) continue;   //零度节点
            flagP3 = smallMapBuild(begin,threadId);
            if (flagP3==-1) continue;   //没有P1或者P2
            //DFS
            unsigned int now = begin;
            uint8_t depth = 1;
            unsigned int path[7] = {begin};
            char pathStr[128];memcpy(pathStr,Comma[begin]+1,16);
            unsigned int pathStrPos[7]{};pathStrPos[0]=Comma[begin][0];
            unsigned int pt[7]{};
            unsigned int ii = 0;
            while (depth) {
                L1:
                unsigned int *gCur = Graph[now];
                unsigned int &q = outDegree[now];
                unsigned int &ptNow = pt[depth];
                vis[now] |= 0x1;
                //普通情况
                if (depth > 2) {
                    if (ptNow == 0 && reachable[threadId][now][1] == begin) {
                        unsigned int pathStrLen =pathStrPos[depth-1];
                        pathStr[pathStrLen-1]='\n';
                        memcpy(resultPosition[depth-3],pathStr,16);
                        memcpy(resultPosition[depth-3]+16,pathStr+16,16);
                        resultPosition[depth-3]+=pathStrLen;pathStr[pathStrLen-1]=',';
                        ++circntArray[threadId];
                    }
                }
                // 遍历邻域
                while (ptNow < q) {
                    unsigned int &next = gCur[ptNow++];
                    if (depth == 3 && flagP3 == 0)
                        break;
                    if (0 == vis[next]) {
                        if (depth < 4) {
                            path[depth] = next;memcpy(pathStr+pathStrPos[depth-1],Comma[next]+1,16);
                            pathStrPos[depth]=pathStrPos[depth-1]+Comma[next][0];
                            depth++;
                            now = next;
                            goto L1;
                        } else {
                            if (reachable[threadId][next][1] == begin) {
                                memcpy(resultPosition[2],pathStr,16);
                                memcpy(resultPosition[2]+16,pathStr+16,16);
                                memcpy(resultPosition[2]+32,pathStr+32,16);
                                resultPosition[2]+=pathStrPos[3];
                                memcpy(resultPosition[2],Comma[next]+1,16);
                                resultPosition[2]+=Comma[next][0];
                                *(resultPosition[2]-1)='\n';
                                ++circntArray[threadId];
                            }
                            if (reachable[threadId][next][2] == begin) {
                                unsigned int *nextCollctions = midPointP2[threadId][reachable[threadId][next][4]];
                                unsigned int &len = reachable[threadId][next][5];
                                memcpy(pathStr+pathStrPos[3],Comma[next]+1,16);
                                pathStrPos[4]=pathStrPos[3]+Comma[next][0];
                                for (ii = 0; ii < len; ++ii) {
                                    unsigned int end = nextCollctions[ii];
                                    if (vis[end]) continue;
                                    memcpy(resultPosition[3],pathStr,16);
                                    memcpy(resultPosition[3]+16,pathStr+16,16);
                                    resultPosition[3]+=pathStrPos[4];
                                    memcpy(resultPosition[3],Comma[end]+1,16);
                                    resultPosition[3]+=Comma[end][0];
                                    *(resultPosition[3]-1)='\n';
                                    ++circntArray[threadId];
                                }
                            }
                            if (reachable[threadId][next][3] == begin) {
                                unsigned long int *nextCollctions = midPointP3[threadId][reachable[threadId][next][6]];
                                unsigned int &len = reachable[threadId][next][7];
                                memcpy(pathStr+pathStrPos[3],Comma[next]+1,16);
                                pathStrPos[4]=pathStrPos[3]+Comma[next][0];
                                for (ii = 0; ii < len; ++ii) {
                                    unsigned int next1 = nextCollctions[ii], next2 = nextCollctions[ii] >> 32;
                                    if (vis[next1] || vis[next2]) continue;
                                    memcpy(resultPosition[4],pathStr,16);
                                    memcpy(resultPosition[4]+16,pathStr+16,16);
                                    resultPosition[4]+=pathStrPos[4];
                                    memcpy(resultPosition[4],Comma[next2]+1,16);
                                    resultPosition[4]+=Comma[next2][0];
                                    memcpy(resultPosition[4],Comma[next1]+1,16);
                                    resultPosition[4]+=Comma[next1][0];
                                    *(resultPosition[4]-1)='\n';
                                    ++circntArray[threadId];
                                }
                            }
                            vis[next] >>= 0x1;
                        }
                    }
                }
                if (ptNow == q) {
                    if (depth == 1)
                        break;
                    ptNow = 0;
                    vis[now] >>= 0x1;
                    now = path[--depth - 1];
                }
            }
        }
        for(i =0;i<5;i++){
            patchtoThread[patchId].address[i] = resultPositionLast[i];
            patchtoThread[patchId].len[i] = resultPosition[i]-resultPositionLast[i];
            resultPositionLast[i] = resultPosition[i];
        }
        last_patch = endId;
    }
}
///**
// * 写入文件
// * @param outputFilePath
// */
static void writeResult(char* outputFilePath) {
    for(unsigned int t=0;t<10;++t){
        circleCnt+=circntArray[t];
    }
    char pathCnt[16];
    pathCnt[0] = circleCnt/1000000+'0';circleCnt%=1000000;
    mempcpy(pathCnt+1,digits1+4*(circleCnt/1000),3);
    mempcpy(pathCnt+4,digits1+4*(circleCnt%1000),3);
    pathCnt[7]='\n';
    int file_fd = open(outputFilePath, O_WRONLY | O_CREAT, 00666);
    write(file_fd,pathCnt,8);
    for(unsigned int k=0;k<5;++k) {
        for(unsigned int patch=0;patch<512;++patch) {
            if(patchtoThread[patch].len[k])
                write(file_fd,patchtoThread[patch].address[k],patchtoThread[patch].len[k]);
        }
    }
    close(file_fd);
}

int main() {
    std::thread threads[10-1];
    int t;
//    char *inputFilePath = (char *) "../100w.txt";
//    char *outputFilePath = (char *) "../result.txt";
    char* inputFilePath = (char*)"/data/test_data.txt";
    char* outputFilePath = (char*)"/projects/student/result.txt";
    readData(inputFilePath);
    std::thread thread1(task1);
    std::thread thread2(task2);
    thread1.join();
    thread2.join();
    for (t = 0; t <4-1; t++) {
        threads[t] = std::thread(sortGraph, t);
    }
    sortGraph(t);
    for (t = 0; t < 4-1; t++) {
        threads[t].join();
    }
    processedId=0;
    for (t = 0; t < 10-1; t++) {
        threads[t] = std::thread(searchResult, t);
    }
    searchResult(t);
    for (t = 0; t < 10-1; t++) {
        threads[t].join();
    }
    writeResult(outputFilePath);
    exit(0);
}
