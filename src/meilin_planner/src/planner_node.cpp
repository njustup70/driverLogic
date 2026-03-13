#include <bits/stdc++.h>

#define TURN_TO 5
#define TAKE_P2_COST 10

#define EMPTY_PRICE 5
#define P1_PRICE 9999
#define P1_CLEAR_PRICE 200
#define P2_PRICE 10
#define PF_PRICE 10000

#define MIN_P2_COUNT 2
struct Point
{
    int coord_x;
    int coord_y;
    int price;
    Point *kid_left;
    Point *kid_right;
    Point *kid_deep;
};

Point CreatPoint(int x, int y, int price) // 创建节点（坐标，方块。相对位置）
{
    Point p;
    p.coord_x = x;
    p.coord_y = y;
    p.price = price;
    p.kid_left = nullptr;
    p.kid_right = nullptr;
    p.kid_deep = nullptr;
    return p;
}

int JudgePrice(const char *get) // 读取方块
{
    if (get[0] == '0' && get[1] == '0') // 00为空
    {
        return EMPTY_PRICE;
    }
    if (get[0] == '0' && get[1] == '1') // 01为P1
    {
        return P1_PRICE;
    }
    if (get[0] == '1' && get[1] == '0') // 10为P2
    {
        return P2_PRICE;
    }
    if (get[0] == '1' && get[1] == '1') // 11为不可接触
    {
        return PF_PRICE;
    }
    else
    {
        return -1;
    }
}

Point list[6][3];
std::vector<std::pair<int,int>> P2_list;
void ListInit() // 初始化数组
{
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            list[i][j] = Point{};
        }
    }
}

void CreatAllPoint(const char get[]) // 构建位置图
{
    ListInit();
    P2_list.clear();//初始化
    for (int i = 1; i <= 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            list[i][j] = CreatPoint(i, j, JudgePrice(get + 2 * ((i - 1) * 3 + j))); // 位置+方块类型
            if (list[i][j].price == P2_PRICE)
                P2_list.push_back({i, j});  //记录所有P2方块
        }
    }
}

void BuildGraph() // 根据位置图，建立有向无环图，生成临接表
{
    // 初始化第0行三个起点
    list[0][0] = CreatPoint(0, 0, EMPTY_PRICE);
    list[0][1] = CreatPoint(0, 1, EMPTY_PRICE);
    list[0][2] = CreatPoint(0, 2, EMPTY_PRICE);

    list[5][0] = CreatPoint(5, 0, EMPTY_PRICE);
    list[5][1] = CreatPoint(5, 1, EMPTY_PRICE);
    list[5][2] = CreatPoint(5, 2, EMPTY_PRICE);

    // 将0行节点连接到1行
    list[0][0].kid_deep = &list[1][0];
    list[0][1].kid_deep = &list[1][1];
    list[0][2].kid_deep = &list[1][2];

    for (int i = 1; i <= 4; i++)
    {
        list[i][1].kid_left = &list[i][0];
        list[i][1].kid_right = &list[i][2];
        list[i][1].kid_deep = (i < 5) ? &list[i + 1][1] : nullptr;

        list[i][0].kid_deep = &list[i + 1][0];
        list[i][0].kid_right = &list[i][1];
        list[i][2].kid_deep = &list[i + 1][2];
        list[i][2].kid_left = &list[i][1];
    }
}
struct RobotStatus // 机器人实时位置/拿走了哪个P2/当前最短路径值/当前朝向
{
    int x, y;
    int mask;
    int dp;
    int dir;
};
struct PointCompare // 优先队列比较器
{
    bool operator()(const RobotStatus &a, const RobotStatus &b)
    {
        return a.dp > b.dp;
    }
};

int bestDp[6][3][1<<4];                                // 记录该状态（坐标/拿走了哪个P2）下的最短路径的代价
int bestDir[6][3][1<<4];                               // 记录该状态（坐标/拿走了哪个P2）下的朝向
std::tuple<int, int, int> PrevRobotStatus[6][3][1<<4]; // 记录最短路径中的前一个状态（前驱节点及状态）

int getP2Index(int x, int y)// 判断是否为某个P2，返回其编号
{
    for (int i = 0; i < (int)P2_list.size(); ++i)
        if (P2_list[i].first == x && P2_list[i].second == y)
            return i;
    return -1;
}
void relaxMove(std::priority_queue<RobotStatus, std::vector<RobotStatus>, PointCompare> &s, RobotStatus &this_point, Point *next_point)
{
    int next_point_x = next_point->coord_x;
    int next_point_y = next_point->coord_y;
    int next_dir;                    // 下一个节点的对应方向
    if (next_point_x > this_point.x) // 向前
    {
        next_dir = 0;
    }
    else if (next_point_y < this_point.y) // 向左
    {
        next_dir = 1;
    }
    else if (next_point_x < this_point.x) // 向后
    {
        next_dir = 2;
    }
    else if (next_point_y > this_point.y) // 向右
    {
        next_dir = 3;
    }
    else
    {
        return;
    }
    int turn_cost = (abs(this_point.dir - next_dir) != 3) ? abs(this_point.dir - next_dir) * TURN_TO : TURN_TO; // 计算转向代价
    int go_price;                                                                                               // 前进代价
    int next_mask = this_point.mask;                                                           // 如果已经拿到P2，则下一个节点依旧持有P2

    switch (next_point->price) // 计算前进代价/更新P2状态
    {
    case EMPTY_PRICE:
        go_price = EMPTY_PRICE;
        break;
    case P1_PRICE:
        //go_price = P1_PRICE;
        //break;
        return;
    case P2_PRICE:
        if ((this_point.mask & (1 << getP2Index(next_point_x , next_point_y))) != 0) // 如果已经拿到该P2，则视为空格
        {
            go_price = EMPTY_PRICE;
            break;
        } 
        else
        {
            return;
        }
    case PF_PRICE:
        return;
    default:
        return;
    }
    int path_price = this_point.dp + turn_cost + go_price;          // 到下一个格子的代价=前进+转向
    if (path_price < bestDp[next_point_x][next_point_y][next_mask]) // 如果有新的最短路径，更新最短路径
    {
        bestDp[next_point_x][next_point_y][next_mask] = path_price;                                                                            // 更新代价
        bestDir[next_point_x][next_point_y][next_mask] = next_dir;                                                                             // 更新方向
        PrevRobotStatus[next_point_x][next_point_y][next_mask] = std::make_tuple(this_point.x, this_point.y, this_point.mask); // 更新前驱节点及状态
        s.push(RobotStatus{next_point_x, next_point_y, next_mask, path_price, next_dir});
    }
}
void tryTakeP2(std::priority_queue<RobotStatus, std::vector<RobotStatus>, PointCompare> &s, const RobotStatus &this_point) // 尝试在当前位置进行取P2操作
{
    const int dx[4] = {1, 0, -1, 0};
    const int dy[4] = {0, -1, 0, 1};
    for (int dir = 0; dir < 4; dir++) // 遍历检查四个方向是否有P2格
    {
        int next_point_x = this_point.x + dx[dir];
        int next_point_y = this_point.y + dy[dir];
        if (next_point_x < 0 || next_point_x > 5 || next_point_y < 0 || next_point_y > 2) // 跳过不存在的格子
        {
            continue;
        }
        Point *next_point_handle = &list[next_point_x][next_point_y];
        if (next_point_handle->price != P2_PRICE) // 跳过不是P2的格子
        {
            continue;
        }
        int turn_cost = (abs(this_point.dir - dir) != 3) ? abs(this_point.dir - dir) * TURN_TO : TURN_TO;
        int path_price = this_point.dp + turn_cost + TAKE_P2_COST; // 取P2代价
        int next_mask = this_point.mask | (1 << getP2Index(next_point_x, next_point_y));                                             
        if (path_price < bestDp[this_point.x][this_point.y][next_mask]) // 如果取P2后的代价小于已有P2状态下的最短路径的代价，更新最短路径
        {
            bestDp[this_point.x][this_point.y][next_mask] = path_price; // 更新代价
            bestDir[this_point.x][this_point.y][next_mask] = dir;       // 取P2操作后改变朝向
            PrevRobotStatus[this_point.x][this_point.y][next_mask] = std::make_tuple(this_point.x, this_point.y, this_point.mask);
            s.push(RobotStatus{this_point.x, this_point.y, next_mask, path_price, dir});
        }
    }
}
int countBits(int mask)
{
    int cnt = 0;
    while (mask)
    {
        cnt += mask & 1; // 如果最低位是1，就加1
        mask >>= 1;      // 右移一位
    }
    return cnt;
}
RobotStatus *FindShortestRoadRobotStatus()
{
    for (int i = 0; i < 6; i++) // 初始化最短路径表
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 16; k++)
            {
                bestDp[i][j][k] = INT_MAX;
                bestDir[i][j][k] = 0;
                PrevRobotStatus[i][j][k] = std::make_tuple(-1, -1, -1);
            }

    std::priority_queue<RobotStatus, std::vector<RobotStatus>, PointCompare> s; // 定义优先队列

    // 多起点初始化
    std::vector<std::pair<int,int>> startPoints = {{0,0},{0,1},{0,2}};
    for (auto &p : startPoints)
    {
        bestDp[p.first][p.second][0] = 0;
        bestDir[p.first][p.second][0] = 0;
        s.push(RobotStatus{p.first, p.second, 0, 0, 0}); // 起点入列
    }

    while (!s.empty()) // Dijkstra算法
    {
        RobotStatus this_point = s.top();
        s.pop();
        if (this_point.dp != bestDp[this_point.x][this_point.y][this_point.mask]) // 跳过非最优解
            continue;

        // 多终点判断，只要到达5行任意列且拿到至少一个P2即可
        if (this_point.x == 5 && (this_point.y == 0 || this_point.y == 1 || this_point.y == 2) && countBits(this_point.mask) >= MIN_P2_COUNT)
        {
            RobotStatus *res = new RobotStatus(this_point);
            return res;
        }

        tryTakeP2(s, this_point); // 尝试取P2
        Point *this_point_handle = &list[this_point.x][this_point.y];
        if (this_point_handle->kid_left != nullptr)
            relaxMove(s, this_point, this_point_handle->kid_left);
        if (this_point_handle->kid_right != nullptr)
            relaxMove(s, this_point, this_point_handle->kid_right);
        if (this_point_handle->kid_deep != nullptr)
            relaxMove(s, this_point, this_point_handle->kid_deep);
    }

    return nullptr;
}
int JudgeDirAndPrint(int prevDir, int newDir) // 判断转向并输出
{
    int diff = (newDir - prevDir + 4) % 4;
    if (diff == 0)
        return 0;
    if (diff == 1)
    {
        std::cout << "(左转) 代价：" << TURN_TO << std::endl;
        return TURN_TO;
    }
    else if (diff == 3)
    {
        std::cout << "(右转) 代价：" << TURN_TO << std::endl;
        return TURN_TO;
    }
    else
    {
        std::cout << "(掉头) 代价：" << (TURN_TO * 2) << std::endl;
        return TURN_TO * 2;
    }
}
void PrintActionSequenceFromRobotStatus(RobotStatus *the_end) // 输出动作序列
{
    if (the_end == nullptr)
    {
        std::cout << "未找到经过P2的可行路径。" << std::endl;
        return;
    }
    int x = the_end->x;
    int y = the_end->y;
    int mask = the_end->mask; // 设置终点坐标和状态
    int finalCost = bestDp[x][y][mask];       // 最终代价
    std::vector<std::tuple<int, int, int>> state_seq; // 状态序列,存储(x,y,mask)
    while (x != -1)
    {
        state_seq.emplace_back(x, y, mask); // 从终点到起点存储状态
        auto t = PrevRobotStatus[x][y][mask];
        int px = std::get<0>(t), py = std::get<1>(t), pg = std::get<2>(t);
        x = px;
        y = py;
        mask = pg;
    }
    reverse(state_seq.begin(), state_seq.end()); // 反转状态序列，使其从起点到终点
    std::cout << "动作列表：" << std::endl;
    int total = 0; //总代价
    for (size_t i = 1; i < state_seq.size(); ++i)
    {
        int this_point_x = std::get<0>(state_seq[i - 1]); //定义当前点坐标/是否拿到P2
        int this_point_y = std::get<1>(state_seq[i - 1]); //
        int this_point_mask = std::get<2>(state_seq[i - 1]); //
        int next_point_x = std::get<0>(state_seq[i]); //定义下一个点坐标/是否拿到P2
        int next_point_y = std::get<1>(state_seq[i]); //
        int next_point_mask = std::get<2>(state_seq[i]); //
        int thisDir = bestDir[this_point_x][this_point_y][this_point_mask];// 当前点朝向
        int nextDir = bestDir[next_point_x][next_point_y][next_point_mask];// 下一个点朝向

        if (this_point_x == next_point_x && this_point_y == next_point_y)//如果坐标不改变，代表进行了取P2
        {
            if (thisDir != nextDir)//朝向不一致，先记录转向
            {
                int c = JudgeDirAndPrint(thisDir, nextDir);
                total += c;
            }
            const int dx[4] = {1, 0, -1, 0};
            const int dy[4] = {0, -1, 0, 1};
            int get_P2_x = -1;
            int get_P2_y = -1;
            bool found_P2 = false;
            for (int dir = 0; dir < 4; dir++)
            {
                int neighbor_point_x = this_point_x + dx[dir]; //遍历前后两个方向，找到P2
                int neighbor_point_y = this_point_y + dy[dir]; //遍历左右两个方向，找到P2
                if (neighbor_point_x < 0 || neighbor_point_x > 5 || neighbor_point_y < 0 || neighbor_point_y > 2) //排除不存在的格子
                    continue;
                if (list[neighbor_point_x][neighbor_point_y].price == P2_PRICE) //检查是不是P2方块
                {
                    if (dir == nextDir) //如果朝向一致，优先提取
                    {
                        get_P2_x = neighbor_point_x;
                        get_P2_y = neighbor_point_y;
                        found_P2 = true;
                        break;
                    }
                    if (found_P2 != true) //朝向不一致的话列为备选
                    {
                        get_P2_x = neighbor_point_x;
                        get_P2_y = neighbor_point_y;
                        found_P2 = true;
                    } 
                }
            }
            std::cout << "在(" << this_point_x << "," << this_point_y << ")取(" << get_P2_x << "," << get_P2_y << ") " << " 代价：" << TAKE_P2_COST << std::endl;
            total += TAKE_P2_COST; //记入总代价
        }
        else //如果有坐标变化，代表移动
        {
            if (thisDir != nextDir)//朝向不一致，先记录转向
            {
                int c = JudgeDirAndPrint(thisDir, nextDir);
                total += c;
            }
            int go_cost = 0; //根据下一个格子的种类确定前进代价
            if (list[next_point_x][next_point_y].price == EMPTY_PRICE)
                go_cost = EMPTY_PRICE;
            else if (list[next_point_x][next_point_y].price == P1_PRICE)
                go_cost = P1_PRICE;
            else if (list[next_point_x][next_point_y].price == P2_PRICE)
                go_cost = EMPTY_PRICE;
            std::cout << "(" << this_point_x << "," << this_point_y << ")->(" << next_point_x << "," << next_point_y << ") 代价：" << go_cost << std::endl;
            total += go_cost;  //记入总代价
        }
    }
    std::cout << "总代价（由分步相加）：" << total << std::endl;
    std::cout << "算法最终计算的最短代价：" << finalCost << std::endl;
}

void TurnBit(uint8_t (&getbits)[3], char (&get)[25])
{
    int pos = 0;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 7; j >= 0; j--)
        {
            get[pos++] = ((getbits[i] >> j) & 1) ? '1' : '0';
        }
    }
    get[24] = '\0';
}

int main()
{
    char get[25];
    if (!(std::cin >> get))
        return 0;
    CreatAllPoint(get);
    BuildGraph();
    RobotStatus *the_end = FindShortestRoadRobotStatus();
    PrintActionSequenceFromRobotStatus(the_end);
    if (the_end)
        delete the_end;
    return 0;
}
