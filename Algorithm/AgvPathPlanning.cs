using AGV_V1._0;
using AGV_V1._0.Agv;
using AGV_V1._0.Algorithm;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;



namespace Agv.PathPlanning
{

    class AgvPathPlanning
    {      
        
        private int Height = 15;       //默认地图高度
        private int Width = 20;       //默认地图宽度
        private const int Reachable = 0;       //可以到达的结点
        private const int Bar = 1;             //障碍物
        private const int Pass = 2;            //需要走的步数
        private const int Source = 3;          //起点
        private const int Destination = 4;     //终点

        

        private const int Right = (1 << 0);
        private const int Down = (1 << 1);
        private const int Left = (1 << 2);
        private const int Up = (1 << 3);

        

        Node[,] graph = null;
        int beginX, beginY, endX, endY; //起始点、终点
        Close[,] close = null;
        Direction beginDir; //当前搜索方向

        void initGraph(ElecMap elc, List<MyPoint> scanner, List<MyPoint> lockNode, int v_num, int beginX, int beginY, int endX, int endY, Direction direction)
        //  public void initGraph(ElecMap elc, List<MyPoint> scanner,ConcurrentQueue<MyPoint> lockNode, int v_num, int sx, int sy, int dx, int dy, Direction direction)
        {

            //地图发生变化时重新构造地
            int i, j;
            this.beginX = beginX;    //起点X坐标
            this.beginY = beginY;    //起点Y坐标
            this.endX = endX;    //终点X坐标
            this.endY = endY;    //终点Y坐标
            this.beginDir = direction;
            Height = elc.HeightNum;
            Width = elc.WidthNum;
            //Width = width;
            //Height = height;

            graph = new Node[Height, Width];

            for (i = 0; i < Height; i++)
            {
                for (j = 0; j < Width; j++)
                {
                    graph[i, j] = new Node { };
                    graph[i, j].x = i; //地图坐标X
                    graph[i, j].y = j; //地图坐标Y
                    graph[i, j].node_Type =  elc.mapnode[i, j].IsAbleCross;    // 节点可到达性
                    graph[i, j].adjoinNodeCount = 0; //邻接节点个数

                    graph[i, j].traCongesIntensity =  elc.mapnode[i, j].TraCongesIntensity;
                    graph[i, j].leftDifficulty =  elc.mapnode[i, j].LeftDifficulty;
                    graph[i, j].rightDifficulty =  elc.mapnode[i, j].RightDifficulty;
                    graph[i, j].upDifficulty =  elc.mapnode[i, j].UpDifficulty;
                    graph[i, j].downDifficulty = elc.mapnode[i, j].DownDifficulty;

                }
            }
            if (NodeDirCount(beginX, beginY) <= lockNode.Count)
            {
                lockNode.Remove(lockNode[0]);
            }
            for (int index = 0; index < lockNode.Count; index++)
            {
                graph[lockNode[index].X, lockNode[index].Y].node_Type = false;
            }
            //Parallel.For(0, lockNode.Count, (int ii) =>
            //{
            //    MyPoint point = null;
            //    if(lockNode.TryPeek(out point)){
            //        graph[point.X, point.Y].node_Type = false;
            //    }
            //});

            for (int index = 0; index < scanner.Count; index++)
            {
                graph[scanner[index].X, scanner[index].Y].node_Type = false;
            }

            for (i = 0; i < Height; i++)
            {
                for (j = 0; j < Width; j++)
                {

                    if ((!graph[i, j].node_Type) && (i != beginX && j != beginY))//&&(i!=srcX&&j!=srcY)即使起点不可达也计算它的邻接点数sur
                    {
                        continue;
                    }
                    if (j > 0)
                    {
                        if (graph[i, j - 1].node_Type && graph[i, j].leftDifficulty < Node.MAX_ABLE_PASS)    // left节点可以到达
                        {
                            graph[i, j].adjoinNodeCount |= Left;
                        }
                    }
                    if (j < Width - 1)
                    {
                        if (graph[i, j + 1].node_Type && graph[i, j].rightDifficulty < Node.MAX_ABLE_PASS)    // right节点可以到达
                        {
                            graph[i, j].adjoinNodeCount |= Right;
                        }
                    }
                    if (i > 0)
                    {
                        if (graph[i - 1, j].node_Type && graph[i, j].upDifficulty<Node.MAX_ABLE_PASS)    // up节点可以到达
                        {
                            graph[i, j].adjoinNodeCount |= Up;
                        }
                    }
                    if (i < Height - 1)
                    {
                        if (graph[i + 1, j].node_Type && graph[i, j].downDifficulty < Node.MAX_ABLE_PASS)    // down节点可以到达
                        {
                            graph[i, j].adjoinNodeCount |= Down;
                        }
                    }
                }
            }
        }
       
        int NodeDirCount(int x, int y)
        {
            int count = 0;
            if (graph[x, y].upDifficulty<Node.MAX_ABLE_PASS)
            {
                count++;
            }
            if (graph[x, y].downDifficulty < Node.MAX_ABLE_PASS)
            {
                count++;
            }
            if (graph[x, y].leftDifficulty < Node.MAX_ABLE_PASS)
            {
                count++;
            }
            if (graph[x, y].rightDifficulty < Node.MAX_ABLE_PASS)
            {
                count++;
            }

            return count;
        }
        
        /// <summary>
        /// //使用某种算法进行路径规划
        /// </summary>
        /// <returns></returns>
         Close PathPlanning()
        {            
            close = new Close[Height, Width];
            initClose(close, beginX, beginY, endX, endY);
            close[beginX, beginY].Node.isSearched = true;

           int result = Astar.Search(close, graph, beginX, beginY, beginDir);
           //  int result = Dijkstra.Search(close, graph, beginX, beginY, beginDir);
          // int result = Bfs.Search(close, graph, beginX, beginY,Height,Width);

            Close p, t, q = null;
            switch (result)
            {
                case SearchUtil.Sequential:  //顺序最近
                    p = (close[endX, endY]);
                    while (p != null)    //转置路径
                    {
                        t = p.From;
                        p.From = q;
                        q = p;
                        p = t;
                    }
                    close[beginX, beginY].From = q.From;
                    return (close[beginX, beginY]);
                case SearchUtil.NoSolution:
                    return null;
            }
            return null;
        }

         // 地图Close表初始化配置
         void initClose(Close[,] cls, int beginX, int beginY, int endX, int endY)
         {
             int i, j;
             for (i = 0; i < Height; i++)
             {
                 for (j = 0; j < Width; j++)
                 {
                     cls[i, j] = new Close { };
                     cls[i, j].Node = graph[i, j];               // Close表所指节点
                     cls[i, j].Node.isSearched = !(graph[i, j].node_Type);  // 是否被访问
                     cls[i, j].From = null;                    // 所来节点
                     cls[i, j].G =  0;  //需要根据前面的点计算
                     cls[i, j].H = Math.Abs(endX - i) + Math.Abs(endY - j);    // 评价函数值
                     cls[i, j].F = 0;   // cls[i, j].G + cls[i, j].H;
                 }
             }
             //cls[endX, endY].G = AstarUtil.Infinity;     //移步花费代价值
             //cls[beginX, beginY].F = cls[beginX, beginY].H;            //起始点评价初始值
         }
        /// <summary>
        /// // 获取最短路径
        /// </summary>
        /// <returns></returns>
         int GetShortestPath(List<MyPoint> route)
        {
            Close start;
            int m = 0;
            Close p;
            int step = 0;
            p = PathPlanning();
            start = p;
            if (p == null)
            {
                return 0;
            }
            else
            {
                while (p.From != null)
                {
                   // graph[p.node.x, p.node.y].value = Pass;
                    route.Add(new MyPoint(p.Node.x, p.Node.y));
                    m++;
                    p = p.From;
                    step++;
                }
                route.Add(new MyPoint(p.Node.x, p.Node.y));
                m++;
                //graph[beginX, beginY].value = Source;
               // graph[endX, endY].value = Destination;
                return step;
            }
        }
        public List<MyPoint> Search(ElecMap elc, List<MyPoint> scannerNode, List<MyPoint> lockNode, int v_num, int width, int height, int firstX, int firstY, int endX, int endY, Direction direction)
        {

            // ChangeMap(elc, width, height);  // 转换寻找路径的可达还是不可达
            initGraph(elc, scannerNode, lockNode, v_num, firstX, firstY, endX, endY, direction);
            List<MyPoint> route = new List<MyPoint>();
            GetShortestPath(route);
            if (route.Count < 1)
            {
                lockNode.Clear();
                initGraph(elc, scannerNode, lockNode, v_num, firstX, firstY, endX, endY, direction);
                route = new List<MyPoint>();
                GetShortestPath(route);
            }
            return route;
        }
    }
}
