using Agv.PathPlanning;
using AGV_V1._0.Agv;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Algorithm
{
    /// <summary>
    /// f=g+h  预计花费=起点到当前点+当前点到终点的预计
    /// </summary>
    class Astar:IAlgorithm
    {
       
         const int SWERVE_COST = 3;
        public int  Search(Close [,]close,Node[,] graph,int beginX,int beginY,Direction beginDir)
        {    // A*算法遍历
            //int times = 0; 
            int i, curX, curY, nextX, nextY;
            float surG=float.MaxValue;
            Close curPoint = new Close();

             List<Close> open = new List<Close>();
             SearchUtil.AsatrPush(open, close, beginX, beginY, 0);
            int times = 0;
            bool isFirstDirection = true;
            while (open.Count > 0)
            {    
                times++;
                curPoint = SearchUtil.shift(open);
                curX = curPoint.Node.x;
                curY = curPoint.Node.y;                

                if (curPoint.From == null)
                {
                    curPoint.Node.direction = beginDir;
                }
                else
                {
                    curPoint.Node.direction = SearchUtil.getDirection(curPoint.From, curPoint);//0525
                }
                for (i = 0; i < 4; i++)
                {
                    if ((curPoint.Node.adjoinNodeCount & (1 << i)) == 0)
                    {
                        continue;
                    }
                    nextX = curX + (int)SearchUtil.dir[i].X;
                    nextY = curY + (int)SearchUtil.dir[i].Y;
                    //if (surX < 0 || surY < 0)
                    //{
                    //    Console.WriteLine("走出场地外");
                    //    continue;
                    //} 
                    if (!close[nextX, nextY].Node.isSearched)
                    {
                        close[nextX, nextY].Node.isSearched = true;
                        close[nextX, nextY].From = curPoint;
                        Direction tempDir = new Direction();
                        int tempPassDifficulty = 2;
                        switch (i)
                        {
                            case 0:
                                tempDir = Direction.Right;
                                tempPassDifficulty = graph[curX, curY].rightDifficulty;
                                break;
                            case 1:
                                tempDir = Direction.Down;
                                tempPassDifficulty = graph[curX, curY].downDifficulty;
                                break;
                            case 2:
                                tempDir = Direction.Left;
                                tempPassDifficulty = graph[curX, curY].leftDifficulty;
                                break;
                            case 3:
                                tempDir = Direction.Up;
                                tempPassDifficulty = graph[curX, curY].upDifficulty;
                                break;
                        }
                        int directionCost = (tempDir == curPoint.Node.direction) ? 0 :2;
                        if (directionCost == 2 && isFirstDirection == true)
                        {
                            directionCost--;
                            isFirstDirection =false;
                        }
                        //  curPoint.node.stopTime = 1+directionCost * 2; 
                        // int tempTraConges = graph[curX, curY].traCongesIntensity;


                        //curPoint.searchDir = close[surX, surY].searchDir;
                        surG = curPoint.G + (float)(Math.Abs(curX - nextX) + Math.Abs(curY - nextY)) + SWERVE_COST * directionCost;
                        SearchUtil.AsatrPush(open, close, nextX, nextY, surG);
                    }
                }
            }
            if (curPoint.H==0)
            {
                System.Console.WriteLine("astar times:" + times);
                return SearchUtil.Sequential;
            }
            else
            {
                return SearchUtil.NoSolution; //无结果
            }
        }


    }
}
