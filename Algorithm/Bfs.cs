using Agv.PathPlanning;
using System;

namespace AGV_V1._0.Algorithm
{
    class Bfs:IAlgorithm
    {
        public int Search(Close[,] close, Node[,] graph, int beginX, int beginY, Agv.Direction beginDir)
        {
            int times = 0;
            int i, curX, curY, surX, surY;
            int height = close.GetLength(0);
            int width = close.GetLength(1);
            int f = 0, r = 1;
            Close p = new Close();
            Close[] q = new Close[height * width];
            int w = 0;
            for (int m = 0; m < height; m++)
            {
                for (int n = 0; n < width; n++)
                {
                    q[w] = new Close ();
                    w++;
                }
            }
            close[beginX, beginY].Node.isSearched = true;

            while (r != f)
            {
                p = q[f];
                f = (f + 1) % SearchUtil.MaxLength;
                curX = p.Node.x;
                curY = p.Node.y;
                for (i = 0; i < 8; i++)
                {
                    if ((p.Node.adjoinNodeCount & (1 << i)) == 0)
                    {
                        continue;
                    }
                    surX = curX + (int)SearchUtil.dir[i].X;
                    surY = curY + (int)SearchUtil.dir[i].Y;
                    if (surX < 0 || surY < 0)
                    {
                        Console.WriteLine("走出场地外");
                        continue;
                    }
                    if (!close[surX, surY].Node.isSearched)
                    {
                        close[surX, surY].From = p;
                        close[surX, surY].Node.isSearched = true;
                        close[surX, surY].G = p.G + 1;
                        q[r] = close[surX, surY];
                        r = (r + 1) % SearchUtil.MaxLength;
                    }
                }
                times++;
            }
            return times;
        }

    }
}
