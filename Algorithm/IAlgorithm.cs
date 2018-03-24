using Agv.PathPlanning;
using AGV_V1._0.Agv;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Algorithm
{
    interface IAlgorithm
    {
        List<MyPoint> Search(Node[,] graph, int beginX, int beginY,int endX,int endY, Direction beginDir);
    }
}
