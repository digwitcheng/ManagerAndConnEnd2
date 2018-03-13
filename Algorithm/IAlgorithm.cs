using Agv.PathPlanning;
using AGV_V1._0.Agv;

namespace AGV_V1._0.Algorithm
{
    interface IAlgorithm
    {
        int Search(Close[,] close, Node[,] graph, int beginX, int beginY, Direction beginDir);
    }
}
