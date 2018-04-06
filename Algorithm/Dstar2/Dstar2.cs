using Agv.PathPlanning;
using AGV_V1._0.Agv;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Algorithm.Dstar2
{
    class Dstar2:IAlgorithm
    {
        //Private Member variables
        private List<Cell> path = new List<Cell>();
        private double C1;
        private double k_m;
        private Cell s_start = new Cell();
        private Cell s_goal = new Cell();
        private Cell s_last = new Cell();
        private int maxSteps;


        private List<Cell> openList = new List<Cell>();
        //Change back to private****
        public Dictionary<Cell, CellInfo> cellHash = new Dictionary<Cell, CellInfo>();
        private Dictionary<Cell, float> openHash = new Dictionary<Cell, float>();

        //Constants
        private double M_SQRT2 = Math.Sqrt(2.0);

        //Default constructor
        public Dstar2()
        {            
            maxSteps = 80000;
            C1 = 1;
        }


        //Calculate Keys
        public void CalculateKeys()
        {

        }
        public List<MyPoint> Search(Node[,] graph, int beginX, int beginY, int endX, int endY, Direction beginDir)
        {
            init(graph, beginX, beginY, endX, endY);
            replan();
            List<MyPoint> route = new List<MyPoint>();
            for (int i = 0; i < path.Count; i++)
            {
                route.Add(new MyPoint(path[i].x, path[i].y));
            }
            return route;
        }
       
        /*
         * Initialise Method
         * @params start and goal coordinates
         */
        public void init(Node[,] graph, int sX, int sY, int gX, int gY)
        {
            k_m = 0;

            s_start.x = sX;
            s_start.y = sY;
            s_goal.x = gX;
            s_goal.y = gY;

            initClose(graph);


            s_start = calculateKey(s_start);
            s_last = s_start;

        }
        void initClose(Node[,] graph)
        {
            int i, j;
            for (i = 0; i < graph.GetLength(0); i++)
            {
                for (j = 0; j < graph.GetLength(1); j++)
                {
                    Cell u = new Cell();
                    u.x = i;
                    u.y = j;
                    makeNewCell(u);                    
                    if (graph[i, j].node_Type)
                    {
                        cellHash[u].cost = C1;
                    }
                    else
                    {
                        cellHash[u].cost = -1;
                    }
                }
            }
            //cls[endX, endY].G = AstarUtil.Infinity;     //移步花费代价值
            //cls[beginX, beginY].F = cls[beginX, beginY].H;            //起始点评价初始值
        }

        /*
         * CalculateKey(state u)
         * As per [S. Koenig, 2002]
         */
        private Cell calculateKey(Cell u)
        {
            double val = Math.Min(getRHS(u), getG(u));

            double key= (val + heuristic(u, s_start) + k_m);

            u.pairK= new KeyValuePair<double, double>(key,val);

            return u;
        }

        /*
         * Returns the rhs value for state u.
         */
        private double getRHS(Cell u)
        {
            if (u == s_goal) return 0;

            //if the cellHash doesn't contain the State u
            if (!cellHash.ContainsKey(u))
                return heuristic(u, s_goal);
            return cellHash[u].rhs;
        }

        /*
         * Returns the g value for the state u.
         */
        private double getG(Cell u)
        {
            //if the cellHash doesn't contain the State u
            if (!cellHash.ContainsKey(u))
                return heuristic(u, s_goal);
            return cellHash[u].g;
        }

        /*
         * Pretty self explanatory, the heuristic we use is the 8-way distance
         * scaled by a constant C1 (should be set to <= min cost)
         */
        private double heuristic(Cell a, Cell b)
        {
            return eightCondist(a, b) * C1;
        }

        /*
         * Returns the 8-way distance between state a and state b
         */
        private double eightCondist(Cell a, Cell b)
        {
            double temp;
            double min = Math.Abs(a.x - b.x);
            double max = Math.Abs(a.y - b.y);
            if (min > max)
            {
                temp = min;
                min = max;
                max = temp;
            }
            return ((M_SQRT2 - 1.0) * min + max);

        }

        public bool replan()
        {
            path.Clear();

            int res = computeShortestPath();
            if (res < 0)
            {
                Console.WriteLine("No Path to Goal");
                return false;
            }

            LinkedList<Cell> nextState = new LinkedList<Cell>();
            Cell cur = s_start;

            if (getG(s_start) == Double.PositiveInfinity)
            {
                Console.WriteLine("No Path to Goal");
                return false;
            }

            while (cur.neq(s_goal))
            {
                path.Add(cur);
                nextState = new LinkedList<Cell>();
                nextState = getSucc(cur);//8 direction
                 
                if (nextState.Count<=0)
                {
                    Console.WriteLine("No Path to Goal");
                    return false;
                }

                double cmin = Double.PositiveInfinity;
                double tmin = 0;
                Cell smin = new Cell();

                foreach (Cell next in nextState)
                {
                    double val = cost(cur, next);
                    double val2 = trueDist(next, s_goal) + trueDist(s_start, next);
                    val += getG(next);

                    if (close(val, cmin))
                    {
                        if (tmin > val2)
                        {
                            tmin = val2;
                            cmin = val;
                            smin = next;
                        }
                    }
                    else if (val < cmin)
                    {
                        tmin = val2;
                        cmin = val;
                        smin = next;
                    }
                }
                nextState.Clear();
                cur = new Cell(smin);
                //cur = smin;
            }
            path.Add(s_goal);

            Console.WriteLine("path:" + path);
            Console.WriteLine("openList:" + openList);
            Console.WriteLine("cellHash:");
            Console.WriteLine(cellHash);
            Console.WriteLine("openHash:");
            Console.WriteLine(openHash);

            return true;
        }
        void PushNodeToOpenList(Cell close)
        {
            for (int i = openList.Count - 1; i >= 0; i--)
            {
                if (close.pairK.Key < openList[i].pairK.Key)
                {
                    continue;
                }
                openList.Insert(i + 1, close);
                return;
            }
            openList.Add(close);
        }
        Cell FetchNodeFromOpenList()
        {
            Cell temp = openList[0];
            openList.RemoveAt(0);
            return temp;
        }

        /*
         * As per [S. Koenig,2002] except for two main modifications:
         * 1. We stop planning after a number of steps, 'maxsteps' we do this
         *    because this algorithm can plan forever if the start is surrounded  by obstacles
         * 2. We lazily remove states from the open list so we never have to iterate through it.
         */
        private int computeShortestPath()
        {
            LinkedList<Cell> s = new LinkedList<Cell>();
            Console.WriteLine("openList:" + openList);
            //if (openList.Count<=0) return 1;

            int k = 0;
            while ((openList.Count>0) &&
                   (openList[0].lt(s_start = calculateKey(s_start))) ||
                   (getRHS(s_start) != getG(s_start)))
            {
                if (k++ > maxSteps)
                {
                    Console.WriteLine("At maxsteps");
                    return -1;
                }
                Cell u;
                bool test = (getRHS(s_start) != getG(s_start));

                //lazy remove
                while (true)
                {
                    if (openList.Count<=0) return 1;
                    u = FetchNodeFromOpenList();

                    if (!isValid(u)) continue;
                    if (!(u.lt(s_start)) && (!test)) return 2;
                    break;
                }

                openHash.Remove(u);

                Cell k_old = new Cell(u);

                if (k_old.lt(calculateKey(u)))
                { //u is out of date
                    insert(u);
                }
                else if (getG(u) > getRHS(u))
                { //needs update (got better)
                    setG(u, getRHS(u));
                    s = getPred(u);
                    foreach (Cell i in s)
                    {
                        updateVertex(i);
                    }
                }
                else
                {                        // g <= rhs, state has got worse
                    setG(u, Double.PositiveInfinity);
                    s = getPred(u);

                    foreach (Cell i in s)
                    {
                        updateVertex(i);
                    }
                    updateVertex(u);
                }
            } //while
            return 0;
        }

        /*
         * Returns a list of successor states for state u, since this is an
         * 8-way graph this list contains all of a cells neighbours. Unless
         * the cell is occupied, in which case it has no successors.
         */
        private LinkedList<Cell> getSucc(Cell u)
        {
            LinkedList<Cell> s = new LinkedList<Cell>();
            Cell tempState;
            if (occupied(u)) return s;
            //Generate the successors, starting at the immediate right,
            //Moving in a clockwise manner
            tempState = new Cell(u.x + 1, u.y, new KeyValuePair<double,double>(-1.0, -1.0));
            s.AddFirst(tempState);
          //  tempState = new Cell(u.x + 1, u.y + 1, new KeyValuePair<double,double>(-1.0, -1.0));
           // s.AddFirst(tempState);
            tempState = new Cell(u.x, u.y + 1, new KeyValuePair<double,double>(-1.0, -1.0));
            s.AddFirst(tempState);
         //  tempState = new Cell(u.x - 1, u.y + 1, new KeyValuePair<double,double>(-1.0, -1.0));
         //  s.AddFirst(tempState);
            tempState = new Cell(u.x - 1, u.y, new KeyValuePair<double,double>(-1.0, -1.0));
            s.AddFirst(tempState);
          //  tempState = new Cell(u.x - 1, u.y - 1, new KeyValuePair<double,double>(-1.0, -1.0));
           // s.AddFirst(tempState);
            tempState = new Cell(u.x, u.y - 1, new KeyValuePair<double,double>(-1.0, -1.0));
            s.AddFirst(tempState);
          //  tempState = new Cell(u.x + 1, u.y - 1, new KeyValuePair<double,double>(-1.0, -1.0));
          //  s.AddFirst(tempState);

            return s;
        }

        /*
         * Returns a list of all the predecessor states for state u. Since
         * this is for an 8-way connected graph, the list contains all the
         * neighbours for state u. Occupied neighbours are not added to the list
         */
        private LinkedList<Cell> getPred(Cell u)
        {
            LinkedList<Cell> s = new LinkedList<Cell>();
            Cell tempState;

            tempState = new Cell(u.x + 1, u.y, new KeyValuePair<double,double>(-1.0, -1.0));
            if (!occupied(tempState)) s.AddFirst(tempState);
            tempState = new Cell(u.x + 1, u.y + 1, new KeyValuePair<double,double>(-1.0, -1.0));
            if (!occupied(tempState)) s.AddFirst(tempState);
            tempState = new Cell(u.x, u.y + 1, new KeyValuePair<double,double>(-1.0, -1.0));
            if (!occupied(tempState)) s.AddFirst(tempState);
            tempState = new Cell(u.x - 1, u.y + 1, new KeyValuePair<double,double>(-1.0, -1.0));
            if (!occupied(tempState)) s.AddFirst(tempState);
            tempState = new Cell(u.x - 1, u.y, new KeyValuePair<double,double>(-1.0, -1.0));
            if (!occupied(tempState)) s.AddFirst(tempState);
            tempState = new Cell(u.x - 1, u.y - 1, new KeyValuePair<double,double>(-1.0, -1.0));
            if (!occupied(tempState)) s.AddFirst(tempState);
            tempState = new Cell(u.x, u.y - 1, new KeyValuePair<double,double>(-1.0, -1.0));
            if (!occupied(tempState)) s.AddFirst(tempState);
            tempState = new Cell(u.x + 1, u.y - 1, new KeyValuePair<double,double>(-1.0, -1.0));
            if (!occupied(tempState)) s.AddFirst(tempState);

            return s;
        }


        /*
         * Update the position of the agent/robot.
         * This does not force a replan.
         */
        public void updateStart(int x, int y)
        {
            s_start.x = x;
            s_start.y = y;

            k_m += heuristic(s_last, s_start);

            s_start = calculateKey(s_start);
            s_last = s_start;

        }

        /*
         * This is somewhat of a hack, to change the position of the goal we
         * first save all of the non-empty nodes on the map, clear the map, move the
         * goal and add re-add all of the non-empty cells. Since most of these cells
         * are not between the start and goal this does not seem to hurt performance
         * too much. Also, it frees up a good deal of memory we are probably not
         * going to use.
         */
        public void updateGoal(int x, int y)
        {
            //List< KeyValuePair< ipoint2, Double>> toAdd = new List<KeyValuePair<ipoint2, Double>>();
            //KeyValuePair<ipoint2, Double> tempPoint;

            //for (Map.Entry<State, CellInfo> entry : cellHash.entrySet())
            //{
            //    if (!close(entry.getValue().cost, C1))
            //    {
            //        tempPoint = new KeyValuePair<double,double>(
            //                    new ipoint2(entry.getKey().x, entry.getKey().y),
            //                    entry.getValue().cost);
            //        toAdd.add(tempPoint);
            //    }
            //}
            //cellHash.clear();
            //openHash.clear();

            //while (!openList.isEmpty())
            //    openList.poll();

            //k_m = 0;

            //s_goal.x = x;
            //s_goal.y = y;

            //CellInfo tmp = new CellInfo();
            //tmp.g = tmp.rhs = 0;
            //tmp.cost = C1;

            //cellHash.Add(s_goal, tmp);

            //tmp = new CellInfo();
            //tmp.g = tmp.rhs = heuristic(s_start, s_goal);
            //tmp.cost = C1;
            //cellHash.Add(s_start, tmp);
            //s_start = calculateKey(s_start);

            //s_last = s_start;

            //Iterator<Pair<ipoint2, Double>> iterator = toAdd.iterator();
            //while (iterator.hasNext())
            //{
            //    tempPoint = iterator.next();
            //    updateCell(tempPoint.Key.x, tempPoint.Key.y, tempPoint.Value);
            //}

            //Console.WriteLine("------------------------------------------------------------------------------");
            //Console.WriteLine("path:" + path);
            //Console.WriteLine("openList:" + openList);
            //Console.WriteLine("cellHash:");
            //Console.WriteLine(cellHash);
            //Console.WriteLine("openHash:");
            //Console.WriteLine(openHash);

        }

        /*
         * As per [S. Koenig, 2002]
         */
        private void updateVertex(Cell u)
        {
            LinkedList<Cell> s = new LinkedList<Cell>();

            if (u.neq(s_goal))
            {
                s = getSucc(u);
                double tmp = Double.PositiveInfinity;
                double tmp2;

                foreach (Cell i in s)
                {
                    tmp2 = getG(i) + cost(u, i);
                    if (tmp2 < tmp) tmp = tmp2;
                }
                if (!close(getRHS(u), tmp)) setRHS(u, tmp);
            }

            if (!close(getG(u), getRHS(u))) insert(u);
        }

        /*
         * Returns true if state u is on the open list or not by checking if
         * it is in the hash table.
         */
        private bool isValid(Cell u)
        {
            if (openHash[u] == null) return false;
            if (!close(keyHashCode(u), openHash[u])) return false;
            return true;
        }

        /*
         * Sets the G value for state u
         */
        private void setG(Cell u, double g)
        {
            makeNewCell(u);
            cellHash[u].g = g;
        }

        /*
         * Sets the rhs value for state u
         */
        private void setRHS(Cell u, double rhs)
        {
            makeNewCell(u);
            cellHash[u].rhs = rhs;
        }

        /*
         * Checks if a cell is in the hash table, if not it adds it in.
         */
        private void makeNewCell(Cell u)
        {
            if (cellHash.ContainsKey(u)) return;
            CellInfo tmp = new CellInfo();
            tmp.g = tmp.rhs = heuristic(u, s_goal);
            tmp.cost = C1;
            cellHash.Add(u, tmp);
        }

        /*
         * updateCell as per [S. Koenig, 2002]
         */
        public void updateCell(int x, int y, double val)
        {
            Cell u = new Cell();
            u.x = x;
            u.y = y;

            if ((u.eq(s_start)) || (u.eq(s_goal))) return;

            makeNewCell(u);
            cellHash[u].cost = val;
            updateVertex(u);
        }

        /*
         * Inserts state u into openList and openHash
         */
        private void insert(Cell u)
        {
            //iterator cur
            float csum;

            u = calculateKey(u);
            //cur = openHash.find(u);
            csum = keyHashCode(u);

            // return if cell is already in list. TODO: this should be
            // uncommented except it introduces a bug, I suspect that there is a
            // bug somewhere else and having duplicates in the openList queue
            // hides the problem...
            //if ((cur != openHash.end()) && (close(csum,cur->second))) return;

            openHash.Add(u, csum);
            PushNodeToOpenList(u);
        }

        /*
         * Returns the key hash code for the state u, this is used to compare
         * a state that has been updated
         */
        private float keyHashCode(Cell u)
        {
            return (float)(u.pairK.Key+ 1193 * u.pairK.Value);
        }

        /*
         * Returns true if the cell is occupied (non-traversable), false
         * otherwise. Non-traversable are marked with a cost < 0
         */
        private bool occupied(Cell u)
        {
            //if the cellHash does not contain the State u
            if (cellHash[u] == null)
                return false;
            return (cellHash[u].cost < 0);
        }

        /*
         * Euclidean cost between state a and state b
         */
        private double trueDist(Cell a, Cell b)
        {
            float x = a.x - b.x;
            float y = a.y - b.y;
            return Math.Sqrt(x * x + y * y);
        }

        /*
         * Returns the cost of moving from state a to state b. This could be
         * either the cost of moving off state a or onto state b, we went with the
         * former. This is also the 8-way cost.
         */
        private double cost(Cell a, Cell b)
        {
            int xd = Math.Abs(a.x - b.x);
            int yd = Math.Abs(a.y - b.y);
            double scale = 1;

            if (xd + yd > 1) scale = M_SQRT2;

            if (cellHash.ContainsKey(a) == false) return scale * C1;
            return scale * cellHash[a].cost;
        }

        /*
         * Returns true if x and y are within 10E-5, false otherwise
         */
        private bool close(double x, double y)
        {
            if (x == Double.PositiveInfinity && y == Double.PositiveInfinity) return true;
            return (Math.Abs(x - y) < 0.00001);
        }

        public List<Cell> getPath()
        {
            return path;
        }
    }
    class CellInfo
    {
    public double g = 0;
    public double rhs = 0;
    public double cost = 0;
        public override string ToString()
        {
            return "g=" + g + ", cost=" + cost + "\r\n";
            //		return "g=" + g + ", rhs=" + rhs + ", cost=" + cost + "\r\n";
        }    
}

class ipoint2
    {
        public int x = 0;
        public int y = 0;

        //default constructor
        public ipoint2()
        {

        }

        //overloaded constructor
        public ipoint2(int x, int y)
        {
            this.x = x;
            this.y = y;
        }
    }
}
