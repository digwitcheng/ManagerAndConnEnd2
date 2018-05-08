using AGV_V1._0.Agv;
using AGV_V1._0.Util;
using Agv.PathPlanning;
using System.Collections.Generic;
using System.Windows.Forms;
using AGV_V1._0.Algorithm;

namespace AGV_V1._0
{
    class SearchManager
    {
        ElecMap Elc;
        AgvPathPlanning astarSearch;
        //private readonly object searchLock = new object();

        private static SearchManager _instance;
        public static SearchManager Instance
        {
            get
            {
                if (_instance == null)
                {
                    _instance = new SearchManager();
                }
                return _instance;
            }
        }
        public SearchManager()
        {
            Elc = ElecMap.Instance;
            astarSearch = new AgvPathPlanning();

        }
        private int ResearchCount
        {
            get;
            set;

        }
        public void ReSearchRoute(Vehicle v)
        {
            ResearchCount++;
            if (v.Route == null || v.TPtr>= v.Route.Count-1)
            {
                return;
            }
            for (int i = v.TPtr + 2; i < v.TPtr + v.ForwordStep; i++)
            {
                if (i <= v.Route.Count - 1)
                {
                    Elc.mapnode[v.Route[i].X, v.Route[i].Y].Free(v.Id);
                }
            }
            v.BeginX = v.Route[v.TPtr].X;
            v.BeginY = v.Route[v.TPtr].Y;
            SearchRoute(v);
        }
        public void SearchRoute(Vehicle v)
        {
            v.RouteIndex = 0;
            v.cost = 0;
            v.TPtr = 0;// tFram = 0;
           // v.StopTime = v.sto;
            if (!checkXY(v))
            {
                v.CurState = State.cannotToDestination;
                MessageBox.Show("起点或终点超出地图界限");
                return;
            }
            if (v.BeginX == v.EndX && v.BeginY == v.EndX)
            {
                v.Arrive = true;
                System.Console.WriteLine("起点==终点");
                return;
            }
            ////AstarSearch astarSearch = new AstarSearch(Elc);
            List<MyPoint> scannerNode = new List<MyPoint>();
            if (!Elc.IsSpecialArea(v.BeginX, v.BeginY))
            {
                scannerNode = Elc.GetScanner();
            }
            List<MyPoint> routeList = astarSearch.Search(Elc,scannerNode, v.LockNode, v.BeginX, v.BeginY, v.EndX, v.EndY, v.Dir,v.algorithm);
            Elc.mapnode[v.BeginX, v.BeginY].Occupyed(v.Id);
            // Elc.mapnode[startX, startY].NodeCanUsed = false;//搜索完,小车自己所在的地方被小车占用           
             if (routeList.Count<1)
            {
                // MessageBox.Show("没有搜索到路线:"+v_num);
                v.CurState = State.cannotToDestination;
                //v.LockNode.cl;
            }
            else
            {
                v.Route = routeList;
                if (Elc.IsQueueEntra(v.EndX, v.EndY))
                {

                    MyPoint nextEnd = ElecMap.Instance.CalculateScannerPoint(new MyPoint(v.EndX, v.EndY));
                    List<MyPoint> addRoute = astarSearch.Search(Elc,new List<MyPoint>(), v.LockNode, v.EndX, v.EndY, nextEnd.X, nextEnd.Y, v.Dir,v.algorithm);
                    if (addRoute != null && addRoute.Count > 1)
                    {
                        for (int i = 1; i < addRoute.Count; i++)
                        {
                            v.Route.Add(addRoute[i]);
                        }
                        //v.EndX = nextEnd.X;
                        //v.EndY = nextEnd.Y;
                        v.EndLoc = "ScanArea";
                    }
                }

            }
        }
        
        bool checkXY(Vehicle v)
        {
            if (v == null)
            {
                return false;
            }
            if (Elc.IsLegalLocation(v.BeginX, v.BeginY) && Elc.IsLegalLocation(v.EndX, v.EndY))
            {
                return true;
            }
            return false;
        }


    }
}
