//#define moni

using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Drawing;
using System.Timers;
using Agv.PathPlanning;
using AGV_V1._0.Agv;
using AGV_V1._0.Util;
using AGVSocket.Network;
using AGVSocket.Network.EnumType;

namespace AGV_V1._0
{

    class Vehicle
    {
        public AgvInfo agvInfo { get; set; }
        private readonly VehicleConfiguration config;
        private Timer timer;
        private int routeIndex = 0;
        public int RouteIndex
        {
            get { return routeIndex; }
            set
            {
                if (route != null)
                {
                    if (value > route.Count - 1)
                    {
                        value = route.Count - 1;
                    }
                    if (value < 0)
                    {
                        value = 0;
                    }
                }
                else
                {
                    value = 0;
                }

                routeIndex = value;
            }
        }
        private int stopTime;
        public int StopTime
        {
            get { return stopTime; }
            set { stopTime = value; }
        }

        private int stoped = -1;//大于0表示被某个小车锁死，停止了。
        public int Stoped { get; set; }

        //判断小车是否到终点
        public bool Arrive
        {
            get;
            set;
        }

        //小车编号
        public int Id
        {
            get;
            private set;
        }

        public State CurState
        {
            get;
            set;
        }
        public int ForwordStep { get; set; }
        public DateTime WaitEndTime;

        //public List<myPoint> route;//起点到终点的路线
        //public ConcurrentDictionary<int, MyLocation> Route { get; set; }//起点到终点的路线, 键表示时钟指针
        private static Object RouteLock = new Object();
        //private  int LockNode = -1;  //-1节点没有被锁定，大于-1表示被锁定


        private static object lockNodeLock = new object();
        private List<MyPoint> lockNode = new List<MyPoint>();
        public List<MyPoint> LockNode
        {
            get
            {
                return lockNode;
            }
            set
            {
                this.lockNode = value;

            }
            //get
            //{
            //    lock (lockNodeLock)
            //    {
            //        return lockNode;
            //    }
            //}
            //set
            //{
            //    lock (lockNodeLock)
            //    {
            //        this.lockNode = value;

            //    }
            //}
        }
        private List<MyPoint> route = new List<MyPoint>();
        public List<MyPoint> Route
        {
            get
            {
                lock (RouteLock)
                {
                    return route;
                }
            }
            set
            {
                lock (RouteLock)
                {
                    this.route = value;

                }
            }
        }

        //起点到终点的路线, 键表示时钟指针
        public int cost;   //截止到当前时间，总共的花费

        private Direction dir;

        public Direction Dir
        {
            get
            {
                return GetAgvDireciton();
            }
            set { dir = value; }

        }

        private Direction GetAgvDireciton()
        {
            if (agvInfo == null)
            {
                return dir;
            }
            else
            {
                int num = (int)((agvInfo.CurLocation.AgvAngle.Angle + 45) % 360 / 90.0);
                switch (num)
                {
                    case 0: return Direction.Right;
                    case 1: return Direction.Up;
                    case 2: return Direction.Left;
                    case 3: return Direction.Down;
                    default: return Direction.Up;
                }
            }
        }
        public int EndX { get; set; }
        public int EndY { get; set; }
        public int BeginX { get; set; }
        public int BeginY { get; set; }
        public int DestX { get; set; }
        public int DestY { get; set; }
        public int RealX { get; set; }
        public int RealY { get; set; }


        //  public float Distance;//上一个节拍所走的距离；
        // public int stopTime;//停留时钟数

        public MyPoint Location
        {
            get;
            set;
        }
        //小车的电量
        public int Electricity
        {
            get;
            set;
        }

        //小车的加速度
        public float Acceleration
        {
            get;
            set;
        }

        //小车的速度
        public float Speed
        {
            get;
            set;
        }

        //小车的最大速度
        public float MaxSpeed
        {
            get;
            set;
        }
        //车的横坐标
        public int X
        {
            get;
            set;
        }

        //车的纵坐标
        public int Y
        {
            get;
            set;
        }

        public Color pathColor = Color.Red;
        public Color showColor = Color.Pink;


        
        private ConcurrentQueue<MyPoint> crossedPoint = new ConcurrentQueue<MyPoint>();
        public int VirtualTPtr
        {
            get;
            set;
        }

        private int tPtr;
        public int TPtr
        {
            get
            {
                return tPtr;
            }
            set
            {
                if (value < 0)
                {
                    tPtr = 0;
                }
                else
                {
                    tPtr = value;
                }
            }

        }//时钟指针

        public string StartLoc
        {
            get;
            set;
        }
        public string EndLoc
        {
            get;
            set;
        }


        public Vehicle(int x, int y, int v_num, bool arrive, Direction direction,VehicleConfiguration vehicleConfig)
        {
            this.BeginX = x;
            this.BeginY = y;
            this.X = y * ConstDefine.g_NodeLength;
            this.Y = x * ConstDefine.g_NodeLength;
            this.Id = v_num;
            this.Arrive = arrive;
            this.Dir = direction;
            this.config = vehicleConfig;
            this.timer = new Timer();
            InitTimer();
            InitAgv();
        }
        void InitAgv()
        {
            StopTime = config.StopTime;
            ForwordStep = config.ForwordStep;
        }
        void InitTimer()
        {
            this.timer.Interval = config.TimerInterval;
            this.timer.Elapsed += Timer_Elapsed;
            this.timer.Start();
        }

        private void Timer_Elapsed(object sender, ElapsedEventArgs e)
        {
            UpdateRealLocation();

            SetCurrentNodeOccpyAndOldNodeFree();

        }
        void SetCurrentNodeOccpyAndOldNodeFree()
        {
            MyPoint cur = new MyPoint(RealX, RealY);
            if (crossedPoint.Count >0)
            {
                IEnumerator<MyPoint> it = crossedPoint.GetEnumerator();
                int index = 0;
                MyPoint crossed = it.Current;
                while (!cur.Equals(crossed)&&it.MoveNext())
                {
                    crossed = it.Current;
                    index++;
                }
                if (!cur.Equals(crossed))//遍历结束都没有找到等于当前真实坐标的，说明队列中的点都还没走过
                {
                    return;
                }
                for (int j = 0; j < index-1; j++)
                {
                    MyPoint realCrossedPoint = null;
                    bool success = crossedPoint.TryDequeue(out realCrossedPoint);
                    if (success)
                    {
                        ElecMap.Instance.mapnode[realCrossedPoint.X, realCrossedPoint.Y].NodeCanUsed = -1;
                    }
                }


                //MyPoint[] crossedList=crossedPoint.ToArray();
                //for (int i = 0; i < crossedList.Length; i++)
                //{
                //    MyPoint crossed = crossedList[i];
                //    if (cur.Equals(crossed))
                //    {
                //        for (int j = 0; j < i; j++)
                //        {
                //            MyPoint realCrossedPoint = null;
                //            bool success= crossedPoint.TryDequeue(out realCrossedPoint);
                //            if (success)
                //            {
                //                ElecMap.Instance.mapnode[realCrossedPoint.X, realCrossedPoint.Y].NodeCanUsed = -1;
                //            }
                //        }
                //    }
                //}
            }
            
            
           
        }

        //public int TPtr
        //{
        //    get { return TPtr; }
        //    set { TPtr = value; }
        //}


        public MapNodeType CurNodeTypy()
        {
            return ElecMap.Instance.mapnode[BeginX, BeginY].Type;
        }
        /// <summary>
        /// 移动
        /// </summary>
        /// <param name="Elc"></param>
        /// <returns>是否移动了</returns>
        public bool Move(ElecMap Elc)
        {
            lock (RouteLock)
            {
                if (route == null || route.Count < 1)
                {
                    return false;
                }
                if (TPtr >= route.Count - 1)
                {
                    Elc.mapnode[route[route.Count - 1].X, route[route.Count - 1].Y].NodeCanUsed = this.Id;
                    Arrive = true;                    
                    return false;
                }
#if moni

#else

                if (ShouldMove(TPtr + 1) == false)
                {
                    //BeginX = route[TPtr].X;
                    //BeginY = route[TPtr].Y;
                    //if (this.WaitEndTime < DateTime.Now)//超过等待时间还不能走，则重新发送一下当前位置
                    //{
                    //    Console.WriteLine("Resend Current location");
                    //    return true;
                    //}
                    return false;
                }
#endif
                List<MyPoint> lockPoint = new List<MyPoint>();
                for (VirtualTPtr = TPtr+1; VirtualTPtr <TPtr + config.ForwordStep; VirtualTPtr++)
                {
                    if (VirtualTPtr <= route.Count - 1)
                    {
                        int tx = (int)route[VirtualTPtr].X;
                        int ty = (int)route[VirtualTPtr].Y;
                        Boolean IsCanMoveTo = Elc.IsVehicleCanMove(this, tx, ty);// Elc.mapnode[tx, ty].NodeCanUsed;
                        if (IsCanMoveTo)
                        {
                            lockPoint.Add(new MyPoint(tx, ty));
                            Elc.mapnode[tx, ty].NodeCanUsed = this.Id;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
                if (lockPoint.Count>0)
                {
#if moni
                    //if (TPtr == 0)
                    //{
                    //    ElecMap.Instance.mapnode[BeginX, BeginY].NodeCanUsed = -1;
                    //}
                    ElecMap.Instance.mapnode[BeginX, BeginY].NodeCanUsed = -1;
                     TPtr++;
                    BeginX = route[TPtr].X;
                    BeginY = route[TPtr].Y;
#else               
                    crossedPoint.Enqueue(new MyPoint(BeginX, BeginY));  
                    TPtr++;
                    BeginX = route[TPtr].X;
                    BeginY = route[TPtr].Y;
                    if (BeginX == EndX && BeginY == EndY)
                    {
                        crossedPoint.Enqueue(new MyPoint(BeginX, BeginY));
                    }
#endif
                    return true;
                }
                else
                {
                    StopTime--;
                    return false;
                }


                if (TPtr == 0)// config.ForwordStep)
                {

                    for (VirtualTPtr = 1; VirtualTPtr < config.ForwordStep; VirtualTPtr++)
                    {
                        if (TPtr + VirtualTPtr <= route.Count - 1)
                        {
                            int tx = (int)route[VirtualTPtr].X;
                            int ty = (int)route[VirtualTPtr].Y;
                            int temp = Elc.mapnode[tx, ty].NodeCanUsed;
                            if (temp > -1)
                            {
                                Stoped = temp;
                                StopTime--;
                                return false;
                            }
                            else
                            {
                                Elc.mapnode[tx, ty].NodeCanUsed = this.Id;
                            }
                        }
                    }
                    StopTime = StopTime;
                    TPtr++;

                }
                else if (TPtr > 0)
                {

                    if (VirtualTPtr <= route.Count - 1)
                    {
                        int tx = (int)route[VirtualTPtr].X;
                        int ty = (int)route[VirtualTPtr].Y;
                        int temp = Elc.mapnode[tx, ty].NodeCanUsed;
                        if (temp > -1)
                        {
                            Stoped = temp;
                            StopTime--;
                            return false;
                        }
                        else
                        {
                            Elc.mapnode[tx, ty].NodeCanUsed = this.Id;
                            StopTime = StopTime;
                            TPtr++;
                            VirtualTPtr++;
                        }

                    }
                    else
                    {
                        StopTime = StopTime;
                        TPtr++;
                    }
                }
                BeginX = route[TPtr].X;
                BeginY = route[TPtr].Y;
                return true;

            }
        }


        private enum MoveDirecion { XDirection, YDirection }
        private MoveDirecion curMoveDirection;
        private MoveDirecion nextMoveDirection;
        private bool SwerveStoped = true;

        bool ShouldMove(int nextTPtr)
        {
            if (!CheckAgvCorrect()) { return false; }
            if (nextTPtr >= route.Count)
            {
                return false;
            }
            if (nextTPtr == 0)
            {
                return true;
            }
            SetCurAndNextDirection(nextTPtr);
            if (nextTPtr > 1)
            {
                if (curMoveDirection != nextMoveDirection)
                {
                    if (agvInfo.AgvMotion == AgvMotionState.StopedNode)
                    {
                        Console.WriteLine("stoped!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1");
                        SwerveStoped = true;
                        curMoveDirection = nextMoveDirection;
                    }
                    else
                    {
                        Console.WriteLine("还没停止...");
                        SwerveStoped = false;
                        return false;
                    }
                }
            }

            int nextX = route[nextTPtr].X;
            int nextY = route[nextTPtr].Y;
            UpdateRealLocation();
            //int RealX = (int)Math.Round(agvInfo.CurLocation.CurNode.X / 1000.0);
            //int RealY = (int)Math.Round(agvInfo.CurLocation.CurNode.Y / 1000.0);
            if (Math.Abs(nextX - RealX) < config.Deviation + config.ForwordStep - 1 && Math.Abs(nextY - RealY) < config.Deviation)//X轴移动
            {
                return true;
            }
            if (Math.Abs(nextX - RealX) < config.Deviation && Math.Abs(nextY - RealY
                ) < config.Deviation + config.ForwordStep - 1)//Y轴移动
            {
                return true;
            }
            return false;

        }
        //public  void SetCurDirectionEqualNext(byte serinum)
        // {
        //     curMoveDirection = nextMoveDirection;

        // }
        void SetCurAndNextDirection(int index)
        {
            if (SwerveStoped == false)
            {
                return;
            }
            if (Math.Abs(route[index].X - route[index - 1].X) == 0 && Math.Abs(route[index].Y - route[index - 1].Y) == 1)//Y轴方向
            {
                curMoveDirection = nextMoveDirection;
                nextMoveDirection = MoveDirecion.YDirection;
            }
            if (Math.Abs(route[index].X - route[index - 1].X) == 1 && Math.Abs(route[index].Y - route[index - 1].Y) == 0)//X轴方向
            {
                curMoveDirection = nextMoveDirection;
                nextMoveDirection = MoveDirecion.XDirection;
            }
        }
        void UpdateRealLocation()
        {
            if (!CheckAgvCorrect()) { return; }
            RealX = (int)Math.Round(agvInfo.CurLocation.CurNode.X / 1000.0);
            RealY = (int)Math.Round(agvInfo.CurLocation.CurNode.Y / 1000.0);
            ElecMap.Instance.mapnode[RealX, RealY].NodeCanUsed =this.Id;
        }
        bool CheckAgvCorrect()
        {
            if (agvInfo == null)
            {
                return false;
            }
            if (agvInfo.Alarm != AlarmState.Normal)
            {
                return false;
            }
            //if (agvInfo.Electricity < 20)
            //{
            //    return false;
            //}
            return true;
        }
        public bool EqualWithRealLocation(int srcX, int srcY)
        {
            if (!CheckAgvCorrect()) { return false; }
            double tempX = agvInfo.CurLocation.CurNode.X / 1000.0;
            double tempY = agvInfo.CurLocation.CurNode.Y / 1000.0;
            if (Math.Abs(srcX - tempX) < config.Deviation && Math.Abs(srcY - tempY) < config.Deviation)
            {
                return true;
            }
            if (Math.Abs(srcX - tempX) < config.Deviation && Math.Abs(srcY - tempY
                ) < config.Deviation)
            {
                return true;
            }
            return false;

        }
    }
}
