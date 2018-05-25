//#define moni

using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Drawing;
using System.Timers;
using Agv.PathPlanning;
using AGV_V1._0.Agv;
using AGV_V1._0.Algorithm;
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
        public IAlgorithm algorithm;
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
                int numTmp = (int)((agvInfo.CurLocation.AgvAngle.Angle + 45) % 360);
                int num = (int)(numTmp / 90.0);
                switch (num)
                {
                    case 0: return Direction.Down;
                    case 1: return Direction.Right;
                    case 2: return Direction.Up;
                    case 3: return Direction.Left;
                    default: return Direction.Down;
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


        public Vehicle(int x, int y, int v_num, bool arrive, Direction direction, VehicleConfiguration vehicleConfig)
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
            algorithm = config.PathPlanningAlgorithm;
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
            MyPoint cur = new MyPoint(RealX, RealY, dir);
            if (null!=route&&route.Count > 0)
            {
                int index = 0;
                for (; index < route.Count; index++)
                {
                    if (cur.Equals(route[index]))
                    {
                        break;
                    }
                }
                if (index >= route.Count) return;//没有找到
                ElecMap.Instance.mapnode[route[index].X, route[index].Y].NodeCanUsed = Id;
                for (int j = 0; j < index; j++)
                {
                 ElecMap.Instance.mapnode[route[j].X,route[j].Y].NodeCanUsed = -1;                
                }

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
        private bool swerverFinished = true;
        public bool SwerverFinished { get { return swerverFinished; } set { swerverFinished = value; } }

        List<MyPoint> lockPoint = new List<MyPoint>();
        /// <summary>
        /// 移动
        /// </summary>
        /// <param name="Elc"></param>
        /// <returns></returns>
        public MoveType Move(ElecMap Elc)
        {
            lock (RouteLock)
            {
                if (route == null || route.Count < 1)
                {
                    return MoveType.arrived;
                }
                if (TPtr >= route.Count - 1)
                {
                    Elc.mapnode[route[route.Count - 1].X, route[route.Count - 1].Y].NodeCanUsed = this.Id;
                    Arrive = true;
                    return MoveType.arrived;
                    //Elc.mapnode[route[route.Count - 1].X, route[route.Count - 1].Y].NodeCanUsed = this.Id;
                    //if (EqualWithRealLocation(route[route.Count - 1].X, route[route.Count - 1].Y))
                    //{
                    //    Arrive = true;
                    //    return MoveType.arrived;
                    //}
                    //else
                    //{
                    //    return MoveType.move;
                    //}
                }
#if moni

#else
                if (!CheckAgvCorrect()) { return MoveType.agvFault; }

                ////TODO://如果当前方向就是小车方向给它发转弯是否会出错？转向完成，翻盘完成的标志是否要用？
                //if (!swerverFinished)
                //{
                //    return MoveType.cannotReceiveRunCommands;
                //}
                //dir = GetAgvDireciton();
                //Console.WriteLine(dir.ToString());
                //if (TPtr == 0 && route[0].Dir != dir)
                //{

                //    if (AgvCanReceiveSwerverCommands())
                //    {
                //        return MoveType.move;
                //        swerverFinished = false;
                //        return GetSwerveAngle(route[0].Dir);
                //    }
                //    else
                //    {
                //        return MoveType.cannotReceiveSwerverCommands;
                //    }
                //}
                //if (route[TPtr + 1].Dir != dir)
                //{
                //    if (AgvCanReceiveSwerverCommands())
                //    {
                //        return MoveType.move;
                //        swerverFinished = false;
                //        return GetSwerveAngle(route[TPtr + 1].Dir);
                //    }
                //    else
                //    {
                //        return MoveType.cannotReceiveSwerverCommands;
                //    }
                //}
                if (!AgvCanReceiveRunCommands())
                {
                    return MoveType.cannotReceiveRunCommands;
                }
                if (ShouldMove(TPtr + 1) == false)
                {
                    BeginX = route[TPtr].X;
                    BeginY = route[TPtr].Y;
                    if (this.WaitEndTime < DateTime.Now)//超过等待时间还不能走，则重新发送一下当前位置
                    {
                        Console.WriteLine("Resend Current location");
                        return MoveType.move;
                    }
                    return MoveType.cannotReceiveRunCommands;
                }
#endif
                MyPoint cur = new MyPoint(RealX, RealY, dir);
                
                    int index = 0;
                    for (; index < route.Count; index++)
                    {
                        if (cur.Equals(route[index]))
                        {
                            break;
                        }
                    }
                if (index >= route.Count)
                {
                    index--;
                }
                bool canMove = false;
                Direction virtualDir = route[index+1].Dir;
                for (VirtualTPtr = index + 1; VirtualTPtr < index + config.ForwordStep; VirtualTPtr++)
                {
                    if (VirtualTPtr <= route.Count - 1)
                    {
                        int tx = (int)route[VirtualTPtr].X;
                        int ty = (int)route[VirtualTPtr].Y;
                        Direction tDir = route[VirtualTPtr].Dir;
                        Boolean IsCanMoveTo = Elc.IsVehicleCanMove(this, tx, ty);// Elc.mapnode[tx, ty].NodeCanUsed;
                        if (IsCanMoveTo&&virtualDir==tDir)
                        {
                            canMove = true;
                            virtualDir = tDir;
                            Elc.mapnode[tx, ty].NodeCanUsed = this.Id;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
                if (canMove)
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
                    TPtr++;
                    BeginX = route[TPtr].X;
                    BeginY = route[TPtr].Y;
#endif
                    return MoveType.move;
                }
                else
                {
                    StopTime--;
                    return MoveType.stopAvoidConflict;
                }
                #region old
                /*
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
                */
                #endregion
            }
        }
        MoveType GetSwerveAngle(Direction direction)
        {
            switch (direction)
            {
                case Direction.Down: return MoveType.Swerve0;
                case Direction.Right: return MoveType.Swerve90;
                case Direction.Up: return MoveType.Swerve180;
                case Direction.Left: return MoveType.Swerve270;
                default: return MoveType.Swerve0;
            }
        }


        bool ShouldMove(int nextTPtr)
        {
            if (nextTPtr >= route.Count || nextTPtr <= 0)
            {
                return false;
            }
            int nextX = route[nextTPtr].X;
            int nextY = route[nextTPtr].Y;
            UpdateRealLocation();

            double distanceX = Math.Abs(nextX * 1000 - agvInfo.CurLocation.CurNode.X);
            double distanceY = Math.Abs(nextY * 1000 - agvInfo.CurLocation.CurNode.Y);
            if (distanceX < config.Deviation + 1000 && distanceY < config.Deviation ||
                distanceX < config.Deviation && distanceY < config.Deviation + 1000
                )//相邻
            {
                if (OrderExecState.Free == agvInfo.OrderExec)
                {
                    return true;
                }
                else//不是停止状态需要判断小车方向
                {
                    if (dir != route[nextTPtr].Dir)//方向不同，必须等它停止
                    {
                        return false;
                    }
                    else//方向相同，说明在一条直线上，
                    {
                        return true;
                    }
                }
            }
            else//需要发送的位置和当前小车实际所在的位置不相邻,必须在小车运动时才能发，防止启动的时候连发
            {
                if (OrderExecState.Run != agvInfo.OrderExec)
                {
                    return false;
                }
                if (distanceX < config.Deviation + (config.ForwordStep - 1) * 1000 && distanceY < config.Deviation ||//X轴移动
                    distanceX < config.Deviation && distanceY < config.Deviation + (config.ForwordStep - 1) * 1000 //Y
                    )
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            if (dir != route[nextTPtr].Dir)//方向不同必须只能当前小车的位置和下一个相邻并且小车处于停止状态
            {
                if (OrderExecState.Free != agvInfo.OrderExec)
                {
                    return false;
                }
                if (Math.Abs(nextX * 1000 - agvInfo.CurLocation.CurNode.X) < config.Deviation +  1000 &&
                    Math.Abs(nextY * 1000 - agvInfo.CurLocation.CurNode.Y) < config.Deviation)//X轴移动
                {
                    return true;
                }
                if (Math.Abs(nextX * 1000 - agvInfo.CurLocation.CurNode.X) < config.Deviation &&
                    Math.Abs(nextY * 1000 - agvInfo.CurLocation.CurNode.Y) < config.Deviation +  1000)//Y轴移动
                {
                    return true;
                }
            }
            else { //方向相同可发送探测范围内的点并且小车处于运动状态               
                if (Math.Abs(nextX * 1000 - agvInfo.CurLocation.CurNode.X) < config.Deviation + (config.ForwordStep - 1) * 1000 &&
                    Math.Abs(nextY * 1000 - agvInfo.CurLocation.CurNode.Y) < config.Deviation)//X轴移动
                {
                    return true;
                }
                if (Math.Abs(nextX * 1000 - agvInfo.CurLocation.CurNode.X) < config.Deviation &&
                    Math.Abs(nextY * 1000 - agvInfo.CurLocation.CurNode.Y) < config.Deviation + (config.ForwordStep - 1) * 1000)//Y轴移动
                {
                    return true;
                }
            }
            return false;

        }
        bool AgvCanReceiveRunCommands()
        {
            Console.WriteLine("agv当前指令执行：" + agvInfo.OrderExec.ToString());
            if (OrderExecState.Run == agvInfo.OrderExec||
                agvInfo.OrderExec == OrderExecState.Free
                )
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        bool AgvCanReceiveSwerverCommands()
        {
            if (OrderExecState.Free == agvInfo.OrderExec
                )
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        //public  void SetCurDirectionEqualNext(byte serinum)
        // {
        //     curMoveDirection = nextMoveDirection;

        // }
        //void SetCurNextDirection(int index)
        //{
        //    if(SwerveStoped==false)return;
        //    if (Math.Abs(route[index].X - route[index - 1].X) == 0 && Math.Abs(route[index].Y - route[index - 1].Y) == 1)//Y轴方向
        //    {
        //        curMoveDirection = nextMoveDirection;
        //        nextMoveDirection = MoveDirecion.YDirection;
        //    }
        //    if (Math.Abs(route[index].X - route[index - 1].X) == 1 && Math.Abs(route[index].Y - route[index - 1].Y) == 0)//X轴方向
        //    {
        //        curMoveDirection = nextMoveDirection;
        //        nextMoveDirection = MoveDirecion.XDirection;
        //    }
        //}
        Direction GetNextDirection(MyPoint cur, MyPoint next)
        {

            if (cur.X - next.X == 1)
            {
                return Direction.Up;// 3 North;
            }
            if (cur.X - next.X == -1)
            {
                return Direction.Down;// 1;//South;
            }
            if (cur.Y - next.Y == 1)
            {
                return Direction.Left;// 2;//West;
            }
            if (cur.Y - next.Y == -1)
            {
                return Direction.Right;// 0;// East;
            }
            return dir;
        }
        void UpdateRealLocation()
        {
            if (!CheckAgvCorrect()) { return; }
            RealX = (int)Math.Round(agvInfo.CurLocation.CurNode.X / 1000.0);
            RealY = (int)Math.Round(agvInfo.CurLocation.CurNode.Y / 1000.0);
            ElecMap.Instance.mapnode[RealX, RealY].NodeCanUsed = this.Id;
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
