﻿//#define moni

using AGV_V1._0.Agv;
using AGV_V1._0.Algorithm;
using AGV_V1._0.Event;
using AGV_V1._0.Network.ThreadCode;
using AGV_V1._0.NLog;
using AGV_V1._0.Queue;
using AGV_V1._0.Util;
using AGVSocket.Network;
using AGVSocket.Network.EnumType;
using AGVSocket.Network.Packet;
using Agv.PathPlanning;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Threading;
using System.Windows.Forms;

namespace AGV_V1._0
{
    class VehicleManager : BaseThread
    {
        //新建两个全局对象  小车
        private static Vehicle[] vehicles;
        List<Vehicle> vFinished = new List<Vehicle>();
        private bool vehicleInited = false;
        private double moveCount = 0;//统计移动的格数，当前地图一格1.5米
        public const int REINIT_COUNT = 20;
        private const int WAIT_TIME = 4;//  等待超时后还没有翻盘完成的消息就重发翻盘报文

        private static Random rand = new Random(1);//5,/4/4 //((int)DateTime.Now.Ticks);//随机数，随机产生坐标


        private static VehicleManager instance;
        public static VehicleManager Instance
        {
            get
            {
                if (null == instance)
                {
                    instance = new VehicleManager();
                }
                return instance;
            }
        }
        private VehicleManager()
        {

        }
        protected override string ThreadName()
        {
            return "VehicleManager";
        }
        byte serinum = 10;
        protected override void Run()
        {
#if moni
            Thread.Sleep(100);
#else
            Thread.Sleep(ConstDefine.STEP_TIME);
#endif

            if (vehicles == null)
            {
                return;
            }
            for (int vnum =4; vnum < vehicles.Length-1; vnum++)
            {
                serinum = (byte)(serinum % 255);
                if (vehicles[vnum].CurState == State.cannotToDestination && vehicles[vnum].Arrive == false)
                {
                    vehicles[vnum].Arrive = true;
                    vFinished.Add(vehicles[vnum]);

                    string str = string.Format("小车" + vnum + ":{0},{1}->{2},{3}没有搜索到路径，", vehicles[vnum].BeginX, vehicles[vnum].BeginY, vehicles[vnum].EndX, vehicles[vnum].EndY);
                    OnShowMessage(this, new MessageEventArgs(str));
                    continue;
                }
                if (vehicles[vnum].Finished == true)
                {
                    continue;
                }
#if moni
                if (vehicles[vnum].Arrive == true && vehicles[vnum].CurState == State.carried)
                {
                    //vehicle[vnum].BeginX = vehicle[vnum].EndX;
                    //vehicle[vnum].BeginY = vehicle[vnum].EndY;
                    vehicles[vnum].CurState = State.unloading;
                    vFinished.Add(vehicles[vnum]);
                    vehicles[vnum].Route.Clear();
                    vehicles[vnum].LockNode.Clear();
                    continue;
                }
                if (vehicles[vnum].Arrive == true)
                {
                    //vehicle[vnum].BeginX = vehicle[vnum].EndX;
                    //vehicle[vnum].BeginY = vehicle[vnum].EndY;
                    //vehicle[vnum].vehical_state = State.unloading;
                    vFinished.Add(vehicles[vnum]);
                    vehicles[vnum].Route.Clear();
                    vehicles[vnum].LockNode.Clear();

                    continue;
                }
#else
                if (vehicles[vnum].Arrive == true && vehicles[vnum].CurState == State.carried)
                {
                    if (vehicles[vnum].EqualWithRealLocation(vehicles[vnum].BeginX, vehicles[vnum].BeginY))
                    {
                        if (vehicles[vnum].agvInfo.AgvMotion == AgvMotionState.StopedNode)
                        {
                            TrayPacket tp = new TrayPacket((byte)(serinum *vnum), (ushort)vnum, TrayMotion.TopLeft);
                            AgvServerManager.Instance.SendTo(tp,vnum);
                            vehicles[vnum].CurState = State.unloading;
                            vehicles[vnum].WaitEndTime = DateTime.Now.AddSeconds(WAIT_TIME*2);
                            Console.WriteLine(vnum+":send TrayMotion:"+(serinum-1));
                        }
                        continue;
                    }      
                    
                   
                }
                //if (vehicles[vnum].Arrive == true && vehicles[vnum].CurState == State.unloading)
                //{
                //    if (vehicles[vnum].WaitEndTime < DateTime.Now)//超过等待时间还不能走，则重新发送一下倒货
                //    {
                //        if (vehicles[vnum].EqualWithRealLocation(vehicles[vnum].BeginX, vehicles[vnum].BeginY))
                //        {
                //            if (vehicles[vnum].agvInfo.AgvMotion == AgvMotionState.StopedNode)
                //            {
                //                TrayPacket tp = new TrayPacket((byte)(serinum * vnum), (ushort)vnum, TrayMotion.TopLeft);
                //                AgvServerManager.Instance.SendTo(tp, vnum);
                //                vehicles[vnum].CurState = State.unloading;
                //                vehicles[vnum].WaitEndTime = DateTime.Now.AddSeconds(WAIT_TIME);
                //                Console.WriteLine(vnum + ":resend TrayMotion:" + (serinum - 1));
                //            }
                //            continue;
                //        }
                //    }
                //}
               if (vehicles[vnum].Arrive == true && vehicles[vnum].CurState == State.Free)
                {    
                     vFinished.Add(vehicles[vnum]);
                     vehicles[vnum].LockNode.Clear();
                    vehicles[vnum].Finished=true;
                     continue;
                }        
#endif
                if (vehicles[vnum].StopTime < 0)
                {
                    if (vehicles[vnum].CurNodeTypy() != MapNodeType.queuingArea && GetDirCount(vehicles[vnum].BeginX, vehicles[vnum].BeginY) > 1)
                    {
                        if (vehicles[vnum].Stoped > -1 && vehicles[vnum].Stoped < vehicles.Length)
                        {
                            vehicles[vehicles[vnum].Stoped].StopTime = 2;
                        }
                        //重新搜索路径
                        SearchRoute(vnum, true);
                    }
                    vehicles[vnum].StopTime = 3;
                }
                else
                {

#if moni
                    vehicles[vnum].Move(ElecMap.Instance);
#else
                   MoveType moveState = vehicles[vnum].Move(ElecMap.Instance);
                    if (moveState==MoveType.move)
                        {
                        //int tPtr = vehicles[vnum].TPtr - 1;
                        //if (tPtr >= 0)
                        //{
                        //    ElecMap.Instance.mapnode[vehicles[vnum].Route[tPtr].X, vehicles[vnum].Route[tPtr].Y].NodeCanUsed = -1;
                        //}
                           uint x = Convert.ToUInt32(vehicles[vnum].BeginX);
                            uint y = Convert.ToUInt32(vehicles[vnum].BeginY);
                            uint endX = Convert.ToUInt32(vehicles[vnum].EndX);
                            uint endY = Convert.ToUInt32(vehicles[vnum].EndY);                            
                            RunPacket rp = new RunPacket((byte)(serinum * vnum), (ushort)vnum, MoveDirection.Forward, 1500, new Destination(new CellPoint(x * ConstDefine.CELL_UNIT, y * ConstDefine.CELL_UNIT), new CellPoint(endX * ConstDefine.CELL_UNIT, endY * ConstDefine.CELL_UNIT), new AgvDriftAngle(0), TrayMotion.None));
                            AgvServerManager.Instance.SendTo(rp,vnum);

                            Console.WriteLine("*--------------------------------------------------------------------------*");
                            Console.WriteLine(vnum+":"+x + "," + y + "->" + endX + "," + endY + " ,实际位置：" + vehicles[vnum].agvInfo.CurLocation.CurNode.X / 1000.0 + "," + vehicles[vnum].agvInfo.CurLocation.CurNode.Y / 1000.0+"序列号："+(serinum-1));

                            CheckAlarmState(vnum);
                            vehicles[vnum].WaitEndTime = DateTime.Now.AddSeconds(WAIT_TIME);

                        ElecMap.Instance.mapnode[vehicles[vnum].BeginX, vehicles[vnum].BeginY].TraCongesIntensity = 100;
                        }
                    //if (moveState == MoveType.Swerve0)
                    //{
                    //    SendSwerveCommand(vnum,0);
                    //}
                    //if (moveState == MoveType.Swerve90)
                    //{
                    //    SendSwerveCommand(vnum, 90);
                    //}
                    //if (moveState == MoveType.Swerve180)
                    //{
                    //    SendSwerveCommand(vnum, 180);
                    //}
                    //if (moveState == MoveType.Swerve270)
                    //{
                    //    SendSwerveCommand(vnum, 270);
                    //}
#endif
                    moveCount++;
                    OnShowMessage(string.Format("{0:N} 公里", (moveCount * 1.5) / 1000.0));
                }




            }
            if (vFinished != null)
            {
                for (int i = 0; i < vFinished.Count; i++)
                {
                    FinishedQueue.Instance.Enqueue(vFinished[i]);
                }
                vFinished.Clear();
            }

        }
        //void ClearCrossedNode(int vnum)
        //{
        //    for (int i = 0; i < vehicles[vnum].Route.Count-1; i++)
        //    {
        //        ElecMap.Instance.mapnode[vehicles[vnum].Route[i].X, vehicles[vnum].Route[i].Y].NodeCanUsed = -1;
        //    }
        //}
        void SendSwerveCommand(int vnum,int angle)
        {
            SwervePacket sp = new SwervePacket((byte)(serinum * vnum), (ushort)vnum,new AgvDriftAngle((ushort)angle));
            AgvServerManager.Instance.SendTo(sp, vnum);
            Console.WriteLine("send Swerver...");
        }
        void CheckAlarmState(int vnum)
        {
            if (vehicles[vnum].agvInfo.Alarm == AlarmState.ScanNone || vehicles[vnum].agvInfo.Alarm == AlarmState.CommunicationFault)
            {
                Logs.Error("通信故障或没扫到码"+vehicles[vnum].agvInfo.Alarm.ToString());
               // MessageBox.Show("通信故障或没扫到码");
            }
        }
        
       
        int GetDirCount(int row, int col)
        {
            int dir = 0;
            if (ElecMap.Instance.mapnode[row, col].RightDifficulty < MapNode.MAX_ABLE_PASS)
            {
                dir++;
            }
            if (ElecMap.Instance.mapnode[row, col].LeftDifficulty < MapNode.MAX_ABLE_PASS)
            {
                dir++;
            }
            if (ElecMap.Instance.mapnode[row, col].DownDifficulty < MapNode.MAX_ABLE_PASS)
            {
                dir++;
            }
            if (ElecMap.Instance.mapnode[row, col].UpDifficulty < MapNode.MAX_ABLE_PASS)
            {
                dir++;
            }
            return dir;
        }
        /// <summary>
        /// 初始化小车
        /// </summary>
        public void InitialVehicle()
        {
            vehicleInited = false;
            //初始化小车位置

            if (null == FileUtil.sendData || FileUtil.sendData.Length < 1)
            {
                throw new ArgumentNullException();
            }
            int vehicleCount = FileUtil.sendData.Length;
            vehicles = new Vehicle[vehicleCount];
            for (int i = 0; i < vehicleCount; i++)
            {
                VehicleConfiguration config = new VehicleConfiguration();
                vehicles[i] = new Vehicle(FileUtil.sendData[i].BeginX, FileUtil.sendData[i].BeginY, i, false, Direction.Right,config);
                //MyPoint endPoint = RouteUtil.RandPoint(ElecMap.Instance);
                //MyPoint mp = SqlManager.Instance.GetVehicleCurLocationWithId(i);
                //if (mp != null)
                //{
                //    vehicles[i].BeginX = mp.X;
                //    vehicles[i].BeginY = mp.Y;
                //}
                int R = rand.Next(20, 225);
                int G = rand.Next(20, 225);
                int B = rand.Next(20, 225);
                vehicles[i].pathColor = Color.FromArgb(80, R, G, B);
                vehicles[i].showColor = Color.FromArgb(255, R, G, B);
            }

            vehicleInited = true;
            ////把小车所在的节点设为占用状态
            RouteUtil.VehicleOcuppyNode(ElecMap.Instance, vehicles);

        }
        public void ReInitWithiRealAgv()
        {
            
            bool res = false;
            int count=1;
            while (res == false && count < REINIT_COUNT)
            {
                Thread.Sleep(count * 50);
                for (int i = 0; i < vehicles.Length; i++)
                {
                    if (vehicles[i].agvInfo != null)
                    {
                        vehicles[i].BeginX = (int)Math.Round(vehicles[i].agvInfo.CurLocation.CurNode.X/1000.0);
                        vehicles[i].BeginY = (int)Math.Round(vehicles[i].agvInfo.CurLocation.CurNode.Y/1000.0);
                        res = true;
                        Console.WriteLine("小车编号" + i + "初始化完成,起点("+vehicles[i].BeginX+","+vehicles[i].BeginY+")");
                    }
                }
                count++;
            }
            ////把小车所在的节点设为占用状态
            RouteUtil.VehicleOcuppyNode(ElecMap.Instance, vehicles);
            if (count >= REINIT_COUNT)
            {
                MessageBox.Show("没有小车连接，请检查ip设置是否有问题");
            }
        }
        public void AddOrUpdate(ushort agvId, AgvInfo info)
        {
            if (vehicleInited == false)
            {
                return;
            }
            //if (agvId >= vehicles.Length)
            //{
            //    Logs.Error("程序预设的agv数量少了");
            //    return;
            //}
            if (info == null)
            {
                return;
            }
           //
            vehicles[(int)agvId].agvInfo = info;
           // Console.WriteLine(info.CurLocation.AgvAngle.Angle);
        }
        public void RandomMove(int Id)
        {

            MyPoint mpEnd =new MyPoint(3,3,Direction.Right);// RouteUtil.RandRealPoint(ElecMap.Instance);
            while (mpEnd.X == vehicles[Id].BeginX && mpEnd.Y == vehicles[Id].BeginY)
            {
                mpEnd = RouteUtil.RandRealPoint(ElecMap.Instance);
            }
            SendData sd = new SendData(Id, vehicles[Id].BeginX, vehicles[Id].BeginY, mpEnd.X, mpEnd.Y);
            sd.Arrive = false;
            sd.EndLoc = "rest";
            sd.State = State.carried;

            SearchRouteQueue.Instance.Enqueue(new SearchData(sd, false));


        }
        void SearchRoute(int num, bool isResarch)
        {
            SendData td = new SendData();
            td.Num = num;
            td.BeginX = vehicles[num].BeginX;
            td.BeginY = vehicles[num].BeginY;
            td.EndX = vehicles[num].EndX;
            td.EndY = vehicles[num].EndY;
            td.Arrive = false;
            td.EndLoc = vehicles[num].EndLoc;
            td.StartLoc = vehicles[num].StartLoc;
            td.State = vehicles[num].CurState;


            //if (!ElecMap.Instance.IsSpecialArea(td.BeginX, td.BeginY) && ElecMap.Instance.IsScanner(td.EndX, td.EndY))
            //{
            //    MessageBox.Show("起点：" + td.BeginX + "," + td.BeginY + "" + "终点：" + td.EndX + "," + td.EndY);
            //}
            SearchRouteQueue.Instance.Enqueue(new SearchData(td, isResarch));

            //Task.Factory.StartNew(() => vehicle[num].SearchRoute(Elc), TaskCreationOptions.LongRunning);
        }

        private static readonly object vehicleLock = new object();
        public Vehicle[] GetVehicles()
        {
            //Vehicle[] v = null;
            //lock (vehicleLock)
            //{
            //    if (vehicles != null)
            //    {
            //        v = new Vehicle[ConstDefine.g_VehicleCount];
            //        for (int i = 0; i < vehicles.Length; i++)
            //        {
            //            v[i] = vehicles[i].CloneDeep();
            //        }
            //    }
            //}
            //return v;
            return vehicles;
        }


    }

}
