using Agv.PathPlanning;
using AGV_V1._0.Util;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace AGV_V1._0
{
    partial class SearchProcess : Form
    {
        
        //public static readonly int FORM_WIDTH = System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.Width;   //框体的宽度
        //public static readonly int FORM_HEIGHT = System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.Height;    //框体的长度  

        private Bitmap surface = null;
        private Graphics g = null;
        private static Node[,] graph;
        private int nodeLength = 10;
        private static int startX;
        private static int startY;
        private static int endX;
        private static int endY;
        private static List<MyPoint> route = new List<MyPoint>();

        public SearchProcess()
        {
            InitializeComponent();
            

        }
        public static void SetGraph(Node[,] g,List<MyPoint>r, int startX1, int startY1, int endX1, int endY1)
        {
            startX = startX1;
            startY = startY1;
            endX = endX1;
            endY = endY1;
            graph = g;
            route = r;
        }
        void LoadView()
        {
            if (graph == null)
            {
                return;
            }
            // this.WindowState = FormWindowState.Maximized;
            nodeLength = (int)(this.Size.Width) /2/ (graph.GetLength(0) + 1);

            SetMapView();
            // SetInfoShowView();
        }
        void SetMapView()
        {
            int w = nodeLength * (graph.GetLength(0));
            int h = nodeLength * (graph.GetLength(1));
            //设置pictureBox的尺寸和位置
            pic.Location = Point.Empty;
            pic.Size = new Size(h, w);
            surface = new Bitmap(h,w);
            g = Graphics.FromImage(surface);
            //将pictureBox加入到panel上
            
            pic.BackColor = Color.FromArgb(100, 0, 0, 0);
            DrawMap();
           
        }
        void DrawMap()
        {

            if (graph == null)
            {
                return;
            }
            //横纵坐标的控制变量
            int point_x, point_y;

            //节点类型

            point_x = 0;
            point_y = 0;

            for (int i = 0; i < graph.GetLength(0); i++)
            {
                point_x = 0;
                for (int j = 0; j < graph.GetLength(1); j++)
                {
                    //graph[i, j] = new MapNode(point_x, point_y, Node_number, point_type);
                    graph[i, j].x = point_x;
                    graph[i, j].y = point_y;
                    point_x += nodeLength;

                }
                point_y += nodeLength;
            }

            for (int i = 0; i < graph.GetLength(0); i++)
            {
                for (int j = 0; j < graph.GetLength(1); j++)
                {
                    //drawArrow(i, j);
                    //绘制表格
                    if (graph[i, j].node_Type)
                    {
                        DrawUtil.FillRectangle(g, Color.LightGray, graph[i, j].x - 1, graph[i, j].y - 1, nodeLength - 2, nodeLength - 2);
                    }
                    else
                    {
                        DrawUtil.FillRectangle(g, Color.Black, graph[i, j].x - 1, graph[i, j].y - 1, nodeLength - 2, nodeLength - 2);
                    }

                    //绘制标尺
                    if (i == 0 || i == graph.GetLength(0) - 1)
                    {
                        DrawUtil.FillRectangle(g, Color.FromArgb(180, 0, 0, 0), graph[i, j].x - 1, graph[i, j].y - 1, nodeLength - 2, nodeLength - 2);
                        DrawUtil.DrawString(g, j, nodeLength / 2, Color.Yellow, graph[i, j].x - 1, graph[i, j].y - 1);
                    }
                    if (j == 0 || j == graph.GetLength(1) - 1)
                    {
                        DrawUtil.FillRectangle(g, Color.FromArgb(180, 0, 0, 0), graph[i, j].x - 1, graph[i, j].y - 1, nodeLength - 2, nodeLength - 2);
                        DrawUtil.DrawString(g, i, nodeLength / 2, Color.Yellow, graph[i, j].x - 1, graph[i, j].y - 1);
                    }

                    if (graph[i, j].isSearched)
                    {
                        DrawUtil.FillRectangle(g, Color.FromArgb(180, 0, 46, 0), graph[i, j].x - 1, graph[i, j].y - 1, nodeLength - 2, nodeLength - 2);
                    }
                    

                }
            }
            if (route != null)
            {
                for (int i = 0; i < route.Count; i++)
                {
                    DrawUtil.FillRectangle(g, Color.LightYellow, route[i].Y * nodeLength, route[i].X * nodeLength, nodeLength - 2, nodeLength - 2);
                }
            }

            DrawUtil.FillEllipse(g, Color.Yellow, startX*nodeLength, startY*nodeLength, nodeLength - 2, nodeLength - 2);
            DrawUtil.FillRectangle(g, Color.Red, endX*nodeLength, endY*nodeLength, nodeLength - 2, nodeLength - 2);

            pic.Image = surface;

        }

        private void SearchProcess_Paint(object sender, PaintEventArgs e)
        {
           
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            LoadView();
        }
    }
}
