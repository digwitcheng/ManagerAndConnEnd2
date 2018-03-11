namespace Agv.PathPlanning
{
    class Close
    {
       private  Node node;

        internal Node Node
        {
            get { return node; }
            set { node = value; }
        }
      
       private   Close from;

        internal Close From
        {
            get { return from; }
            set { from = value; }
        }
        private  float f,g,h;  

        public float G
        {
            get { return g; }
            set { g = value; }
        }

        public float F
        {
            get { return f; }
            set { f = value; }
        }
       public float H
        {
            get { return h; }
            set { h = value; }
        }

      
    }
}
