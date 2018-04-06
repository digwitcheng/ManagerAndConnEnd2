using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Algorithm.Dstar2
{
    class Cell
    {
        public int x = 0;
        public int y = 0;
        public KeyValuePair<Double, Double> pairK = new KeyValuePair<double, double>(0.0, 0.0);




        public override string ToString()
        {
            return "[" + x + "," + y + String.Format("| %.2f", pairK.Key) + "," + String.Format("%.2f", pairK.Value) + "]\r\n";
            //		return "[" + x + "," +y + "]";
        }
        //Default constructor
        public Cell()
        {

        }

        //Overloaded constructor
        public Cell(int x, int y, KeyValuePair<Double, Double> k)
        {
            this.x = x;
            this.y = y;
            this.pairK = k;
        }

        //Overloaded constructor
        public Cell(Cell other)
        {
            this.x = other.x;
            this.y = other.y;
            this.pairK = other.pairK;
        }

        //Equals
        public bool eq( Cell s2)
        {
            return ((this.x == s2.x) && (this.y == s2.y));
        }

        //Not Equals
        public bool neq( Cell s2)
        {
            return ((this.x != s2.x) || (this.y != s2.y));
        }

        //Greater than
        public bool gt(Cell s2)
        {
            if (pairK.Key- 0.00001 > s2.pairK.Key) return true;
            else if (pairK.Key< s2.pairK.Key- 0.00001) return false;
            return pairK.Value > s2.pairK.Value;
        }

        //Less than or equal to
        public bool lte(Cell s2)
        {
            if (pairK.Key< s2.pairK.Key) return true;
            else if (pairK.Key> s2.pairK.Key) return false;
            return pairK.Value < s2.pairK.Value + 0.00001;
        }

        //Less than
        public bool lt( Cell s2)
        {
            if (pairK.Key+ 0.000001 < s2.pairK.Key) return true;
            else if (pairK.Key- 0.000001 > s2.pairK.Key) return false;
            return pairK.Value < s2.pairK.Value;
        }

        //CompareTo Method. This is necessary when this class is used in a priority queue
        public int compareTo(Object that)
        {
            //This is a modified version of the gt method
            Cell other = (Cell)that;
            if (pairK.Key- 0.00001 > other.pairK.Key) return 1;
            else if (pairK.Key< other.pairK.Key- 0.00001) return -1;
            if (pairK.Value > other.pairK.Value) return 1;
            else if (pairK.Value < other.pairK.Value) return -1;
            return 0;
            //		State other=(State)that;
            //		return x-other.x;

        }        
        public override int GetHashCode()
        {
            return this.x + 34245 * this.y;
        }
        public override bool Equals(object aThat)
        {
            //check for self-comparison
            if (this == aThat) return true;

            //use instanceof instead of getClass here for two reasons
            //1. if need be, it can match any supertype, and not just one class;
            //2. it renders an explict check for "that == null" redundant, since
            //it does the check for null already - "null instanceof [type]" always
            //returns false. (See Effective Java by Joshua Bloch.)
            if (!(aThat is Cell) ) return false;
            //Alternative to the above line :
            //if ( aThat == null || aThat.getClass() != this.getClass() ) return false;

            //cast to native object is now safe
            Cell that = (Cell)aThat;

            //now a proper field-by-field evaluation can be made
            if (this.x == that.x && this.y == that.y) return true;
            return false;

        }
    }
}
