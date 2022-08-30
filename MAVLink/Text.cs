using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LibDrone.Frames
{
    internal class Text
    {
        public class Units : Attribute
        {
            public Units(string unit)
            {
                Unit = unit;
            }

            public string Unit { get; set; }
        }

        public class Description : Attribute
        {
            public Description(string desc)
            {
                Text = desc;
            }

            public string Text { get; set; }
        }
    }
}
