using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MGroup.XFEM.Cracks.PropagationCriteria
{
    public class ConstantIncrement2D : ICrackGrowthLengthCriterion
    {
        private readonly double length;

        /// <summary>
        /// 
        /// </summary>
        /// <param name="incrementLength">Usually 0.1 is used</param>
        public ConstantIncrement2D(double incrementLength)
        {
            if (incrementLength <= 0.0)
            {
                throw new ArgumentException("The increment must be positive, but was: " + incrementLength);
            }
            this.length = incrementLength;
        }

        public double ComputeGrowthLength(double[] modeSifs)
        {
            return length;
        }
    }
}
