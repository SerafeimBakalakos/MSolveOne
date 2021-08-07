using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MGroup.XFEM.Cracks.PropagationCriteria
{
    // TODO: Research whether this work for heterogenegous materials as well.
    public class MaximumCircumferentialTensileStressCriterion: ICrackGrowthDirectionCriterion
    {
        public MaximumCircumferentialTensileStressCriterion()
        {
        }

        public double ComputeGrowthAngle(double[] modeSifs)
        {
            if (modeSifs[0] > 0)
            {
                double ratio = modeSifs[1] / modeSifs[0];
                return 2 * Math.Atan((-2 * ratio) / (1 + Math.Sqrt(1 + 8 * ratio * ratio)));
            }
            else throw new NotImplementedException("SIF 1 = " + modeSifs[0] + " <= 0. What happens in this case?");
        }
    }
}
