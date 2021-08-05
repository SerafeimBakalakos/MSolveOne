using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MGroup.XFEM.Cracks.PropagationCriteria
{
    // TODO: Perhaps it should be (-pi/2, pi/2)
    public interface ICrackGrowthDirectionCriterion
    {
        /// <summary>
        /// Computes the angle coordinate of the next crack tip in the local polar system of the current crack tip.
        /// The angle belongs to the interval (-pi, pi]. However very sharp direction changes 
        /// are suspicious.
        /// </summary>
        /// <param name="modeSifs">The stress intensity factors for all modes of fracture.</param>
        /// <returns>The angle belonging in the interval (-pi, pi] </returns>
        double ComputeGrowthAngle(double[] modeSifs);
    }
}
