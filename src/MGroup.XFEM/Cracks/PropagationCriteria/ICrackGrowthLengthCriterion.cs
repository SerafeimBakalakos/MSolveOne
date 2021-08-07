using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MGroup.XFEM.Cracks.PropagationCriteria
{
    // TODO: Not sure what arguments must be used tbh. Using Paris's law for the increment needs the SIFs.
    public interface ICrackGrowthLengthCriterion
    {
        /// <summary>
        /// Computes the length of the next crack growth increment.
        /// </summary>
        /// <param name="modeSifs">The stress intensity factors for all modes of fracture.</param>
        /// <returns>A positive double value</returns>
        double ComputeGrowthLength(double[] modeSifs);
    }
}
