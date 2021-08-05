using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Cracks.PropagationTermination
{
    public interface IEquivalentSifCalculator
    {
        double CalculateEquivalentSIF(double[] modeSifs);
    }
}
