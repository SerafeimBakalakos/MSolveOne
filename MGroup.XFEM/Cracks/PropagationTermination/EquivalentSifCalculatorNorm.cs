using System;
using System.Collections.Generic;
using System.Text;

//TODO: Also add Tanaka_1974 version and unit tests, which are trivial
namespace MGroup.XFEM.Cracks.PropagationTermination
{
    public class EquivalentSifCalculatorNorm : IEquivalentSifCalculator
    {
        public double CalculateEquivalentSIF(double[] modeSifs)
        {
            double sumSquares = 0.0;
            for (int i = 0; i < modeSifs.Length; ++i)
            {
                sumSquares += modeSifs[i] * modeSifs[i];
            }
            return Math.Sqrt(sumSquares);
        }
    }
}
