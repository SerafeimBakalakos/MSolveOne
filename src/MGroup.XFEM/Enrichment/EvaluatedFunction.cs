using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Enrichment
{
    public class EvaluatedFunction
    {
        public EvaluatedFunction(double value, double[] cartesianDerivatives)
        {
            this.Value = value;
            this.CartesianDerivatives = cartesianDerivatives;
        }

        public double Value { get; }
        public double[] CartesianDerivatives { get; }
    }
}
