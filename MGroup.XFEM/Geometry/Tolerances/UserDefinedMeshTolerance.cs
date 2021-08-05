using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Elements;

namespace MGroup.XFEM.Geometry.Tolerances
{
    public class UserDefinedMeshTolerance : IMeshTolerance
    {
        private readonly double tolerance;

        public UserDefinedMeshTolerance(double elementSize, double coeff = 1E-8)
        {
            this.tolerance = elementSize * coeff;
        }

        public double CalcTolerance(IXFiniteElement element) => tolerance;
    }
}
