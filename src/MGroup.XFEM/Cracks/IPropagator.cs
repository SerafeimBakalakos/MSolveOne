using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Geometry.HybridFries;

namespace MGroup.XFEM.Cracks
{
    public interface IPropagator
    {
        (double growthAngle, double growthLength) Propagate(IAlgebraicModel algebraicModel, IGlobalVector totalDisplacements, 
            ICrackTipSystem crackTipSystem);
    }
}
