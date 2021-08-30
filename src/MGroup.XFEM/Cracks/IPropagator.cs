using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;

namespace MGroup.XFEM.Cracks
{
    public interface IPropagator
    {
        (double growthAngle, double growthLength) Propagate(IAlgebraicModel algebraicModel, IGlobalVector totalDisplacements, 
            double[] crackTip, double[] extensionVector, IEnumerable<IXCrackElement> tipElements);
    }
}
