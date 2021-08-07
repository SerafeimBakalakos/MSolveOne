using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;

namespace MGroup.XFEM.Cracks
{
    public interface IPropagator
    {
        PropagationLogger Logger { get; }

        (double growthAngle, double growthLength) Propagate(IGlobalVector totalFreeDisplacements, 
            double[] globalCrackTip, TipCoordinateSystem tipSystem, IEnumerable<IXCrackElement> tipElements);
    }
}
