using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;

namespace MGroup.XFEM.Cracks
{
    public interface IPropagator
    {
        PropagationLogger Logger { get; }

        (double growthAngle, double growthLength) Propagate(Dictionary<int, Vector> subdomainFreeDisplacements, 
            double[] globalCrackTip, TipCoordinateSystem tipSystem, IEnumerable<IXCrackElement> tipElements);
    }
}
