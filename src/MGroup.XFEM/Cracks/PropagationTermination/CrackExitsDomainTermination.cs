using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Geometry.Boundaries;

namespace MGroup.XFEM.Cracks.PropagationTermination
{
    public class CrackExitsDomainTermination : IPropagationTermination
    {
        private readonly IDomainBoundary2D boundary;

        private bool mustTerminate;

        public CrackExitsDomainTermination(IDomainBoundary2D boundary)
        {
            this.boundary = boundary;
        }

        public string Description { get; } = "The crack has propagated throughout the domain and exited it.";

        public bool MustTerminate(ICrack crack)
        {
            crack.CheckPropagation(this);
            return mustTerminate;
        }

        public void Update(double[] sifs, double[] newCrackTip)
        {
            if (boundary.SurroundsPoint(newCrackTip)) mustTerminate = false;
            else mustTerminate = true;
        }
    }
}
