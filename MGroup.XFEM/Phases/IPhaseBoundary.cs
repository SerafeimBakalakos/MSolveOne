using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.LSM;

namespace MGroup.XFEM.Phases
{
    public interface IPhaseBoundary : IXDiscontinuity
    {
        IClosedGeometry Geometry { get; }

        EnrichmentItem StepEnrichment { get; set; } //TODO: Not only step enrichment. Also ramp or ridge.

        IPhase NegativePhase { get; set; }
        IPhase PositivePhase { get; set; }

    }
}
