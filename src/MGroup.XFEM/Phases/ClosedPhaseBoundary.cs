using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Geometry.LSM;

namespace MGroup.XFEM.Phases
{
    /// <summary>
    /// Represents the closed boundary of a material phase. It cannot move.
    /// </summary>
    public class ClosedPhaseBoundary : IPhaseBoundary
    {
        public ClosedPhaseBoundary(int id, IClosedGeometry geometry, IPhase positivePhase, IPhase negativePhase)
        {
            this.ID = id;
            this.Geometry = geometry;
            this.PositivePhase = positivePhase;
            this.NegativePhase = negativePhase;
        }

        public int ID { get; }

        public EnrichmentItem StepEnrichment { get; set; } 

        public IPhase NegativePhase { get; set; }
        public IPhase PositivePhase { get; set; }

        public IClosedGeometry Geometry { get; }

        public ILsmGeometry LsmGeometry { get; }

        public void InitializeGeometry() 
        {
            // Initialized in constructor
        }

        public void UpdateGeometry(IAlgebraicModel algebraicModel, IGlobalVector totalDisplacements)
        {
            // This boundary does not move.
        }
    }
}
