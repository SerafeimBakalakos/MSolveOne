using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Phases;

namespace MGroup.XFEM.Enrichment.Functions
{
    public class PhaseStepEnrichmentOLD : IEnrichmentFunction
    {
        private readonly IPhase internalPhase, externalPhase;
        private readonly IPhase minPhase, maxPhase;

        public PhaseStepEnrichmentOLD(IPhase internalPhase, IPhase externalPhase)
        {
            this.internalPhase = internalPhase;
            this.externalPhase = externalPhase;
            (this.minPhase, this.maxPhase) = FindMinMaxPhases(internalPhase, externalPhase);
            this.Dof = new EnrichedDof(this, ThermalDof.Temperature);
        }

        public EnrichedDof Dof { get; }

        public IReadOnlyList<IPhase> Phases => new IPhase[] { maxPhase, minPhase };

        public double EvaluateAt(XNode node)
        {
            if (internalPhase.ID == node.PhaseID) return -1.0;
            else return 1.0;
        }

        public double EvaluateAt(XPoint point)
        {
            //if (point.Phase != null) return EvaluateAt(point.Phase);

            if (internalPhase.Contains(point)) return -1;
            else return +1;
        }

        public EvaluatedFunction EvaluateAllAt(XPoint point)
        {
            if (internalPhase.Contains(point))
            {
                return new EvaluatedFunction(-1, new double[point.Dimension]);
            }
            else
            {
                return new EvaluatedFunction(+1, new double[point.Dimension]);
            }
        }

        public double EvaluateJumpAcross(IXDiscontinuity discontinuity, XPoint point)
        {
            //TODO: Cannot I just store the boundary, for which this enrichment is applied an then check reference equality?
            if (discontinuity is ClosedPhaseBoundary phaseBoundary) //TODO: Find a better way than casting
            {
                (IPhase boundaryMinPhase, IPhase boundaryMaxPhase) =
                    FindMinMaxPhases(phaseBoundary.PositivePhase, phaseBoundary.NegativePhase);
                if ((boundaryMinPhase == this.minPhase) && (boundaryMaxPhase == this.maxPhase))
                {
                    return 2.0; // +1 - (-1)
                }
                else return 0.0;
            }
            else return 0.0;
        }

        private static (IPhase minPhase, IPhase maxPhase) FindMinMaxPhases(IPhase phase1, IPhase phase2)
        {
            IPhase minPhase, maxPhase;
            if (phase1.ID < phase2.ID)
            {
                minPhase = phase1;
                maxPhase = phase2;
            }
            else
            {
                minPhase = phase2;
                maxPhase = phase1;
            }
            return (minPhase, maxPhase);
        }
    }
}
