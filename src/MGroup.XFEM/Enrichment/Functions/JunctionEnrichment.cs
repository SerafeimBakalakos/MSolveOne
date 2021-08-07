using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Phases;

namespace MGroup.XFEM.Enrichment.Functions
{
    public class JunctionEnrichment : IEnrichmentFunction
    {
        private readonly IPhase[] descendingPhases;
        private readonly int[] descendingPhaseCoeffs;

        public JunctionEnrichment(int id, ClosedPhaseBoundary boundary, IEnumerable<IPhase> allPhases)
        {
            this.ID = id;
            this.Dof = new EnrichedDof(this, ThermalDof.Temperature);
            this.Boundary = boundary;

            int numPhases = allPhases.Count();
            this.descendingPhases = new IPhase[numPhases];
            this.descendingPhaseCoeffs = new int[numPhases];
            this.descendingPhases[0] = boundary.PositivePhase; 
            this.descendingPhaseCoeffs[0] = +1;
            this.descendingPhases[1] = boundary.NegativePhase;
            this.descendingPhaseCoeffs[1] = -1;

            int i = 2;
            foreach (IPhase phase in allPhases)
            {
                if ((phase != boundary.PositivePhase) && (phase != boundary.NegativePhase))
                {
                    this.descendingPhases[i] = phase;
                    this.descendingPhaseCoeffs[i] = 0;
                    ++i;
                }
            }
            Array.Sort(descendingPhases, descendingPhaseCoeffs, new PhaseComparer());
        }

        public EnrichedDof Dof { get; }

        public int ID { get; }

        public ClosedPhaseBoundary Boundary { get; }

        public IReadOnlyList<IPhase> Phases => descendingPhases;

        public double EvaluateAt(XNode node)
        {
            for (int i = 0; i < descendingPhases.Length; ++i)
            {
                if (node.PhaseID == descendingPhases[i].ID) return descendingPhaseCoeffs[i];
            }
            throw new ArgumentException();
        }

        public double EvaluateAt(XPoint point)
        {
            for (int i = 0; i < descendingPhases.Length - 1; ++i)
            {
                if (descendingPhases[i].ID == point.PhaseID) return descendingPhaseCoeffs[i];
            }
            return descendingPhaseCoeffs[descendingPhases.Length - 1];

            //if (point.PhaseID != null) return EvaluateAt(point.Phase);

            //for (int i = 0; i < descendingPhases.Length - 1; ++i)
            //{
            //    if (descendingPhases[i].Contains(point)) return descendingPhaseCoeffs[i];
            //}
            //return descendingPhaseCoeffs[descendingPhases.Length - 1];
        }

        public EvaluatedFunction EvaluateAllAt(XPoint point)
        {
            for (int i = 0; i < descendingPhases.Length - 1; ++i)
            {
                if (descendingPhases[i].ID == point.PhaseID)
                {
                    return new EvaluatedFunction(descendingPhaseCoeffs[i], new double[point.Dimension]);
                }
            }
            return new EvaluatedFunction(descendingPhaseCoeffs[descendingPhases.Length - 1], new double[point.Dimension]);

            //if (point.Phase != null)
            //{
            //    double psi = EvaluateAt(point.Phase);
            //    return new EvaluatedFunction(psi, new double[point.Dimension]);
            //}

            //for (int i = 0; i < descendingPhases.Length - 1; ++i)
            //{
            //    if (descendingPhases[i].Contains(point))
            //    {
            //        return new EvaluatedFunction(descendingPhaseCoeffs[i], new double[point.Dimension]);
            //    }
            //}
            //return new EvaluatedFunction(descendingPhaseCoeffs[descendingPhases.Length - 1], new double[point.Dimension]);
        }
        public double EvaluateAt(IPhase phaseAtPoint)
        {
            //WARNING: This does not work for a blending element that is intersected by another unrelated to the junction interface
            for (int i = 0; i < descendingPhases.Length; ++i)
            {
                if (phaseAtPoint == descendingPhases[i]) return descendingPhaseCoeffs[i];
            }
            throw new ArgumentException();
        }

        public double EvaluateJumpAcross(IXDiscontinuity discontinuity, XPoint point)
        {
            if (discontinuity is ClosedPhaseBoundary phaseBoundary) //TODO: Find a better way than casting
            {
                return FindPhaseCoeff(phaseBoundary.PositivePhase) - FindPhaseCoeff(phaseBoundary.NegativePhase);
            }
            else return 0.0;
        }

        private int FindPhaseCoeff(IPhase phase)
        {
            for (int i = 0; i < 3; ++i)
            {
                if (descendingPhases[i] == phase) return descendingPhaseCoeffs[i];
            }
            throw new ArgumentException();
        }

        private class PhaseComparer : IComparer<IPhase>
        {
            public int Compare(IPhase x, IPhase y) => y.ID - x.ID; // Descending order
        }
    }
}
