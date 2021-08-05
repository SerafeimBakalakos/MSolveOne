using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Phases;

namespace MGroup.XFEM.Elements
{
    public interface IXMultiphaseElement : IXFiniteElement
    {
        IReadOnlyList<GaussPoint> BulkIntegrationPoints { get; } //TODO: This should probably be in IXFiniteElement

        //TODO: This should probably be in IXFiniteElement.
        //TODO: This is only used for cohesive interfaces. Otherwise their computation is avoided. Not the best design.
        IReadOnlyList<GaussPoint> BoundaryIntegrationPoints { get; }

        //TODO: This should probably be in IXFiniteElement.
        //TODO: This is only used for cohesive interfaces. Otherwise their computation is avoided. Not the best design.
        IReadOnlyList<double[]> BoundaryIntegrationPointNormals { get; }


        HashSet<IPhase> Phases { get; }

        //TODO: If the interfaces are not cohesive, this is not used in the XFEM analysis. Integration classes use
        //      Dictionary<int, IElementDiscontinuityInteraction> IXFiniteElement.InteractingDiscontinuities. Therefore, there is
        //      no reason to compute and store this for the XFEM analysis. Perhaps we do need it for the geometric classes though.
        Dictionary<IPhaseBoundary, IElementDiscontinuityInteraction> PhaseIntersections { get; }
    }

    public static class XMultiphaseElementExtensions
    {
        /// <summary>
        /// Finds the phase that contains <paramref name="point"/>. In addition <see cref="XPoint.PhaseID"/> of 
        /// <paramref name="point"/> is set to <see cref="IPhase.ID"/> of the phase found.
        /// </summary>
        /// <param name="element"></param>
        /// <param name="point"></param>
        /// <returns></returns>
        public static IPhase FindPhaseAt(this IXMultiphaseElement element, XPoint point)
        {
            // If there is only 1 phase, then that will be returned
            Debug.Assert(element.Phases.Count > 0);
            if (element.Phases.Count == 1)
            {
                IPhase phase = element.Phases.First();
                point.PhaseID = phase.ID;
                return phase;
            }

            // Search which phase contains this point
            IPhase defaultPhase = null;
            foreach (IPhase phase in element.Phases)
            {
                // Avoid searching for the point in the default phase, since its shape is highly irregular.
                if (phase is DefaultPhase)
                {
                    defaultPhase = phase;
                    continue;
                }
                else if (phase.Contains(point))
                {
                    point.PhaseID = phase.ID;
                    return phase;
                }
            }

            // If the point is not contained in any other phases, it must be in the default phase 
            if (defaultPhase == null)
            {
                throw new ArgumentException("The provided point does not belong to any of this element's phases");
            }
            else
            {
                point.PhaseID = defaultPhase.ID;
                return defaultPhase;
            }
        }
    }
}
