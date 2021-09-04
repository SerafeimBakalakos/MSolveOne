using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Cracks.PropagationTermination;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;

//TODO: Extend to the case where there are 2 tips! Perhaps abstract the number of tips by using a general ICrackTip that can
//      have an implementation with 2 tips. 
//TODO: Perhaps use IDs instead of references to IXCrackElement.
namespace MGroup.XFEM.Cracks.Geometry
{
	public interface ICrack : IXDiscontinuity
	{
		/// <summary>
		/// Elements whose edges conform to the crack, instead of being intersected by it. This does not include elements 
		/// belonging to <see cref="TipElements"/>.
		/// </summary>
		HashSet<IXCrackElement> ConformingElements { get; }

		EnrichmentItem CrackBodyEnrichment { get; }

		IXGeometryDescription CrackGeometry { get; }

		EnrichmentItem CrackTipEnrichments { get; }

		/// <summary>
		/// Elements that are intersected by the crack, but do not belong to <see cref="TipElements"/>.
		/// </summary>
		HashSet<IXCrackElement> IntersectedElements { get; }


		double[] TipCoordinates { get; }

		/// <summary>
		/// Elements containing the crack tip, not elements whose nodes are enriched with crack tip functions. In 2D cracks there 
		/// is only 1 usually. However it is possible for the tip to lie on the boundary between multiple elements.
		/// </summary>
		HashSet<IXCrackElement> TipElements { get; }

		TipCoordinateSystemExplicit TipSystem { get; } //TODO: This should probably be provided by the Geometry property

		void CheckPropagation(IPropagationTermination termination);

		//TODO: Perhaps everything enrichment related should be calculated, stored and exposed by INodeEnricher.
		//      Also cracks are geometric components and do not necessarily have to define their enrichments. E.g. the exact same
		//      crack class should be usable for brittle and cohesive cracks, although the tip enrichment functions and the SIF
		//      calculation are different. For that matter the NodeEnricher component should be the same as well, as it just 
		//      locates which nodes to enrich with the crack's enrichments funcs
		IList<EnrichmentItem> DefineEnrichments(int numCurrentEnrichments);

		HashSet<XNode> FindNodesNearFront(double maxDistance);

		void InteractWithMesh(); 
	}
}
