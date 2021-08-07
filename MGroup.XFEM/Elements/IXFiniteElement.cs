using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.XFEM.ElementGeometry;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.ConformingMesh;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Interpolation;

//TODO: LSM/element interactions should probably be stored in a GeometricModel class
namespace MGroup.XFEM.Elements
{
	public interface IXFiniteElement : IElement, IElementType
	{
		/// <summary>
		/// Will be null for elements not intersected by any interfaces
		/// </summary>
		IElementSubcell[] ConformingSubcells { get; set; }

		ElementEdge[] Edges { get; }

		ElementFace[] Faces { get; }

		IBulkIntegration IntegrationBulk { get; }

		IQuadrature IntegrationStandard { get; }

		IIsoparametricInterpolation Interpolation { get; }

		IReadOnlyList<XNode> Nodes { get; }

		void SetSubdomainID(int subdomainID); //TODO: refactor this to be a property without hiding base member

		//TODO: Use a reference to the discontinuity itself instead of its ID
		Dictionary<int, IElementDiscontinuityInteraction> InteractingDiscontinuities { get; }

		double CalcBulkSizeCartesian();

		double CalcBulkSizeNatural();

		XPoint EvaluateFunctionsAt(double[] naturalPoint);

		double[] FindCentroidCartesian();

		void IdentifyDofs();

		void IdentifyIntegrationPointsAndMaterials();
	}

	//TODO: These should be converted to default interface implementations
	public static class XFiniteElementExtensions
	{
		public static HashSet<IEnrichmentFunction> FindEnrichments(this IXFiniteElement element)
		{
			var elementEnrichments = new HashSet<IEnrichmentFunction>();
			foreach (XNode node in element.Nodes) elementEnrichments.UnionWith(node.EnrichmentFuncs.Keys);
			return elementEnrichments;
		}

		public static bool HasEnrichedNodes(this IXFiniteElement element)
		{
			foreach (XNode node in element.Nodes)
			{
				if (node.IsEnriched) return true;
			}
			return false;
		}

		public static double[] FindCentroidNatural(this IXFiniteElement element)
		{
			IReadOnlyList<double[]> nodesNatural = element.Interpolation.NodalNaturalCoordinates;
			return Utilities.FindCentroid(nodesNatural);
		}
	}
}
