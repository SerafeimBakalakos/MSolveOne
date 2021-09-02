using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Integration;

namespace MGroup.XFEM.Geometry.HybridFries
{
	public class ImplicitCrackElementInteraction : IElementDiscontinuityInteraction
	{
		private readonly IntersectionMesh intersectionMesh;

		public ImplicitCrackElementInteraction(int parentGeometryID, IXFiniteElement element,
			RelativePositionCurveElement relativePosition, bool frontInteractsWithElement, IntersectionMesh intersectionMesh)
		{
			this.ParentGeometryID = parentGeometryID;
			this.Element = element;
			if (relativePosition == RelativePositionCurveElement.Disjoint)
			{
				throw new ArgumentException("There is no intersection between the curve and element");
			}
			this.RelativePosition = relativePosition;
			this.BoundaryOfGeometryInteractsWithElement = frontInteractsWithElement;
			this.intersectionMesh = intersectionMesh;
		}

		public bool BoundaryOfGeometryInteractsWithElement { get; }

		public IXFiniteElement Element { get; }

		public int ParentGeometryID { get; }

		public RelativePositionCurveElement RelativePosition { get; }

		public IIntersectionMesh ApproximateGlobalCartesian()
		{
			throw new NotImplementedException();
		}

		public IReadOnlyList<GaussPoint> GetBoundaryIntegrationPoints(int order)
		{
			throw new NotImplementedException();
		}

		public IReadOnlyList<double[]> GetNormalsAtBoundaryIntegrationPoints(int order)
		{
			throw new NotImplementedException();
		}

		public IList<double[]> GetVerticesForTriangulation()
		{
			throw new NotImplementedException();
		}
	}
}
